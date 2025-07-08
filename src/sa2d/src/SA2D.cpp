/////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2024, Precision Innovations Inc.
// All rights reserved.
//
// BSD 3-Clause License
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
///////////////////////////////////////////////////////////////////////////////

#include "sa2d/SA2D.h"

#include "dpl/Opendp.h"
#include "infrastructure/network.h"
#include "ThreadSafeGrid.h"
#include "Worker.h"
#include "WorkerManager.h"
#include "odb/db.h"
#include "odb/util.h"
#include "utl/Logger.h"
#include <algorithm>

namespace sa2d {

SA2D::SA2D()
{
  grid_info_ = std::make_unique<ImmutableGridInfo>();
}

SA2D::~SA2D() = default;

void SA2D::init(odb::dbDatabase* db, utl::Logger* logger)
{
  db_ = db;
  logger_ = logger;
  // Don't access block here, it might not be loaded yet
  // We'll get it when actually needed in runSA()
  
  logger_->info(utl::SA2D, 1, "SA2D initialized");
}

void SA2D::setMaxDisplacement(int max_displacement_x, int max_displacement_y)
{
  max_displacement_x_ = max_displacement_x;
  max_displacement_y_ = max_displacement_y;
  
  logger_->info(utl::SA2D, 2, "Max displacement set to ({}, {}) sites", 
                max_displacement_x_, max_displacement_y_);
}

void SA2D::runSA()
{
  if (!dpl_) {
    logger_->error(utl::SA2D, 3, "DPL engine not set. Call setDplEngine() first.");
  }
  
  // Get the current block
  if (!db_ || !db_->getChip()) {
    logger_->error(utl::SA2D, 4, "No chip found in database.");
  }
  
  block_ = db_->getChip()->getBlock();
  if (!block_) {
    logger_->error(utl::SA2D, 4, "No design block found.");
  }
  
  logger_->info(utl::SA2D, 5, "Starting SA2D placement optimization");
  logger_->info(utl::SA2D, 6, "Parameters:");
  logger_->info(utl::SA2D, 7, "  Max temperature: {}", max_temp_);
  logger_->info(utl::SA2D, 8, "  Min temperature: {}", min_temp_);
  logger_->info(utl::SA2D, 9, "  Cooling rate: {}", cooling_rate_);
  logger_->info(utl::SA2D, 10, "  Max iterations: {}", max_iter_);
  logger_->info(utl::SA2D, 11, "  Max displacement: ({}, {}) sites", 
                max_displacement_x_, max_displacement_y_);
  logger_->info(utl::SA2D, 12, "  Move budget: {}", move_budget_);
  logger_->info(utl::SA2D, 13, "  Random seed: {}", seed_);
  
  // Initialize grid from DPL using public getters
  if (!dpl_) {
    logger_->error(utl::SA2D, 14, "DPL engine not set");
  }
  
  auto* dpl_grid = dpl_->getGrid();
  auto* dpl_arch = dpl_->getArchitecture();
  auto* dpl_network = dpl_->getNetwork();
  
  if (!dpl_grid || !dpl_arch || !dpl_network) {
    logger_->error(utl::SA2D, 15, "DPL not properly initialized. Run detailed placement first.");
  }
  
  // Store DPL references
  arch_ = dpl_arch;
  network_ = dpl_network;
  
  logger_->info(utl::SA2D, 16, "Initializing SA2D grid from DPL");
  
  // Initialize immutable grid info from DPL
  grid_info_->initFromDPL(dpl_grid, arch_, block_, logger_);
  
  // Debug logging
  logger_->info(utl::SA2D, 20, "Grid initialized: {} rows x {} sites, site_width = {} DBU",
                grid_info_->getRowCount(), 
                grid_info_->getRowSiteCount(),
                grid_info_->getSiteWidth());
  
  // Debug: Log core area (moved after grid init to have site_width available)
  odb::Rect core = block_->getCoreArea();
  logger_->info(utl::SA2D, 24, "Core area: ({}, {}) to ({}, {})",
                core.xMin(), core.yMin(), core.xMax(), core.yMax());
  
  // Check if core origin is site-aligned
  if (grid_info_->getSiteWidth() > 0 && core.xMin() % grid_info_->getSiteWidth() != 0) {
    logger_->warn(utl::SA2D, 25, "Core xMin {} is not site-aligned (site width = {})",
                  core.xMin(), grid_info_->getSiteWidth());
  }
  
  // Choose between single and parallel execution
  if (num_workers_ == 1) {
    runSingleWorkerSA();
  } else {
    runParallelSA();
  }
  
  logger_->info(utl::SA2D, 22, "SA2D placement optimization completed");
}

void SA2D::runSingleWorkerSA()
{
  logger_->info(utl::SA2D, 17, "Running single-worker SA");
  
  SAWorker* worker = new SAWorker(this, 0);
  worker->initFromDPL(network_, arch_, grid_info_.get());
  worker->setSeed(seed_);
  worker->setMaxDisplacement(max_displacement_x_, max_displacement_y_);
  worker->setTemp(max_temp_);
  worker->setCoolingRate(cooling_rate_);
  worker->setMaxIter(max_iter_);
  worker->setMoveBudget(move_budget_);
  
  // Report initial state
  logger_->info(utl::SA2D, 18, "Initial HPWL: {:.1f} u", 
                block_->dbuToMicrons(worker->getTotalHPWL()));
    
  // Debug: Compare with odb::WireLengthEvaluator
  odb::WireLengthEvaluator eval(block_);
  int64_t odb_hpwl = eval.hpwl();
  logger_->info(utl::SA2D, 19, "ODB HPWL check: {:.1f} u (difference: {:.1f} u)", 
                block_->dbuToMicrons(odb_hpwl),
                block_->dbuToMicrons(odb_hpwl - worker->getTotalHPWL()));
  
  // Run SA on single worker
  worker->run();
  
  // Apply results back to DPL
  logger_->info(utl::SA2D, 21, "Applying best solution from SA2D to DPL network");
  worker->applyToDPL(network_);
  
  // Update database instance locations (similar to what DPL does)
  // We need to do this manually since updateDbInstLocations() is private
  int updated_count = 0;
  odb::Rect core = block_->getCoreArea();
  for (auto& cell : network_->getNodes()) {
      if (!cell->isFixed() && cell->isStdCell()) {
          odb::dbInst* db_inst = cell->getDbInst();
          // Only move the instance if necessary to avoid triggering callbacks
          if (db_inst->getOrient() != cell->getOrient()) {
              db_inst->setOrient(cell->getOrient());
          }
          const int x = core.xMin() + cell->getLeft().v;
          const int y = core.yMin() + cell->getBottom().v;
          int inst_x, inst_y;
          db_inst->getLocation(inst_x, inst_y);
          if (x != inst_x || y != inst_y) {
              db_inst->setLocation(x, y);
              updated_count++;
          }
      }
  }
  logger_->info(utl::SA2D, 22, "Updated {} instance locations in database", updated_count);
  
  // Calculate final HPWL using ODB to verify
  odb::WireLengthEvaluator eval_final(block_);
  int64_t final_odb_hpwl = eval_final.hpwl();
  logger_->info(utl::SA2D, 20, "Final ODB HPWL: {:.1f} u (SA2D reported: {:.1f} u, difference: {:.1f} u)", 
                block_->dbuToMicrons(final_odb_hpwl),
                block_->dbuToMicrons(worker->getBestHPWL()),
                block_->dbuToMicrons(final_odb_hpwl - worker->getBestHPWL()));
  
  delete worker;
}

void SA2D::runParallelSA()
{
  logger_->info(utl::SA2D, 301, "Starting parallel SA with {} workers", num_workers_);
  logger_->info(utl::SA2D, 302, "GWTW interval: {}, elite ratio: {}", gwtw_interval_, elite_ratio_);
  
  // Create worker manager
  worker_manager_ = std::make_unique<WorkerManager>(num_workers_, this);
  worker_manager_->setGWTWInterval(gwtw_interval_);
  worker_manager_->setEliteRatio(elite_ratio_);
  
  // Initialize all workers
  worker_manager_->initializeWorkers(network_, grid_info_.get(),
                                     seed_, max_displacement_x_, max_displacement_y_,
                                     max_temp_, cooling_rate_, max_iter_, move_budget_);
  
  // Report initial state
  auto initial_costs = worker_manager_->getWorkerCosts();
  int64_t min_initial = *std::min_element(initial_costs.begin(), initial_costs.end());
  logger_->info(utl::SA2D, 303, "Initial HPWL (best worker): {:.1f} u", 
                block_->dbuToMicrons(min_initial));
  
  // Run parallel SA with GWTW
  int total_iterations = max_iter_;
  int sync_iterations = gwtw_interval_;
  
  for (int iter = 0; iter < total_iterations; iter += sync_iterations) {
      int remaining = std::min(sync_iterations, total_iterations - iter);
      
      // Run workers for sync_iterations
      worker_manager_->runWorkers(remaining);
      
      // Perform GWTW synchronization
      if (iter + sync_iterations < total_iterations) {  // Don't sync on last iteration
          worker_manager_->performGWTW();
      }
      
      // Report progress
      worker_manager_->reportProgress(iter + remaining);
  }
  
  // Apply best solution
  logger_->info(utl::SA2D, 304, "Applying best solution to DPL network");
  worker_manager_->applyBestSolution(network_);
  
  // Update database instance locations
  int updated_count = 0;
  odb::Rect core = block_->getCoreArea();
  for (auto& cell : network_->getNodes()) {
      if (!cell->isFixed() && cell->isStdCell()) {
          odb::dbInst* db_inst = cell->getDbInst();
          if (db_inst->getOrient() != cell->getOrient()) {
              db_inst->setOrient(cell->getOrient());
          }
          const int x = core.xMin() + cell->getLeft().v;
          const int y = core.yMin() + cell->getBottom().v;
          int inst_x, inst_y;
          db_inst->getLocation(inst_x, inst_y);
          if (x != inst_x || y != inst_y) {
              db_inst->setLocation(x, y);
              updated_count++;
          }
      }
  }
  logger_->info(utl::SA2D, 305, "Updated {} instance locations in database", updated_count);
  
  // Calculate final HPWL using ODB to verify
  odb::WireLengthEvaluator eval_final(block_);
  int64_t final_odb_hpwl = eval_final.hpwl();
  logger_->info(utl::SA2D, 306, "Final ODB HPWL: {:.1f} u (parallel SA best: {:.1f} u)", 
                block_->dbuToMicrons(final_odb_hpwl),
                block_->dbuToMicrons(worker_manager_->getBestCost()));
}

void SA2D::initTestGrid()
{
  // Temporary grid initialization for testing
  // This creates a minimal grid based on the core area
  
  odb::Rect core = block_->getCoreArea();
  
  // Find first row to get site information
  odb::dbRow* first_row = nullptr;
  for (auto row : block_->getRows()) {
    if (row->getSite()->getClass() != odb::dbSiteClass::PAD) {
      first_row = row;
      break;
    }
  }
  
  if (!first_row) {
    logger_->error(utl::SA2D, 14, "No rows found in design");
  }
  
  int site_width = first_row->getSite()->getWidth();
  int site_height = first_row->getSite()->getHeight();
  
  logger_->info(utl::SA2D, 15, "Test grid: core ({}, {}) to ({}, {}), site {}x{}", 
                core.xMin(), core.yMin(), core.xMax(), core.yMax(),
                site_width, site_height);
  
  // For now, just log that we would initialize the grid
  // Real implementation needs access to DPL internals
}

}  // namespace sa2d 