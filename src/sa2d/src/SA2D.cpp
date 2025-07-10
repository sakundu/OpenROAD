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
  logger_->info(utl::SA2D, 6, "Parameters: temp={:.1f}, cooling={}, iter={}, displacement=({},{}) sites, seed={}",
                max_temp_, cooling_rate_, max_iter_, 
                max_displacement_x_, max_displacement_y_, seed_);
  
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
  
  // Initialize immutable grid info from DPL
  grid_info_->initFromDPL(dpl_grid, arch_, block_, logger_);
  
  // Report initial HPWL from ODB before starting SA
  odb::WireLengthEvaluator eval_initial(block_);
  int64_t initial_odb_hpwl = eval_initial.hpwl();
  logger_->info(utl::SA2D, 23, "Initial HPWL (ODB): {:.1f} u", 
                block_->dbuToMicrons(initial_odb_hpwl));
  
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
  worker->setMovesPerIter(moves_per_iter_);
  
  // Set LSMC parameters
  worker->setKickInterval(kick_interval_);
  worker->setKickThreshold(kick_threshold_);
  worker->setKickStrength(kick_strength_);
  worker->setKickTempMultiplier(kick_temp_multiplier_);
  worker->setEnableKicks(enable_kicks_);
  worker->setEnableChainMoves(enable_chain_moves_);
  worker->setChainMoveInterval(chain_move_interval_);
  worker->setChainMovesPerRound(chain_moves_per_round_);
  
  // Report initial state
  int64_t initial_hpwl = worker->getTotalHPWL();
  logger_->info(utl::SA2D, 18, "Initial HPWL: {:.1f} u", 
                block_->dbuToMicrons(initial_hpwl));
  
  // Run SA on single worker
  worker->run();
  
  // Apply results back to DPL
  worker->applyToDPL(network_);
  
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
  
  // Calculate final HPWL
  odb::WireLengthEvaluator eval_final(block_);
  int64_t final_odb_hpwl = eval_final.hpwl();
  logger_->info(utl::SA2D, 20, "Final HPWL: {:.1f} u (improvement: {:.2f}%)", 
                block_->dbuToMicrons(final_odb_hpwl),
                100.0 * (1.0 - (double)final_odb_hpwl / initial_hpwl));
  
  delete worker;
}

void SA2D::runParallelSA()
{
  logger_->info(utl::SA2D, 301, "Running parallel SA with {} workers", num_workers_);
  
  // Create worker manager
  worker_manager_ = std::make_unique<WorkerManager>(num_workers_, this);
  worker_manager_->setGWTWInterval(gwtw_interval_);
  worker_manager_->setEliteRatio(elite_ratio_);
  
  // Initialize all workers
  worker_manager_->initializeWorkers(network_, grid_info_.get(),
                                     seed_, max_displacement_x_, max_displacement_y_,
                                     max_temp_, cooling_rate_, max_iter_, move_budget_,
                                     moves_per_iter_,
                                     // LSMC parameters
                                     kick_interval_, kick_threshold_, kick_strength_,
                                     kick_temp_multiplier_, enable_kicks_,
                                     enable_chain_moves_, chain_move_interval_,
                                     chain_moves_per_round_);
  
  // Report initial state
  auto initial_costs = worker_manager_->getWorkerCosts();
  int64_t min_initial = *std::min_element(initial_costs.begin(), initial_costs.end());
  logger_->info(utl::SA2D, 303, "Initial HPWL: {:.1f} u", 
                block_->dbuToMicrons(min_initial));
  
  // Run parallel SA with GWTW
  int total_iterations = max_iter_;
  int sync_iterations = gwtw_interval_;
  
  // Print initial progress header
  worker_manager_->reportProgress(0, total_iterations);
  
  for (int iter = 0; iter < total_iterations; iter += sync_iterations) {
      int remaining = std::min(sync_iterations, total_iterations - iter);
      
      // Run workers for sync_iterations
      worker_manager_->runWorkers(remaining);
      
      // Perform GWTW synchronization
      if (iter + sync_iterations < total_iterations) {  // Don't sync on last iteration
          worker_manager_->performGWTW();
      }
      
      // Report progress
      worker_manager_->reportProgress(iter + remaining, total_iterations);
  }
  
  // IMPORTANT: Update best solution tracking after final iteration
  // Workers may have found better solutions after the last GWTW sync
  worker_manager_->updateGlobalBest();
  
  // Debug: Report which worker has the best solution
  logger_->info(utl::SA2D, 308, "Best solution found by worker {} with HPWL: {:.1f} u", 
                worker_manager_->getBestWorkerId(),
                block_->dbuToMicrons(worker_manager_->getGlobalBestCost()));
  
  // Apply best solution
  worker_manager_->applyBestSolution(network_);
  
  // Verify: Calculate HPWL from DPL network before updating ODB
  // Simple HPWL calculation from DPL network (for verification)
  // This mimics what the worker does but using DPL's current state
  logger_->info(utl::SA2D, 309, "DPL network updated, proceeding to update ODB instances");
  
  // Report kick statistics
  worker_manager_->reportMoveStatistics();
  
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
  
  // Calculate final HPWL using ODB to verify
  odb::WireLengthEvaluator eval_final(block_);
  int64_t final_odb_hpwl = eval_final.hpwl();
  
  logger_->info(utl::SA2D, 307, "");  // Empty line for separation
  logger_->info(utl::SA2D, 306, "Final HPWL: {:.1f} u (improvement: {:.2f}%)", 
                block_->dbuToMicrons(final_odb_hpwl),
                100.0 * (1.0 - (double)final_odb_hpwl / min_initial));
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