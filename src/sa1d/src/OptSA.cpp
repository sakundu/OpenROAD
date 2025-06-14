/////////////////////////////////////////////////////////////////////////////
// Authors: Sayak Kundu (sakundu@ucsd.edu), Zhiang Wang (zhw033@ucsd.edu)
//          Dooseok Yoon (d3yoon@ucsd.edu)
// Copyright (c) 2024, The Regents of the University of California
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

#include "odb/dbTypes.h"
#include "sa1d/Objects.h"
#include "sa1d/OptSA.h"
#include "Worker.h"

#include <cfloat>
#include <cmath>
#include <iostream>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <string>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <omp.h>
#include "odb/util.h"
#include "utl/Logger.h"

using odb::dbOrientType;
using odb::Point;
using odb::Rect;
using utl::SA1D;

namespace sa1d {

OptSA::OptSA()
{
}

OptSA::~OptSA() = default;

void OptSA::init(odb::dbDatabase* db, utl::Logger* logger)
{
  db_ = db;
  logger_ = logger;
}

// --------------------------------------------------------------------------
// Hyperparameter settings
// --------------------------------------------------------------------------
void OptSA::setNumWorkers(int num_workers) {
  num_workers_ = num_workers;
}

void OptSA::setNumThreads(int num_threads) {
  num_threads_ = num_threads;
}

void OptSA::setMoveProbs(std::vector<std::vector<float> > move_probs) 
{ 
  move_probs_ = move_probs; 
}

void OptSA::setMaxTemp(float max_temp)
{
  max_temp_ = max_temp;
}

void OptSA::setMinTemp(float min_temp)
{
  min_temp_ = min_temp;
}

void OptSA::setCoolingRate(float cooling_rate)
{
  cooling_rate_ = cooling_rate;
}

void OptSA::setMaxIter(int max_iter)
{
  max_iter_ = max_iter;
}

void OptSA::setMoveBudget(int move_budget)
{
  move_budget_ = move_budget;
}

void OptSA::setNumMovePerIter(int num_move_per_iter)
{
  num_move_per_iter_ = num_move_per_iter;
}


void OptSA::setGWTWIter(int gwtw_iter)
{
  gwtw_iter_ = gwtw_iter;
}

void OptSA::setMaxTempDerateFactor(float derate_factor)
{
  max_temp_derate_factor_ = derate_factor;
}
  
void OptSA::setTopK(int top_k) 
{
  top_k_ = top_k;
}

void OptSA::setTopKRatio(std::vector<float> top_k_ratio)
{
  top_k_ratio_ = top_k_ratio;
}

void OptSA::setSyncFreq(float sync_freq)
{
  sync_freq_ = sync_freq;
}  

void OptSA::setSeed(int seed)
{
  seed_ = seed;
}

void OptSA::setIncrementalFlag(bool incremantal_flag)
{
  incremantal_flag_ = incremantal_flag;
}

void OptSA::setSAParams(const char* filename)
{
  // Read the SA parameters from the json file
  // and set the parameters
  using boost::property_tree::ptree;
  ptree pt;

  // Read the JSON file into the property tree
  try {
    boost::property_tree::read_json(filename, pt);
  } catch (const std::exception& e) {
    logger_->error(utl::SA1D, 9, "Error reading JSON file: {}", e.what());
    return;
  }

  num_workers_ = pt.get<int>("num_workers");
  num_threads_ = pt.get<int>("num_threads");
  
  move_probs_.clear();
  for (const auto& row : pt.get_child("move_probs")) {
    std::vector<float> inner_vector;
    for (const auto& value : row.second) {
      inner_vector.push_back(value.second.get_value<float>());
    }
    move_probs_.push_back(inner_vector);
  }

  max_temp_ = pt.get<float>("max_temp");
  min_temp_ = pt.get<float>("min_temp"); 
  cooling_rate_ = pt.get<float>("cooling_rate");
  max_iter_ = pt.get<int>("max_iter");
  move_budget_ = pt.get<int>("move_budget"); 
  num_move_per_iter_ = move_budget_ / max_iter_;
  gwtw_iter_ = pt.get<int>("gwtw_iter");
  max_temp_derate_factor_ = pt.get<float>("max_temp_derate_factor"); 
  top_k_ = pt.get<int>("top_k"); 

  top_k_ratio_.clear();
  for (const auto& value : pt.get_child("top_k_ratio")) {
    top_k_ratio_.push_back(value.second.get_value<float>());
  }

  sync_freq_ = pt.get<float>("sync_freq"); 
  seed_ = pt.get<int>("seed"); 
  incremantal_flag_ = pt.get<bool>("incremental_flag");
}

void OptSA::checkParams()
{
  // print all the settings
  logger_->report("[Params] num_workers = {}", num_workers_);
  logger_->report("[Params] num_threads = {}", num_threads_);
  
  if (move_probs_.size() == num_workers_) {
    logger_->report("Each worker will be assigned with a unique move probability vector !");
    for (int worker_id = 0; worker_id < num_workers_; worker_id++) {
      auto& work_prob = move_probs_[worker_id];
      if ((std::accumulate(work_prob.begin(), work_prob.end(), 0.0) - 1.0) <= 0.00001) {
        if (work_prob.size() == num_move_types) {
          std::string info = "[Params] Move probability vector for worker ";
          info += std::to_string(worker_id);
          info += " is : [ ";
          for (auto& prob : work_prob) {
            info += std::to_string(prob) + " ";
          }
          info += "]";
          logger_->report(info);
        } else {
          logger_->error(utl::SA1D, 6, "The size of move probability dose not match number of move types !");
        }
      } else {
        logger_->error(utl::SA1D, 7, "The summation of move probability is not equal to 1.0 !");
      }
    }
  } else {
    logger_->report("All the worker will be assigned with the same move probability vector !");
    auto& work_prob = move_probs_[0];
    if ((std::accumulate(work_prob.begin(), work_prob.end(), 0.0) - 1.0) <= 0.00001) {
      if (work_prob.size() == num_move_types) {
        std::string info = "[Params] Move probability vector for worker ";
        info += "is : [ ";
        for (auto& prob : work_prob) {
          info += std::to_string(prob) + " ";
        }
        info += "]";
        logger_->report(info);
        unique_move_prob_flag_ = true;
      } else {
        logger_->error(utl::SA1D, 8, "The size of move probability dose not match number of move types !");
      }
    } else {
      std::cout << "Probability sum = " <<  std::accumulate(work_prob.begin(), work_prob.end(), 0.0) << std::endl;
      logger_->error(utl::SA1D, 9, "The summation of move probability is not equal to 1.0 !");
    }
  }

  logger_->report("[Params] max_temp = {}", max_temp_);
  logger_->report("[Params] max_iter = {}", max_iter_);
  logger_->report("[Params] move_budget = {}", move_budget_);
  logger_->report("[Params] num_move_per_iter = {}", num_move_per_iter_);
  
  // checking cooling rate
  float cooling_rate = std::exp(std::log(min_temp_ / max_temp_) / max_iter_);
  if (cooling_rate >= cooling_rate_) {
    logger_->report("[Params] reset the cooling_rate to {}", cooling_rate);
    cooling_rate_ = cooling_rate;
  } else {
    logger_->report("[Params] cooling_rate = {}", cooling_rate_);
  }

  logger_->report("[Params] gwtw_iter = {}", gwtw_iter_);
  logger_->report("[Params] max_temp_derate_factor = {}", max_temp_derate_factor_);
  logger_->report("[Params] top_k = {}", top_k_);
  
  // win_ratio_size should be equal to top_replica_count_
  if (top_k_ratio_.size() != top_k_) {
    logger_->error(utl::SA1D, 4, "Invalid size of top_k_ratio !");
  }

  // Ensure the sum of win_ratios_ is equal to worker_count_
  if ((std::accumulate(top_k_ratio_.begin(), top_k_ratio_.end(), 0.0) - 1.0) > 0.00001) {
    logger_->error(utl::SA1D, 5, "Invalid top_k_ratio ! (summation should be 1.0)");
  }

  logger_->report("[Params] sync_freq = {}", sync_freq_);
  logger_->report("[Params] seed = {}", seed_);
  logger_->report("[Params] incremental_flag = {}", incremantal_flag_);
  if (incremantal_flag_ == true) {
    for (const auto& db_inst : block_->getInsts()) { 
      if (db_inst->isPlaced() == false) {
        logger_->error(utl::SA1D, 1, "All cells should be placed for incremental SA !");
      }
    }
  }

  logger_->info(utl::SA1D, 10, "All SA parameters are valid");
}


// --------------------------------------------------------------------------
// Loading DB
// --------------------------------------------------------------------------
void OptSA::makeCells()
{
  // First allocate total number of cells in the vector
  cells_.clear();
  cells_.reserve(block_->getInsts().size());
  int cell_id = 0;
  for (const auto& db_inst : block_->getInsts()) {
    // Create a new Cell object directly in the vector
    cells_.emplace_back();
    auto& cell = cells_.back(); // Get a reference to the newly added Cell
    odb::dbIntProperty::create(db_inst, "cell_id", cell_id++);
    cell.db_inst = db_inst;
    // Handle the Master object
    auto cell_master = db_inst->getMaster();
    cell.width = cell_master->getWidth();
    cell.height = cell_master->getHeight();
    auto& mterm_locs = cell.mterm_locs;
    mterm_locs.clear();
    // Update the mterm locations
    for (const auto& mterm : cell_master->getMTerms()) {
      auto mterm_loc = mterm->getBBox();
      auto mterm_x = (mterm_loc.xMin() + mterm_loc.xMax())/2;
      auto mterm_y = (mterm_loc.yMin() + mterm_loc.yMax())/2;
      odb::dbIntProperty::create(mterm, "mterm_id",
        static_cast<int>(mterm_locs.size()));
      mterm_locs.emplace_back(mterm_x, mterm_y);
    }
    cell.nets.clear();
    cell.nets.resize(mterm_locs.size(), -1);
  }
  logger_->report("[INFO] Number of cells : {}", cells_.size());
}


void OptSA::makeNets()
{ 
  // First allocate total number of nets in the vector
  nets_.reserve(block_->getNets().size());
  int net_id = 0;
  for (const auto& db_net : block_->getNets()) {
    // Ignore Special Nets (power and ground net)
    if (db_net->isSpecial()) {
      continue; 
    }

    odb::dbIntProperty::create(db_net, "net_id", net_id);
    // Create a new Net object directly in the vector
    nets_.emplace_back();
    auto& net = nets_.back(); // Get a reference to the newly added Net
    net.db_net = db_net;

    // Handle the BTerm object
    net.updateBTerm();

    // Handle the Cells in the Net
    for (const auto& db_inst_term : db_net->getITerms()) {
      auto db_inst = db_inst_term->getInst();
      auto db_mterm = db_inst_term->getMTerm();
      int cell_id = odb::dbIntProperty::find(db_inst, "cell_id")->getValue();
      int mterm_id = odb::dbIntProperty::find(db_mterm, "mterm_id")->getValue();
      net.terms.emplace_back(cell_id, mterm_id);
      //cells_[cell_id].nets.push_back(net_id);
      cells_[cell_id].nets[mterm_id] = net_id;
    }
    net_id++;
  }
  logger_->report("[INFO] Number of nets : {}", nets_.size());
}


void OptSA::importDb()
{
  block_ = db_->getChip()->getBlock();
  
  if (!block_) {
    logger_->error(utl::SA1D, 2, "No block found in the database");
  }

  site_count_ = 0;
  row_count_ = 0;
  // Update Site count, row count, site width and row height
  auto first_row = *block_->getRows().begin();
  auto site = first_row->getSite();
  site_count_ = first_row->getSiteCount();
  orient_ = first_row->getOrient();
  row_count_ = block_->getRows().size();
  // If row_count_ is not 1, then it is not 1D SA
  // Print the current row count and site count
  if (row_count_ != 1) {
    logger_->error(utl::SA1D, 3, "Row count is {}, expected 1", row_count_);
  }
  
  row_height_ = site->getHeight();
  site_width_ = site->getWidth();
  Rect core_area = block_->getCoreArea();
  core_llx_ = core_area.xMin();
  core_lly_ = core_area.yMin();

  // Import cells
  makeCells();

  // Import nets
  makeNets();
}


// get the cell ordering from database
void OptSA::cellOrdering(
  std::vector<int>& cell_order, 
  std::vector<odb::dbOrientType>& orients)
{
  // Ensure that all cells are placed
  // Initialize the cell order
  cell_order.clear();
  cell_order.resize(cells_.size());
  std::iota(cell_order.begin(), cell_order.end(), 0);

  // Sort Cell_order based on cells_[i].db_inst->getOrigin().x()
  std::sort(cell_order.begin(), cell_order.end(),
      [this](int i, int j) {
      return cells_[i].db_inst->getOrigin().x() < 
            cells_[j].db_inst->getOrigin().x();
  });

  orients.clear();
  orients.reserve(cells_.size());
  for (auto& cell : cells_) {
    orients.push_back(cell.db_inst->getOrient());
  }
}


void OptSA::updateOpenDB(const CellLocMap& cell_locs)
{
  for (size_t cell_id = 0; cell_id < cell_locs.size(); cell_id++) {
    auto db_inst = cells_[cell_id].db_inst;
    auto& cell_loc = cell_locs[cell_id];
    db_inst->setOrient(cell_loc.orient);
    db_inst->setLocation(cell_loc.x, cell_loc.y);
  }
}

void OptSA::reportPackHPWL()
{
  if (cells_.size() != block_->getInsts().size()) {
    importDb(); // Avoid the re-import of the database
  }

  std::vector<int> cell_order;
  std::vector<odb::dbOrientType> orients;
  cellOrdering(cell_order, orients);
  SAWorker worker(this, logger_, 0);
  worker.initCellOrder(cell_order, orients);
  int64_t hpwl = worker.getTotalHPWL();
  odb::WireLengthEvaluator eval(block_);
  int64_t hpwl_db = eval.hpwl();
  logger_->report("[INFO] Note that there may be minor differences in HPWL due to y-direction calculation.");
  logger_->report("[INFO] We only consider x-direction HPWL");
  logger_->report("[INFO] HPWL before packing: {}", block_->dbuToMicrons(hpwl_db));
  logger_->report("[INFO] HPWL after packing: {}", block_->dbuToMicrons(hpwl));

  // Check total cell width
  int total_width = 0;
  for (int i = 0; i < cell_order.size(); i++) {
    total_width += cells_[cell_order[i]].getWidth();
  }
  // Ensure that total_width is less or equal to site_width_*site_count_
  if (total_width > site_width_*site_count_) {
    logger_->report("Total width exceeds site width");
  }
}

void OptSA::runSA()
{  
  logger_->info(utl::SA1D, 1, "Running simulated annealing");
  // Import the database
  importDb();
  logger_->info(SA1D, 11, "Imported the database");

  // Check SA parameters before running SA
  checkParams();

  std::vector<int> cell_order;
  std::vector<odb::dbOrientType> orients;
  
  if (incremantal_flag_ == true) {
    cellOrdering(cell_order, orients);
    logger_->report("[INFO] Finish initial cell ordering");
  }
 
  // Initialize the workers
  std::vector<std::unique_ptr<SAWorker> > workers;
  // try to workers with different cooling rate
  float delta_cooling_rate = (0.995 - cooling_rate_) / (num_workers_ / 2 + 1);
  for (int worker_id = 0; worker_id < num_workers_; worker_id++) {
    std::unique_ptr<SAWorker> worker = std::make_unique<SAWorker>(this, logger_, worker_id);
    worker->setRandomSeed(seed_ + worker_id);
    worker->setTemp(max_temp_); // Initial Temperature
    worker->setCoolingRate(cooling_rate_ + (worker_id - num_workers_ / 2) * delta_cooling_rate);
    worker->setNumMovePerIter(num_move_per_iter_);
    worker->setMaxIter(static_cast<int>(max_iter_ * sync_freq_));
    //worker->setSaveFlag(true);
    if (unique_move_prob_flag_ == false) {
      std::string info = "[INFO] Worker_id = ";
      info += std::to_string(worker_id) + " Move Prob = [ ";
      for (auto& prob : move_probs_[worker_id]) {
        info += std::to_string(prob) + " ";
      }
      info += "]";
      logger_->report(info); 
      worker->setMoveProbs(move_probs_[worker_id]);
    } else {
      std::string info = "[INFO] Worker_id = ";
      info += std::to_string(worker_id) + " Move Prob = [ ";
      for (auto& prob : move_probs_[0]) {
        info += std::to_string(prob) + " ";
      }
      info += "]";
      logger_->report(info);    
      worker->setMoveProbs(move_probs_[0]);
    }

    std::cout << "set the move prob correctly" << std::endl;

    if (incremantal_flag_) {
      worker->initCellOrder(cell_order, orients);
    } else {
      worker->initCellOrderRandom();
    }
    workers.push_back(std::move(worker));    
  }

  logger_->report("Initialized workers ... ");
  omp_set_num_threads(std::max(8, num_threads_));
  
  std::vector<int> copied_cnt;
  int copied_cnt_sum = 0;
  for (int i = 0; i < top_k_; i++) {
    int cnt = static_cast<int>(std::floor(num_workers_ * top_k_ratio_[i]));
    if (cnt == 0) {
      logger_->report("[INFO] The {} element of top_k_ratio is too samll (cnt = 0)!", i);
    }
    copied_cnt_sum += cnt;
    copied_cnt.push_back(cnt);
  }
  copied_cnt[0] += num_workers_ - copied_cnt_sum;
  std::string copied_vec_string("[INFO] The copied cnt for top vector = [ ");
  for (int i = 0; i < top_k_; i++) {
    copied_vec_string += std::to_string(copied_cnt[i]) + " ";
  }
  copied_vec_string += "]";
  logger_->report(copied_vec_string);

  for (int gwtw_iter_id = 0; gwtw_iter_id < gwtw_iter_; gwtw_iter_id++ ) {
    logger_->report("[INFO] GWTW Iteration {} starts ...", gwtw_iter_id);        
    if (gwtw_iter_id > 0) {
      max_temp_ *= max_temp_derate_factor_;
    }

    logger_->report("[INFO] Set the max temperature to {}", max_temp_);
    for (auto& worker : workers) {
      worker->setTemp(max_temp_);
    }
    
    // in each GWTW iteration, there are multiple sync up
    int sync_iter = static_cast<int>(max_iter_ * sync_freq_);
    for (int iter = 0; iter < max_iter_; iter += sync_iter) {
      #pragma omp parallel for schedule(dynamic)
      for (int j = 0; j < workers.size(); j++) {
        workers[j]->run();
      }

      // Get the top K replicas based on HPWL
      std::sort(workers.begin(), workers.end(),
        [](const std::unique_ptr<SAWorker>& a, const std::unique_ptr<SAWorker>& b) {
          return a->getTotalHPWL() < b->getTotalHPWL();
        });

      // print the statistcis of the results
      logger_->report("********************* the current results after iteration {} *********************", iter);
      for (auto& worker : workers) {
        worker->reportDetails();
        logger_->report("[INFO] worker_id = {}, HPWL = {}", worker->getWorkerId(), worker->getTotalHPWL());
      
      }

      int worker_cnt = top_k_; 
      for (int j = 0; j < top_k_; j++) {
        const std::vector<int>& cell_order = workers[j]->getFinalOrdering();
        std::vector<odb::dbOrientType> orients = workers[j]->getFinalOrients();
        for (int i = 1; i < copied_cnt[j]; i++) {
          workers[worker_cnt++]->initCellOrder(cell_order, orients);
        }      
      }
    }
  }
    
  updateOpenDB(workers[0]->getCellLocs());
}

}  // namespace sa1d
