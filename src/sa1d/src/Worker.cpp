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

#include "Worker.h"
#include <cstdint>
#include <limits>
#include <numeric>
#include "odb/dbTypes.h"
#include "sa1d/Objects.h"
#include "sa1d/OptSA.h"
#include <iostream>
#include <fstream>

namespace sa1d {

using odb::dbOrientType;
using odb::Point;
using odb::Rect;
using utl::SA1D;

SAWorker::SAWorker(OptSA* opt, Logger* logger, int worker_id)
  : opt_(opt),  logger_(logger), worker_id_(worker_id),
    cells_(opt->getCells()), nets_(opt->getNets())
{
  // Constructor implementation
  // Initialize other member variables if needed
  total_cells_ = static_cast<int>(cells_.size());
  num_nets_ = static_cast<int>(nets_.size());
  net_hpwl_.resize(num_nets_);
  affected_cells_.reserve(total_cells_);
}

SAWorker::~SAWorker()
{
  // Destructor implementation
  // Clean up resources if necessary
}

void SAWorker::setRandomSeed(int seed) 
{
  seed_ = seed;
  rng_.seed(seed_);
}

void SAWorker::setTemp(float temp)
{
  temp_ = temp;
}

void SAWorker::setCoolingRate(float cooling_rate)
{
  cooling_rate_ = cooling_rate;
}

void SAWorker::setNumMovePerIter(int num_move_per_iter)
{
  num_move_per_iter_ = num_move_per_iter;
}

void SAWorker::setMaxIter(int max_iter)
{
  max_iter_ = max_iter;
}

void SAWorker::setMoveProbs(const std::vector<float>& move_probs)
{
  action_prob_ = move_probs;
  for (int i = 1; i < action_prob_.size(); i++) {
    action_prob_[i] += action_prob_[i-1];
  }
}

void SAWorker::initCellOrderRandom()
{
  cell_order_.resize(total_cells_);
  std::iota(cell_order_.begin(), cell_order_.end(), 0);
  std::shuffle(cell_order_.begin(), cell_order_.end(), rng_);
  std::vector<odb::dbOrientType> cell_orient;
  cell_orient.reserve(total_cells_);
  auto default_orient = odb::dbOrientType::R0;
  for (int i = 0; i < total_cells_; i++) {
    if (distribution_(rng_) <= 0.5) {
      cell_orient.push_back(default_orient);
    } else {
      cell_orient.push_back(orientMirrorY(default_orient));
    }
  }
  updateCellLocs(cell_orient);
  norm_hpwl_ = static_cast<float>(getTotalHPWL());
  
  // Initialize overlap normalization
  if (enable_overlap_) {
    peak_overlap_ = overlap_calc_.computePeakOverlap(net_hpwl_);
    norm_overlap_ = static_cast<float>(std::max(1, peak_overlap_)); // Avoid division by 0
    logger_->report("[INFO] Norm Overlap = {}", static_cast<int>(norm_overlap_));
  }
  
  logger_->report("[INFO] Norm HPWL = {}", static_cast<int>(norm_hpwl_));
}


void SAWorker::initCellOrder(
  const std::vector<int>& cell_order, 
  const std::vector<odb::dbOrientType>& cell_orients)
{
  cell_order_ = cell_order;
  updateCellLocs(cell_orients);
  norm_hpwl_ = static_cast<float>(getTotalHPWL());
  
  // Initialize overlap normalization
  if (enable_overlap_) {
    peak_overlap_ = overlap_calc_.computePeakOverlap(net_hpwl_);
    norm_overlap_ = static_cast<float>(std::max(1, peak_overlap_)); // Avoid division by 0
    logger_->report("[INFO] Norm Overlap = {}", static_cast<int>(norm_overlap_));
  }
  
  logger_->report("[INFO] Norm HPWL = {}", static_cast<int>(norm_hpwl_));
}

void SAWorker::updateCellLocs(const std::vector<odb::dbOrientType>& orients)
{
  cell_locs_.clear();
  cell_locs_.resize(total_cells_);
  int prev_width = opt_->getCoreLLX();
  int y = opt_->getCoreLLY();
  int x = 0.0;
  for (int i = 0; i < total_cells_; i++) {
    int cell_id = cell_order_[i];
    const Cell& cell = cells_[cell_id];
    x += prev_width;
    cell_locs_[cell_id] = CellLoc(x, y, orients[cell_id]);
    prev_width = cell.getWidth();
  }
  updateNetHPWL();
}

// We only consider x dim (y dim is fixed)
void SAWorker::calcNetHPWL(int net_id)
{
  int lx = std::numeric_limits<int>::max();
  int ux = std::numeric_limits<int>::min();  
  const auto& net = nets_[net_id];
  if (net.bterm_flag == true) {
    lx = std::min(lx, net.bterm_box.xMin());
    ux = std::max(ux, net.bterm_box.xMax());
  }  

  for (const auto& term : net.getTerms()) {
    int cell_id = term.cell_id;
    int mterm_id = term.mterm_id;
    const auto& cell_loc = cell_locs_[cell_id];
    auto pin_loc = cells_[cell_id].getPinLocation(
      cell_loc.x, cell_loc.y, mterm_id, cell_loc.orient);
    lx = std::min(lx, pin_loc.first);      
    ux = std::max(ux, pin_loc.first);
  }

  net_hpwl_[net_id] = netHPWL(ux - lx, lx, ux);
}

void SAWorker::updateNetHPWL()
{
  for (int net_id = 0; net_id < num_nets_; net_id++) {
    calcNetHPWL(net_id);
  } 
}

int64_t SAWorker::getTotalHPWL()
{
  updateNetHPWL();
  int64_t total_hpwl = 0;
  for (const auto& net : net_hpwl_) {
    total_hpwl += net.hpwl;
  }
  return total_hpwl;
}


void SAWorker::checkCellOrder()
{
  int prev_width = opt_->getCoreLLX();
  int expected_x = 0;
  for (int i = 0; i < total_cells_; i++) {
    int cell_id = cell_order_[i];
    const Cell& cell = cells_[cell_id];
    expected_x += prev_width;
    int x = cell_locs_[cell_id].x;
    if (x != expected_x) {
      logger_->error(utl::SA1D, 1, "Cell order mismatch at index: {} (cell id = {}) "
                      "Expected x: {} Found x: {}", i, cell_id, expected_x, x);
    }
    prev_width = cell.getWidth();
  }
}


int SAWorker::selectCell()
{
  int cell_id = static_cast<int>(distribution_(rng_) * total_cells_);
  return cell_id;
}


int SAWorker::selectMoveType()
{
  float rand_num = distribution_(rng_);
  if (rand_num <= action_prob_[0]) {
    return 0;
  } else if (rand_num <= action_prob_[1]) {
    return 1;
  } else {
    return 2;
  }
}


int SAWorker::swapCells(int id1, int id2)
{
  if (id1 == id2) {
    return 0;
  }

  // id1 is less than id2
  if (id1 > id2) {
    std::swap(id1, id2);
  }

  int cell_id1 = cell_order_[id1];
  int cell_id2 = cell_order_[id2];

  auto& cell1 = cells_[cell_order_[id1]];
  auto& cell2 = cells_[cell_order_[id2]];
  bool isSameWidth = cell1.getWidth() == cell2.getWidth();

  affected_cells_.clear();
  affected_nets_.clear();
  cell_locs_[cell_id1].backupX();
  cell_locs_[cell_id2].backupX();
  std::swap(cell_locs_[cell_id1].x, cell_locs_[cell_id2].x);
  std::swap(cell_order_[id1], cell_order_[id2]);
  affected_cells_.push_back(cell_id1);
  affected_cells_.push_back(cell_id2);
  
  if (isSameWidth == false) {
    int delta_width = cell1.getWidth() - cell2.getWidth();
    cell_locs_[cell_id1].x -= delta_width;
    for (int i = id1+1; i < id2; i++ ) {
      cell_locs_[cell_order_[i]].backupX();
      cell_locs_[cell_order_[i]].x -= delta_width;
      affected_cells_.push_back(cell_order_[i]);
    }
  }

  // update the affected nets
  for (auto& cell_id : affected_cells_) {
    for (auto& net_id : cells_[cell_id].nets) {
      if (net_id == -1) {
        continue;
      }
      affected_nets_.insert(net_id);
    }
  }

  for (auto& net_id : affected_nets_) {
    net_hpwl_[net_id].backup();
  }

  // for (auto& cell_id : affected_cells_) {
  //   updateDeltaHPWLUtil(cell_id);
  // }

  int delta_hpwl = 0;
  for (auto& net_id : affected_nets_) {
    delta_hpwl += updateDeltaHPWL(net_id);
  }

  return delta_hpwl;
}


void SAWorker::swapCellsReject(int id1, int id2)
{
  for (auto& cell_id : affected_cells_) {
    cell_locs_[cell_id].restoreX();
  }
  
  for (auto& net_id : affected_nets_) {
    net_hpwl_[net_id].restore();
  }
  
  std::swap(cell_order_[id1], cell_order_[id2]);
}


int SAWorker::moveCell(int id, int n)
{
  // If new location is same as the current location return 0
  if (n == id ) {
    return 0;
  }

  // Get the cell width
  int cell_id1 = cell_order_[id];
  int cell_id2 = cell_order_[n];

  int width1 = cells_[cell_id1].getWidth();
  int width2 = cells_[cell_id2].getWidth();
  cell_locs_[cell_id1].backupX();
  cell_locs_[cell_id2].backupX();

  affected_cells_.clear();
  affected_nets_.clear();
  // We need to consider two possible cases:
  // if id > n, we need to insert the cell in front of n and shift the cells in between towards the right
  // if id < n, we need to insert the cell behind n and shift the cells in between towards the left
  if (id > n) {
    int new_x = cell_locs_[cell_id2].x; // Match the left edge of the cell
    cell_locs_[cell_id1].x = new_x;
    for (int i = n; i < id; i++) {
      auto& cell_id_temp = cell_order_[i];
      cell_locs_[cell_id_temp].backupX();
      cell_locs_[cell_id_temp].x += width1;
    }
    for (int i = id; i > n; i--) {
      cell_order_[i] = cell_order_[i - 1];
      affected_cells_.push_back(cell_order_[i]);
    }
    cell_order_[n] = cell_id1;
    affected_cells_.push_back(cell_id1);
  } else { // id < n
    int new_x = cell_locs_[cell_id2].x + width2 - width1; // Match the right edge of the cell
    cell_locs_[cell_id1].x = new_x;
    for (int i = id + 1; i <= n; i++) {
      auto& cell_id_temp = cell_order_[i];
      cell_locs_[cell_id_temp].backupX();
      cell_locs_[cell_id_temp].x -= width1;
    }
    for (int i = id; i < n; i++) {
      cell_order_[i] = cell_order_[i + 1];
      affected_cells_.push_back(cell_order_[i]);
    }
    cell_order_[n] = cell_id1;
    affected_cells_.push_back(cell_id1);
  }

  // update the affected nets
  // update the affected nets
  for (auto& cell_id : affected_cells_) {
    for (auto& net_id : cells_[cell_id].nets) {
      if (net_id == -1) {
        continue;
      }
      affected_nets_.insert(net_id);
    }
  }

  for (auto& net_id : affected_nets_) {
    net_hpwl_[net_id].backup();
  }

  // for (auto& cell_id : affected_cells_) {
  //   updateDeltaHPWLUtil(cell_id);
  // }

  int delta_hpwl = 0;
  for (auto& net_id : affected_nets_) {
    delta_hpwl += updateDeltaHPWL(net_id);
  }

  return delta_hpwl;
}


void SAWorker::moveCellReject(int id, int n)
{
  // If new location is same as the current location return 0
  if (n == id ) {
    return;
  }
  
  for (auto& cell_id : affected_cells_) {
    cell_locs_[cell_id].restoreX();
  }
  
  for (auto& net_id : affected_nets_) {
    net_hpwl_[net_id].restore();
  }
   
  int cell_id2 = cell_order_[n];
  if (id > n) {
    for (int i = n; i < id; i++) {
      cell_order_[i] = cell_order_[i + 1];
    }
    cell_order_[id] = cell_id2;    
  } else { // id < n
    for (int i = n; i > id; i--) {
      cell_order_[i] = cell_order_[i - 1];
    }
    cell_order_[id] = cell_id2;
  } 
}


// Key point: use the boundary condition to calculate the delta HPWL
void SAWorker::updateDeltaHPWLUtil(int cell_id)
{
  auto& cell_loc = cell_locs_[cell_id];
  const auto& cell_x = cell_loc.x;
  const auto& cell_y = cell_loc.y;  
  const auto& orient = cell_loc.orient;

  const auto& cell_pre_x = cell_locs_[cell_id].pre_x;
  auto& cell = cells_[cell_id];
  for (int mterm_id = 0; mterm_id < cell.getMtermCount(); mterm_id++) {
    auto mterm_loc = cell.getPinLocation(cell_x, cell_y, mterm_id, orient);
    auto pre_mterm_loc = cell.getPinLocation(cell_pre_x, cell_y, mterm_id, orient);
    int net_id = cell.nets[mterm_id];
    if (net_id == -1)  {
      continue;
    }

    auto& net = net_hpwl_[net_id];
    if (net.scratch_flag == true) {
      continue;
    }  

    if (mterm_loc.first <= net.lx && pre_mterm_loc.first < net.ux) {
      net.lx = mterm_loc.first;
    } else if (mterm_loc.first >= net.ux && pre_mterm_loc.first > net.lx) {
      net.ux = mterm_loc.first;
    } else {
      net.scratch_flag = true;  
    }
  }
}

// We only consider x dim (y dim is fixed)
int SAWorker::updateDeltaHPWL(int net_id)
{
  auto& net_hpwl = net_hpwl_[net_id];
  if (net_hpwl.scratch_flag == false) {
    net_hpwl.hpwl = net_hpwl.ux - net_hpwl.lx;
  } else {
    int lx = std::numeric_limits<int>::max();
    int ux = std::numeric_limits<int>::min();  
    const auto& net = nets_[net_id];
    if (net.bterm_flag == true) {
      lx = std::min(lx, net.bterm_box.xMin());
      ux = std::max(ux, net.bterm_box.xMax());
    }  
    for (const auto& term : net.getTerms()) {
      int cell_id = term.cell_id;
      int mterm_id = term.mterm_id;
      const auto& cell_loc = cell_locs_[cell_id];
      auto pin_loc = cells_[cell_id].getPinLocation(
        cell_loc.x, cell_loc.y, mterm_id, cell_loc.orient);
      lx = std::min(lx, pin_loc.first);      
      ux = std::max(ux, pin_loc.first);
    }
    net_hpwl.ux = ux;
    net_hpwl.lx = lx;
    net_hpwl.hpwl = ux - lx;
  }
  return net_hpwl.getDeltaHPWL();
}

int SAWorker::computeDeltaOverlap()
{
  if (!enable_overlap_ || overlap_weight_ <= 0.0) {
    return 0;
  }
  
  int old_peak = peak_overlap_;
  int new_peak = overlap_calc_.computeIncrementalOverlap(affected_nets_, net_hpwl_);
  peak_overlap_ = new_peak; // Update current peak overlap
  return new_peak - old_peak;
}


int SAWorker::flipCell(int id)
{
  int cell_id = cell_order_[id];
  cell_locs_[cell_id].backupOrient();
  cell_locs_[cell_id].flipOrient();
  affected_nets_.clear();

  // To speed up the calculation of delta HPWL,
  // we calculate the change of HPWL in a cell-based manner
  for (auto net_id : cells_[cell_id].getNets()) {
    if (net_id == -1) {
      continue;
    }
    
    affected_nets_.insert(net_id);
    net_hpwl_[net_id].backup();
    updateDeltaHPWLUtil(cell_id);
  }

  int delta_hpwl = 0;
  for (auto& net_id : affected_nets_) {
    delta_hpwl += updateDeltaHPWL(net_id);
  }

  return delta_hpwl;
}


void SAWorker::flipCellReject(int id)
{
  int cell_id = cell_order_[id];
  cell_locs_[cell_id].restoreOrient();
  for (auto net_id : affected_nets_) {
    net_hpwl_[net_id].restore();
  }
}


// define the same guardband
float safeExp(float x) 
{
  x = std::min(x, 50.0f);
  x = std::max(x, -50.0f);
  return std::exp(x);  
}

// Define the guard for exp to avoid overflow
void SAWorker::run()
{
  accept_count_ = 0;
  swap_op_count_ = 0;
  move_op_count_ = 0;
  flip_op_count_ = 0;
  if (save_flag_ == true) {
    cost_vec_.clear();
    cost_vec_.reserve(max_iter_ * num_move_per_iter_);
  }

  auto start_time = std::chrono::high_resolution_clock::now();
  int64_t curr_hpwl = getTotalHPWL();
  for (int iter = 0; iter < max_iter_; iter++) {
    for (int move_id = 0; move_id < num_move_per_iter_; move_id++) {
      int move_type = selectMoveType();
      int id1 = selectCell();
      int id2 = selectCell();
      int delta_hpwl = 0;
      switch (move_type) {
        case 0:
          delta_hpwl = swapCells(id1, id2);
          swap_op_count_++;
          break;
        case 1:
          delta_hpwl = moveCell(id1, id2);
          move_op_count_++;
          break;
        case 2:
          delta_hpwl = flipCell(id1);
          flip_op_count_++;
          break;
        default:
          logger_->error(utl::SA1D, 4, "Invalid move type selected.");
      }

      // Compute delta overlap
      int delta_overlap = computeDeltaOverlap();
      
      // Calculate acceptance probability with normalized terms
      bool accept_flag = false;
      if (enable_overlap_ && overlap_weight_ > 0.0) {
        // Multi-objective cost with normalized terms
        float normalized_delta_hpwl = static_cast<float>(delta_hpwl) / norm_hpwl_;
        float normalized_delta_overlap = static_cast<float>(delta_overlap) / norm_overlap_;
        float total_normalized_delta = normalized_delta_hpwl + overlap_weight_ * normalized_delta_overlap;
        
        accept_flag = total_normalized_delta <= 0.0 || 
          distribution_(rng_) < safeExp(-1.0 * total_normalized_delta / temp_);
      } else {
        // Original HPWL-only acceptance
        accept_flag = delta_hpwl <= 0.0 || 
          distribution_(rng_) < safeExp(-1.0 * delta_hpwl / norm_hpwl_ / temp_);
      }
      if (accept_flag == false) {
        // Reject the move
        switch (move_type) {
          case 0:
            swapCellsReject(id1, id2);
            break;
          case 1:
            moveCellReject(id1, id2);
            break;
          case 2:
            flipCellReject(id1);
            break;
          default:
            break;
        }
        
        // Restore peak overlap if it was changed
        if (enable_overlap_ && overlap_weight_ > 0.0 && delta_overlap != 0) {
          peak_overlap_ -= delta_overlap; // Revert the change
        }
      } else {
        accept_count_++;
        curr_hpwl += delta_hpwl;
      }
      
      if (save_flag_ == true) {
        cost_vec_.push_back(curr_hpwl);
      }
    }
    temp_ = temp_ * cooling_rate_;
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  elapsed_time_ = std::chrono::duration_cast<std::chrono::milliseconds>(
      end_time - start_time).count() / 1000.0;
}

void SAWorker::reportDetails() {
  // Report Current Iteration and Temperature
  logger_->report("###########################################");
  logger_->report("## Worker ID: {:<26} ##", worker_id_);
  logger_->report("## cooling_rate: {:<15.3f} ##", cooling_rate_);
  logger_->report("## Eelapsed Time: {:<15.3f} second ##", elapsed_time_);
  logger_->report("## Flip operation count : {:<15} ##", flip_op_count_);
  logger_->report("## Move operation count : {:<15} ##", move_op_count_);
  logger_->report("## Swap operation count : {:<15} ##", swap_op_count_);
  checkCellOrder();

  if (save_flag_ == true) {
    std::string file_name = "cost_" + std::to_string(worker_id_) + ".txt";
    std::ofstream file(file_name);
    for (auto& cost : cost_vec_) {
      file << cost << std::endl;
    }
    file.close();
  }
}

}  // namespace sa1d
