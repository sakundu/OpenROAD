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

#pragma once

#include <cstdint>
#include <random>
#include <vector>
#include <chrono>

#include "odb/dbTypes.h"
#include "sa1d/Objects.h"
#include "sa1d/OptSA.h"

namespace sa1d {

using std::map;

using odb::dbOrientType;
using odb::dbSite;


// store the lx and ux for quick check
// Note that for 1D placement, either lx or ux can corresponds to 
// one cell
struct netHPWL {
  int hpwl = 0;
  int lx = 0;
  int ux = 0;
  int pre_lx = 0;
  int pre_ux = 0;
  int pre_hpwl = 0;  
  bool scratch_flag = true;

  netHPWL() 
  { 
    hpwl = 0;
    lx = 0;
    ux = 0;
    pre_lx = lx;
    pre_ux = ux;
    pre_hpwl = hpwl;
  }

  netHPWL(int _hpwl, int _lx, int _ux)
  {
    hpwl = _hpwl;
    lx = _lx;
    ux = _ux;
    pre_lx = lx;
    pre_ux = ux;
    pre_hpwl = hpwl;
  }

  void backup()
  {
    pre_lx = lx;
    pre_ux = ux;
    pre_hpwl = hpwl;
    scratch_flag = true;
  }

  void restore()
  {
    lx = pre_lx;
    ux = pre_ux;
    hpwl = pre_hpwl;
    scratch_flag = true;
  }

  int getDeltaHPWL() {
    return hpwl - pre_hpwl;
  }
};

class SAWorker
{
  public:
    SAWorker(OptSA* opt, Logger* logger, int worker_id);
    ~SAWorker();

    void setRandomSeed(int seed);
    void setTemp(float temp);
    void setCoolingRate(float cooling_rate);
    void setNumMovePerIter(int num_move_per_iter);
    void setMaxIter(int max_iter);
    void setMoveProbs(const std::vector<float>& move_probs);
    void initCellOrder(
      const std::vector<int>& cell_order, 
      const std::vector<dbOrientType>& orients);
    void initCellOrderRandom();

    // Run SA
    void run();
    int64_t getTotalHPWL();
    
    int getWorkerId() const { return worker_id_; }

    // Get the final ordering
    const std::vector<int>& getFinalOrdering() { return cell_order_ ;}
    
    std::vector<odb::dbOrientType> getFinalOrients() {
      std::vector<odb::dbOrientType> orients;
      orients.reserve(total_cells_);
      for (auto& cell_loc : cell_locs_) {
        orients.push_back(cell_loc.orient);
      }
      return orients;
    }

    const CellLocMap& getCellLocs() { return cell_locs_; }
    void reportDetails();

    void setSaveFlag(bool flag) { save_flag_ = flag; }
    
  private:
    OptSA* opt_;
    Logger* logger_;
    int worker_id_;

    float temp_;
    float cooling_rate_;
    int seed_;
    int max_iter_;
    std::vector<float> action_prob_;
    int num_move_per_iter_;

    // basic statistics
    float elapsed_time_;
    int flip_op_count_ = 0;
    int move_op_count_ = 0;
    int swap_op_count_ = 0;

    int accept_count_;
    int total_cells_;
    int num_nets_;
    int move_count_ = 3;
    float norm_hpwl_ = 0.0;
    std::mt19937 rng_;  // Random number generator
    std::uniform_real_distribution<float> distribution_;
    std::vector<int> cell_order_;
    std::vector<netHPWL> net_hpwl_;

    CellLocMap cell_locs_;   
    std::vector<Cell>& cells_;
    std::vector<Net>& nets_;

    void updateCellLocs(const std::vector<odb::dbOrientType>& orients);
    void updateNetHPWL();
    void checkCellOrder();

    // Randomly select a cell id
    int selectCell();

    // Select a move type based on action_prob_
    int selectMoveType();

    // Moves
    // store all the affected nets for a move
    std::set<int> affected_nets_;
    std::vector<int> affected_cells_;

    int swapCells(int id1, int id2);
    void swapCellsReject(int id1, int id2);
    //void swapCellsAccept(int id1, int id2);
    int moveCell(int id, int n);
    void moveCellReject(int id, int n);
    //void moveCellAccept(int id, int n);
    int flipCell(int id);
    //void flipCellAccept(int id);
    void flipCellReject(int id);

    void calcNetHPWL(int net_id);
    // Note that for 1D placement,
    // we can calculate the delta HPWL for a cell perspective
    void updateDeltaHPWLUtil(int cell_id);   
    int updateDeltaHPWL(int net_id);

    bool save_flag_ = false;
    std::vector<float> cost_vec_;
  
  };

}  // namespace sa1d
