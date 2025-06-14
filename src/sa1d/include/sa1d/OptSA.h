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

#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>  // pair
#include <vector>

#include "odb/db.h"
#include "odb/dbTypes.h"
#include "sa1d/Objects.h"

namespace utl {
class Logger;
}

namespace sa1d {

using std::map;
using std::unordered_map;
using std::vector;

using utl::Logger;

using odb::dbBlock;
using odb::dbDatabase;
using odb::dbInst;
using odb::dbMaster;
using odb::dbNet;
using odb::dbOrientType;


struct CellLoc {
  int x;
  int y;
  odb::dbOrientType orient;

  // for bookkeeping
  int pre_x;
  odb::dbOrientType pre_orient;

  CellLoc() {
    x = 0;
    y = 0;
    orient = odb::dbOrientType::R0;
    pre_x = x;
    pre_orient = orient;
  }

  CellLoc(int _x, int _y, const odb::dbOrientType& _orient) {
    x = _x;
    y = _y;
    orient = _orient;
    pre_x = x;
    pre_orient = orient;
  }

  void backupX() {
    pre_x = x;
  }

  void restoreX() {
    x = pre_x;
  }

  void backupOrient() {
    pre_orient = orient;
  }

  void restoreOrient() {
    orient = pre_orient;
  }

  void flipOrient() {
    orient = orientMirrorY(orient);
  }

};

using CellLocMap = std::vector<CellLoc>;


struct Cell
{
  dbInst* db_inst = nullptr;
  int width;
  int height;
  std::vector<Point> mterm_locs;
  std::vector<int> nets; // stores the net id

  std::pair<int, int> getPinLocation(
    int llx, int lly, 
    int mterm_id,
    const odb::dbOrientType& orient);
  
  int getWidth() const { return width; }
  int getHeight() const { return height; }
  const std::vector<int>& getNets() const { return nets; }
  int getMtermCount() const { return mterm_locs.size(); }

};

struct netTerm {
  int cell_id;
  int mterm_id; 

  netTerm() { 
    cell_id = -1;
    mterm_id = -1;
  }

  netTerm(int _cell_id, int _mterm_id) {
    cell_id = _cell_id;
    mterm_id = _mterm_id;
  }

};


struct Net
{
  dbNet* db_net = nullptr;
  bool bterm_flag = false;
  Rect bterm_box; // Fixed the contributions from bterm
  std::vector<netTerm> terms;

  void updateBTerm();
  const std::vector<netTerm>& getTerms() const { return terms; }
};




////////////////////////////////////////////////////////////////
class OptSA
{
 public:
  OptSA();
  ~OptSA();

  OptSA(const OptSA&) = delete;
  OptSA& operator=(const OptSA&) = delete;
  void runSA();
  void init(dbDatabase* db, Logger* logger);
  
  // total number of workers for each iteration of GWTW
  void setNumWorkers(int num_workers);
  // number of workers in parallel
  void setNumThreads(int num_threads);
  // Add SA Parameters here
  // Moves are defined as follows:
  //  0: Global Swap (Swap any two cells)
  //  1: Move (Move a cell to a new location)
  //  2: Flip (Flip a cell)
  // if move_probs_.size() == num_workers, then the move_prob of worker i
  // is move_probs_[i]  
  void setMoveProbs(vector<vector<float> > move_probs);
  // All the workers has exactly the same temperature schedule
  // Setting for each SA worker
  void setMaxTemp(float max_temp);
  void setMinTemp(float min_temp);
  void setCoolingRate(float cooling_rate);
  void setMaxIter(int max_iter);
  void setMoveBudget(int move_budget);
  void setNumMovePerIter(int num_move_per_iter);

  // Setting for GWTW
  void setGWTWIter(int gwtw_iter);
  void setMaxTempDerateFactor(float derate_factor);
  void setTopK(int top_k);
  void setTopKRatio(std::vector<float> top_k_ratio);
  void setSyncFreq(float sync_freq);  

  // random seed
  void setSeed(int seed);
  void setIncrementalFlag(bool incremantal_flag);

  void setSAParams(const char* filename); 
  void checkParams();
  
  void reportPackHPWL();
  odb::dbBlock* getBlock() { return block_; }
  utl::Logger* getLogger() { return logger_; }
  odb::dbDatabase* getDB() { return db_; }

  std::vector<Cell>& getCells() { return cells_; }
  std::vector<Net>& getNets() { return nets_; }

  int getCoreLLX() const { return core_llx_; }
  int getCoreLLY() const { return core_lly_; }

 private:  
  Logger* logger_ = nullptr;
  dbDatabase* db_ = nullptr;
  dbBlock* block_ = nullptr;

  // Hyperparameters
  int num_workers_ = 20;  // total number of workers for each iteration of GWTW
  int num_threads_ = 10;  // number of workers in parallel
  
  // Add SA Parameters here
  // Moves are defined as follows:
  //  0: Global Swap (Swap any two cells)
  //  1: Move (Move a cell to a new location)
  //  2: Flip (Flip a cell)
  // if move_probs_.size() == num_workers, then the move_prob of worker i
  // is move_probs_[i]
  std::vector<std::vector<float>> move_probs_;
  bool unique_move_prob_flag_ = false;

  // All the workers has exactly the same temperature schedule
  // Setting for each SA worker
  float max_temp_ = 100.0;  
  float min_temp_ = 1e-12;  // for sensity check only
  float cooling_rate_ = 0.95; // gemoetric cooling schedule
  int max_iter_ = 5000;  // number of iterations for a single SA worker
  int move_budget_ = 1000000; // total number of moves for each SA worker
  int num_move_per_iter_ = 200; // ( = max_budget / max_iter )

  // Setting for GWTW
  int gwtw_iter_ = 1;  
  float max_temp_derate_factor_ = 1.0;
  int top_k_ = 10; // each time copy the first top_k_ walkers
  std::vector<float> top_k_ratio_; // the copy ratio for each picked worker: sum(top_k_ratio_) = 1.0
  float sync_freq_ = 0.1; // sync up betwen workers for every max_iter * sync_freq iterations

  // random seed
  int seed_ = 0;

  // incremantal flag or random initialization
  bool incremantal_flag_ = true;

  const int num_move_types = 3;

  // Other information
  std::vector<Cell> cells_;
  std::vector<Net> nets_;

  // Row and Site details
  int row_count_;
  int site_count_;
  int site_width_;
  int row_height_;
  int core_llx_;
  int core_lly_;
  dbOrientType orient_;

  void importDb();
  void makeCells();
  void makeNets();
  void updateOpenDB(const CellLocMap& cell_locs); 
  void cellOrdering(std::vector<int>& cell_order, 
    std::vector<odb::dbOrientType>& orients);
};


}  // namespace sa1d
