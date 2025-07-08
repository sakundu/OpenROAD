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

#pragma once

#include <memory>
#include <vector>

// Forward declarations
namespace odb {
class dbDatabase;
class dbBlock;
}  // namespace odb

namespace utl {
class Logger;
}

namespace dpl {
class Opendp;
class Architecture;
class Network;
}  // namespace dpl

namespace sa2d {

// Forward declarations
class ImmutableGridInfo;
class SAWorker;
class WorkerManager;

class SA2D
{
public:
  SA2D();
  ~SA2D();

  void init(odb::dbDatabase* db, utl::Logger* logger);
  void runSA();
  
  // SA Parameters (following sa1d pattern)
  void setNumWorkers(int num_workers) { num_workers_ = num_workers; }
  void setNumThreads(int num_threads) { num_threads_ = num_threads; }
  void setMaxTemp(float max_temp) { max_temp_ = max_temp; }
  void setMinTemp(float min_temp) { min_temp_ = min_temp; }
  void setCoolingRate(float cooling_rate) { cooling_rate_ = cooling_rate; }
  void setMaxIter(int max_iter) { max_iter_ = max_iter; }
  void setMoveBudget(int move_budget) { move_budget_ = move_budget; }
  void setSeed(int seed) { seed_ = seed; }
  
  // Displacement parameters (following DPL pattern)
  void setMaxDisplacement(int max_displacement_x, int max_displacement_y);
  
  // Parallel SA parameters
  void setGWTWInterval(int interval) { gwtw_interval_ = interval; }
  void setEliteRatio(float ratio) { elite_ratio_ = ratio; }
  
  // DPL integration
  void setDplEngine(dpl::Opendp* dpl) { dpl_ = dpl; }
  
  // Getters for worker access
  utl::Logger* getLogger() const { return logger_; }
  odb::dbBlock* getBlock() const { return block_; }
  
  // Initialize for testing
  void initTestGrid();

private:
  utl::Logger* logger_;
  odb::dbDatabase* db_;
  odb::dbBlock* block_;
  
  // Reuse DPL infrastructure (read-only)
  dpl::Opendp* dpl_;
  dpl::Architecture* arch_;
  dpl::Network* network_;
  
  // Thread-safe grid infrastructure
  std::unique_ptr<ImmutableGridInfo> grid_info_;  // Shared, immutable
  
  // Worker management
  std::unique_ptr<WorkerManager> worker_manager_;
  
  // SA parameters
  int num_workers_ = 1;
  int num_threads_ = 1;  // For future OpenMP support
  float max_temp_ = 100.0;
  float min_temp_ = 1e-6;
  float cooling_rate_ = 0.95;
  int max_iter_ = 1000;
  int move_budget_ = 100000;
  int seed_ = 0;
  
  // Parallel SA parameters
  int gwtw_interval_ = 100;  // Iterations between GWTW sync
  float elite_ratio_ = 0.2f;  // Top 20% are "winners"
  
  // Max displacement (in grid sites, following DPL)
  int max_displacement_x_ = 500;  // Default from DPL
  int max_displacement_y_ = 100;  // Default from DPL
  
  // Private methods
  void runSingleWorkerSA();
  void runParallelSA();
};

}  // namespace sa2d 