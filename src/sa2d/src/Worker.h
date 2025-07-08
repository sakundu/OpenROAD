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
#include <random>
#include <vector>
#include <unordered_map>
#include <atomic>

#include "CellState.h"
#include "ThreadSafeGrid.h"

// Forward declarations
namespace dpl {
class Network;
class Architecture;
class Node;
class Net;
class Pin;
class Group;
}  // namespace dpl

namespace sa2d {

// Forward declarations
class SA2D;
class SimpleBarrier;

class SAWorker {
public:
    SAWorker(SA2D* sa2d, int worker_id);
    ~SAWorker() = default;
    
    // SA parameters
    void setTemp(float temp) { temp_ = temp; }
    void setCoolingRate(float cooling_rate) { cooling_rate_ = cooling_rate; }
    void setMaxIter(int max_iter) { max_iter_ = max_iter; }
    void setMaxDisplacement(int max_x, int max_y);
    void setMoveBudget(int move_budget) { move_budget_ = move_budget; }
    void setSeed(int seed);
    
    // Main SA run function
    void run();
    
    // Parallel execution
    void runParallel(int iterations, SimpleBarrier& sync_barrier, 
                    const std::atomic<bool>& should_stop);
    void copyStateFrom(const SAWorker& other);
    
    // Get costs
    int64_t getTotalHPWL() const { return state_.total_hpwl; }
    int64_t getBestHPWL() const { return best_state_.total_hpwl; }
    int64_t getCurrentCost() const { return state_.total_hpwl; }
    
    // For GWTW
    void setAsWinner() { is_winner_ = true; }
    void resetWinner() { is_winner_ = false; }
    bool isWinner() const { return is_winner_; }
    
    // Initialize from DPL current state
    void initFromDPL(dpl::Network* network,
                     const dpl::Architecture* arch,
                     const ImmutableGridInfo* grid_info);
    
    // Apply best solution back to DPL
    void applyToDPL(dpl::Network* network);
    
private:
    SA2D* sa2d_;
    int worker_id_;
    
    // Lightweight state (parallel-ready)
    WorkerState state_;
    WorkerState best_state_;
    
    // Grid for accurate legality checking
    std::unique_ptr<WorkerGrid> grid_;
    std::unique_ptr<WorkerGrid> best_grid_;
    
    // References to shared read-only structures
    dpl::Network* network_;  // Non-const to access non-const methods
    const dpl::Architecture* arch_;
    const ImmutableGridInfo* grid_info_;
    
    // SA parameters
    float temp_;
    float cooling_rate_;
    int max_iter_;
    int move_budget_;
    
    // Max displacement in grid sites (from SA2D)
    int max_displacement_x_;
    int max_displacement_y_;
    
    // Parallel execution state
    bool is_winner_ = false;
    
    // Move operations with full legality
    bool tryMove(int cell_id);
    bool trySwap(int cell1_id, int cell2_id);
    
    // Legality checking using WorkerGrid
    bool canPlaceCell(int cell_id, GridX x, GridY y);
    
    // Get the correct orientation for a position
    odb::dbOrientType getCellOrientation(int cell_id, GridX x, GridY y);
    
    // Cost evaluation
    int64_t calcInitialHPWL();
    int64_t calcDeltaHPWL(const std::vector<int>& affected_nets);
    void updateHPWLCache(const std::vector<int>& affected_nets);
    std::vector<int> getAffectedNets(int cell_id);
    
    // SA acceptance
    bool acceptMove(int64_t delta_cost, float temp);
    
    // Generate random position within displacement limits
    GridPt generateRandomPosition(int cell_id);
    
    // Update best solution if current is better
    void updateBestSolution();
    
    // Random number generation
    std::mt19937 rng_;
    std::uniform_real_distribution<float> distribution_;
    
    // Move statistics
    int accepted_moves_ = 0;
    int rejected_moves_ = 0;
    int illegal_moves_ = 0;
};

}  // namespace sa2d 