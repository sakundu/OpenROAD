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
    void setMoveBudget(int move_budget) { move_budget_ = move_budget; }
    void setMovesPerIter(int moves_per_iter) { moves_per_iter_ = moves_per_iter; }
    void setMaxDisplacement(int max_x, int max_y);
    void setSeed(int seed);
    
    // LSMC parameters
    void setKickInterval(int interval) { kick_interval_ = interval; }
    void setKickThreshold(float threshold) { kick_threshold_ = threshold; }
    void setKickStrength(int strength) { kick_strength_ = strength; }
    void setKickTempMultiplier(float multiplier) { kick_temp_multiplier_ = multiplier; }
    void setEnableKicks(bool enable) { enable_kicks_ = enable; }
    
    // Chain move control
    void setEnableChainMoves(bool enable) { enable_chain_moves_ = enable; }
    void setChainMoveInterval(int interval) { chain_move_interval_ = interval; }
    void setChainMovesPerRound(int moves) { chain_moves_per_round_ = moves; }
    void setEnableSlides(bool enable) { enable_slides_ = enable; }
    
    // Main SA run function
    void run();
    
    // Parallel execution
    void runParallel(int iterations, SimpleBarrier& sync_barrier, 
                    const std::atomic<bool>& should_stop);
    void copyStateFrom(const SAWorker& other);
    
    // State accessors
    int64_t getTotalHPWL() { return state_.total_hpwl; }
    int64_t getBestHPWL() { return best_state_.total_hpwl; }
    // Get current cost for GWTW
    int64_t getCurrentCost() const { return state_.total_hpwl; }
    
    // Get best cost found by this worker
    int64_t getBestCost() const { return best_state_.total_hpwl; }
    
    // Current temperature
    float getCurrentTemp() const { return temp_; }
    
    // Acceptance rate tracking
    double getAcceptRate() const { 
        int total_moves = accepted_moves_ + rejected_moves_;
        return total_moves > 0 ? (double)accepted_moves_ / total_moves : 0.0;
    }
    
    // Winner status for GWTW
    void setAsWinner() { is_winner_ = true; }
    void resetWinner() { is_winner_ = false; }
    
    // For GWTW
    bool isWinner() const { return is_winner_; }
    
    // Initialize from DPL current state
    void initFromDPL(dpl::Network* network,
                     const dpl::Architecture* arch,
                     const ImmutableGridInfo* grid_info);
    
    // Apply best solution back to DPL
    void applyToDPL(dpl::Network* network);
    
    // Debug: Check grid/state consistency
    bool checkGridStateConsistency();
    
    // Get move statistics
    int getAcceptedSingleMoves() const { return accepted_single_moves_; }
    int getAcceptedSwaps() const { return accepted_swaps_; }
    int getAcceptedFlips() const { return accepted_flips_; }
    int getAcceptedSlides() const { return accepted_slides_; }
    int getAcceptedChainMoves() const { return accepted_chain_moves_; }
    int getAttemptedSingleMoves() const { return attempted_single_moves_; }
    int getAttemptedSwaps() const { return attempted_swaps_; }
    int getAttemptedFlips() const { return attempted_flips_; }
    int getAttemptedSlides() const { return attempted_slides_; }
    int getAttemptedChainMoves() const { return attempted_chain_moves_; }
    int getTotalCellsShifted() const { return total_cells_shifted_; }
    int getMaxChainLength() const { return max_chain_length_; }
    
    // Different-size swap statistics
    int getAttemptedDiffSizeSwaps() const { return attempted_diff_size_swaps_; }
    int getAcceptedDiffSizeSwaps() const { return accepted_diff_size_swaps_; }
    
    // Get kick statistics
    int getKickAttempts() const { return kick_attempts_; }
    int getKickAccepted() const { return kick_accepted_; }
    int getTotalSwapsApplied() const { return total_swaps_applied_; }
    
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
    int moves_per_iter_; // New member variable
    
    // Max displacement in grid sites (from SA2D)
    int max_displacement_x_;
    int max_displacement_y_;
    
    // LSMC parameters
    int kick_interval_ = 100;
    float kick_threshold_ = 0.05f;
    int kick_strength_ = 10;
    float kick_temp_multiplier_ = 1.5f;
    bool enable_kicks_ = true;
    
    // Chain move control  
    bool enable_chain_moves_ = true;  // Enable/disable chain moves
    int chain_move_interval_ = 50;    // How often to attempt chain moves  
    int chain_moves_per_round_ = 5;   // Number of chain moves to try when triggered
    bool enable_slides_ = true; // Enable/disable slide moves
    
    // LSMC tracking
    int stagnation_counter_ = 0;
    double rolling_accept_rate_ = 1.0;  // Start optimistic
    int last_kick_iteration_ = -100;   // Allow early kick
    int last_chain_iteration_ = -50;   // Allow early chain moves
    int64_t best_hpwl_at_last_improvement_ = 0;
    
    // Parallel execution state
    bool is_winner_ = false;
    
    // Move operations with full legality
    bool tryMove(int cell_id);
    bool trySwap(int cell1_id, int cell2_id);
    bool tryFlip(int cell_id);  // Y-axis flip - supports multi-height cells
    bool trySlide(int cell_id);  // Slide cell left/right to best HPWL position
    
    // Multi-height cell operations
    bool tryMoveMultiHeight(int cell_id);
    bool trySwapMultiHeight(int cell1_id, int cell2_id);
    bool canPlaceMultiHeightCell(int cell_id, GridX x, GridY y);
    
    // Chain/ripple move operations
    bool tryChainMove(int cell_id);
    bool tryRippleLeft(int cell_id, GridPt target_pos, int max_chain_length = 3);
    bool tryRippleRight(int cell_id, GridPt target_pos, int max_chain_length = 3);
    
    struct ChainedMove {
        int cell_id;
        GridPt old_pos;
        GridPt new_pos;
        odb::dbOrientType old_orient;
        odb::dbOrientType new_orient;
    };
    
    bool executeChain(const std::vector<ChainedMove>& chain);
    void revertChain(const std::vector<ChainedMove>& chain);
    int64_t calculateChainDelta(const std::vector<ChainedMove>& chain);
    bool validateChain(const std::vector<ChainedMove>& chain);
    
    // Legality checking using WorkerGrid
    bool canPlaceCell(int cell_id, GridX x, GridY y);
    
    // Get the correct orientation for a position
    odb::dbOrientType getCellOrientation(int cell_id, GridX x, GridY y);
    
    // Cost evaluation
    int64_t calcInitialHPWL();
    int64_t calcDeltaHPWL(const std::vector<int>& affected_nets);
    void updateHPWLCache(const std::vector<int>& affected_nets);
    int64_t calcNetHPWL(int net_id);  // Calculate HPWL for a single net
    std::vector<int> getAffectedNets(int cell_id);
    
    // SA acceptance
    bool acceptMove(int64_t delta_cost, float temp);
    
    // Generate random position within displacement limits
    GridPt generateRandomPosition(int cell_id);
    
    // Update best solution if current is better
    void updateBestSolution();
    
    // LSMC kick move operations
    bool shouldPerformKick(int iteration);
    bool shouldPerformChainMoves(int iteration);
    bool tryRegionShuffle(int region_size);
    void performSwapInState(int cell1_id, int cell2_id);
    void applySwapToGrid(int cell1_id, int cell2_id);
    
    // Statistics
    int accepted_moves_{0};
    int rejected_moves_{0};
    int illegal_moves_{0};
    int accepted_flips_{0};  // Track successful flips
    int accepted_swaps_{0};  // Track successful swaps
    int accepted_single_moves_{0};  // Track successful single moves
    int accepted_slides_{0};  // Track successful slide moves
    
    // Attempt counters
    int attempted_flips_{0};
    int attempted_swaps_{0};
    int attempted_single_moves_{0};
    int attempted_slides_{0};
    int attempted_chain_moves_{0};  // Track chain move attempts
    
    // Different-size swap tracking
    int attempted_diff_size_swaps_{0};
    int accepted_diff_size_swaps_{0};
    
    // LSMC kick statistics
    int kick_attempts_{0};
    int kick_accepted_{0};
    int kick_rejected_{0};
    int total_swaps_applied_{0};
    
    // Chain move specific statistics
    int accepted_chain_moves_{0};
    int total_cells_shifted_{0};  // Total cells affected by chain moves
    int max_chain_length_{0};     // Track longest successful chain
    
    // Performance optimization: cache affected nets
    mutable std::unordered_map<int, std::vector<int>> affected_nets_cache_;
    
    // Random number generation
    std::mt19937 rng_;
    std::uniform_real_distribution<float> distribution_;
};

}  // namespace sa2d 