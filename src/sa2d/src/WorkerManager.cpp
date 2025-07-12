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

#include "WorkerManager.h"
#include "Worker.h"
#include "sa2d/SA2D.h"
#include "utl/Logger.h"
#include <algorithm>
#include <random>

namespace sa2d {

WorkerManager::WorkerManager(int num_workers, SA2D* sa2d)
    : sa2d_(sa2d),
      gwtw_rng_(0)  // Will be properly seeded in initializeWorkers
{
    workers_.reserve(num_workers);
    
    // Create sync barrier for num_workers threads
    sync_barrier_ = std::make_unique<SimpleBarrier>(num_workers);
}

WorkerManager::~WorkerManager() = default;

void WorkerManager::initializeWorkers(dpl::Network* network, 
                                    const ImmutableGridInfo* grid_info,
                                    int seed,
                                    int max_displacement_x,
                                    int max_displacement_y,
                                    float max_temp,
                                    float cooling_rate,
                                    int max_iter,
                                    int move_budget,
                                    int moves_per_iter,
                                    int kick_interval,
                                    float kick_threshold,
                                    int kick_strength,
                                    float kick_temp_multiplier,
                                    bool enable_kicks,
                                    bool enable_chain_moves,
                                    int chain_move_interval,
                                    int chain_moves_per_round,
                                    bool enable_slides,
                                    bool use_sa1d_operators,
                                    const std::vector<float>& sa1d_move_probs,
                                    bool use_best_orderings_1d,
                                    float sa1d_overlap_weight)
{
    // Seed GWTW RNG
    gwtw_rng_.seed(seed + 1000);  // Different from worker seeds
    
    // Store grid info reference
    grid_info_ = grid_info;
    
    // Get architecture from SA2D's stored reference
    const dpl::Architecture* arch = nullptr;  // Will be passed through grid_info
    
    // Create and initialize workers
    for (size_t i = 0; i < workers_.capacity(); ++i) {
        auto worker = std::make_unique<SAWorker>(sa2d_, i);
        
        // Initialize from DPL state
        worker->initFromDPL(network, arch, grid_info);
        
        // Set SA parameters
        worker->setSeed(seed + i);  // Each worker gets unique seed
        worker->setMaxDisplacement(max_displacement_x, max_displacement_y);
        worker->setTemp(max_temp);
        worker->setCoolingRate(cooling_rate);
        worker->setMaxIter(max_iter);
        worker->setMoveBudget(move_budget);
        worker->setMovesPerIter(moves_per_iter);
        
        // Set LSMC parameters
        worker->setKickInterval(kick_interval);
        worker->setKickThreshold(kick_threshold);
        worker->setKickStrength(kick_strength);
        worker->setKickTempMultiplier(kick_temp_multiplier);
        worker->setEnableKicks(enable_kicks);
        
        // Chain move control
        worker->setEnableChainMoves(enable_chain_moves);
        worker->setChainMoveInterval(chain_move_interval);
        worker->setChainMovesPerRound(chain_moves_per_round);
        worker->setEnableSlides(enable_slides);
        
        // Set SA1D operator parameters
        worker->setUseSA1DOperators(use_sa1d_operators);
        worker->setSA1DMoveProbs(sa1d_move_probs);
        worker->setUseBestOrderings1D(use_best_orderings_1d);
        worker->setSA1DOverlapWeight(sa1d_overlap_weight);
        
        workers_.push_back(std::move(worker));
    }
    
    // Initialize global best cost with the best of all workers
    std::vector<int64_t> initial_costs = getWorkerCosts();
    global_best_cost_ = *std::min_element(initial_costs.begin(), initial_costs.end());
    best_worker_id_ = std::distance(initial_costs.begin(), 
                                   std::min_element(initial_costs.begin(), initial_costs.end()));
}

void WorkerManager::runWorkers(int iterations)
{
    // Launch worker threads
    std::vector<std::thread> threads;
    threads.reserve(workers_.size());
    
    for (auto& worker : workers_) {
        threads.emplace_back([this, iterations, &worker]() {
            worker->runParallel(iterations, *sync_barrier_, should_stop_);
        });
    }
    
    // Wait for all threads to complete
    for (auto& thread : threads) {
        thread.join();
    }
}

void WorkerManager::performGWTW()
{
    // 1. Collect costs from all workers
    std::vector<std::pair<int64_t, int>> worker_costs;
    worker_costs.reserve(workers_.size());
    
    for (size_t i = 0; i < workers_.size(); ++i) {
        worker_costs.emplace_back(workers_[i]->getCurrentCost(), i);
    }
    
    // 2. Sort by cost (ascending - lower is better)
    std::sort(worker_costs.begin(), worker_costs.end());
    
    // 3. Identify winners (top elite_ratio)
    int num_winners = std::max(1, static_cast<int>(workers_.size() * elite_ratio_));
    std::vector<int> winner_ids;
    winner_ids.reserve(num_winners);
    
    // Reset winner status for all workers
    for (auto& worker : workers_) {
        worker->resetWinner();
    }
    
    // Mark winners
    for (int i = 0; i < num_winners; ++i) {
        int winner_id = worker_costs[i].second;
        winner_ids.push_back(winner_id);
        workers_[winner_id]->setAsWinner();
    }
    
    // 4. Losers copy from random winners
    std::uniform_int_distribution<int> winner_dist(0, num_winners - 1);
    
    for (size_t i = num_winners; i < workers_.size(); ++i) {
        int loser_id = worker_costs[i].second;
        int winner_idx = winner_dist(gwtw_rng_);
        int winner_id = winner_ids[winner_idx];
        
        // Copy winner's state to loser
        workers_[loser_id]->copyStateFrom(*workers_[winner_id]);
    }
    
    // 5. Update global best - check ALL workers' best solutions, not just current
    // This ensures we track the best solution ever found across all workers
    for (size_t i = 0; i < workers_.size(); ++i) {
        int64_t worker_best = workers_[i]->getBestCost();
        if (worker_best < global_best_cost_) {
            global_best_cost_ = worker_best;
            best_worker_id_ = static_cast<int>(i);
        }
    }
}

void WorkerManager::updateGlobalBest()
{
    // Find the worker with the best solution after all iterations complete
    // This is critical because workers may have found better solutions after the last GWTW sync
    
    int64_t best_cost = std::numeric_limits<int64_t>::max();
    int best_id = -1;
    
    for (size_t i = 0; i < workers_.size(); ++i) {
        // Get the best cost from each worker (not current cost)
        int64_t worker_best = workers_[i]->getBestCost();
        if (worker_best < best_cost) {
            best_cost = worker_best;
            best_id = static_cast<int>(i);
        }
    }
    
    // Update global tracking
    if (best_id >= 0) {
        global_best_cost_ = best_cost;
        best_worker_id_ = best_id;
    }
}

void WorkerManager::reportProgress(int iteration, int total_iterations)
{
    utl::Logger* logger = sa2d_->getLogger();
    
    // Print header on first iteration
    if (iteration == 0) {
        logger->info(utl::SA2D, 410, "");
        logger->info(utl::SA2D, 411, "Iteration    Progress    Temperature    Current HPWL    Best HPWL    Accept Rate");
        logger->info(utl::SA2D, 412, "---------    --------    -----------    ------------    ---------    -----------");
    }
    
    // Report at regular intervals (every 10% or every 100 iterations, whichever is smaller)
    int report_interval = std::min(100, std::max(1, total_iterations / 10));
    
    if (iteration % report_interval == 0 || iteration == total_iterations - 1) {
        // Collect statistics from all workers
        std::vector<int64_t> costs = getWorkerCosts();
        int64_t min_cost = *std::min_element(costs.begin(), costs.end());
        
        // Get temperature from first worker (all should be similar)
        float current_temp = workers_[0]->getCurrentTemp();
        
        // Calculate average accept rate
        double total_accept_rate = 0.0;
        for (const auto& worker : workers_) {
            total_accept_rate += worker->getAcceptRate();
        }
        double avg_accept_rate = total_accept_rate / workers_.size();
        
        // Calculate progress percentage
        int progress = (iteration * 100) / total_iterations;
        
        // Update global best to ensure we show the true best found so far
        // Check all workers' best solutions
        for (size_t i = 0; i < workers_.size(); ++i) {
            int64_t worker_best = workers_[i]->getBestCost();
            if (worker_best < global_best_cost_) {
                global_best_cost_ = worker_best;
                best_worker_id_ = static_cast<int>(i);
            }
        }
        
        // Format the output in aligned columns
        logger->info(utl::SA2D, 413, "{:>9}    {:>7}%    {:>11.2e}    {:>12.1f}    {:>9.1f}    {:>10.1f}%",
                    iteration,
                    progress,
                    current_temp,
                    sa2d_->getBlock()->dbuToMicrons(min_cost),
                    sa2d_->getBlock()->dbuToMicrons(global_best_cost_),
                    avg_accept_rate * 100.0);
    }
}

std::vector<int64_t> WorkerManager::getWorkerCosts() const
{
    std::vector<int64_t> costs;
    costs.reserve(workers_.size());
    
    for (const auto& worker : workers_) {
        costs.push_back(worker->getCurrentCost());
    }
    
    return costs;
}

void WorkerManager::applyBestSolution(dpl::Network* network)
{
    if (best_worker_id_ >= 0 && best_worker_id_ < static_cast<int>(workers_.size())) {
        workers_[best_worker_id_]->applyToDPL(network);
    }
}

void WorkerManager::reportMoveStatistics()
{
    utl::Logger* logger = sa2d_->getLogger();
    
    // Aggregate kick statistics from all workers
    int total_kick_attempts = 0;
    int total_kick_accepted = 0;
    int total_swaps_applied = 0;
    
    for (const auto& worker : workers_) {
        total_kick_attempts += worker->getKickAttempts();
        total_kick_accepted += worker->getKickAccepted();
        total_swaps_applied += worker->getTotalSwapsApplied();
    }
    
    if (total_kick_attempts > 0) {
        logger->info(utl::SA2D, 331, "Aggregate LSMC kicks: {} attempted, {} accepted ({:.1f}% success), {} total swaps",
                    total_kick_attempts, total_kick_accepted,
                    100.0 * total_kick_accepted / total_kick_attempts,
                    total_swaps_applied);
        
        // Report if specialized low-row kick moves were used
        if (grid_info_ && grid_info_->getRowCount() <= 5) {
            logger->info(utl::SA2D, 332, "Used specialized kick strategies for {}-row design: horizontal chain swap, row compression, inter-row transfer, sliding window",
                        grid_info_->getRowCount());
        }
    }
    
    // Report runtime statistics for each worker
    //for (const auto& worker : workers_) {
    //    worker->reportRuntimeStatistics();
    //}
}

}  // namespace sa2d 