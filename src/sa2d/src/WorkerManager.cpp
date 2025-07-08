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
                                    int move_budget)
{
    utl::Logger* logger = sa2d_->getLogger();
    
    // Seed GWTW RNG
    gwtw_rng_.seed(seed + 1000);  // Different from worker seeds
    
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
        
        workers_.push_back(std::move(worker));
    }
    
    logger->info(utl::SA2D, 401, "Initialized {} workers for parallel SA", workers_.size());
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
    utl::Logger* logger = sa2d_->getLogger();
    
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
    
    // 5. Update global best
    if (worker_costs[0].first < global_best_cost_) {
        global_best_cost_ = worker_costs[0].first;
        best_worker_id_ = worker_costs[0].second;
        
        logger->info(utl::SA2D, 402, "New global best: HPWL = {:.1f} u (worker {})",
                    sa2d_->getBlock()->dbuToMicrons(global_best_cost_),
                    best_worker_id_);
    }
    
    // Log GWTW results
    logger->info(utl::SA2D, 403, "GWTW: {} winners, best cost = {:.1f} u, worst cost = {:.1f} u",
                num_winners,
                sa2d_->getBlock()->dbuToMicrons(worker_costs[0].first),
                sa2d_->getBlock()->dbuToMicrons(worker_costs.back().first));
}

void WorkerManager::reportProgress(int iteration)
{
    utl::Logger* logger = sa2d_->getLogger();
    
    // Collect current costs
    std::vector<int64_t> costs = getWorkerCosts();
    
    // Calculate statistics
    int64_t min_cost = *std::min_element(costs.begin(), costs.end());
    int64_t max_cost = *std::max_element(costs.begin(), costs.end());
    double avg_cost = 0;
    for (int64_t cost : costs) {
        avg_cost += cost;
    }
    avg_cost /= costs.size();
    
    logger->info(utl::SA2D, 404, "Iteration {}: min={:.1f} u, avg={:.1f} u, max={:.1f} u",
                iteration,
                sa2d_->getBlock()->dbuToMicrons(min_cost),
                sa2d_->getBlock()->dbuToMicrons(avg_cost),
                sa2d_->getBlock()->dbuToMicrons(max_cost));
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
        utl::Logger* logger = sa2d_->getLogger();
        logger->info(utl::SA2D, 405, "Applying best solution from worker {}", best_worker_id_);
        workers_[best_worker_id_]->applyToDPL(network);
    }
}

}  // namespace sa2d 