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

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <random>
#include <thread>
#include <vector>

// Forward declarations
namespace dpl {
class Network;
}

namespace sa2d {

// Forward declarations
class SA2D;
class SAWorker;
class ImmutableGridInfo;

// Simple barrier implementation for C++17 compatibility
class SimpleBarrier {
public:
    explicit SimpleBarrier(std::size_t count) 
        : threshold_(count), count_(count), generation_(0) {}
    
    void arrive_and_wait() {
        auto gen = generation_.load();
        {
            std::unique_lock<std::mutex> lock(mutex_);
            if (--count_ == 0) {
                generation_++;
                count_ = threshold_;
                cv_.notify_all();
            } else {
                cv_.wait(lock, [this, gen] { return generation_ != gen; });
            }
        }
    }
    
private:
    std::mutex mutex_;
    std::condition_variable cv_;
    const std::size_t threshold_;
    std::size_t count_;
    std::atomic<std::size_t> generation_;
};

class WorkerManager {
public:
    WorkerManager(int num_workers, SA2D* sa2d);
    ~WorkerManager();
    
    // Public methods
    void initializeWorkers(dpl::Network* network, 
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
                          bool enable_slides);
    
    void runWorkers(int iterations);
    void performGWTW();
    
    // Update global best after all workers complete
    void updateGlobalBest();
    
    // Reporting
    void reportProgress(int iteration, int total_iterations);
    void reportMoveStatistics();
    
    // Apply best solution
    void applyBestSolution(dpl::Network* network);
    
    // Get aggregate costs
    std::vector<int64_t> getWorkerCosts() const;
    int64_t getBestCost() const { return global_best_cost_; }
    int getBestWorkerId() const { return best_worker_id_; }
    SAWorker* getBestWorker() const { return best_worker_id_ >= 0 ? workers_[best_worker_id_].get() : nullptr; }
    
    // GWTW configuration
    void setGWTWInterval(int interval) { gwtw_interval_ = interval; }
    void setEliteRatio(float ratio) { elite_ratio_ = ratio; }
    
    // Control
    void stop() { should_stop_ = true; }
    
    // Getters for debugging
    int64_t getGlobalBestCost() const { return global_best_cost_; }
    
private:
    SA2D* sa2d_;
    std::vector<std::unique_ptr<SAWorker>> workers_;
    
    // Synchronization
    std::unique_ptr<SimpleBarrier> sync_barrier_;
    std::atomic<bool> should_stop_{false};
    
    // GWTW parameters
    int gwtw_interval_ = 100;
    float elite_ratio_ = 0.2f;
    
    // Best global solution tracking
    int best_worker_id_ = -1;
    int64_t global_best_cost_ = std::numeric_limits<int64_t>::max();
    
    // Random number generation for GWTW
    std::mt19937 gwtw_rng_;
};

}  // namespace sa2d 