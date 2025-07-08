# Parallel SA Implementation Plan for SA2D

## Overview

This document outlines the implementation plan for adding parallel simulated annealing with Go-With-The-Winners (GWTW) synchronization to SA2D. The current single-worker implementation provides the foundation for parallelization.

## Goals

1. **Multiple concurrent SA workers** - Each exploring the solution space independently
2. **GWTW synchronization** - Periodic sharing of best solutions among workers
3. **Scalable architecture** - Support 2-20+ workers efficiently
4. **Minimal overhead** - Low synchronization cost, efficient memory usage
5. **Deterministic results** - Given same seed and worker count

## Current Architecture (Single Worker)

```
SA2D
  ├── ImmutableGridInfo (shared, read-only)
  ├── Network (shared, read-only from DPL)
  └── SAWorker
       ├── WorkerState (positions, HPWL cache)
       ├── WorkerGrid (occupancy tracking)
       └── RNG (per-worker random state)
```

## Target Parallel Architecture

```
SA2D
  ├── ImmutableGridInfo (shared across all workers)
  ├── Network (shared, read-only)
  ├── WorkerManager (new)
  │    ├── Worker synchronization
  │    ├── GWTW logic
  │    └── Progress tracking
  └── SAWorker[] (array of workers)
       ├── Independent state
       ├── Independent grid
       └── Independent RNG
```

## Implementation Phases

### Phase 1: Multi-Worker Infrastructure

#### 1.1 Update SA2D Class
```cpp
class SA2D {
public:
    // Existing methods...
    
    // New parallel control
    void runParallelSA();
    
private:
    // Worker management
    std::vector<std::unique_ptr<SAWorker>> workers_;
    std::unique_ptr<WorkerManager> worker_manager_;
    
    // Parallel parameters
    int gwtw_interval_ = 100;  // Iterations between GWTW sync
    float elite_ratio_ = 0.2;  // Top 20% are "winners"
};
```

#### 1.2 Create WorkerManager Class
```cpp
class WorkerManager {
public:
    WorkerManager(int num_workers, SA2D* sa2d);
    
    // Worker lifecycle
    void initializeWorkers(dpl::Network* network, 
                          const ImmutableGridInfo* grid_info);
    void runWorkers(int iterations);
    void applyBestSolution(dpl::Network* network);
    
    // GWTW synchronization
    void performGWTW();
    
    // Progress monitoring
    void reportProgress();
    std::vector<int64_t> getWorkerCosts() const;
    
private:
    SA2D* sa2d_;
    std::vector<SAWorker*> workers_;  // Non-owning pointers
    
    // Synchronization
    std::barrier<> sync_barrier_;  // C++20 barrier
    std::atomic<bool> should_stop_{false};
    
    // Best global solution tracking
    int best_worker_id_ = -1;
    int64_t global_best_cost_ = std::numeric_limits<int64_t>::max();
};
```

#### 1.3 Update SAWorker for Parallel Execution
```cpp
class SAWorker {
public:
    // Existing methods...
    
    // New parallel methods
    void runParallel(int iterations, std::barrier<>& sync_barrier);
    void copyStateFrom(const SAWorker& other);
    int64_t getCurrentCost() const { return state_.total_hpwl; }
    
    // For GWTW
    void setAsWinner() { is_winner_ = true; }
    bool isWinner() const { return is_winner_; }
    
private:
    // Parallel execution state
    bool is_winner_ = false;
    std::atomic<bool>* should_stop_ = nullptr;
};
```

### Phase 2: GWTW Implementation

#### 2.1 GWTW Algorithm
```cpp
void WorkerManager::performGWTW() {
    // 1. Collect costs from all workers
    std::vector<std::pair<int64_t, int>> worker_costs;
    for (int i = 0; i < workers_.size(); ++i) {
        worker_costs.emplace_back(workers_[i]->getCurrentCost(), i);
    }
    
    // 2. Sort by cost (ascending)
    std::sort(worker_costs.begin(), worker_costs.end());
    
    // 3. Identify winners (top elite_ratio)
    int num_winners = std::max(1, (int)(workers_.size() * elite_ratio_));
    std::vector<int> winner_ids;
    for (int i = 0; i < num_winners; ++i) {
        winner_ids.push_back(worker_costs[i].second);
        workers_[worker_costs[i].second]->setAsWinner();
    }
    
    // 4. Losers copy from random winners
    std::uniform_int_distribution<int> winner_dist(0, num_winners - 1);
    std::mt19937 rng(seed_);
    
    for (int i = num_winners; i < workers_.size(); ++i) {
        int loser_id = worker_costs[i].second;
        int winner_idx = winner_dist(rng);
        int winner_id = winner_ids[winner_idx];
        
        // Copy winner's state to loser
        workers_[loser_id]->copyStateFrom(*workers_[winner_id]);
    }
    
    // 5. Update global best
    if (worker_costs[0].first < global_best_cost_) {
        global_best_cost_ = worker_costs[0].first;
        best_worker_id_ = worker_costs[0].second;
    }
}
```

#### 2.2 State Copying
```cpp
void SAWorker::copyStateFrom(const SAWorker& other) {
    // Copy cell positions and orientations
    state_ = other.state_;
    
    // Copy grid occupancy
    grid_->copyFrom(*other.grid_);
    
    // Note: Don't copy RNG state - maintain diversity
    // Note: Don't copy best_state_ - let worker find its own
    
    // Reset winner status
    is_winner_ = false;
}
```

### Phase 3: Parallel Execution

#### 3.1 Main Parallel Loop
```cpp
void SA2D::runParallelSA() {
    logger_->info(utl::SA2D, 301, "Starting parallel SA with {} workers", num_workers_);
    
    // Create worker manager
    worker_manager_ = std::make_unique<WorkerManager>(num_workers_, this);
    
    // Create workers
    workers_.clear();
    for (int i = 0; i < num_workers_; ++i) {
        auto worker = std::make_unique<SAWorker>(this, i);
        workers_.push_back(std::move(worker));
    }
    
    // Initialize all workers
    worker_manager_->initializeWorkers(network_, grid_info_.get());
    
    // Run parallel SA with GWTW
    int total_iterations = max_iter_;
    int sync_iterations = gwtw_interval_;
    
    for (int iter = 0; iter < total_iterations; iter += sync_iterations) {
        // Run workers for sync_iterations
        worker_manager_->runWorkers(sync_iterations);
        
        // Perform GWTW synchronization
        worker_manager_->performGWTW();
        
        // Report progress
        worker_manager_->reportProgress();
    }
    
    // Apply best solution
    worker_manager_->applyBestSolution(network_);
}
```

#### 3.2 Worker Thread Function
```cpp
void SAWorker::runParallel(int iterations, std::barrier<>& sync_barrier) {
    // Run SA for specified iterations
    for (int iter = 0; iter < iterations && !should_stop_->load(); ++iter) {
        int moves_per_temp = /* ... */;
        
        for (int move = 0; move < moves_per_temp; ++move) {
            // Same move logic as before
            if (/* do swap */) {
                trySwap(/* ... */);
            } else {
                tryMove(/* ... */);
            }
        }
        
        // Update best solution
        updateBestSolution();
        
        // Cool down
        temp_ *= cooling_rate_;
    }
    
    // Wait for all workers to finish this round
    sync_barrier.arrive_and_wait();
}
```

### Phase 4: Thread Management

#### 4.1 Thread Pool Approach
```cpp
void WorkerManager::runWorkers(int iterations) {
    // Launch worker threads
    std::vector<std::thread> threads;
    
    for (auto* worker : workers_) {
        threads.emplace_back([worker, iterations, this]() {
            worker->runParallel(iterations, sync_barrier_);
        });
    }
    
    // Wait for all threads to complete
    for (auto& thread : threads) {
        thread.join();
    }
}
```

#### 4.2 Alternative: OpenMP
```cpp
void WorkerManager::runWorkers(int iterations) {
    #pragma omp parallel for num_threads(workers_.size())
    for (int i = 0; i < workers_.size(); ++i) {
        workers_[i]->runParallel(iterations, sync_barrier_);
    }
}
```

## Memory Considerations

### Per-Worker Memory
- Cell states: 1M cells × 24 bytes = 24 MB
- Grid: 10M pixels × 4 bytes = 40 MB
- HPWL cache: ~8 MB
- **Total per worker: ~72 MB**

### Shared Memory
- ImmutableGridInfo: ~100 MB
- Network (read-only): Existing DPL memory

### Total for 10 Workers
- Workers: 10 × 72 MB = 720 MB
- Shared: 100 MB
- **Total: ~820 MB additional**

## TCL Interface Updates

```tcl
# Existing commands work as before
sa2d_set_num_workers 10        # Now actually creates 10 workers
sa2d_set_gwtw_interval 100     # New: iterations between GWTW
sa2d_set_elite_ratio 0.2       # New: fraction of winners

# Run command automatically uses parallel if num_workers > 1
sa2d_run
```

## Testing Strategy

### 1. Correctness Tests
- Verify deterministic results with same seed
- Compare single vs multi-worker quality
- Check no race conditions

### 2. Performance Tests
- Measure speedup vs worker count
- Profile synchronization overhead
- Memory usage scaling

### 3. Quality Tests
- HPWL improvement vs DPL
- Convergence rate analysis
- Parameter sensitivity

## Implementation Timeline

### Week 1: Multi-Worker Infrastructure
- [ ] Update SA2D class for multiple workers
- [ ] Create WorkerManager class
- [ ] Update SAWorker for parallel execution
- [ ] Basic thread management

### Week 2: GWTW Implementation
- [ ] Implement GWTW algorithm
- [ ] Add state copying between workers
- [ ] Test synchronization correctness
- [ ] Add progress reporting

### Week 3: Optimization & Testing
- [ ] Profile and optimize synchronization
- [ ] Tune GWTW parameters
- [ ] Comprehensive testing
- [ ] Memory usage optimization

### Week 4: Integration & Documentation
- [ ] Update TCL interface
- [ ] Write user documentation
- [ ] Performance benchmarking
- [ ] Code review and cleanup

## Success Metrics

1. **Speedup**: Near-linear speedup up to 8 workers
2. **Quality**: Better solutions than single worker in same time
3. **Memory**: <100 MB per worker overhead
4. **Stability**: No crashes or race conditions
5. **Determinism**: Reproducible results

## Future Enhancements

1. **Dynamic worker count** - Adjust based on problem size
2. **Adaptive GWTW interval** - Change based on convergence
3. **Hierarchical GWTW** - Groups of workers
4. **GPU acceleration** - For HPWL calculation
5. **Distributed execution** - MPI support 