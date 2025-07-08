# Simulated Annealing-Based 2D Detailed Placer Plan

**Note: This document describes the long-term vision for SA2D. For the initial implementation (v0), see [PLAN_v0.md](PLAN_v0.md) which focuses on a simplified version that reuses DPL infrastructure.**

## Overview

This document outlines the comprehensive plan for implementing a simulated annealing-based 2D detailed placer as an enhancement/alternative to the existing detailed placement engine in OpenROAD. The implementation will build upon the existing infrastructure in `src/dpl` while introducing simulated annealing as the core optimization algorithm.

## Understanding of Existing DPL Implementation

### Current Architecture

The existing detailed placement (dpl) implementation follows this architecture:

1. **Core Data Structures**:
   - `Node`: Represents cells/instances with position, dimensions, and connectivity
   - `Edge`: Represents nets connecting cells
   - `Pin`: Connection points on cells
   - `Group`: Placement regions/groups
   - `Grid`: Pixel-based representation of the placement area
   - `Architecture`: Row and site information
   - `Network`: Complete netlist representation

2. **Optimization Approach**:
   - Multiple optimization passes: MIS (Maximum Independent Set), global swaps, vertical swaps, reordering
   - Random moves with HPWL objective
   - Greedy acceptance of improvements
   - Script-based optimization flow

3. **Key Features**:
   - Handles multi-row cells
   - Respects placement regions and blockages
   - Maintains legality constraints (row alignment, no overlaps)
   - Supports incremental updates via journaling

## Proposed SA2D Architecture

### Design Philosophy

The simulated annealing-based placer will:
- Reuse existing infrastructure (Grid, Node, Network, Architecture)
- Replace the greedy optimization with simulated annealing
- Provide better escape from local minima
- Support multi-objective optimization (HPWL, congestion, timing)

### Core Components

#### 1. SA Engine (`sa2d/src/sa_engine.h/.cpp`)
```cpp
class SAEngine {
    // Core SA parameters
    double initial_temperature_;
    double cooling_rate_;
    double min_temperature_;
    
    // Move generation and evaluation
    std::unique_ptr<MoveGenerator> move_gen_;
    std::unique_ptr<CostEvaluator> cost_eval_;
    
    // Main SA loop
    void anneal(DetailedMgr* mgr);
    bool acceptMove(double delta_cost, double temperature);
};
```

#### 2. Move Generation (`sa2d/src/moves/`)
- **SingleCellMove**: Move one cell to a new legal position
- **CellSwapMove**: Swap two cells
- **ChainMove**: Shift a chain of cells
- **RippleMove**: Move cell and ripple-adjust neighbors
- **WindowMove**: Optimize within a window

#### 3. Cost Functions (`sa2d/src/objectives/`)
- **HPWLCost**: Half-perimeter wirelength (primary)
- **DisplacementCost**: Penalize large movements
- **DensityCost**: Maintain uniform density
- **CongestionCost**: Consider routing congestion
- **TimingCost**: Critical path optimization (future)

#### 4. Temperature Schedule (`sa2d/src/schedule/`)
- **AdaptiveSchedule**: Adjust based on acceptance rate
- **GeometricSchedule**: Classic geometric cooling
- **FastSchedule**: Aggressive cooling for quick results

### Implementation Phases

#### Phase 1: Core Infrastructure (Weeks 1-2)
- [ ] Set up directory structure and CMake integration
- [ ] Create base classes for SA engine
- [ ] Implement basic move generators (single cell, swap)
- [ ] Port HPWL calculation from dpl

#### Phase 2: Basic SA Implementation (Weeks 3-4)
- [ ] Implement temperature scheduling
- [ ] Create move acceptance logic
- [ ] Add legalization after moves
- [ ] Basic cost function (HPWL only)

#### Phase 3: Advanced Moves (Weeks 5-6)
- [ ] Chain move implementation
- [ ] Ripple move with efficient updates
- [ ] Window-based optimization
- [ ] Move probability adaptation

#### Phase 4: Multi-Objective (Weeks 7-8)
- [ ] Displacement penalty
- [ ] Density balancing
- [ ] Congestion awareness
- [ ] Weighted cost combination

#### Phase 5: Performance Optimization (Weeks 9-10)
- [ ] Incremental cost updates
- [ ] Parallel move evaluation
- [ ] Efficient data structures
- [ ] Move caching

#### Phase 6: Integration & Testing (Weeks 11-12)
- [ ] TCL command interface
- [ ] Integration with OpenROAD flow
- [ ] Comprehensive testing
- [ ] Performance benchmarking

## Key Design Decisions

### 1. Incremental Updates
- Maintain incremental HPWL computation
- Use spatial data structures (R-tree) for efficient queries
- Cache move evaluations when possible

### 2. Legalization Strategy
- **Immediate**: Legalize after each move (slower but always legal)
- **Deferred**: Allow temporary illegality, legalize periodically
- **Hybrid**: Keep cells in legal positions but allow temporary overlaps

### 3. Parallelization
- Parallel move evaluation for independent moves
- Speculative execution of moves
- GPU acceleration for cost computation (future)

### 4. Adaptive Behavior
- Dynamic temperature adjustment based on solution quality
- Move type selection based on success rates
- Problem-specific parameter tuning

## File Structure

```
sa2d/
├── README.md                 # User documentation
├── CMakeLists.txt           # Build configuration
├── include/sa2d/
│   ├── sa_placer.h         # Main interface
│   └── sa_engine.h         # SA engine interface
├── src/
│   ├── sa_placer.cpp       # Main implementation
│   ├── sa_engine.cpp       # SA core algorithm
│   ├── moves/              # Move generators
│   │   ├── move_base.h
│   │   ├── single_move.cpp
│   │   ├── swap_move.cpp
│   │   ├── chain_move.cpp
│   │   └── window_move.cpp
│   ├── objectives/         # Cost functions
│   │   ├── objective_base.h
│   │   ├── hpwl_objective.cpp
│   │   ├── density_objective.cpp
│   │   └── timing_objective.cpp
│   ├── schedule/           # Temperature schedules
│   │   ├── schedule_base.h
│   │   ├── geometric_schedule.cpp
│   │   └── adaptive_schedule.cpp
│   └── utils/              # Helper utilities
│       ├── incremental_hpwl.cpp
│       └── move_cache.cpp
├── test/                   # Unit and integration tests
│   ├── test_moves.cpp
│   ├── test_objectives.cpp
│   └── test_sa_engine.cpp
└── doc/                    # Detailed documentation
    ├── algorithm.md
    └── api.md
```

## Integration with OpenROAD

### TCL Interface
```tcl
# Run SA-based detailed placement
sa_detailed_placement
    [-temperature_schedule geometric|adaptive]
    [-initial_temp 1000]
    [-cooling_rate 0.95]
    [-moves_per_temp 1000]
    [-target_hpwl_improvement 5.0]
    [-max_displacement 10]
    [-seed 42]
```

### C++ API
```cpp
// Create SA placer
auto sa_placer = std::make_unique<SA2DPlacer>();
sa_placer->init(db, logger);
sa_placer->setTemperatureSchedule(SASchedule::Adaptive);
sa_placer->setMaxDisplacement(10);

// Run placement
sa_placer->place();

// Get statistics
auto stats = sa_placer->getStats();
```

## Performance Targets

- **Runtime**: Comparable to existing detailed placement (within 2x)
- **Quality**: 5-10% HPWL improvement over greedy approach
- **Scalability**: Handle designs with 10M+ cells
- **Memory**: Linear memory usage with design size

## Risk Mitigation

1. **Performance Risk**: SA traditionally slower than greedy
   - Mitigation: Aggressive optimizations, hybrid approaches
   
2. **Quality Risk**: May not always beat sophisticated greedy
   - Mitigation: Ensemble approach, multiple cooling schedules

3. **Integration Risk**: Complex integration with existing flow
   - Mitigation: Reuse existing infrastructure, incremental integration

## Success Metrics

1. **Primary**: HPWL improvement over existing detailed placement
2. **Secondary**: Runtime efficiency, robustness across benchmarks
3. **Tertiary**: Extensibility for future objectives (timing, power)

## Next Steps

1. Review and refine this plan with stakeholders
2. Set up development environment and directory structure
3. Begin Phase 1 implementation
4. Establish testing infrastructure early

## References

- Classic SA for placement: Kirkpatrick et al. (1983)
- TimberWolf: Sechen & Sangiovanni-Vincentelli (1986)
- Modern SA techniques: Wong et al. (2019)
- OpenROAD DPL documentation and source code 