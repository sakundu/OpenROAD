# SA1D Vertex Ordering Implementation Plan

## Overview
This document outlines the implementation plan for integrating SAIT vertex ordering algorithms into the SA1D detailed placement framework in OpenROAD. The goal is to replace random initial placement with high-quality vertex orderings derived from hypergraph analysis.

## Architecture Overview

### Current SA1D Structure
- **Database**: OpenDB (ODB) contains physical design data (cells, nets, terminals)
- **SA Framework**: Multi-worker simulated annealing with GWTW (Go-With-The-Winners)
- **Placement**: 1D placement (single row) with orientation optimization
- **Existing Integration**: Basic vertex ordering infrastructure already implemented

### SAIT Integration Status
- **Implemented**: Basic vertex ordering interface in `VertexOrdering.cpp`
- **Implemented**: Best orderings framework in `BestOrderings.cpp`  
- **Implemented**: SAIT algorithm wrappers in `src/sait/` directory
- **Implemented**: Hypergraph conversion from ODB to SAIT format

## Hypergraph Generation from ODB Database

### Current Implementation Analysis

The hypergraph generation is implemented in `BestOrderingsInterface::convertToSAITHypergraph()` (line 260-304):

```cpp
std::unique_ptr<Hypergraph> BestOrderingsInterface::convertToSAITHypergraph() {
    const auto& cells = opt_sa_->getCells();  // SA1D Cell objects
    const auto& nets = opt_sa_->getNets();    // SA1D Net objects
    
    // Create hypergraph: vertices = cells, hyperedges = nets
    auto hg = std::make_unique<Hypergraph>(cells.size(), nets.size());
    
    // Create bidirectional mapping between cell IDs and vertex IDs
    for (size_t cell_id = 0; cell_id < cells.size(); ++cell_id) {
        cell_to_vertex_map_[cell_id] = static_cast<int>(cell_id);
        vertex_to_cell_map_[static_cast<int>(cell_id)] = static_cast<int>(cell_id);
    }
    
    // Convert nets to hyperedges
    for (size_t net_id = 0; net_id < nets.size(); ++net_id) {
        const auto& net = nets[net_id];
        std::vector<int> hyperedge_vertices;
        
        // Add all cells connected to this net
        for (const auto& term : net.getTerms()) {
            int cell_id = term.cell_id;
            hyperedge_vertices.push_back(cell_to_vertex_map_[cell_id]);
        }
        
        // Only add hyperedge if it has at least 2 vertices (nets must connect multiple cells)
        if (hyperedge_vertices.size() >= 2) {
            hg->addHyperedge(hyperedge_vertices);
        }
    }
    
    hg->buildVertexToHyperedgeMapping();
    return hg;
}
```

### ODB Database Structure Mapping

#### 1. Cell to Vertex Mapping
- **ODB Cell** (`sa1d::Cell`): Physical cell instance from `opt_sa_->getCells()`
  - `db_inst`: Pointer to OpenDB instance
  - `width`, `height`: Physical dimensions  
  - `mterm_locs`: Pin locations on the cell
  - `nets`: List of net IDs connected to this cell

- **SAIT Vertex** (`int`): Abstract graph node representing a cell
  - Direct 1:1 mapping: `vertex_id = cell_id`
  - Vertex IDs are 0-indexed consecutive integers

#### 2. Net to Hyperedge Mapping  
- **ODB Net** (`sa1d::Net`): Electrical connection between cells
  - `db_net`: Pointer to OpenDB net
  - `bterm_flag`: Whether net connects to I/O terminals
  - `bterm_box`: Bounding box for I/O terminals
  - `terms`: List of `netTerm` objects (cell_id, mterm_id pairs)

- **SAIT Hyperedge** (`std::vector<int>`): Set of vertices (cells) connected by the net
  - Contains all `vertex_id`s corresponding to cells connected by the net
  - Only includes hyperedges with ≥2 vertices (multi-point nets)

#### 3. Terminal Handling (bTerms)
SA1D properly extracts I/O terminal information from the ODB database through the `Net::updateBTerm()` method:

```cpp
void Net::updateBTerm() {
    bterm_flag = false;
    bterm_box.mergeInit();
    for (dbBTerm* bterm : db_net->getBTerms()) {
        bterm_flag = true;
        for (dbBPin* bpin : bterm->getBPins()) {
            Rect pin_bbox = bpin->getBBox();
            int center_x = (pin_bbox.xMin() + pin_bbox.xMax()) / 2;
            int center_y = (pin_bbox.yMin() + pin_bbox.yMax()) / 2;
            Rect pin_center(center_x, center_y, center_x, center_y);
            bterm_box.merge(pin_center);
        }
    }
}
```

**Current Status**: I/O terminals are properly detected and their coordinates computed, but they are **not** modeled as separate vertices in the hypergraph. The `bterm_box` provides the fixed coordinate constraints needed for advanced algorithms.

## SAIT Algorithm Integration

### Available Algorithms (from main.cpp:1041-1050)

#### Core Algorithms (Always Available)
1. **Fiedler Ordering**: Spectral method using Fiedler vector
2. **RCM (Reverse Cuthill-McKee)**: Bandwidth reduction 
3. **RCM-Boost**: Boost library implementation
4. **BFS**: Breadth-first search from optimal start vertex
5. **DFS**: Depth-first search from optimal start vertex  
6. **SFC-Hilbert2D**: Space-filling curve (Hilbert)
7. **SFC-ZOrder2D**: Space-filling curve (Z-order)
8. **Random**: Random permutation baseline

#### Advanced Algorithms (Require Terminal File)
9. **Dirichlet**: Harmonic embedding with fixed I/O constraints
10. **Soft-Penalty**: Penalty method for soft I/O anchoring
11. **Soft-Springs**: Virtual springs method for soft I/O anchoring

### Missing Algorithms in Current SA1D Implementation

The current `BestOrderings.cpp` is missing several algorithms from the SAIT main.cpp:

#### Missing Core Algorithms
- **SFC-Hilbert2D**: Space-filling curve orderings
- **SFC-ZOrder2D**: Space-filling curve orderings  

#### Missing Advanced Algorithms
- **Soft-Penalty**: Soft I/O anchoring with penalty method
- **Soft-Springs**: Soft I/O anchoring with virtual springs

#### Missing Infrastructure
- **I/O Terminal Integration**: Modeling bTerms as special vertices in hypergraph
- **Refinement Modes**: IO-Anchored Constrained vs Standard refinement
- **Parallel Best-Orderings**: The main.cpp uses parallel execution to test all algorithms

## Implementation Plan

### Phase 1: Complete Algorithm Integration ✅ (Partially Done)

#### 1.1 Add Missing Core Algorithms
**File**: `src/sa1d/src/BestOrderings.cpp`
**Method**: `getAvailableAlgorithms()`

Add missing algorithms to the algorithm list:
```cpp
// Add to getAvailableAlgorithms()
algorithms.push_back({"SFC-Hilbert2D", 
    [](const Hypergraph& hg) { return computeSFCOrdering(hg, SFCStrategy::HILBERT_2D); }});
algorithms.push_back({"SFC-ZOrder2D", 
    [](const Hypergraph& hg) { return computeSFCOrdering(hg, SFCStrategy::ZORDER_2D); }});
```

#### 1.2 Add I/O Terminal Detection from ODB
**Files**: `src/sa1d/src/BestOrderings.cpp`

```cpp
bool BestOrderingsInterface::hasIOTerminals() {
    const auto& nets = opt_sa_->getNets();
    for (const auto& net : nets) {
        if (net.bterm_flag) {
            return true;
        }
    }
    return false;
}

std::vector<FixedVertex> BestOrderingsInterface::extractIOCoordinates() {
    std::vector<FixedVertex> fixed_vertices;
    const auto& nets = opt_sa_->getNets();
    
    int io_vertex_id = opt_sa_->getCells().size(); // Start after regular cells
    for (size_t net_id = 0; net_id < nets.size(); ++net_id) {
        const auto& net = nets[net_id];
        if (net.bterm_flag) {
            FixedVertex fv;
            fv.vertex_id = io_vertex_id++;
            fv.x_coord = (net.bterm_box.xMin() + net.bterm_box.xMax()) / 2.0;
            fv.y_coord = (net.bterm_box.yMin() + net.bterm_box.yMax()) / 2.0;
            fixed_vertices.push_back(fv);
        }
    }
    return fixed_vertices;
}
```

#### 1.3 Add Advanced I/O-Aware Algorithms
**Prerequisites**: I/O terminal detection from ODB bTerms

```cpp
// Add to getAvailableAlgorithms() when I/O terminals exist
if (hasIOTerminals()) {
    algorithms.push_back({"Soft-Penalty", [&](const Hypergraph& hg) {
        auto fixed_vertices = extractIOCoordinates();
        return computeSoftAnchoredOrdering(hg, fixed_vertices, PENALTY_METHOD);
    }});
    
    algorithms.push_back({"Soft-Springs", [&](const Hypergraph& hg) {
        auto fixed_vertices = extractIOCoordinates();
        return computeSoftAnchoredOrdering(hg, fixed_vertices, VIRTUAL_SPRINGS);
    }});
}
```

### Phase 2: Enhanced Hypergraph Generation 

#### 2.1 I/O Terminal Integration
**Enhancement**: Model I/O terminals (bTerms) as special vertices in the hypergraph

**Current Issue**: I/O terminals are properly detected via `Net::updateBTerm()` but not modeled as separate vertices in the hypergraph. This is crucial for:
- Dirichlet embedding (requires fixed boundary conditions)
- Soft-anchored methods (requires I/O coordinate constraints)

**Solution**: 
```cpp
std::unique_ptr<Hypergraph> BestOrderingsInterface::convertToSAITHypergraph() {
    const auto& cells = opt_sa_->getCells();
    const auto& nets = opt_sa_->getNets();
    
    // Count I/O terminals (nets with bterm_flag = true)
    int num_io_terminals = 0;
    for (const auto& net : nets) {
        if (net.bterm_flag) {
            num_io_terminals++;
        }
    }
    
    // Create hypergraph: vertices = cells + I/O terminals
    int total_vertices = cells.size() + num_io_terminals;
    auto hg = std::make_unique<Hypergraph>(total_vertices, nets.size());
    
    // Map cells to vertices [0, cells.size())
    for (size_t cell_id = 0; cell_id < cells.size(); ++cell_id) {
        cell_to_vertex_map_[cell_id] = static_cast<int>(cell_id);
        vertex_to_cell_map_[static_cast<int>(cell_id)] = static_cast<int>(cell_id);
    }
    
    // Map I/O terminals to vertices [cells.size(), total_vertices)
    int io_vertex_id = cells.size();
    std::unordered_map<size_t, int> io_net_to_vertex_map;
    
    for (size_t net_id = 0; net_id < nets.size(); ++net_id) {
        const auto& net = nets[net_id];
        std::vector<int> hyperedge_vertices;
        
        // Add cells connected to this net
        for (const auto& term : net.getTerms()) {
            hyperedge_vertices.push_back(cell_to_vertex_map_[term.cell_id]);
        }
        
        // Add I/O terminal vertex if this net has bTerms
        if (net.bterm_flag) {
            if (io_net_to_vertex_map.find(net_id) == io_net_to_vertex_map.end()) {
                io_net_to_vertex_map[net_id] = io_vertex_id++;
            }
            hyperedge_vertices.push_back(io_net_to_vertex_map[net_id]);
        }
        
        if (hyperedge_vertices.size() >= 2) {
            hg->addHyperedge(hyperedge_vertices);
        }
    }
    
    hg->buildVertexToHyperedgeMapping();
    return hg;
}
```

#### 2.2 Physical Coordinate Integration
**Enhancement**: Use actual physical coordinates from ODB bTerms for coordinate-aware algorithms

The SA1D framework already extracts bTerm coordinates via `Net::updateBTerm()`. This information can be directly used:

```cpp
struct FixedVertex {
    int vertex_id;
    double x_coord;
    double y_coord;
};

std::vector<FixedVertex> BestOrderingsInterface::extractIOCoordinates() {
    std::vector<FixedVertex> fixed_vertices;
    const auto& nets = opt_sa_->getNets();
    
    int io_vertex_id = opt_sa_->getCells().size(); // Start after regular cells
    for (size_t net_id = 0; net_id < nets.size(); ++net_id) {
        const auto& net = nets[net_id];
        if (net.bterm_flag) {
            FixedVertex fv;
            fv.vertex_id = io_vertex_id++;
            // Use the center of the bterm_box computed by updateBTerm()
            fv.x_coord = (net.bterm_box.xMin() + net.bterm_box.xMax()) / 2.0;
            fv.y_coord = (net.bterm_box.yMin() + net.bterm_box.yMax()) / 2.0;
            fixed_vertices.push_back(fv);
        }
    }
    
    return fixed_vertices;
}
```

**Key Insight**: No external terminal file parsing is needed. The ODB database provides all necessary bTerm coordinate information through the existing SA1D infrastructure.

### Phase 3: Parallel Best-Orderings Implementation

#### 3.1 Parallel Algorithm Execution
**File**: `src/sa1d/src/BestOrderings.cpp`
**Method**: `computeBestOrderings()`

**Current Implementation**: Sequential algorithm testing
**Target**: Parallel execution like SAIT main.cpp (lines 1041-1106)

```cpp
BestOrderingsResult BestOrderingsInterface::computeBestOrderings(const BestOrderingsParams& params) {
    // ... existing setup code ...
    
    // Parallel algorithm execution
    std::vector<std::tuple<std::string, std::vector<int>, double, int, int>> algorithm_results(algorithms.size());
    
    #pragma omp parallel for schedule(dynamic)
    for (size_t i = 0; i < algorithms.size(); ++i) {
        const auto& [name, compute_func, requires_terminal] = algorithms[i];
        
        auto start_time = std::chrono::high_resolution_clock::now();
        
        try {
            // Compute base ordering
            auto vertex_ordering = compute_func(*hg);
            
            // Compute initial cutwidth
            auto initial_result = computeCutwidthCurve(*hg, vertex_ordering, false);
            int initial_peak = initial_result.peak_cutwidth;
            
            // Apply refinement
            auto refined_ordering = vertex_ordering;
            if (params.apply_refinement) {
                refined_ordering = applyRefinement(*hg, vertex_ordering, name, params.use_constrained_refinement);
            }
            
            // Compute final cutwidth
            auto final_result = computeCutwidthCurve(*hg, refined_ordering, false);
            int final_peak = final_result.peak_cutwidth;
            
            auto end_time = std::chrono::high_resolution_clock::now();
            double time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            
            algorithm_results[i] = std::make_tuple(name, refined_ordering, time_ms, initial_peak, final_peak);
            
        } catch (const std::exception& e) {
            algorithm_results[i] = std::make_tuple(name, std::vector<int>(), 0.0, INT_MAX, INT_MAX);
        }
    }
    
    // Sort by final peak cutwidth and select top orderings
    // ... existing result processing code ...
}
```

### Phase 4: Worker Initialization Enhancement

#### 4.1 Distributed Best Orderings
**File**: `src/sa1d/src/OptSA.cpp`
**Method**: `runSA()` (lines 579-595)

**Current**: Workers initialized with single best ordering or random
**Target**: Distribute top 3 orderings across all workers for diversity

```cpp
// Enhanced worker initialization (in runSA())
if (use_best_orderings_ && best_orderings_result.success) {
    auto worker_init_data = best_orderings_->prepareWorkerInitialization(best_orderings_result, num_workers_);
    
    for (int worker_id = 0; worker_id < num_workers_; worker_id++) {
        // ... existing worker setup code ...
        
        if (worker_id < static_cast<int>(worker_init_data.size()) && worker_init_data[worker_id].use_ordering) {
            // Initialize with specific best ordering
            const auto& init_data = worker_init_data[worker_id];
            worker->initCellOrder(init_data.cell_ordering, init_data.orientations);
            
            logger_->info(utl::SA1D, 218, "Worker {} initialized with {} ordering (peak cutwidth: {})", 
                         worker_id, init_data.algorithm_name, init_data.peak_cutwidth);
        } else {
            // Fall back to random initialization
            worker->initCellOrderRandom();
        }
    }
}
```

#### 4.2 Worker Distribution Strategy
**Method**: `prepareWorkerInitialization()`

```cpp
std::vector<WorkerInitData> BestOrderingsInterface::prepareWorkerInitialization(
    const BestOrderingsResult& result, int num_workers) {
    
    std::vector<WorkerInitData> worker_data(num_workers);
    
    if (result.top_orderings.empty()) {
        // All workers use random initialization
        for (auto& data : worker_data) {
            data.use_ordering = false;
        }
        return worker_data;
    }
    
    // Distribute best orderings across workers
    int orderings_count = result.top_orderings.size();
    int workers_per_ordering = num_workers / orderings_count;
    int remaining_workers = num_workers % orderings_count;
    
    int worker_idx = 0;
    for (int ord_idx = 0; ord_idx < orderings_count; ++ord_idx) {
        const auto& ordering_info = result.top_orderings[ord_idx];
        
        int workers_for_this_ordering = workers_per_ordering;
        if (ord_idx < remaining_workers) {
            workers_for_this_ordering++;
        }
        
        for (int i = 0; i < workers_for_this_ordering && worker_idx < num_workers; ++i, ++worker_idx) {
            worker_data[worker_idx].cell_ordering = ordering_info.cell_ordering;
            worker_data[worker_idx].orientations = ordering_info.orientations;
            worker_data[worker_idx].algorithm_name = ordering_info.algorithm_name;
            worker_data[worker_idx].use_ordering = true;
        }
    }
    
    return worker_data;
}
```

## Quality Metrics and Validation

### 1. Cutwidth Analysis
- **Peak Cutwidth**: Maximum number of nets crossing any cut in the ordering
- **Cutwidth Curve**: Profile of net crossings across all cuts
- **Improvement Percentage**: Reduction in peak cutwidth after refinement

### 2. HPWL (Half-Perimeter Wire Length)  
- **Initial HPWL**: Wire length with current cell positions
- **Projected HPWL**: Wire length after applying vertex ordering
- **HPWL Improvement**: Reduction in total wire length

### 3. SA Convergence Quality
- **Best Final Cost**: Lowest cost achieved by any SA worker
- **Convergence Speed**: Number of iterations to reach target quality
- **Worker Diversity**: Spread of final costs across workers

## Testing and Validation Plan

### Unit Tests
1. **Hypergraph Conversion**: Verify ODB → SAIT conversion correctness
2. **Algorithm Execution**: Test each SAIT algorithm individually  
3. **Worker Initialization**: Verify proper distribution of orderings

### Integration Tests
1. **End-to-End**: Complete SA1D run with vertex ordering initialization
2. **Quality Comparison**: Compare against random initialization baseline
3. **Performance**: Measure computation time overhead

### Benchmark Designs
1. **Simple Designs**: Small circuits for correctness verification
2. **Industrial Designs**: Real-world circuits for quality assessment
3. **Pathological Cases**: Designs that challenge specific algorithms

## Expected Benefits

### Quality Improvements
- **Better Initial Solutions**: Replace random placement with optimized orderings
- **Faster Convergence**: Start SA closer to optimal solutions
- **Improved Final Quality**: Better final placement results

### Algorithmic Diversity
- **Multiple Strategies**: 8-11 different ordering algorithms
- **Worker Specialization**: Different workers start with different orderings
- **Robust Performance**: Reduced dependence on random initialization

### Computational Efficiency
- **Parallel Execution**: Compute multiple orderings simultaneously
- **Early Convergence**: Reduced SA iterations needed
- **Quality-Time Trade-off**: Invest upfront computation for better results

## Implementation Status

### ✅ Completed
- Basic vertex ordering interface (`VertexOrdering.cpp`)
- Best orderings framework (`BestOrderings.cpp`)
- SAIT algorithm wrappers (`src/sait/`)
- Hypergraph conversion from ODB
- Worker initialization infrastructure
- **Enhanced hypergraph generation with bTerms as vertices (`BestOrderings.cpp`)**
- **I/O terminal coordinate extraction from ODB bTerms**
- **FixedVertex structure for I/O constraints**

### 🔄 In Progress
- Update VertexOrdering to include bTerms as vertices
- Missing algorithm implementations (SFC, Soft-Anchored)
- Parallel best-orderings execution

### 📋 TODO
- Advanced refinement methods (IO-Anchored Constrained)
- Comprehensive testing and validation
- Performance optimization
- Documentation and examples

## Risk Assessment

### Technical Risks
- **Algorithm Failures**: Some SAIT algorithms may fail on certain hypergraphs
- **Memory Usage**: Large hypergraphs may consume significant memory
- **Computation Time**: Vertex ordering computation may be expensive

### Mitigation Strategies
- **Graceful Fallback**: Always provide fallback to simpler methods
- **Resource Limits**: Set timeouts and memory limits for algorithms
- **Incremental Implementation**: Add algorithms progressively

## Conclusion

The SA1D vertex ordering integration leverages the sophisticated SAIT hypergraph algorithms to provide high-quality initial placements for the simulated annealing optimization. The implementation builds on existing infrastructure while adding the comprehensive "best-orderings" approach from the SAIT framework.

The key innovation is replacing random initial placement with optimized vertex orderings, distributed across multiple SA workers to maintain diversity while improving overall solution quality.