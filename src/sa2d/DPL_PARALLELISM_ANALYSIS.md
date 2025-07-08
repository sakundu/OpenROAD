# DPL Data Structures Parallelism Analysis for SA2D

## Executive Summary

The DPL data structures have significant limitations for direct parallel usage in a go-with-the-winners (GWTW) SA implementation. Most core structures are designed for single-threaded access and contain shared mutable state. This analysis identifies which structures can be reused, which need modifications, and proposes solutions for parallel SA.

## Core Data Structures Analysis

### 1. Node (Cell) - **NOT Thread-Safe**

**Structure:**
```cpp
class Node {
    DbuX left_{0};        // Mutable position
    DbuY bottom_{0};      // Mutable position
    dbOrientType orient_; // Mutable orientation
    bool placed_{false};  // Mutable state
    // ... other mutable fields
};
```

**Issues:**
- All position and state fields are mutable
- No synchronization mechanisms
- Direct pointer access throughout codebase

**Parallel Limitations:**
- Multiple threads cannot modify different nodes' positions simultaneously
- Race conditions on position updates
- No copy-on-write semantics

### 2. Edge (Net) - **Partially Thread-Safe for Read**

**Structure:**
```cpp
class Edge {
    int id_{0};
    std::vector<Pin*> pins_;  // Mostly read-only after initialization
};
```

**Analysis:**
- HPWL calculation is read-only operation
- Pin connectivity rarely changes during detailed placement
- Can be shared for read access

**Parallel Potential:**
- Safe for concurrent HPWL calculations
- Requires synchronization if pins are modified

### 3. Grid & Pixel - **NOT Thread-Safe**

**Structure:**
```cpp
struct Pixel {
    Node* cell = nullptr;     // Mutable occupancy
    Group* group = nullptr;   // Mutable
    double util = 0.0;        // Mutable
    // ...
};

class Grid {
    std::vector<std::vector<Pixel>> pixels_;  // 2D mutable grid
};
```

**Critical Issues:**
- Grid is the central placement data structure
- Every cell movement updates multiple pixels
- No locking or atomic operations
- Painting/erasing cells modifies shared state

**Parallel Limitations:**
- Cannot have multiple threads updating grid simultaneously
- Overlap checking requires consistent grid state
- Legal position finding needs synchronized access

### 4. Network - **Read-Only After Construction**

**Structure:**
```cpp
class Network {
    std::vector<std::unique_ptr<Node>> nodes_;
    std::vector<std::unique_ptr<Edge>> edges_;
    std::vector<std::unique_ptr<Pin>> pins_;
    // Hash maps for lookups
};
```

**Analysis:**
- Topology is fixed during detailed placement
- Only Node positions change, not connectivity
- Can be shared across threads for read access

### 5. Architecture - **Thread-Safe for Read**

**Structure:**
```cpp
class Architecture {
    std::vector<Row*> rows_;
    std::vector<Group*> regions_;
    // Row information - static
};
```

**Analysis:**
- Row structure doesn't change
- Safe for concurrent read access
- Power compatibility checks are read-only

### 6. DetailedMgr - **NOT Thread-Safe**

**Structure:**
```cpp
class DetailedMgr {
    Journal journal;                           // Mutable state tracking
    std::vector<std::vector<Node*>> cellsInSeg_; // Mutable segments
    // Move generation and evaluation state
};
```

**Issues:**
- Centralized move management
- Journal tracks all changes
- Segment assignments are mutable

### 7. Journal - **Designed for Single-Thread**

**Structure:**
```cpp
class Journal {
    std::vector<std::unique_ptr<JournalAction>> actions_;
    std::set<Node*> affected_nodes_;
    std::set<Edge*> affected_edges_;
};
```

**Issues:**
- Sequential action tracking
- Cannot merge parallel journals easily
- Undo/redo assumes single timeline

## Parallelism Classification

### Thread-Safe for Read (Can Share)
1. **Network** - After initial construction
2. **Edge** - For HPWL calculation
3. **Pin** - Connectivity information
4. **Master** - Cell type information
5. **Architecture** - Row structure

### NOT Thread-Safe (Cannot Share)
1. **Node** - Position and state changes
2. **Grid/Pixel** - Occupancy changes
3. **DetailedMgr** - Move generation state
4. **Journal** - Change tracking
5. **Group** - Cell assignments

### Requires Synchronization
1. **Padding** - If dynamically modified
2. **PlacementDRC** - If spacing rules change

## Parallel SA Implementation Strategies

### Strategy 1: Copy-on-Write Workers (Recommended)

Each SA worker maintains:
```cpp
class SAWorker {
    // Private copies
    std::vector<CellPosition> positions_;    // Cell locations
    std::unique_ptr<Grid> private_grid_;     // Worker's grid copy
    std::vector<int> cell_to_segment_;       // Segment assignments
    
    // Shared read-only
    const Network* shared_network_;          // Topology
    const Architecture* shared_arch_;        // Rows
    const PlacementDRC* shared_drc_;        // Rules
};
```

**Advantages:**
- No synchronization needed during SA iterations
- Each worker has consistent view
- Can evaluate moves independently

**Disadvantages:**
- Memory overhead (Grid copies)
- Need to sync best solutions periodically

### Strategy 2: Lightweight Position Tracking

Instead of full Node copies:
```cpp
struct LightweightCell {
    DbuX x;
    DbuY y;
    dbOrientType orient;
    int segment_id;
};

class SAWorker {
    std::vector<LightweightCell> cells_;
    // Lazy grid evaluation
};
```

**Advantages:**
- Much lower memory overhead
- Faster copying between workers
- Sufficient for HPWL calculation

**Disadvantages:**
- Need to rebuild grid for legality checks
- May miss some DRC violations during SA

### Strategy 3: Shared Grid with Locking (Not Recommended)

Add synchronization to Grid:
```cpp
class ThreadSafeGrid {
    mutable std::shared_mutex grid_mutex_;
    // Fine-grained pixel locks
};
```

**Issues:**
- High contention on popular regions
- Deadlock potential
- Performance degradation

## Recommended Approach for SA2D v0

For the GWTW parallel SA implementation:

1. **Shared Read-Only Structures:**
   - Network (topology)
   - Architecture (rows)
   - Master information
   - DRC rules

2. **Per-Worker Copies:**
   - Cell positions (lightweight)
   - Segment assignments
   - HPWL cache

3. **Lazy Grid Evaluation:**
   - Don't maintain full grid per worker
   - Evaluate legality only when needed
   - Use shared grid for final legalization

4. **Periodic Synchronization:**
   - Share best solutions between workers
   - Update worker states based on winners
   - Minimize synchronization frequency

## Implementation Recommendations

### Phase 1: Lightweight Workers
```cpp
namespace sa2d {

struct WorkerState {
    std::vector<DbuX> cell_x;
    std::vector<DbuY> cell_y;
    std::vector<dbOrientType> cell_orient;
    std::vector<uint64_t> net_hpwl_cache;
};

class ParallelSAWorker {
    WorkerState state_;
    const dpl::Network* network_;  // Shared
    
    // Evaluate moves without Grid
    int64_t evaluateMove(int cell_id, DbuX new_x, DbuY new_y);
    
    // Periodic legality check
    bool checkLegalityLazy(int cell_id, DbuX x, DbuY y);
};

}
```

### Phase 2: Efficient State Copying
```cpp
// Fast state copy for GWTW
void copyWinnerState(const WorkerState& winner, WorkerState& target) {
    // Use memcpy for POD arrays
    std::memcpy(target.cell_x.data(), winner.cell_x.data(), 
                sizeof(DbuX) * num_cells);
    // ...
}
```

### Phase 3: Final Legalization
```cpp
// Apply best solution to DPL structures
void applyBestSolution(const WorkerState& best,
                       dpl::DetailedMgr* mgr) {
    // Single-threaded update
    for (int i = 0; i < num_cells; ++i) {
        mgr->tryMove(cells[i], 
                     current_x[i], current_y[i], current_seg[i],
                     best.cell_x[i], best.cell_y[i], new_seg[i]);
    }
}
```

## Memory Overhead Estimation

For N cells, K workers:
- Position storage: K × N × 12 bytes (x, y, orient)
- HPWL cache: K × num_nets × 8 bytes
- Total: ~K × N × 20 bytes

For 1M cells, 20 workers: ~400 MB additional memory

## Conclusions

1. **Direct reuse is limited** - Most DPL structures assume single-threaded access
2. **Lightweight copying is feasible** - Position-only copies have acceptable overhead
3. **Shared read-only is safe** - Network topology and architecture can be shared
4. **Grid is the bottleneck** - Need alternative legality checking for parallel SA
5. **GWTW is implementable** - With proper state management and periodic sync

## Next Steps

1. Implement lightweight worker state structure
2. Create efficient state copying mechanisms
3. Develop lazy legality checking
4. Design synchronization protocol for GWTW
5. Benchmark memory and performance impact 