# SA2D v0 Implementation Plan - Simple Simulated Annealing 2D Placer

## Overview

Version 0 of SA2D focuses on implementing a basic simulated annealing placer that:
- Reuses existing DPL infrastructure (not copying, but directly using)
- Implements basic moves (single cell move, swap, **and flip**)
- **Simplified legalization**: Only overlap, row alignment, site alignment, and site orientation checks
- Single objective: HPWL minimization
- Follows sa1d implementation pattern

**Important Update**: Based on parallelism analysis (see [DPL_PARALLELISM_ANALYSIS.md](DPL_PARALLELISM_ANALYSIS.md)) and accuracy concerns, v0 will use a thread-safe Grid implementation (see [THREAD_SAFE_GRID_DESIGN.md](THREAD_SAFE_GRID_DESIGN.md)) for accurate legality checking.

## Architecture Following sa1d Pattern

### Core Classes (Following sa1d Structure)

#### 1. Main SA2D Class (`include/sa2d/SA2D.h`)
```cpp
namespace sa2d {

class SA2D {
public:
    SA2D();
    ~SA2D();
    
    void init(odb::dbDatabase* db, utl::Logger* logger);
    void runSA();
    
    // SA Parameters (following sa1d pattern)
    void setNumWorkers(int num_workers);
    void setNumThreads(int num_threads);
    void setMaxTemp(float max_temp);
    void setMinTemp(float min_temp);
    void setCoolingRate(float cooling_rate);
    void setMaxIter(int max_iter);
    void setMoveBudget(int move_budget);
    void setSeed(int seed);
    
    // Displacement parameters (following DPL pattern)
    void setMaxDisplacement(int max_displacement_x, int max_displacement_y);
    
    // DPL integration
    void setDplEngine(dpl::Opendp* dpl) { dpl_ = dpl; }
    
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
    
    // SA parameters
    int num_workers_ = 1;  // v0: single worker
    float max_temp_ = 100.0;
    float min_temp_ = 1e-6;
    float cooling_rate_ = 0.95;
    int max_iter_ = 1000;
    int move_budget_ = 100000;
    int seed_ = 0;
    
    // Max displacement (in grid sites, following DPL)
    int max_displacement_x_ = 500;  // Default from DPL
    int max_displacement_y_ = 100;  // Default from DPL
};

}  // namespace sa2d
```

#### 2. Lightweight Cell State (`src/CellState.h`)
```cpp
namespace sa2d {

// Lightweight cell state for parallel-ready implementation
struct CellState {
    DbuX x;
    DbuY y;
    odb::dbOrientType orient;
    
    // For tracking changes
    DbuX prev_x;
    DbuY prev_y;
    odb::dbOrientType prev_orient;
};

// Complete worker state
struct WorkerState {
    std::vector<CellState> cells;
    std::vector<uint64_t> net_hpwl_cache;
    double total_hpwl;
};

}  // namespace sa2d
```

#### 3. Thread-Safe Grid (`src/ThreadSafeGrid.h`)
```cpp
namespace sa2d {

// Immutable grid information (shared across all workers)
class ImmutableGridInfo {
public:
    void initFromDPL(const dpl::Grid* dpl_grid, 
                     const dpl::Architecture* arch,
                     odb::dbBlock* block);
    
    // All conversion functions are const and thread-safe
    GridX gridX(DbuX x) const;
    GridY gridSnapDownY(DbuY y) const;
    GridX gridPaddedWidth(const dpl::Node* cell) const;
    GridY gridHeight(const dpl::Node* cell) const;
    // ... other conversion functions
    
    // Basic grid properties
    GridY getRowCount() const { return row_count_; }
    GridX getRowSiteCount() const { return row_site_count_; }
    DbuX getSiteWidth() const { return site_width_; }
    DbuY gridYToDbu(GridY y) const;
    
    // Site information access
    const std::map<dbSite*, dbOrientType>& getSitesAt(GridX x, GridY y) const;
    dbOrientType getValidOrientation(dbSite* site, GridX x, GridY y) const;
    
private:
    // Immutable after initialization
    Rect core_;
    DbuX site_width_;
    GridY row_count_;
    GridX row_site_count_;
    std::vector<DbuY> row_index_to_y_dbu_;
    std::optional<DbuY> uniform_row_height_;
    
    // Site information for each pixel
    struct PixelSiteInfo {
        std::map<dbSite*, dbOrientType> sites;  // Available sites and their orientations
    };
    std::vector<std::vector<PixelSiteInfo>> pixel_sites_;  // [y][x]
};

// Mutable grid state per worker
class WorkerGrid {
public:
    WorkerGrid(const ImmutableGridInfo* info);
    
    // Check occupancy
    bool isOccupied(GridX x, GridY y) const;
    int getCellAt(GridX x, GridY y) const;
    
    // Modifications
    void placeCell(int cell_id, GridX x, GridY y, GridX width, GridY height);
    void removeCell(int cell_id);
    void clear();
    
    // Efficient copying
    void copyFrom(const WorkerGrid& other);
    
private:
    const ImmutableGridInfo* info_;
    std::vector<std::vector<int>> pixels_;  // cell_id or -1
    std::unordered_map<int, GridRect> cell_locations_;
};

}  // namespace sa2d
```

#### 4. SA Worker (`src/Worker.h`)
```cpp
namespace sa2d {

class SAWorker {
public:
    SAWorker(SA2D* sa2d, int worker_id);
    
    void setTemp(float temp);
    void setCoolingRate(float cooling_rate);
    void setMaxIter(int max_iter);
    void setMaxDisplacement(int max_x, int max_y);
    
    void run();
    int64_t getTotalHPWL();
    
    // Initialize from DPL current state
    void initFromDPL(const dpl::Network* network,
                     const ImmutableGridInfo* grid_info);
    
    // Apply best solution back to DPL
    void applyToDPL(dpl::DetailedMgr* mgr);
    
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
    const dpl::Network* network_;
    const dpl::Architecture* arch_;
    const ImmutableGridInfo* grid_info_;
    
    float temp_;
    float cooling_rate_;
    int max_iter_;
    
    // Max displacement in grid sites (from SA2D)
    int max_displacement_x_;
    int max_displacement_y_;
    
    // Move operations with full legality
    bool tryMove(int cell_id);
    bool trySwap(int cell1_id, int cell2_id);
    bool tryFlip(int cell_id);  // Flip cell orientation for HPWL improvement
    
    // Legality checking using WorkerGrid
    bool canPlaceCell(int cell_id, GridX x, GridY y);
    
    // Get the correct orientation for a position
    dbOrientType getCellOrientation(int cell_id, GridX x, GridY y);
    
    // Cost evaluation
    int64_t calcDeltaHPWL(const std::vector<int>& affected_nets);
    
    // SA acceptance
    bool acceptMove(int64_t delta_cost, float temp);
    
    // Generate random position within displacement limits
    GridPt generateRandomPosition(int cell_id);
    
    std::mt19937 rng_;
    std::uniform_real_distribution<float> distribution_;
};

}  // namespace sa2d
```

## Key Design Decisions for v0

### 1. Maximum Displacement Handling (Following DPL)

**Key Points**:
- Max displacement is specified in **sites** (not microns or DBU)
- Default values: 500 sites horizontal, 100 sites vertical (from DPL)
- Used to limit cell movement during optimization
- Respects group boundaries if cell belongs to a placement group
- Converted from user input (microns) to sites in TCL interface

**TCL Interface** (following DPL pattern):
```tcl
# Basic usage
sa2d_simple_place \
    -max_displacement 50 \
    -max_temp 10.0 \
    -cooling_rate 0.95 \
    -max_iter 100 \
    -seed 42

# Disable chain moves for faster runtime
sa2d_set_enable_chain_moves 0
sa2d_simple_place \
    -max_displacement 50 \
    -max_temp 10.0

# Or reduce chain move frequency
sa2d_set_chain_move_interval 100  # Every 100 iterations instead of 50
sa2d_set_chain_moves_per_round 3  # Only 3 chain moves per round
sa2d_simple_place \
    -max_displacement 50 \
    -max_temp 10.0
```

**Conversion in TCL**:
```tcl
if { [info exists keys(-max_displacement)] } {
    set max_displacement $keys(-max_displacement)
    if { [llength $max_displacement] == 1 } {
        set max_displacement_x $max_displacement
        set max_displacement_y $max_displacement
    } elseif { [llength $max_displacement] == 2 } {
        lassign $max_displacement max_displacement_x max_displacement_y
    }
    # Convert from microns to sites
    set site [dpl::get_row_site]
    set max_displacement_x [expr [ord::microns_to_dbu $max_displacement_x] / [$site getWidth]]
    set max_displacement_y [expr [ord::microns_to_dbu $max_displacement_y] / [$site getHeight]]
} else {
    # Use DPL defaults
    set max_displacement_x 500
    set max_displacement_y 100
}
```

### 2. Thread-Safe Grid Implementation

**Why**: Lazy legality checking can miss critical violations

**Approach**:
- Separate immutable grid info (shared) from mutable state (per-worker)
- Each worker maintains its own grid occupancy
- Full accuracy in legality checking

```cpp
void SAWorker::initFromDPL(const dpl::Network* network,
                           const ImmutableGridInfo* grid_info) {
    network_ = network;
    grid_info_ = grid_info;
    
    // Initialize cell states
    state_.cells.resize(network->getNumCells());
    
    // Initialize worker grid
    grid_ = std::make_unique<WorkerGrid>(grid_info_);
    
    // Copy current placement
    for (int i = 0; i < network->getNumNodes(); i++) {
        const dpl::Node* node = network->getNode(i);
        if (node->getType() == dpl::Node::CELL && node->isPlaced()) {
            state_.cells[i].x = node->getLeft();
            state_.cells[i].y = node->getBottom();
            state_.cells[i].orient = node->getOrient();
            
            // Place in grid
            GridX gx = grid_info_->gridX(node->getLeft());
            GridY gy = grid_info_->gridSnapDownY(node->getBottom());
            GridX gw = grid_info_->gridPaddedWidth(node);
            GridY gh = grid_info_->gridHeight(node);
            
            grid_->placeCell(i, gx, gy, gw, gh);
        }
    }
}
```

### 2. Move Operations with Full Legality

**Move Type Distribution:**
- 70% single cell moves
- 15% cell swaps
- 15% cell flips

Chain/ripple moves are now separate periodic operations (not part of regular move mix).

**Chain Move Strategy (Periodic Operation):**
- Executed periodically every N iterations (default: 50)
- Only performed at temperatures > 1.0
- Performs M chain moves per round (default: 5)
- Can be disabled via `sa2d_set_enable_chain_moves 0`
- Interval controlled by `sa2d_set_chain_move_interval N`
- Moves per round controlled by `sa2d_set_chain_moves_per_round M`

**Chain Move Optimizations:**
- Not part of regular move distribution (huge performance gain)
- Periodic execution reduces overhead dramatically
- Skip entirely at temperatures < 1.0
- Proper grid validation ensures no overlaps
- Early termination if target position is free

**Single Cell Move with Displacement Limits:**
```cpp
bool SAWorker::tryMove(int cell_id) {
    const dpl::Node* node = network_->getNode(cell_id);
    
    // Generate new position within displacement limits
    GridPt new_pos = generateRandomPosition(cell_id);
    GridX grid_x = new_pos.x;
    GridY grid_y = new_pos.y;
    
    // Full legality check with grid (including site compatibility)
    if (!canPlaceCell(cell_id, grid_x, grid_y)) {
        return false;
    }
    
    // Get the correct orientation for this position
    dbOrientType new_orient = getCellOrientation(cell_id, grid_x, grid_y);
    
    // Calculate cost change
    std::vector<int> affected_nets = getAffectedNets(cell_id);
    int64_t delta = calcDeltaHPWL(affected_nets);
    
    if (acceptMove(delta, temp_)) {
        // Update grid occupancy
        grid_->removeCell(cell_id);
        GridX gw = grid_info_->gridPaddedWidth(node);
        GridY gh = grid_info_->gridHeight(node);
        grid_->placeCell(cell_id, grid_x, grid_y, gw, gh);
        
        // Update position (snapped to grid)
        state_.cells[cell_id].x = gridToDbu(grid_x, grid_info_->getSiteWidth());
        state_.cells[cell_id].y = grid_info_->gridYToDbu(grid_y);
        state_.cells[cell_id].orient = new_orient;  // Update orientation
        
        updateHPWLCache(affected_nets);
        return true;
    }
    return false;
}

GridPt SAWorker::generateRandomPosition(int cell_id) {
    const dpl::Node* node = network_->getNode(cell_id);
    
    // Current position in grid coordinates
    GridX curr_x = grid_info_->gridX(state_.cells[cell_id].x);
    GridY curr_y = grid_info_->gridSnapDownY(state_.cells[cell_id].y);
    
    // Generate random displacement within limits
    std::uniform_int_distribution<int> dist_x(-max_displacement_x_, max_displacement_x_);
    std::uniform_int_distribution<int> dist_y(-max_displacement_y_, max_displacement_y_);
    
    int dx = dist_x(rng_);
    int dy = dist_y(rng_);
    
    // New position
    GridX new_x = curr_x + dx;
    GridY new_y = curr_y + dy;
    
    // Clip to grid bounds
    new_x = std::max(GridX{0}, new_x);
    new_y = std::max(GridY{0}, new_y);
    
    GridX width = grid_info_->gridPaddedWidth(node);
    GridY height = grid_info_->gridHeight(node);
    
    new_x = std::min(new_x, grid_info_->getRowSiteCount() - width);
    new_y = std::min(new_y, grid_info_->getRowCount() - height);
    
    // Restrict to group boundary if cell is in a group
    if (node->inGroup()) {
        const auto group_rect = grid_info_->gridWithin(node->getGroup()->getBBox());
        new_x = std::max(new_x, group_rect.xlo);
        new_y = std::max(new_y, group_rect.ylo);
        new_x = std::min(new_x, group_rect.xhi - width);
        new_y = std::min(new_y, group_rect.yhi - height);
    }
    
    return GridPt{new_x, new_y};
}
```

**Cell Flipping (Following DPL Pattern):**
```cpp
bool SAWorker::tryFlip(int cell_id) {
    const dpl::Node* node = network_->getNode(cell_id);
    
    // Only flip single-height cells
    if (!grid_info_->isSingleHeightCell(node)) {
        return false;
    }
    
    // Check if row supports Y symmetry
    GridY gy = grid_info_->gridSnapDownY(state_.cells[cell_id].y);
    if (!grid_info_->isRowYSymmetric(gy)) {
        return false;
    }
    
    // Calculate flipped orientation
    dbOrientType current = state_.cells[cell_id].orient;
    dbOrientType flipped;
    switch (current) {
        case dbOrientType::R0:   flipped = dbOrientType::MY; break;
        case dbOrientType::R180: flipped = dbOrientType::MX; break;
        case dbOrientType::MY:   flipped = dbOrientType::R0; break;
        case dbOrientType::MX:   flipped = dbOrientType::R180; break;
        default: return false;  // Can't flip other orientations
    }
    
    // Calculate HPWL change (only accept improvements)
    state_.cells[cell_id].orient = flipped;
    int64_t delta = calcDeltaHPWL(getAffectedNets(cell_id));
    
    if (delta < 0) {
        // Accept flip - improves HPWL
        updateHPWLCache(getAffectedNets(cell_id));
        state_.total_hpwl += delta;
        return true;
    } else {
        // Reject flip - restore orientation
        state_.cells[cell_id].orient = current;
        return false;
    }
}
```

**Chain/Ripple Moves (Following DPL Pattern):**
```cpp
bool SAWorker::tryChainMove(int cell_id) {
    // Generate target position
    GridPt target_pos = generateRandomPosition(cell_id);
    
    // If position is free, just do regular move
    if (!grid_->isOccupied(target_pos.x, target_pos.y)) {
        return tryMove(cell_id);
    }
    
    // Try ripple in both directions
    bool left_success = tryRippleLeft(cell_id, target_pos);
    bool right_success = tryRippleRight(cell_id, target_pos);
    
    return left_success || right_success;
}

bool SAWorker::tryRippleLeft(int cell_id, GridPt target_pos, int max_chain_length) {
    std::vector<ChainedMove> chain;
    
    // Build chain by shifting cells left
    GridX curr_x = target_pos.x;
    for (int i = 0; i < max_chain_length; ++i) {
        int blocking_cell = grid_->getCellAt(curr_x, target_pos.y);
        if (blocking_cell == -1) break;  // Found space
        
        if (network_->getNode(blocking_cell)->isFixed()) {
            return false;  // Can't move fixed cells
        }
        
        // Add to chain
        ChainedMove move;
        move.cell_id = blocking_cell;
        // ... calculate new position ...
        chain.push_back(move);
        
        curr_x = move.new_pos.x;  // Move to next position
    }
    
    // Validate entire chain (displacement limits, no overlaps)
    if (!validateChain(chain)) return false;
    
    // Calculate total HPWL change
    int64_t total_delta = calculateChainDelta(chain);
    
    // SA acceptance
    if (acceptMove(total_delta, temp_)) {
        executeChain(chain);
        return true;
    }
    
    return false;
}
```

Key features of chain moves:
- Adaptive chain length: 3 at high temp (>10°), 2 at low temp
- Bidirectional rippling (left and right)
- Simplified validation without temporary grid (performance optimization)
- Combined HPWL calculation for all affected cells
- Respects displacement limits for each cell in chain
- Handles multi-height cells appropriately
- Temperature-dependent usage (disabled below 1.0°)

### 3. Simplified Legality Checking (v0)

```cpp
bool SAWorker::canPlaceCell(int cell_id, GridX x, GridY y) {
    const dpl::Node* node = network_->getNode(cell_id);
    
    // 1. Check row alignment (y must be valid row)
    if (y < 0 || y >= grid_info_->getRowCount()) {
        return false;
    }
    
    // 2. Check site alignment (x must be valid site)
    GridX width = grid_info_->gridPaddedWidth(node);
    if (x < 0 || x + width > grid_info_->getRowSiteCount()) {
        return false;
    }
    
    // 3. Check site compatibility
    dbSite* cell_site = node->getSite();
    const auto& available_sites = grid_info_->getSitesAt(x, y);
    if (available_sites.find(cell_site) == available_sites.end()) {
        return false;  // Site type not available at this location
    }
    
    // 4. Check no overlaps
    GridY height = grid_info_->gridHeight(node);
    for (GridY yi = y; yi < y + height; yi++) {
        for (GridX xi = x; xi < x + width; xi++) {
            if (grid_->isOccupied(xi, yi)) {
                int other_id = grid_->getCellAt(xi, yi);
                if (other_id != cell_id) {  // Not self
                    return false;
                }
            }
        }
    }
    
    return true;
}

// Get the correct orientation for a cell at a position
dbOrientType SAWorker::getCellOrientation(int cell_id, GridX x, GridY y) {
    const dpl::Node* node = network_->getNode(cell_id);
    dbSite* cell_site = node->getSite();
    
    // Get the orientation from the pixel's site information
    return grid_info_->getValidOrientation(cell_site, x, y);
}
```

**Note**: v0 includes these essential checks:
- **Row alignment**: Cell must align with valid rows
- **Site alignment**: Cell must align with site grid
- **Site compatibility**: Cell's site type must be available at location
- **Site orientation**: Cell gets correct orientation based on row
- **No overlaps**: Cells cannot overlap

v0 deliberately omits these checks for simplicity:
- Blockage checking
- Region/group constraints  
- Multi-height row patterns
- Power rail compatibility
- Hopeless pixel checking

These can be added in future versions as needed.

### 4. Efficient State Management

```cpp
// Track best solution
void SAWorker::updateBestSolution() {
    if (state_.total_hpwl < best_state_.total_hpwl) {
        best_state_ = state_;
        best_grid_->copyFrom(*grid_);
    }
}

// For future GWTW
void SAWorker::copyStateFrom(const SAWorker& other) {
    state_ = other.state_;
    grid_->copyFrom(*other.grid_);
}
```

### 5. Final Legalization

After SA completes, apply best solution to DPL:
```cpp
void SAWorker::applyToDPL(dpl::DetailedMgr* mgr) {
    // Apply best solution
    for (int i = 0; i < network_->getNumNodes(); i++) {
        dpl::Node* node = network_->getNode(i);
        if (node->getType() == dpl::Node::CELL) {
            // Move to best position
            node->setLeft(best_state_.cells[i].x);
            node->setBottom(best_state_.cells[i].y);
            node->setOrient(best_state_.cells[i].orient);
        }
    }
    
    // DPL can run additional legalization if needed
}
```

## Implementation Steps

### Phase 1: Setup & Grid Infrastructure (Week 1)
- [x] Create directory structure following sa1d pattern
- [ ] Set up CMakeLists.txt to link with DPL
- [ ] Create SA2D main class with DPL integration
- [ ] **Implement ImmutableGridInfo class**
- [ ] **Implement WorkerGrid class**
- [ ] Create TCL interface with max displacement handling

### Phase 2: Basic SA Worker (Week 2)
- [ ] Implement SAWorker class with thread-safe grid
- [ ] Implement state initialization from DPL
- [ ] Implement move generation with displacement limits
- [ ] Implement legality checking with site orientation
- [x] Implement cell flipping for HPWL improvement (like DPL)
- [ ] Test grid operations
- [ ] Test basic moves

### Phase 3: SA Algorithm (Week 3)
- [ ] Implement temperature scheduling
- [ ] Implement Metropolis acceptance criterion
- [ ] Add incremental HPWL calculation
- [ ] Basic SA loop with displacement-limited moves
- [ ] Add best solution tracking

### Phase 4: Testing & Refinement (Week 4)
- [ ] Test on small benchmarks
- [ ] Verify legality checking accuracy
- [ ] Verify displacement limits are respected
- [ ] Compare with DPL's greedy approach
- [ ] Performance tuning
- [ ] Memory usage profiling
- [ ] Documentation

## File Structure for v0

```
sa2d/
├── CMakeLists.txt           # Links with dpl
├── include/sa2d/
│   ├── SA2D.h              # Main interface
│   └── MakeSA2D.h          # Factory pattern
├── src/
│   ├── SA2D.cpp            # Main implementation
│   ├── SA2D.tcl            # TCL commands
│   ├── SA2D.i              # SWIG interface
│   ├── CellState.h         # Lightweight state
│   ├── ThreadSafeGrid.h    # Grid classes
│   ├── ThreadSafeGrid.cpp  # Grid implementation
│   ├── Worker.h            # SA worker
│   ├── Worker.cpp          # Worker implementation
│   └── MakeSA2D.cpp        # Factory implementation
└── test/
    └── test_basic.tcl      # Basic test
```

## Memory Analysis

For 1M cells, single worker:
- Cell states: 1M × 12 bytes = 12 MB
- Grid (10K × 1K pixels): 10M × 4 bytes = 40 MB
- HPWL cache: ~8 MB
- **Total per worker: ~60 MB**

This is very reasonable and enables future scaling to 20+ workers.

## Key Differences from Original Plan

1. **Full Grid implementation** instead of lazy checking
2. **Thread-safe design** from the start
3. **Accurate legality checking** matching DPL's quality
4. **Slightly more memory** but still reasonable (~60MB/worker)
5. **Better foundation** for parallel GWTW

## Success Metrics for v0

1. **Functional**: Successfully runs SA with legal placements
2. **Quality**: Shows improvement over initial placement
3. **Accuracy**: Zero legality violations
4. **Integration**: Works within OpenROAD flow
5. **Performance**: Reasonable runtime (within 5x of DPL)
6. **Memory**: Low overhead (~60MB for single worker)

## Path to Parallel (Future)

The thread-safe grid design enables future parallel GWTW:
1. Multiple workers with independent grids
2. Shared ImmutableGridInfo (no duplication)
3. Fast grid state copying between workers
4. Single-threaded final result application

## Next Steps After v0

Once v0 is working:
1. Implement multiple workers
2. Add GWTW synchronization
3. Optimize grid copying performance
4. Add more sophisticated moves
5. Profile and optimize memory usage 