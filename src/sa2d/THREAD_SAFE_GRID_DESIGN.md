# Thread-Safe Grid Design for SA2D

## Overview

This document describes the design of a thread-safe Grid structure that replicates DPL's Grid functionality while enabling parallel SA workers to perform accurate legality checking.

## Core Requirements

The thread-safe Grid must support:
1. **Concurrent Read Access**: Multiple workers reading pixel state
2. **Copy-on-Write Semantics**: Each worker maintains independent state
3. **Efficient State Copying**: Fast state transfer for GWTW
4. **Simplified Legality Checking**: Only overlap, row alignment, site alignment, and site orientation (as per v0 requirements)

## Design Approach

### 1. Immutable Base Grid + Worker State

```cpp
namespace sa2d {

// Immutable grid information (shared across all workers)
class ImmutableGridInfo {
public:
    // Row and site information (never changes)
    struct RowInfo {
        DbuY bottom;
        DbuY height;
        dbOrientType orient;  // Row orientation
        std::map<dbSite*, dbOrientType> sites;  // Site type to orientation
    };
    
    // Static grid properties
    Rect core_;
    DbuX site_width_;
    GridY row_count_;
    GridX row_site_count_;
    std::vector<RowInfo> rows_;
    std::vector<DbuY> row_index_to_y_dbu_;
    std::optional<DbuY> uniform_row_height_;
    
    // Site information for each pixel (row y, site x)
    struct PixelSiteInfo {
        std::map<dbSite*, dbOrientType> sites;  // Available sites and their orientations
    };
    std::vector<std::vector<PixelSiteInfo>> pixel_sites_;  // [y][x]
    
    // Conversion functions (all const, thread-safe)
    GridX gridX(DbuX x) const;
    GridY gridSnapDownY(DbuY y) const;
    // ... other conversion functions
    
    // Site query functions
    const std::map<dbSite*, dbOrientType>& getSitesAt(GridX x, GridY y) const;
    dbOrientType getValidOrientation(dbSite* site, GridX x, GridY y) const;
};

// Mutable pixel state per worker
struct WorkerPixel {
    int cell_id = -1;  // Index into cell array (-1 = empty)
    // Note: We use cell_id instead of pointer for efficiency
};

// Complete worker grid state
class WorkerGrid {
public:
    WorkerGrid(const ImmutableGridInfo* info);
    
    // Pixel access
    bool isOccupied(GridX x, GridY y) const;
    int getCellAt(GridX x, GridY y) const;
    
    // Modifications (for single worker)
    void placeCell(int cell_id, GridX x, GridY y, GridX width, GridY height);
    void removeCell(int cell_id);
    
    // Efficient state copying for GWTW
    void copyFrom(const WorkerGrid& other);
    
private:
    const ImmutableGridInfo* info_;  // Shared, immutable
    std::vector<std::vector<WorkerPixel>> pixels_;  // Per-worker state
    
    // Optional: sparse representation for memory efficiency
    std::unordered_map<int, GridRect> cell_locations_;  // cell_id -> location
};

}  // namespace sa2d
```

### 2. Simplified Legality Checking (v0)

```cpp
class WorkerLegalityChecker {
public:
    WorkerLegalityChecker(const ImmutableGridInfo* info, 
                         const WorkerGrid* grid,
                         const dpl::Network* network);
    
    // Simplified legality checking for v0
    bool canPlaceCell(int cell_id, GridX x, GridY y) const {
        // 1. Check row alignment
        if (!checkRowAlignment(cell_id, y)) return false;
        
        // 2. Check site alignment
        if (!checkSiteAlignment(x)) return false;
        
        // 3. Check site compatibility (orientation)
        if (!checkSiteCompatibility(cell_id, x, y)) return false;
        
        // 4. Check no overlaps
        if (!checkNoOverlap(cell_id, x, y)) return false;
        
        return true;
    }
    
private:
    bool checkRowAlignment(int cell_id, GridY y) const {
        // Check if y coordinate aligns with a valid row
        return y >= 0 && y < info_->row_count_;
    }
    
    bool checkSiteAlignment(GridX x) const {
        // Check if x coordinate aligns with site grid
        // Grid coordinates are already site-aligned by construction
        return x >= 0 && x < info_->row_site_count_;
    }
    
    bool checkSiteCompatibility(int cell_id, GridX x, GridY y) const {
        const dpl::Node* cell = network_->getNode(cell_id);
        dbSite* cell_site = cell->getSite();
        
        // Check if the cell's site type is available at this location
        const auto& available_sites = info_->getSitesAt(x, y);
        auto it = available_sites.find(cell_site);
        if (it == available_sites.end()) {
            return false;  // Site type not available here
        }
        
        // The orientation will be determined by the pixel's site orientation
        // This is handled during cell placement
        return true;
    }
    
    bool checkNoOverlap(int cell_id, GridX x, GridY y) const {
        const auto& cell_info = getCellInfo(cell_id);
        GridX x_end = x + cell_info.grid_width;
        GridY y_end = y + cell_info.grid_height;
        
        // Check bounds
        if (x_end > info_->row_site_count_ || y_end > info_->row_count_) {
            return false;
        }
        
        // Check each pixel for overlap
        for (GridY yi = y; yi < y_end; yi++) {
            for (GridX xi = x; xi < x_end; xi++) {
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
    
    // Get the appropriate orientation for a cell at a position
    dbOrientType getCellOrientation(int cell_id, GridX x, GridY y) const {
        const dpl::Node* cell = network_->getNode(cell_id);
        dbSite* cell_site = cell->getSite();
        
        // Get the orientation from the pixel's site map
        return info_->getValidOrientation(cell_site, x, y);
    }
};
```

### 3. Integration with SA Worker

```cpp
class SAWorker {
private:
    // Lightweight cell state
    struct CellState {
        DbuX x;
        DbuY y;
        dbOrientType orient;  // Track orientation
    };
    std::vector<CellState> cell_states_;
    
    // Grid for overlap checking
    std::unique_ptr<WorkerGrid> grid_;
    std::unique_ptr<WorkerLegalityChecker> legality_checker_;
    
    // Shared immutable data
    const ImmutableGridInfo* grid_info_;
    const dpl::Network* network_;
    
public:
    bool tryMove(int cell_id) {
        // Generate new position
        DbuX new_x = generateNewX(cell_id);
        DbuY new_y = generateNewY(cell_id);
        
        // Convert to grid coordinates (ensures site/row alignment)
        GridX grid_x = grid_info_->gridX(new_x);
        GridY grid_y = grid_info_->gridSnapDownY(new_y);
        
        // Check simplified legality (overlap, row, site, orientation)
        if (!legality_checker_->canPlaceCell(cell_id, grid_x, grid_y)) {
            return false;
        }
        
        // Get the correct orientation for this position
        dbOrientType new_orient = legality_checker_->getCellOrientation(cell_id, grid_x, grid_y);
        
        // Calculate cost
        int64_t delta_hpwl = calcDeltaHPWL(cell_id, new_x, new_y);
        
        if (acceptMove(delta_hpwl, temp_)) {
            // Update grid
            grid_->removeCell(cell_id);
            grid_->placeCell(cell_id, grid_x, grid_y, 
                           getCellGridWidth(cell_id), 
                           getCellGridHeight(cell_id));
            
            // Update position (snapped to grid)
            cell_states_[cell_id].x = gridToDbu(grid_x, info_->site_width_);
            cell_states_[cell_id].y = info_->gridYToDbu(grid_y);
            cell_states_[cell_id].orient = new_orient;  // Update orientation
            
            return true;
        }
        return false;
    }
};
```

### 4. Initializing Immutable Grid Info from DPL

```cpp
void ImmutableGridInfo::initFromDPL(const dpl::Grid* dpl_grid,
                                   odb::dbBlock* block) {
    // Copy basic grid properties
    core_ = dpl_grid->getCore();
    site_width_ = dpl_grid->getSiteWidth();
    row_count_ = dpl_grid->getRowCount();
    row_site_count_ = dpl_grid->getRowSiteCount();
    
    // Initialize pixel site information
    pixel_sites_.resize(row_count_.v);
    for (int y = 0; y < row_count_.v; y++) {
        pixel_sites_[y].resize(row_site_count_.v);
    }
    
    // Populate site information from rows
    for (odb::dbRow* db_row : block->getRows()) {
        if (db_row->getSite()->getClass() == odb::dbSiteClass::PAD) {
            continue;
        }
        
        const odb::Point orig = db_row->getOrigin();
        const GridX x_start{(orig.x() - core_.xMin()) / site_width_.v};
        const GridX x_end{x_start + db_row->getSiteCount()};
        const GridY y_row = dpl_grid->gridSnapDownY(DbuY{orig.y() - core_.yMin()});
        
        // Store site and orientation for each pixel in this row
        for (GridX x = x_start; x < x_end; x++) {
            if (x >= 0 && x < row_site_count_ && y_row >= 0 && y_row < row_count_) {
                pixel_sites_[y_row.v][x.v].sites[db_row->getSite()] = db_row->getOrient();
            }
        }
    }
}
```

## Memory Optimization

### 1. Sparse Representation Option

For designs with low utilization:
```cpp
class SparseWorkerGrid {
    // Only store occupied pixels
    std::unordered_map<GridPt, int> occupied_pixels_;
    
    // Fast lookup by cell
    std::unordered_map<int, std::vector<GridPt>> cell_pixels_;
};
```

### 2. Compressed Pixel State

```cpp
// Pack pixel state into single integer
union CompressedPixel {
    struct {
        uint32_t cell_id : 24;    // Support up to 16M cells
        uint32_t occupied : 1;     // Occupied flag
        uint32_t reserved : 7;     // Future use
    };
    uint32_t packed;
};
```

## Memory Overhead Analysis

For a typical design:
- Grid size: 10,000 x 1,000 pixels
- Workers: 20

**Dense representation**:
- Per worker: 10M pixels × 4 bytes = 40 MB
- Total: 20 workers × 40 MB = 800 MB
- Shared immutable (including site info): ~100 MB
- **Total overhead: ~900 MB**

**Sparse representation** (10% utilization):
- Per worker: 1M occupied pixels × 12 bytes = 12 MB
- Total: 20 workers × 12 MB = 240 MB
- **Total overhead: ~340 MB**

## Implementation Phases

### Phase 1: Basic Thread-Safe Grid (v0)
- Implement ImmutableGridInfo with site orientation support
- Basic WorkerGrid with dense representation
- Simplified legality checking (overlap, row, site, orientation)

### Phase 2: Optimization
- Add sparse representation option
- Optimize state copying
- Profile memory usage

### Phase 3: Advanced Features (future versions)
- DRC checking integration
- Multi-height cell support
- Region constraints
- Blockage handling
- Power rail compatibility

## Key Simplifications for v0

1. **No blockage checking** - Assume valid placement regions
2. **No region constraints** - All cells can go anywhere legal
3. **Basic orientation handling** - Use row's orientation for cells
4. **No multi-height complexity** - Basic row alignment only
5. **No power rail checking** - Assume power compatibility

## Advantages of Thread-Safe Grid

1. **Accurate overlap detection**: No missed overlaps
2. **Guaranteed alignment**: Site and row alignment enforced
3. **Correct orientation**: Cells placed with valid orientations
4. **Better SA quality**: Avoid wasting iterations on illegal moves
5. **Parallel-ready**: Foundation for GWTW

## Comparison with Lazy Checking

| Aspect | Lazy Checking | Thread-Safe Grid |
|---------|---------------|------------------|
| Overlap Detection | Might miss | Always accurate |
| Site Compatibility | Not checked | Always verified |
| Orientation | May be invalid | Always correct |
| Memory Usage | Minimal | ~40MB/worker |
| Move Evaluation | Fast but risky | Fast and accurate |
| Final Legalization | Heavy cleanup | Minimal cleanup |

## Conclusion

This thread-safe Grid design provides:
- **Accurate overlap checking** throughout SA
- **Enforced site/row alignment** via grid coordinates
- **Correct site orientation** based on row configuration
- **Thread safety** for parallel workers
- **Reasonable memory overhead** (< 1GB for 20 workers)

The simplified v0 implementation focuses on essential checks (overlap, row, site, orientation) while maintaining the infrastructure for future enhancements. 