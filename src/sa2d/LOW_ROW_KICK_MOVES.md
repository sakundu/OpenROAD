# Specialized Kick Move Framework for Low-Row Designs

## Overview

When a design has 5 or fewer rows (detected automatically), SA2D switches from standard region-based kick moves to specialized strategies optimized for the unique challenges of low-row placements.

## Challenges with Low-Row Designs

1. **Limited vertical movement**: With only 3-5 rows, cells have minimal Y-direction options
2. **High horizontal density**: All cells are compressed into few rows, creating dense horizontal packing
3. **Small effective regions**: Standard 10×10 regions become effectively 10×3 in a 3-row design
4. **Few free locations**: High density means minimal empty space for single moves

## Specialized Kick Move Strategies

### 1. Horizontal Chain Swap
- **Purpose**: Permute cells within a single row to escape local minima
- **Operation**: Selects a chain of 3-8 adjacent cells in a row and rotates their positions
- **Example**: `[A B C D] → [D A B C]`
- **Benefits**: Maintains legality while exploring different orderings

### 2. Row Compression
- **Purpose**: Create free space by removing gaps between cells
- **Operation**: 
  - Identifies a segment of cells with gaps between them
  - Compresses them to remove gaps
  - Only accepts if HPWL improves
- **Benefits**: Creates space for future moves while potentially improving wirelength

### 3. Inter-Row Transfer
- **Purpose**: Balance density across rows
- **Operation**:
  - Finds single-height cells that can move to other rows
  - Searches for free space in target rows near current X position
  - Transfers cell if space is found and SA accepts
- **Benefits**: Redistributes cells vertically for better optimization opportunities

### 4. Sliding Window Move
- **Purpose**: Locally rearrange cells while maintaining relative positions
- **Operation**:
  - Selects a window of 5-15 cells in a row
  - Randomly permutes cells within the window
  - Maintains some spacing between cells
- **Benefits**: Explores local permutations without disturbing global structure

## Implementation Details

### Detection
```cpp
bool isLowRowDesign() const { 
    return grid_info_->getRowCount() <= 5; 
}
```

### Strategy Selection
- Equal probability (25%) for each strategy
- Strategies are randomly selected for each kick attempt
- All strategies validate legality before applying changes

### Key Parameters
- **Chain length**: 3-8 cells (adaptive based on row population)
- **Compression segment**: 20-50 sites
- **Inter-row search range**: ±50 sites from current position
- **Sliding window size**: 5-15 cells

## Performance Optimizations

1. **Pre-validation**: All moves check legality before modifying state
2. **Efficient search**: Limited search ranges to avoid excessive computation
3. **Early termination**: Strategies abort early if insufficient cells/space
4. **Targeted acceptance**: Some strategies (like compression) only accept improvements

## Usage

The framework activates automatically when:
- Grid has 5 or fewer rows
- Kick moves are enabled (`enable_kicks = true`)
- Temperature is above kick threshold

No manual configuration needed - SA2D detects and adapts automatically.

## Monitoring

Look for these log messages:
```
[INFO SA2D-0485] Low row design detected (3 rows) - using specialized kick moves
[INFO SA2D-0486] Kick strategies: horizontal chain swap, row compression, inter-row transfer, sliding window
[INFO SA2D-0332] Used specialized kick strategies for 3-row design: horizontal chain swap, row compression, inter-row transfer, sliding window
```

## Future Enhancements

1. **Adaptive parameters**: Adjust chain lengths and window sizes based on density
2. **Multi-row chains**: Coordinate moves across multiple rows
3. **Density-aware strategies**: Focus on high-density regions
4. **Learning**: Track which strategies work best for specific designs 