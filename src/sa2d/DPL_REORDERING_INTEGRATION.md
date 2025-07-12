# DPL Reordering Integration for SA2D

## Problem

SA2D's initial reordering implementation was causing overlaps despite efforts to match DPL's algorithm. The main issues were:

1. **Incomplete validation**: Missing some of DPL's internal placement checks
2. **Grid synchronization**: Complex coordination between SA2D grid and DPL network
3. **Segment handling**: SA2D processes entire rows while DPL processes segments between blockages
4. **Edge cases**: Various corner cases not handled properly

## Solution

Instead of duplicating DPL's complex reordering logic, we exposed DPL's native reordering functionality through a new public API.

### Changes Made

#### 1. Added public method to DPL (`dpl/include/dpl/Opendp.h`):
```cpp
void runReorderingOnly(int max_displacement_x = 500,
                       int max_displacement_y = 100,
                       int passes = 10,
                       double tolerance = 0.005);
```

#### 2. Implemented the method in DPL (`dpl/src/Optdp.cpp`):
- Uses DPL's internal `DetailedMgr` and `Detailed` classes
- Runs only the "ro" (reordering) optimization script
- Maintains all DPL's internal validation and legality checks
- Guaranteed to not produce overlaps

#### 3. Updated SA2D to use DPL's reordering:
- SA2D now calls `dpl_->runReorderingOnly()` when DPL reordering is enabled
- SA2D's own reordering implementation is disabled with warning messages

## Usage

### To use DPL's native reordering BEFORE SA2D starts:
```tcl
# Enable pre-SA reordering
sa2d_set_pre_sa_reordering 1

# Configure reordering parameters (optional)
sa2d_set_dpl_reordering_passes 10      # Default: 5
sa2d_set_dpl_reordering_tolerance 0.01 # Default: 0.01
sa2d_set_reorder_window_size 3         # Default: 3 (range: 2-4)

# Run SA2D - DPL reordering will run BEFORE SA starts
sa2d_simple_place -max_displacement 50 -max_temp 10.0
```

### To use DPL's native reordering AFTER SA2D:
```tcl
# Enable reordering and use DPL's implementation
sa2d_set_enable_reordering 1
sa2d_set_use_dpl_reordering 1

# Configure reordering parameters (optional)
sa2d_set_dpl_reordering_passes 10      # Default: 5
sa2d_set_dpl_reordering_tolerance 0.01 # Default: 0.01
sa2d_set_reorder_window_size 3         # Default: 3 (range: 2-4)

# Run SA2D - DPL reordering will run automatically after SA completes
sa2d_simple_place -max_displacement 50 -max_temp 10.0
```

### To use reordering BOTH before and after SA2D:
```tcl
# Enable both pre and post reordering
sa2d_set_pre_sa_reordering 1          # Before SA
sa2d_set_enable_reordering 1          # After SA
sa2d_set_use_dpl_reordering 1         # Use DPL's implementation

# Run SA2D with reordering both before and after
sa2d_simple_place -max_displacement 50 -max_temp 10.0
```

### To run DPL reordering manually:
```tcl
# Run SA2D without reordering
sa2d_set_enable_reordering 0
sa2d_simple_place -max_displacement 50

# Then run DPL reordering separately
sa2d_run_dpl_reordering_only
```

## Benefits

1. **No overlaps**: DPL's reordering is battle-tested and guaranteed safe
2. **Better quality**: DPL's implementation handles all edge cases properly
3. **Less code duplication**: No need to maintain parallel implementations
4. **Future improvements**: Automatically benefit from DPL enhancements

## Technical Details

DPL's reordering algorithm:
- Processes segments (not full rows) to respect blockages
- Window sizes 2-4 (default 3)
- Tries all permutations within windows
- Validates site alignment and displacement limits
- Uses X-direction HPWL for cost evaluation
- Reverts changes if no improvement after alignment

## Notes

- SA2D's built-in reordering (`sa2d_set_use_dpl_reordering 0`) is disabled due to overlap issues
- The window size parameter (`sa2d_set_reorder_window_size`) now works with DPL's reordering (added in latest update)
- Window size must be between 2 and 4 (DPL constraint)
- Default window size is 3 