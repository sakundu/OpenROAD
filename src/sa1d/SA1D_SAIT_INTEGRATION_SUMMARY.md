# SA1D + SAIT Vertex Ordering Integration

## Overview

This document describes the successful integration of the SAIT (Spectral AI Iterative Techniques) vertex ordering algorithms with OpenROAD's SA1D (1D Simulated Annealing) placer. The integration allows SA1D to use advanced graph-theoretic vertex ordering algorithms as initial placement solutions before starting the simulated annealing refinement process.

## Architecture

```
OpenROAD Database → Hypergraph Extraction → SAIT Ordering → Cell Position Mapping → SA1D Initialization → SA Refinement
```

### Key Components

1. **VertexOrderingInterface** (`src/VertexOrdering.cpp/.h`): Main interface that bridges SA1D and SAIT
2. **SAIT Core Files** (`src/sait/`): Copied and adapted SAIT algorithms
3. **OptSA Extensions**: Modified to support custom vertex ordering initialization
4. **TCL Interface**: Commands to control vertex ordering from OpenROAD scripts

## Supported Algorithms

| Algorithm | Description | Use Case |
|-----------|-------------|----------|
| `random` | Random vertex ordering | Baseline comparison |
| `size_based` | Size-based ordering (smallest first) | Simple heuristic |
| `fiedler` | Fiedler vector (spectral) ordering | Minimize wire length |
| `rcm` | Reverse Cuthill-McKee ordering | Bandwidth reduction |
| `rcm_boost` | RCM using Boost Graph Library | Alternative RCM implementation |

## Files Modified/Added

### Core Integration Files
- `include/sa1d/VertexOrdering.h` - Main interface header
- `src/VertexOrdering.cpp` - Main interface implementation
- `include/sa1d/OptSA.h` - Added vertex ordering methods
- `src/OptSA.cpp` - Added vertex ordering implementation

### SAIT Algorithm Files
- `include/sait/hypergraph.hpp` - Hypergraph data structure
- `src/sait/hypergraph.cpp` - Hypergraph implementation
- `include/sait/fiedler_ordering.hpp` - Fiedler ordering interface
- `src/sait/fiedler_ordering.cpp` - Fiedler ordering implementation
- `include/sait/rcm_ordering.hpp` - RCM ordering interface
- `src/sait/rcm_ordering.cpp` - RCM ordering implementation
- `include/sait/random_ordering.hpp` - Random ordering interface
- `src/sait/random_ordering.cpp` - Random ordering implementation
- `include/sait/graph_conversion.hpp` - Graph conversion utilities
- `src/sait/graph_conversion.cpp` - Graph conversion implementation
- `include/sait/io_utils.hpp` - I/O utilities
- `src/sait/io_utils.cpp` - I/O utilities implementation

### Interface Files
- `src/OptSA.i` - SWIG interface with vertex ordering commands
- `src/OptSA.tcl` - TCL commands for vertex ordering

### Build System
- `CMakeLists.txt` - Updated to include SAIT sources and dependencies

## Usage

### TCL Commands

```tcl
# Set vertex ordering method
set_vertex_ordering_method <method> [-verbose]

# Enable/disable custom ordering
enable_custom_ordering <true|false>

# Compute custom ordering (standalone)
compute_custom_ordering
```

### Example Usage

```tcl
# Use Fiedler ordering with verbose output
set_vertex_ordering_method fiedler -verbose

# Enable custom ordering for SA1D
enable_custom_ordering true

# Run SA1D with custom initialization
opt_sa_1d
```

### C++ API Usage

```cpp
#include "sa1d/OptSA.h"
#include "sa1d/VertexOrdering.h"

// Configure vertex ordering
sa1d::VertexOrderingParams params;
params.method = sa1d::OrderingMethod::FIEDLER;
params.verbose = true;

// Set up SA1D with custom ordering
sa1d::OptSA opt_sa;
opt_sa.init(db, logger);
opt_sa.setVertexOrderingMethod(params);
opt_sa.enableCustomOrdering(true);

// Run SA with custom initialization
opt_sa.runSA();
```

## Data Flow

### 1. Hypergraph Conversion
- SA1D cells and nets are converted to SAIT hypergraph format
- Cell IDs are mapped to vertex IDs (0-indexed for SAIT)
- Net connectivity becomes hyperedges

### 2. SAIT Algorithm Execution
- Selected algorithm (Fiedler, RCM, etc.) computes vertex ordering
- Returns ordered list of vertex IDs

### 3. Position Mapping
- Vertex ordering is converted back to cell ordering
- Cells are placed left-to-right according to ordering
- X positions computed based on cumulative cell widths

### 4. SA Initialization
- SA1D workers are initialized with custom cell positions
- SA refinement proceeds normally from this initial solution

## Integration Points

### In OptSA::runSA()
```cpp
// INTEGRATION POINT: Custom vertex ordering initialization
if (use_custom_ordering_) {
    auto ordering_result = computeCustomOrdering();
    if (ordering_result.success) {
        initializeCellOrderingFromCustom(ordering_result);
        cellOrdering(cell_order, orients);
    }
}
```

## Validation and Testing

### Build Validation
```bash
cd /path/to/OpenROAD
cmake --build build --target sa1d_lib --parallel
```

### Functionality Testing
```bash
cd src/sa1d/test
./openroad -no_init test_vertex_ordering_integration.tcl
```

## Performance Metrics

The integration tracks several metrics:

- **Initial HPWL**: Wire length before ordering
- **Final HPWL**: Wire length after ordering  
- **Computation Time**: Time spent in ordering algorithm
- **Vertices/Hyperedges Processed**: Graph size statistics
- **Algorithm Used**: Which ordering method was applied

## Dependencies

### Required Libraries
- **Eigen3**: For spectral algorithms (Fiedler ordering)
- **Boost Graph**: For RCM-Boost implementation
- **OpenMP**: For parallelization (already in SA1D)

### CMake Dependencies
```cmake
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS graph)

target_link_libraries(sa1d_lib
  PUBLIC
    Eigen3::Eigen
  PRIVATE
    Boost::graph
)
```

## Error Handling

The integration includes comprehensive error handling:

- **Hypergraph Validation**: Checks for valid graph structure
- **Algorithm Failures**: Graceful fallback to default initialization
- **Memory Management**: Smart pointers for safe resource management
- **Logging**: Detailed debug output when verbose mode enabled

## Future Enhancements

### Potential Improvements
1. **IO-Aware Ordering**: Consider I/O pin constraints
2. **Multi-objective Optimization**: Balance wire length and other metrics
3. **Incremental Updates**: Support for incremental ordering updates
4. **More Algorithms**: Add additional SAIT algorithms (e.g., bandwidth minimization)
5. **Parallel Ordering**: Leverage OpenMP for algorithm parallelization

### Configuration Extensions
1. **Algorithm Parameters**: Expose algorithm-specific tuning parameters
2. **Hybrid Methods**: Combine multiple ordering strategies
3. **Quality Metrics**: Additional placement quality assessments

## Troubleshooting

### Common Issues
1. **Compilation Errors**: Ensure Eigen3 and Boost are properly installed
2. **Runtime Failures**: Check that design has valid connectivity
3. **Performance Issues**: Monitor hypergraph size for large designs

### Debug Output
Enable verbose mode to see detailed execution traces:
```tcl
set_vertex_ordering_method fiedler -verbose
```

## Summary

The SA1D + SAIT integration successfully provides:

✅ **Complete Integration**: Full integration of SAIT algorithms into SA1D workflow  
✅ **Multiple Algorithms**: Support for 5 different vertex ordering methods  
✅ **Robust Interface**: Both C++ and TCL APIs available  
✅ **Error Handling**: Comprehensive validation and fallback mechanisms  
✅ **Performance Monitoring**: Detailed metrics and timing information  
✅ **Maintainable Code**: Clean separation of concerns and modularity  

The integration enables users to leverage advanced graph-theoretic algorithms for improved initial placement quality, potentially leading to better final SA1D results with faster convergence. 