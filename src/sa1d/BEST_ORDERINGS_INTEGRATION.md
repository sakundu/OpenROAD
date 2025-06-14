# SA1D + SAIT Best-Orderings Integration

## Understanding the "Best-Orderings" Functionality

### What "best-orderings" Does:

1. **Runs 8-11 SAIT algorithms in parallel:**
   - **Basic algorithms:** Fiedler, RCM (2 versions), BFS, DFS, Random
   - **Space-filling curves:** SFC-Hilbert2D, SFC-ZOrder2D  
   - **Advanced methods** (if terminal file exists): Dirichlet, Soft-Penalty, Soft-Springs

2. **Applies optimal refinement to each ordering**

3. **Sorts results by peak cutwidth quality**

4. **Returns top 3 best orderings** with performance metrics

### Perfect Match for SA1D:

SA1D uses **multiple workers** (typically 20) with **Go-With-The-Winners (GWTW)** strategy:
- **Top 3 workers** get the best SAIT orderings as starting points
- **Remaining workers** use random/default initialization  
- **GWTW naturally favors** the workers that perform best
- **Result:** Much better SA convergence from superior starting points

## Implementation Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────────┐
│   OpenROAD DB   │ -> │ SAIT Best-Order  │ -> │ SA1D Multi-Worker   │
│                 │    │                  │    │                     │
│ • Cells & Nets  │    │ • 8-11 Algorithms│    │ • Worker 0: Best    │
│ • Connectivity  │    │ • Parallel Exec  │    │ • Worker 1: 2nd     │
│ • Constraints   │    │ • Refinement     │    │ • Worker 2: 3rd     │
└─────────────────┘    │ • Top 3 Results  │    │ • Worker 3-19: Rand │
                       └──────────────────┘    └─────────────────────┘
```

## Example Usage

### TCL Interface (Simple)
```tcl
# Method 1: Use best-orderings directly
enable_best_orderings true
set_best_orderings_params -verbose true -top_count 3

# Run SA1D with top 3 orderings
opt_sa_1d

# Method 2: Step-by-step control
compute_best_orderings
initialize_workers_from_best_orderings
opt_sa_1d
```

### C++ API (Advanced)
```cpp
#include "sa1d/OptSA.h"
#include "sa1d/BestOrderings.h"

// Setup
sa1d::OptSA opt_sa;
opt_sa.init(db, logger);

// Configure best-orderings
sa1d::BestOrderingsParams params;
params.verbose = true;
params.top_count = 3;
params.use_parallel = true;
params.include_advanced_methods = true;

opt_sa.setBestOrderingsParams(params);
opt_sa.enableBestOrderings(true);

// Run SA with best orderings initialization
opt_sa.runSA();
```

## How Worker Initialization Works

### Traditional SA1D:
```cpp
// All workers start randomly or from current placement
for (int i = 0; i < num_workers; ++i) {
    workers[i]->initCellOrderRandom();  // or from current
}
```

### Enhanced SA1D with Best-Orderings:
```cpp
// Get top 3 orderings from SAIT
auto best_result = computeBestOrderings();

// Initialize workers strategically  
for (int i = 0; i < num_workers; ++i) {
    if (i < best_result.getCount()) {
        // Top workers get best orderings
        auto& ordering_info = best_result.top_orderings[i];
        workers[i]->initCellOrder(ordering_info.cell_ordering, 
                                  ordering_info.orientations);
    } else {
        // Remaining workers use random
        workers[i]->initCellOrderRandom();
    }
}
```

## Expected Performance Benefits

### 1. **Better Initial Solutions**
- **Fiedler:** Spectral ordering optimizes for wire length
- **RCM:** Bandwidth reduction improves locality
- **SFC:** Space-filling curves provide excellent locality

### 2. **Faster SA Convergence**  
- SA starts from much better initial points
- GWTW favors workers with good starts
- Fewer iterations needed to reach high quality

### 3. **Higher Final Quality**
- Better starting points → better local optima
- Multiple diverse starting strategies
- Best of both worlds: global optimization + local refinement

## Detailed Workflow

### Phase 1: Best-Orderings Computation
```
1. Convert SA1D netlist → SAIT hypergraph
2. Launch algorithms in parallel:
   - Fiedler (spectral)            ~100-500ms
   - RCM (bandwidth)               ~50-200ms  
   - SFC-Hilbert (space-filling)   ~200-800ms
   - BFS/DFS (graph traversal)     ~10-50ms
   - Dirichlet (if IOs available)  ~300-1000ms
   - Advanced methods...           ~100-2000ms
3. Apply greedy refinement to each (~50-200ms per algorithm)
4. Sort by peak cutwidth quality
5. Return top 3 with metrics
```

### Phase 2: SA1D Worker Initialization  
```
1. Assign top 3 orderings to workers 0, 1, 2
2. Convert vertex orderings → cell positions
3. Initialize remaining workers randomly
4. All workers start SA with their assigned initial state
```

### Phase 3: SA1D Execution
```
1. Workers run SA iterations
2. GWTW copies best performers (likely workers 0, 1, 2 initially)
3. Competition drives all workers toward high quality
4. Final result benefits from both good starts and SA exploration
```

## Configuration Options

### BestOrderingsParams
```cpp
struct BestOrderingsParams {
    bool verbose = false;                    // Debug output
    bool use_parallel = true;                // Parallel algorithm execution
    int max_threads = 0;                     // 0 = auto-detect
    int top_count = 3;                       // Number of top orderings
    bool include_advanced_methods = true;    // Include Dirichlet, etc.
    bool apply_refinement = true;            // Apply greedy refinement
    bool use_constrained_refinement = false; // IO-constrained refinement
};
```

### Performance Tuning
```cpp
// For fast exploration (development)
params.top_count = 1;
params.include_advanced_methods = false;
params.apply_refinement = false;

// For maximum quality (production)  
params.top_count = 3;
params.include_advanced_methods = true;
params.apply_refinement = true;
params.use_constrained_refinement = true;  // If IO constraints exist
```

## Validation and Debugging

### TCL Commands for Analysis
```tcl
# Compute and analyze best orderings
compute_best_orderings -verbose

# Check worker initialization
report_worker_initialization

# Compare with baseline
run_sa_comparison -baseline random -enhanced best_orderings
```

### Debug Output Example
```
=== BEST ORDERINGS COMPUTATION ===
[+] Algorithms: 8 (including advanced methods)  
[+] Parallel threads: 8
[+] Refinement: IO-constrained

>> Processing algorithms in parallel...
[OK] Fiedler completed (peak: 145)
[OK] RCM completed (peak: 167)  
[OK] SFC-Hilbert2D completed (peak: 139)
[OK] Dirichlet completed (peak: 142)
...

>> TOP 3 BEST ORDERINGS:
+-----+-----------------+---------+---------+-------------+----------+
| Rank| Algorithm       | Initial | Final   | Improvement | Time(ms) |
+-----+-----------------+---------+---------+-------------+----------+
| 1   | SFC-Hilbert2D   | 187     | 139     | 25.7%       | 445      |
| 2   | Dirichlet       | 203     | 142     | 30.0%       | 692      |  
| 3   | Fiedler         | 198     | 145     | 26.8%       | 287      |
+-----+-----------------+---------+---------+-------------+----------+

>> WORKER INITIALIZATION:
  Worker 0: SFC-Hilbert2D (peak: 139)
  Worker 1: Dirichlet (peak: 142)
  Worker 2: Fiedler (peak: 145) 
  Workers 3-19: Random initialization
```

## Integration Status

✅ **Architecture Designed:** Complete interface and workflow defined  
⚠️ **SAIT Files:** Need to copy missing algorithms (dirichlet, sfc, etc.)  
⚠️ **Implementation:** BestOrderingsInterface needs implementation  
⚠️ **SA1D Integration:** Worker initialization needs modification  
⚠️ **Testing:** Validation and performance comparison needed  

## Next Steps for Full Implementation

### 1. Copy Missing SAIT Files
```bash
cd /home/fetzfs_projects/SAITPlacement/bodhi/OpenROAD/src/sa1d
chmod +x scripts/copy_missing_sait_files.sh
./scripts/copy_missing_sait_files.sh
```

### 2. Implement BestOrderingsInterface
- Parallel algorithm execution
- SAIT algorithm integration  
- Refinement application
- Result ranking and selection

### 3. Modify SA1D Worker Initialization
- Add worker initialization options
- Integrate with runSA() method
- Handle mixed initialization strategies

### 4. Add TCL Interface
- `enable_best_orderings`
- `compute_best_orderings`  
- `set_best_orderings_params`

### 5. Validation Testing
- Compare baseline vs best-orderings
- Measure convergence speed
- Analyze final quality improvement

## Summary

The **best-orderings** functionality provides a **multi-algorithm approach** that:

1. **Runs 8-11 SAIT algorithms in parallel**
2. **Returns top 3 best orderings** with quality metrics  
3. **Integrates perfectly with SA1D's multi-worker architecture**
4. **Provides superior starting points** for SA refinement
5. **Leverages GWTW strategy** to favor the best-performing workers

This creates a **hybrid global-local optimization system** that combines:
- **Global optimization:** SAIT's graph-theoretic algorithms
- **Local refinement:** SA1D's simulated annealing
- **Multi-start strategy:** Multiple diverse initial solutions  
- **Adaptive selection:** GWTW promotes the best approaches

**Expected result:** Significantly improved placement quality with faster convergence! 