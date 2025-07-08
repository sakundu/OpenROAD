# Analysis of Existing DPL Implementation

## Overview

The DPL (Detailed Placement) module in OpenROAD performs legalization and local optimization of cell placements after global placement. It focuses on improving placement quality while maintaining legal constraints.

## Core Concepts

### 1. Placement Representation

**Grid-Based System**:
- The placement area is discretized into a pixel grid
- Each pixel represents a placement site
- Cells occupy one or more pixels based on their size
- Multi-row cells are supported

**Coordinate Systems**:
- DBU (Database Units): Actual physical coordinates
- Grid coordinates: Site-based coordinates for placement
- Conversion utilities handle mapping between systems

### 2. Data Structures

**Node (Cell)**:
```cpp
class Node {
    int id_;
    DbuX left_, bottom_;        // Current position
    DbuX orig_left_, orig_bottom_; // Original position
    DbuX width_, height_;       // Dimensions
    bool fixed_, placed_, hold_; // Status flags
    Master* master_;            // Cell type information
    Group* group_;              // Placement group/region
    std::vector<Pin*> pins_;    // Connectivity
};
```

**Edge (Net)**:
```cpp
class Edge {
    int id_;
    std::vector<Pin*> pins_;  // Connected pins
    uint64_t hpwl() const;    // Calculate HPWL
};
```

**Architecture**:
- Manages row structure and site information
- Handles power rail patterns
- Defines legal placement locations

**Network**:
- Complete netlist representation
- Manages all nodes, edges, and pins
- Provides connectivity queries

### 3. Placement Operations

**Basic Moves**:
1. **Shift**: Move a cell to a new location
2. **Swap**: Exchange positions of two cells
3. **Mirror**: Flip cell orientation

**Advanced Operations**:
1. **Segment-based placement**: Cells are organized into horizontal segments
2. **Chain moves**: Moving connected cells together
3. **Window optimization**: Local optimization within a region

### 4. Optimization Strategies

The current implementation uses a **script-based optimization flow**:

```cpp
// From Optdp.cpp
dtParams.script_ = "";
dtParams.script_ += "mis -p 10 -t 0.005;";    // Maximum Independent Set
dtParams.script_ += "gs -p 10 -t 0.005;";     // Global Swaps
dtParams.script_ += "vs -p 10 -t 0.005;";     // Vertical Swaps
dtParams.script_ += "ro -p 10 -t 0.005;";     // Reordering
dtParams.script_ += "default -p 5 -f 20 -gen rng -obj hpwl -cost (hpwl);";
```

**Optimization Techniques**:

1. **Maximum Independent Set (MIS)**:
   - Identifies non-overlapping sets of movable cells
   - Optimizes positions independently

2. **Global Swaps**:
   - Considers swapping cells across the entire design
   - Targets long-range wirelength reduction

3. **Vertical Swaps**:
   - Swaps cells between adjacent rows
   - Improves vertical net connections

4. **Reordering**:
   - Reorders cells within segments
   - Local wirelength optimization

5. **Random Moves**:
   - Stochastic optimization with HPWL objective
   - Provides diversity in search

### 5. Legalization

**Constraints Enforced**:
1. **No overlaps**: Cells cannot occupy same sites
2. **Row alignment**: Cells must align to row boundaries
3. **Site alignment**: Cells snap to legal site positions
4. **Region boundaries**: Cells stay within assigned regions
5. **Fixed obstacles**: Respect macro and blockage locations

**Legalization Strategy**:
- Shift legalization: Minimal movement to resolve overlaps
- Maintains relative ordering when possible
- Handles multi-row cells specially

### 6. Cost Functions

**Primary Objective - HPWL**:
```cpp
uint64_t Edge::hpwl() const {
    // Calculate bounding box of all pins
    // Return half-perimeter of bounding box
}
```

**Additional Metrics**:
- Displacement from original position
- Cell density uniformity
- Edge spacing violations

### 7. Incremental Updates

**Journal System**:
- Tracks all placement changes
- Enables undo/redo operations
- Supports incremental cost updates

**Efficient Updates**:
- Only recalculate affected nets
- Maintain spatial data structures (R-tree)
- Cache frequently used computations

## Key Algorithms

### 1. DetailedManager
- Central coordinator for optimization
- Manages segments and placement constraints
- Handles move generation and evaluation

### 2. Move Generation
```cpp
bool tryMove(Node* cell, DbuX new_x, DbuY new_y) {
    // 1. Check if move is legal
    // 2. Calculate cost change
    // 3. Apply if beneficial
    // 4. Update data structures
}
```

### 3. Cost Evaluation
- Incremental HPWL computation
- Delta cost calculation for efficiency
- Multi-objective support (weighted sum)

## Strengths of Current Implementation

1. **Modular Design**: Clear separation of concerns
2. **Flexibility**: Script-based optimization flow
3. **Efficiency**: Incremental updates and spatial indexing
4. **Robustness**: Handles various cell types and constraints
5. **Integration**: Well-integrated with OpenROAD infrastructure

## Limitations

1. **Greedy Nature**: Accepts only improving moves
2. **Local Minima**: Can get stuck in suboptimal solutions
3. **Limited Exploration**: Doesn't accept temporary degradation
4. **Sequential Processing**: Limited parallelization

## Opportunities for SA-Based Improvement

1. **Global Search**: SA can escape local minima
2. **Solution Diversity**: Probabilistic acceptance explores more solutions
3. **Adaptive Strategy**: Temperature schedule can adapt to problem
4. **Multi-Objective**: Natural framework for competing objectives
5. **Parallel Potential**: Independent move evaluation

## Key Takeaways for SA2D Implementation

1. **Reuse Infrastructure**:
   - Grid, Node, Edge, Network classes
   - Legalization algorithms
   - HPWL computation

2. **Replace Optimization Core**:
   - SA engine instead of greedy acceptance
   - Temperature-based move acceptance
   - Adaptive move generation

3. **Enhance Features**:
   - Better handling of competing objectives
   - More sophisticated move types
   - Learning-based parameter adaptation

4. **Maintain Compatibility**:
   - Same input/output interface
   - Compatible with existing flows
   - Similar runtime characteristics 