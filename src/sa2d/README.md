# SA2D - Simulated Annealing-Based 2D Detailed Placer

## Overview

SA2D is a simulated annealing-based detailed placement engine for OpenROAD. It aims to improve placement quality (minimize HPWL) while maintaining legal placement constraints, offering an alternative optimization approach to the existing greedy-based detailed placer.

## Current Status: v0 Implementation

The initial version (v0) focuses on a simplified implementation that:
- **Reuses DPL infrastructure** directly rather than reimplementing
- **Basic moves only**: Single cell move and swap operations
- **Simple legalization**: Overlap, row, and site alignment checks
- **Single objective**: HPWL minimization only
- **Follows sa1d pattern**: Similar structure to the existing SA-based 1D placer

See [PLAN_v0.md](PLAN_v0.md) for the detailed v0 implementation plan.

## Motivation

The current detailed placement (DPL) implementation uses greedy optimization techniques that can get stuck in local minima. Simulated annealing provides:

- **Global optimization**: Ability to escape local minima through probabilistic acceptance
- **Better solution quality**: Potential for 5-10% HPWL improvement
- **Flexibility**: Natural framework for multi-objective optimization
- **Adaptivity**: Temperature-based control of optimization aggressiveness

## Key Features (v0)

- **Direct DPL integration**: Uses existing DPL classes and infrastructure
- **SA-based optimization**: Temperature-controlled probabilistic acceptance
- **Basic move types**: Single moves and swaps using DPL's move infrastructure
- **Incremental updates**: Efficient HPWL computation from DPL
- **Legal placement guarantee**: Uses DPL's legalization checks

## Architecture (v0)

```
SA2D (v0)
    ├── Main SA2D Class
    │   └── Interfaces with DPL
    ├── SA Worker
    │   ├── Move Generation (using DPL)
    │   ├── Cost Evaluation (using DPL)
    │   └── SA Acceptance
    └── TCL Interface
```

## Usage (v0)

```tcl
# Run SA-based detailed placement
sa2d_detailed_placement \
    -max_temp 100.0 \
    -min_temp 0.001 \
    -cooling_rate 0.95 \
    -max_iter 1000 \
    -seed 42
```

## Development Status

- **v0**: In planning phase - see [PLAN_v0.md](PLAN_v0.md)
- **Future versions**: See [PLAN.md](PLAN.md) for long-term vision

## Documentation

- [PLAN_v0.md](PLAN_v0.md) - v0 implementation plan (simplified, DPL-integrated)
- [PLAN.md](PLAN.md) - Long-term comprehensive plan
- [DPL_ANALYSIS.md](DPL_ANALYSIS.md) - Analysis of existing DPL implementation
- Additional documentation will be added as development progresses

## Contributing

This project is part of OpenROAD. Contributions should follow OpenROAD's coding standards and review process.

## References

- Kirkpatrick, S., Gelatt, C. D., & Vecchi, M. P. (1983). Optimization by simulated annealing. Science, 220(4598), 671-680.
- Sechen, C., & Sangiovanni-Vincentelli, A. (1986). TimberWolf3.2: A new standard cell placement and global routing package. IEEE Design & Test of Computers, 3(3), 44-54. 