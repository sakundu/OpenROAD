# Test SA2D placement optimization on simple design
source "../../dpl/test/helpers.tcl"

# Read input files
read_lef ../../dpl/test/Nangate45/Nangate45.lef
read_def ../../dpl/test/simple01.def

# First run standard detailed placement to get initial placement
detailed_placement

# Configure SA2D parameters
sa2d_set_num_workers 4
sa2d_set_max_temp 100.0
sa2d_set_min_temp 0.1
sa2d_set_cooling_rate 0.95
sa2d_set_max_iter 1000
sa2d_set_move_budget 100
sa2d_set_seed 42
sa2d_set_max_displacement 10.0 10.0  ;# 10 microns max displacement

# Run SA2D optimization
sa2d_run

# Check placement after SA2D
check_placement

# Write output
set def_file [make_result_file sa2d_simple01.def]
write_def $def_file

# Report results
puts "SA2D placement optimization completed" 