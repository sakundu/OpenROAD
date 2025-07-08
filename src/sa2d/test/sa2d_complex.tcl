# Test SA2D placement optimization on more complex design
source "../../dpl/test/helpers.tcl"

# Read input files
read_lef ../../dpl/test/Nangate45/Nangate45.lef
read_def ../../dpl/test/simple04.def

# First run standard detailed placement to get initial placement
detailed_placement

# Report initial placement metrics
puts "Initial placement metrics:"
check_placement

# Configure SA2D parameters for more aggressive optimization
sa2d_set_num_workers 8
sa2d_set_max_temp 200.0
sa2d_set_min_temp 0.01
sa2d_set_cooling_rate 0.98
sa2d_set_max_iter 5000
sa2d_set_move_budget 500
sa2d_set_seed 123
sa2d_set_max_displacement 20.0 20.0  ;# 20 microns max displacement

# Run SA2D optimization
puts "Running SA2D optimization..."
sa2d_run

# Check placement after SA2D
puts "Final placement metrics:"
check_placement

# Write output
set def_file [make_result_file sa2d_complex.def]
write_def $def_file

# Report results
puts "SA2D placement optimization completed"
report_design_area 