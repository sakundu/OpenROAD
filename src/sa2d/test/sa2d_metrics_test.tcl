# SA2D metrics comparison test
source "../../dpl/test/helpers.tcl"

puts "=== SA2D Metrics Comparison Test ==="

# Load design
read_lef ../../dpl/test/Nangate45/Nangate45.lef
read_def ../../dpl/test/simple04.def

# Get initial placement metrics
puts "\n--- Initial Placement ---"
detailed_placement

puts "\nInitial placement metrics:"
check_placement -verbose

# Save initial DEF for comparison
set initial_def [make_result_file sa2d_metrics_initial.def]
write_def $initial_def

# Configure SA2D for optimization
puts "\n--- Configuring SA2D ---"
sa2d_set_num_workers 4
sa2d_set_max_temp 150.0
sa2d_set_min_temp 0.05
sa2d_set_cooling_rate 0.97
sa2d_set_max_iter 2000
sa2d_set_move_budget 200
sa2d_set_seed 42
sa2d_set_max_displacement 15.0 15.0

puts "SA2D parameters configured"

# Run SA2D optimization
puts "\n--- Running SA2D Optimization ---"
sa2d_run

# Check final placement metrics
puts "\nFinal placement metrics after SA2D:"
check_placement -verbose

# Save final DEF
set final_def [make_result_file sa2d_metrics_final.def]
write_def $final_def

# Report design area (useful metric)
puts "\nDesign area report:"
report_design_area

puts "\n=== Test completed ==="
puts "Initial placement saved to: $initial_def"
puts "Final placement saved to: $final_def" 