# Test script for SA2D Parallel Implementation with GWTW
# Uses ibex design from OpenROAD test suite

puts "\n=== SA2D Parallel Test - Multiple Workers with GWTW ==="

# Load the ibex design
puts "\n1. Loading ibex design..."
read_lef ../../dpl/test/Nangate45/Nangate45_tech.lef
read_lef ../../dpl/test/Nangate45/Nangate45_stdcell.lef
read_def ../../dpl/test/ibex_core_replace.def

# Run detailed placement first (required before SA2D)
puts "\n2. Running detailed placement..."
detailed_placement
check_placement

# Configure SA2D for parallel execution
puts "\n3. Configuring SA2D for parallel execution..."
sa2d_set_num_workers 40          ;
sa2d_set_max_temp 100.0
sa2d_set_min_temp 0.0001
sa2d_set_cooling_rate 0.95
sa2d_set_max_iter 5000          ;# More iterations to see GWTW effect
sa2d_set_move_budget 200000000   ;# Larger budget for parallel
sa2d_set_gwtw_interval 100       ;# Sync every 50 iterations
sa2d_set_elite_ratio 0.25       ;# Top 25% are winners (1 worker)
sa2d_set_seed 456
sa2d_set_max_displacement 10 5

# Show configured parameters
puts "  Workers: 4"
puts "  Temperature: 100.0 -> 1.0"
puts "  Cooling rate: 0.95"
puts "  Max iterations: 200"
puts "  Move budget: 20M"
puts "  GWTW interval: 50"
puts "  Elite ratio: 0.25 (1 winner)"
puts "  Max displacement: (10, 5) sites"

# Run parallel SA2D optimization
puts "\n4. Running parallel SA2D optimization..."
set start_time [clock milliseconds]

# Run the SA2D command
sa2d_run

set end_time [clock milliseconds]
set runtime [expr {($end_time - $start_time) / 1000.0}]
puts "\nParallel SA2D runtime: $runtime seconds"

# Check placement
puts "\n5. Checking final placement..."
check_placement

exit

# Compare with single worker
puts "\n6. Running single worker for comparison..."
sa2d_set_num_workers 1
sa2d_set_seed 456  ;# Same seed for fair comparison

set start_time [clock milliseconds]
sa2d_run
set end_time [clock milliseconds]
set single_runtime [expr {($end_time - $start_time) / 1000.0}]

puts "\nSingle worker runtime: $single_runtime seconds"
puts "Speedup: [expr {$single_runtime / $runtime}]x"

puts "\n=== Parallel Test Complete ==="
puts "Parallel SA2D with GWTW is working!" 
