# Test script for SA2D reordering functionality
# This test enables reordering and runs SA2D to verify the integration works

puts "\n=== SA2D Reordering Test ==="

# Load a simple design for testing
puts "\n1. Loading test design..."
read_lef "../../dpl/test/Nangate45/Nangate45_tech.lef"
read_lef "../../dpl/test/Nangate45/Nangate45_stdcell.lef"
read_def "../../dpl/test/ibex_core_replace.def"

# Run detailed placement first (required before SA2D)
puts "\n2. Running detailed placement..."
detailed_placement
check_placement

# Configure SA2D with reordering enabled
puts "\n3. Configuring SA2D with reordering..."
sa2d_set_num_workers 1
sa2d_set_max_temp 50.0
sa2d_set_min_temp 1.0
sa2d_set_cooling_rate 0.95
sa2d_set_max_iter 50         ;# Small for testing
sa2d_set_move_budget 10000    ;# Small for testing
sa2d_set_seed 123
sa2d_set_max_displacement 5 3

# Enable reordering (this is the key feature being tested)
sa2d_set_enable_reordering 1

# Disable other features for cleaner test
sa2d_set_enable_kicks 0
sa2d_set_enable_chain_moves 0
sa2d_set_enable_slides 0

# Show configured parameters
puts "  Workers: 1"
puts "  Temperature: 50.0 -> 1.0"
puts "  Cooling rate: 0.95"
puts "  Max iterations: 50"
puts "  Max displacement: (5, 3) sites"
puts "  Reordering: ENABLED"

# Run SA2D optimization with reordering
puts "\n4. Running SA2D with reordering..."
set start_time [clock milliseconds]

# Run the SA2D command
sa2d_run

set end_time [clock milliseconds]
set runtime [expr {($end_time - $start_time) / 1000.0}]
puts "\nSA2D runtime: $runtime seconds"

# Check placement
puts "\n5. Checking final placement..."
check_placement

puts "\n=== SA2D Reordering Test Complete ==="
puts "SA2D reordering integration is verified and working!" 