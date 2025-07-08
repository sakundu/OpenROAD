# Basic SA2D functionality test
source "../../dpl/test/helpers.tcl"

puts "=== SA2D Basic Functionality Test ==="

# Test setting parameters without loading a design
puts "\n1. Testing SA2D parameter setting..."

sa2d_set_num_workers 2
puts "  - Set num_workers to 2"

sa2d_set_max_temp 50.0
puts "  - Set max_temp to 50.0"

sa2d_set_min_temp 0.5
puts "  - Set min_temp to 0.5"

sa2d_set_cooling_rate 0.9
puts "  - Set cooling_rate to 0.9"

sa2d_set_max_iter 100
puts "  - Set max_iter to 100"

sa2d_set_move_budget 50
puts "  - Set move_budget to 50"

sa2d_set_seed 999
puts "  - Set seed to 999"

# Now load a design
puts "\n2. Loading design..."
read_lef ../../dpl/test/Nangate45/Nangate45.lef
read_def ../../dpl/test/simple01.def

# Run detailed placement first
puts "\n3. Running initial detailed placement..."
detailed_placement
check_placement -verbose

# Set max displacement after design is loaded (needs site info)
puts "\n4. Setting max displacement..."
sa2d_set_max_displacement 5.0 5.0  ;# 5 microns

# Try to run SA2D
puts "\n5. Running SA2D optimization..."
sa2d_run

puts "\n=== Test completed successfully ===" 