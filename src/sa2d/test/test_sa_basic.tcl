source "helpers.tcl"

# Test basic SA2D functionality with a small design
read_lef Nangate45/Nangate45_tech.lef
read_lef Nangate45/Nangate45_stdcell.lef

read_def "gcd_nangate45_placed.def"

# Run detailed placement first
detailed_placement

# Run SA2D with basic parameters
puts "Running SA2D optimization..."

sa2d_detailed_placement \
    -max_temp 50.0 \
    -min_temp 0.01 \
    -cooling_rate 0.95 \
    -max_iter 10 \
    -move_budget 1000 \
    -seed 42 \
    -max_displacement {100 20}

puts "SA2D optimization completed."

# Optionally check placement
check_placement

puts "Test completed successfully." 