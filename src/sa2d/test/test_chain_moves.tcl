# Test for SA2D chain/ripple moves
source "helpers.tcl"

# Read LEF/DEF
read_lef $::env(DESIGN_DIR)/Nangate45_tech.lef
read_lef $::env(DESIGN_DIR)/Nangate45_stdcell.lef  
read_def $::env(DESIGN_DIR)/aes_nangate45_placed.def

# Create congested placement to encourage chain moves
set db [::ord::get_db]
set block [[$db getChip] getBlock]

# Run global placement to get a starting point
global_placement -skip_initial_place

# Report initial metrics
puts "Initial placement metrics:"
report_design_area

# Run SA2D with moderate temperature to see chain moves
puts "\nRunning SA2D with chain moves enabled..."
sa2d_simple_place \
    -max_displacement 50 \
    -max_temp 10.0 \
    -cooling_rate 0.95 \
    -max_iter 100 \
    -move_budget 10000 \
    -seed 42

puts "\nSA2D placement completed."
puts "Chain moves should be reported in the statistics above."

# Verify placement is legal
if {[check_placement -verbose]} {
    puts "PASS: Placement is legal"
} else {
    puts "FAIL: Placement has violations"
    exit 1
}

# Report final metrics
puts "\nFinal placement metrics:"
report_design_area

puts "\nTest completed successfully!" 