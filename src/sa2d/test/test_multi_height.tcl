#!/usr/bin/env tclsh
# Test multi-height cell support in SA2D

# Initialize
source "helpers.tcl"
read_lef ../../../test/Nangate45/Nangate45_tech.lef
read_lef ../../../test/Nangate45/Nangate45_stdcell.lef
read_def ../../../test/gcd_nangate45_placed.def

# Get initial HPWL
set initial_hpwl [dpl::get_total_hpwl]
puts "Initial HPWL: $initial_hpwl"

# Check if we have any multi-height cells
set num_multi_height 0
foreach inst [[ord::get_db_block] getInsts] {
    set master [$inst getMaster]
    set height [$master getHeight]
    set site_height [[[lindex [[ord::get_db_tech] getSites] 0] getHeight]]
    
    if {$height > $site_height} {
        incr num_multi_height
        puts "Multi-height cell: [$inst getName] (height: $height, spans [expr $height/$site_height] rows)"
    }
}

puts "Found $num_multi_height multi-height cells"

# Run SA2D placement with multi-height support
sa2d_simple_place \
    -max_displacement 10 \
    -max_temp 2.0 \
    -cooling_rate 0.95 \
    -max_iter 20 \
    -seed 42

# Get final HPWL
set final_hpwl [dpl::get_total_hpwl]
puts "Final HPWL: $final_hpwl"
puts "HPWL improvement: [expr 100.0 * (1.0 - double($final_hpwl) / double($initial_hpwl))]%"

# Verify all cells are legally placed
if {[check_placement -verbose]} {
    puts "All cells are legally placed"
} else {
    puts "ERROR: Placement has violations!"
    exit 1
}

puts "Multi-height cell test completed successfully"
exit 0 