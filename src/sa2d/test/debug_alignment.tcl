# Debug script to identify site alignment issues
puts "\n=== SA2D Site Alignment Debug ==="

# Load the design
read_lef ../../dpl/test/Nangate45/Nangate45_tech.lef
read_lef ../../dpl/test/Nangate45/Nangate45_stdcell.lef
read_def ../../dpl/test/ibex_core_replace.def

# Get site width
set block [ord::get_db_block]
set rows [$block getRows]
set first_row [lindex $rows 0]
set site [$first_row getSite]
set site_width [$site getWidth]
puts "Site width: $site_width DBU"

# Run detailed placement
detailed_placement

# Check all cells before SA2D
puts "\nChecking cells before SA2D..."
set cells [$block getInsts]
set misaligned_before 0
foreach cell $cells {
    if {[$cell isCore]} {
        set x [$cell getLocation_x]
        set y [$cell getLocation_y]
        if {[expr $x % $site_width] != 0} {
            puts "  MISALIGNED: [$cell getName] at x=$x (not divisible by $site_width)"
            incr misaligned_before
        }
    }
}
puts "Total misaligned before SA2D: $misaligned_before"

# Configure and run SA2D
sa2d_set_num_workers 1
sa2d_set_max_temp 100.0
sa2d_set_min_temp 1.0
sa2d_set_cooling_rate 0.95
sa2d_set_max_iter 5
sa2d_set_move_budget 1000000
sa2d_set_seed 123

puts "\nRunning SA2D..."
sa2d_run

# Check all cells after SA2D
puts "\nChecking cells after SA2D..."
set misaligned_after 0
foreach cell $cells {
    if {[$cell isCore]} {
        set x [$cell getLocation_x]
        set y [$cell getLocation_y]
        if {[expr $x % $site_width] != 0} {
            puts "  MISALIGNED: [$cell getName] at x=$x (not divisible by $site_width)"
            incr misaligned_after
        }
    }
}
puts "Total misaligned after SA2D: $misaligned_after"

# Run the check_placement to confirm
puts "\nRunning check_placement..."
check_placement

puts "\n=== Debug Complete ===" 