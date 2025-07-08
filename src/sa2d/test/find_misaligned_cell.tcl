# Script to find and analyze the misaligned cell
puts "\n=== Finding Misaligned Cell ==="

# Load the design
read_lef ../../dpl/test/Nangate45/Nangate45_tech.lef
read_lef ../../dpl/test/Nangate45/Nangate45_stdcell.lef
read_def ../../dpl/test/ibex_core_replace.def

# Get core and site info
set block [ord::get_db_block]
set core_area [$block getCoreArea]
set core_x [$core_area xMin]
set core_y [$core_area yMin]
puts "Core area starts at: ($core_x, $core_y)"

set rows [$block getRows]
set first_row [lindex $rows 0]
set site [$first_row getSite]
set site_width [$site getWidth]
puts "Site width: $site_width DBU"

# Run detailed placement
detailed_placement

# Check all cells BEFORE SA2D
puts "\n=== Checking cells BEFORE SA2D ==="
set cells [$block getInsts]
set misaligned_before {}
foreach cell $cells {
    if {[$cell isCore]} {
        set name [$cell getName]
        set loc [$cell getLocation]
        set x [$loc x]
        set y [$loc y]
        set master [$cell getMaster]
        set width [$master getWidth]
        
        # Calculate core-relative position
        set rel_x [expr $x - $core_x]
        
        # Check alignment
        if {[expr $x % $site_width] != 0} {
            lappend misaligned_before $cell
            puts "MISALIGNED BEFORE: $name"
            puts "  Absolute position: ($x, $y)"
            puts "  Core-relative position: ($rel_x, [expr $y - $core_y])"
            puts "  Width: $width"
            puts "  X % site_width = [expr $x % $site_width]"
            puts "  Master: [$master getName]"
        }
    }
}
puts "Total misaligned before SA2D: [llength $misaligned_before]"

# Run SA2D
puts "\n=== Running SA2D ==="
sa2d_set_num_workers 1
sa2d_set_max_temp 100.0
sa2d_set_min_temp 1.0
sa2d_set_cooling_rate 0.95
sa2d_set_max_iter 5
sa2d_set_move_budget 1000000
sa2d_set_seed 123
sa2d_run

# Check all cells AFTER SA2D
puts "\n=== Checking cells AFTER SA2D ==="
set misaligned_after {}
foreach cell $cells {
    if {[$cell isCore]} {
        set name [$cell getName]
        set loc [$cell getLocation]
        set x [$loc x]
        set y [$loc y]
        set master [$cell getMaster]
        set width [$master getWidth]
        
        # Calculate core-relative position
        set rel_x [expr $x - $core_x]
        
        # Check alignment
        if {[expr $x % $site_width] != 0} {
            lappend misaligned_after $cell
            puts "MISALIGNED AFTER: $name"
            puts "  Absolute position: ($x, $y)"
            puts "  Core-relative position: ($rel_x, [expr $y - $core_y])"
            puts "  Width: $width"
            puts "  X % site_width = [expr $x % $site_width]"
            puts "  Master: [$master getName]"
            
            # Check if this cell was in the misaligned_before list
            set was_misaligned_before 0
            foreach before_cell $misaligned_before {
                if {[$before_cell getName] == $name} {
                    set was_misaligned_before 1
                    break
                }
            }
            if {$was_misaligned_before} {
                puts "  NOTE: This cell was already misaligned before SA2D"
            } else {
                puts "  ERROR: This cell became misaligned during SA2D!"
            }
        }
    }
}
puts "\nTotal misaligned after SA2D: [llength $misaligned_after]"

puts "\n=== Analysis Complete ===" 