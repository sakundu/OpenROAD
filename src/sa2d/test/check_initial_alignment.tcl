# Script to verify site alignment issue exists before SA2D
puts "\n=== Checking Initial Placement Alignment ==="

# Load the design
read_lef ../../dpl/test/Nangate45/Nangate45_tech.lef
read_lef ../../dpl/test/Nangate45/Nangate45_stdcell.lef
read_def ../../dpl/test/ibex_core_replace.def

# Run detailed placement
puts "\n1. Running detailed placement..."
detailed_placement

# Check placement BEFORE SA2D
puts "\n2. Checking placement alignment BEFORE SA2D..."
check_placement

puts "\n=== Initial placement check complete ==="
puts "If there's already a site alignment failure, it's not caused by SA2D." 