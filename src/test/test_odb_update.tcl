source "helpers.tcl"
read_lef Nangate45/Nangate45_tech.lef
read_lef Nangate45/Nangate45_stdcell.lef
read_def design_nangate45.def

# 1. Run global placement
puts "\n1. Running global placement..."
global_placement -skip_nesterov_place

# 2. Run detailed placement
puts "\n2. Running detailed placement..."
detailed_placement

# 3. Configure SA2D parameters
puts "\n3. Configuring SA2D parameters..."
sa2d_set_num_workers 1
sa2d_set_temperature 100.0 1.0
sa2d_set_cooling_rate 0.95
sa2d_set_max_iterations 10
sa2d_set_move_budget 5000
sa2d_set_max_displacement 100 50
sa2d_set_seed 123

# 4. Run SA2D optimization
puts "\n4. Running SA2D optimization..."
sa2d_run

# 5. Check final placement after SA2D
puts "\n5. Checking final placement after SA2D..."
check_placement -verbose

puts "\n=== ODB Update Test Complete ==="
puts "If the Final ODB HPWL matches the SA2D reported best HPWL, then database update is working correctly!"

exit 