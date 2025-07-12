set proj_dir "/home/fetzfs_projects/SAITPlacement/sakundu/Testcases/"
read_lef "${proj_dir}/lef/asap7_tech_1x_201209.lef"
read_lef "${proj_dir}/lef/asap7sc7p5t_27_R_1x_201211.lef"
read_lef "${proj_dir}/lef/asap7sc7p5t_27_L_1x_201211.lef"
read_lef "${proj_dir}/lef/asap7sc7p5t_27_SL_1x_201211.lef"
read_lef "${proj_dir}/lef/sram_asap7_16x256_1rw.lef"
# foreach mbff_lef [glob "${proj_dir}/lef/mbff_lef/*.lef"] {
#     read_lef $mbff_lef
# }
set design "$::env(DESIGN)"
read_def "${proj_dir}/def/${design}_placed.def"


## Add command
## Add DESIGN="aes_cipher_top", "ibex_core", "jpeg_encoder", "ariane"
detailed_placement
#sa2d_run_dpl_reordering_only
#improve_placement

#exit

check_placement

puts "\n3. Configuring SA2D for parallel execution..."
sa2d_set_num_workers 80         ;
sa2d_set_max_temp 100.0
sa2d_set_min_temp 1e-18
sa2d_set_cooling_rate 0.998
sa2d_set_moves_per_iter 10000   ;
sa2d_set_max_iter 6000          ;# More iterations to see GWTW effect
sa2d_set_move_budget 10000000   ;# Larger budget for parallel
sa2d_set_gwtw_interval 300       ;# Sync every 50 iterations
sa2d_set_elite_ratio 0.25       ;# Top 25% are winners (1 worker)
sa2d_set_seed 42
#sa2d_set_use_sa1d_operators 1   ;
sa2d_set_max_displacement 1 16
sa2d_set_enable_kicks 1
sa2d_set_kick_threshold 0.02
sa2d_set_chain_move_interval 15        ;# More frequent chain moves
sa2d_set_chain_moves_per_round 10      ;# More chain moves per round
sa2d_set_kick_strength 20              ;# Larger kick regions
sa2d_set_kick_temp_multiplier 2.5      ;# Higher kick temperature
sa2d_set_kick_interval 100             ;# More frequent kicks
sa2d_set_enable_reordering 1           ;
sa2d_set_use_dpl_reordering 1               ;# Enable reordering
sa2d_set_reorder_window_size 2         ;
sa2d_set_dpl_reordering_passes 10
sa2d_set_enable_chain_moves 1
sa2d_set_enable_slides 1


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

#improve_placement

write_def ${design}_sa.def
