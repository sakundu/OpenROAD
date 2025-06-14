set top_module ArtNet

set stamp1 [clock seconds]

set lef_dir "./lefs"
# put input DEF file path
# set def_dir "./defs/testcase1"

read_lef ${lef_dir}/asap7_tech_1x_201209.lef
read_lef ${lef_dir}/asap7sc7p5t_27_L_1x_201211.lef
read_lef ${lef_dir}/asap7sc7p5t_27_R_1x_201211.lef
read_lef ${lef_dir}/asap7sc7p5t_27_SL_1x_201211.lef

# read_def ${def_dir}/ArtNet.def
# read_def ./ArtNet_dpo1.def
# read_def "./ArtNet_1500_30_20_45_40_10.def"
# read_def "/home/fetzfs_projects/SAITPlacement/bodhi/OpenROAD/src/sa1d/test/ArtNet_dpo.def"
set design $::env(SA1D_DEF_FILE)
if { $design == "1"} {
  read_def "./defs/testcase1/ArtNet.def"
} elseif { $design == "2" } {
  read_def "./defs/testcase2/ArtNet.def"
} elseif { $design == "3" } {
  read_def "./defs/testcase3/ArtNet.def"
} elseif { $design == "4" } {
  read_def "./ArtNet_302_30_20_45_40_10.def"
} elseif { $design == "5" } {
  read_def "./ArtNet_510_30_20_45_40_10.def"
} elseif { $design == "6" } {
  read_def "./ArtNet_1000_30_20_45_40_10.def"
} elseif { $design == "7" } {
  read_def "./ArtNet_1500_30_20_45_40_10.def"
} else {
  puts "\[ERROR\] Invalid design number: $design"
  exit 1
}

if { $::env(SA1D_ENABLE_UPDATED_COST) == 1 } {
  setSAParams -json_file "./setSAParam.json"
} else {
  setSAParams -json_file "./setSAParam_dc.json"
}

# enable_best_orderings true
# set_best_orderings_params -verbose -use_parallel -include_advanced -apply_refinement
# In your TCL script

if { $::env(SA1D_ENABLE_BEST_ORDERINGS) == 1 } {
  enable_best_orderings true
  set_best_orderings_params \
      -verbose true \
      -use_parallel true \
      -max_threads 4 \
      -top_count 6 \
      -include_advanced true \
      -apply_refinement true
  compute_best_orderings -verbose
}
# check_placement
opt_sa_1d

check_placement

report_pack_hpwl

check_placement -verbose

# detailed_placement
improve_placement

# write_def ArtNet_dpo.def

set stamp2 [clock seconds]

puts "\[INFO\] Running time:   [expr $stamp2 - $stamp1] seconds"

exit
