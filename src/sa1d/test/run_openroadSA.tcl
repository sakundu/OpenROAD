set top_module ArtNet

set stamp1 [clock seconds]

set lef_dir "./lefs"
# put input DEF file path
set def_dir "./defs/testcase1"

read_lef ${lef_dir}/asap7_tech_1x_201209.lef
read_lef ${lef_dir}/asap7sc7p5t_27_L_1x_201211.lef
read_lef ${lef_dir}/asap7sc7p5t_27_R_1x_201211.lef
read_lef ${lef_dir}/asap7sc7p5t_27_SL_1x_201211.lef

read_def ${def_dir}/ArtNet.def

setSAParams -json_file "./setSAParam.json" 

opt_sa_1d

check_placement

report_pack_hpwl

write_def ArtNet_dpo.def

set stamp2 [clock seconds]

puts "\[INFO\] Running time:   [expr $stamp2 - $stamp1] seconds"

exit
