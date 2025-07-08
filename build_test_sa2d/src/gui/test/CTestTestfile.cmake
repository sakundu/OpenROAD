# CMake generated Testfile for 
# Source directory: /home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/src/gui/test
# Build directory: /home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/build_test_sa2d/src/gui/test
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(gui.supported.tcl "/usr/bin/bash" "/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/test/regression_test.sh")
set_tests_properties(gui.supported.tcl PROPERTIES  ENVIRONMENT "OPENROAD_EXE=/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/build_test_sa2d/src/openroad;TEST_NAME=supported;TEST_EXT=tcl;TEST_TYPE=tcl;TEST_CHECK_LOG=True;TEST_CHECK_PASSFAIL=False" LABELS "IntegrationTest tcl gui log_compare" WORKING_DIRECTORY "/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/src/gui/test" _BACKTRACE_TRIPLES "/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/src/cmake/testing.cmake;19;add_test;/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/src/cmake/testing.cmake;86;or_integration_test_single;/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/src/gui/test/CMakeLists.txt;1;or_integration_tests;/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/src/gui/test/CMakeLists.txt;0;")
