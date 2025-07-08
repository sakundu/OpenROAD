add_test([=[TestHconn.ConnectionMade]=]  /home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/build_test_sa2d/src/dbSta/test/cpp/TestHconn [==[--gtest_filter=TestHconn.ConnectionMade]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[TestHconn.ConnectionMade]=]  PROPERTIES WORKING_DIRECTORY /home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/src/dbSta/test/cpp/.. SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==])
set(  TestHconn_TESTS TestHconn.ConnectionMade)
