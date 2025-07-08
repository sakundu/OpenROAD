# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/build_test_sa2d/_deps/googletest-src"
  "/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/build_test_sa2d/_deps/googletest-build"
  "/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/build_test_sa2d/_deps/googletest-subbuild/googletest-populate-prefix"
  "/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/build_test_sa2d/_deps/googletest-subbuild/googletest-populate-prefix/tmp"
  "/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/build_test_sa2d/_deps/googletest-subbuild/googletest-populate-prefix/src/googletest-populate-stamp"
  "/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/build_test_sa2d/_deps/googletest-subbuild/googletest-populate-prefix/src"
  "/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/build_test_sa2d/_deps/googletest-subbuild/googletest-populate-prefix/src/googletest-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/build_test_sa2d/_deps/googletest-subbuild/googletest-populate-prefix/src/googletest-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/build_test_sa2d/_deps/googletest-subbuild/googletest-populate-prefix/src/googletest-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
