# Install script for directory: /home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "RELEASE")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "0")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/home/tool/binutils/binutils-2.27/bin/objdump")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/build_test_sa2d/src/ifp/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/build_test_sa2d/src/pad/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/build_test_sa2d/src/odb/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/build_test_sa2d/src/sta/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/build_test_sa2d/src/dbSta/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/build_test_sa2d/src/rsz/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/build_test_sa2d/src/stt/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/build_test_sa2d/src/gpl/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/build_test_sa2d/src/dpl/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/build_test_sa2d/src/exa/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/build_test_sa2d/src/fin/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/build_test_sa2d/src/ppl/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/build_test_sa2d/src/rmp/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/build_test_sa2d/src/cts/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/build_test_sa2d/src/grt/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/build_test_sa2d/src/tap/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/build_test_sa2d/src/rcx/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/build_test_sa2d/src/psm/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/build_test_sa2d/src/ant/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/build_test_sa2d/src/gui/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/build_test_sa2d/src/drt/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/build_test_sa2d/src/upf/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/build_test_sa2d/src/utl/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/build_test_sa2d/src/dst/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/build_test_sa2d/src/pdn/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/build_test_sa2d/src/dft/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/build_test_sa2d/src/mpl/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/build_test_sa2d/src/par/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/build_test_sa2d/src/sa1d/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/build_test_sa2d/src/sa2d/cmake_install.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openroad" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openroad")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openroad"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/fetzfs_projects/SAITPlacement/bodhi/SAIT-2D-Placer-v2/OpenROAD/build_test_sa2d/src/openroad")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openroad" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openroad")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/home/tool/binutils/binutils-2.27/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openroad")
    endif()
  endif()
endif()

