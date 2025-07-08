# SA2D - Simulated Annealing 2D Placer

include("openroad")

swig_lib(NAME sa2d
  NAMESPACE sa2d
  LANGUAGE tcl
  I_FILE src/SA2D.i
  SCRIPTS src/SA2D.tcl
)

target_sources(sa2d
  PRIVATE
    src/MakeSA2D.cpp
    src/SA2D.cpp
    src/ThreadSafeGrid.cpp
    src/Worker.cpp
    src/WorkerManager.cpp
)

target_include_directories(sa2d
  PUBLIC
    include
  PRIVATE
    ${PROJECT_SOURCE_DIR}/src/dpl/src
    ${PROJECT_SOURCE_DIR}/src/dpl/src/infrastructure
)

target_link_libraries(sa2d
  PRIVATE
    odb
    utl_lib
    dpl_lib
)

messages(
  TARGET sa2d
  OUTPUT_DIR ..
)

# Unit tests
if (enable_tests)
  add_subdirectory(test)
endif() 