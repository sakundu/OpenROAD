#!/bin/bash

# Copy missing SAIT files for best-orderings functionality
set -e

SAIT_SOURCE_DIR="/home/fetzfs_projects/TritonPart/bodhi/SAIT/src"
SA1D_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "=== Copying Missing SAIT Files for Best-Orderings ==="
echo "SAIT source: $SAIT_SOURCE_DIR"
echo "SA1D root: $SA1D_ROOT"

# Copy additional algorithm files
echo "Copying additional algorithm files..."

# Dirichlet ordering (major algorithm)
if [ -f "${SAIT_SOURCE_DIR}/dirichlet_ordering.hpp" ]; then
    cp "${SAIT_SOURCE_DIR}/dirichlet_ordering.hpp" "${SA1D_ROOT}/include/sait/"
    echo "✓ Copied dirichlet_ordering.hpp"
fi

if [ -f "${SAIT_SOURCE_DIR}/dirichlet_ordering.cpp" ]; then
    cp "${SAIT_SOURCE_DIR}/dirichlet_ordering.cpp" "${SA1D_ROOT}/src/sait/"
    echo "✓ Copied dirichlet_ordering.cpp"
fi

# Peak cutwidth ordering (major algorithm)
if [ -f "${SAIT_SOURCE_DIR}/peak_cutwidth_ordering.hpp" ]; then
    cp "${SAIT_SOURCE_DIR}/peak_cutwidth_ordering.hpp" "${SA1D_ROOT}/include/sait/"
    echo "✓ Copied peak_cutwidth_ordering.hpp"
fi

if [ -f "${SAIT_SOURCE_DIR}/peak_cutwidth_ordering.cpp" ]; then
    cp "${SAIT_SOURCE_DIR}/peak_cutwidth_ordering.cpp" "${SA1D_ROOT}/src/sait/"
    echo "✓ Copied peak_cutwidth_ordering.cpp"
fi

# SFC ordering (space-filling curves)
if [ -f "${SAIT_SOURCE_DIR}/sfc_ordering.hpp" ]; then
    cp "${SAIT_SOURCE_DIR}/sfc_ordering.hpp" "${SA1D_ROOT}/include/sait/"
    echo "✓ Copied sfc_ordering.hpp"
fi

if [ -f "${SAIT_SOURCE_DIR}/sfc_ordering.cpp" ]; then
    cp "${SAIT_SOURCE_DIR}/sfc_ordering.cpp" "${SA1D_ROOT}/src/sait/"
    echo "✓ Copied sfc_ordering.cpp"
fi

# BFS/DFS orderings
if [ -f "${SAIT_SOURCE_DIR}/bfs_ordering.hpp" ]; then
    cp "${SAIT_SOURCE_DIR}/bfs_ordering.hpp" "${SA1D_ROOT}/include/sait/"
    echo "✓ Copied bfs_ordering.hpp"
fi

if [ -f "${SAIT_SOURCE_DIR}/bfs_ordering.cpp" ]; then
    cp "${SAIT_SOURCE_DIR}/bfs_ordering.cpp" "${SA1D_ROOT}/src/sait/"
    echo "✓ Copied bfs_ordering.cpp"
fi

if [ -f "${SAIT_SOURCE_DIR}/dfs_ordering.hpp" ]; then
    cp "${SAIT_SOURCE_DIR}/dfs_ordering.hpp" "${SA1D_ROOT}/include/sait/"
    echo "✓ Copied dfs_ordering.hpp"
fi

if [ -f "${SAIT_SOURCE_DIR}/dfs_ordering.cpp" ]; then
    cp "${SAIT_SOURCE_DIR}/dfs_ordering.cpp" "${SA1D_ROOT}/src/sait/"
    echo "✓ Copied dfs_ordering.cpp"
fi

# Cutwidth analysis
if [ -f "${SAIT_SOURCE_DIR}/cutwidth_analysis.hpp" ]; then
    cp "${SAIT_SOURCE_DIR}/cutwidth_analysis.hpp" "${SA1D_ROOT}/include/sait/"
    echo "✓ Copied cutwidth_analysis.hpp"
fi

if [ -f "${SAIT_SOURCE_DIR}/cutwidth_analysis.cpp" ]; then
    cp "${SAIT_SOURCE_DIR}/cutwidth_analysis.cpp" "${SA1D_ROOT}/src/sait/"
    echo "✓ Copied cutwidth_analysis.cpp"
fi

# Update include paths in copied files
echo "Updating include paths..."
find "${SA1D_ROOT}/src/sait" -name "*.cpp" -exec sed -i 's/#include "\([^"]*\.hpp\)"/#include "sait\/\1"/g' {} \;
find "${SA1D_ROOT}/include/sait" -name "*.hpp" -exec sed -i 's/#include "\([^"]*\.hpp\)"/#include "sait\/\1"/g' {} \;

echo ""
echo "=== Additional SAIT files copied! ==="
echo "Next: Update CMakeLists.txt to include these files" 