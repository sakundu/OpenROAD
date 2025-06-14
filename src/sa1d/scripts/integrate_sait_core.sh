#!/bin/bash

# SA1D + SAIT Core Integration Script
set -e

SAIT_SOURCE_DIR="/home/fetzfs_projects/TritonPart/bodhi/SAIT/src"
SA1D_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "=== SA1D + SAIT Integration Script ==="
echo "SAIT source: $SAIT_SOURCE_DIR"
echo "SA1D root: $SA1D_ROOT"

# Validate source directory exists
if [ ! -d "$SAIT_SOURCE_DIR" ]; then
    echo "Error: SAIT source directory not found: $SAIT_SOURCE_DIR"
    exit 1
fi

# Create SAIT directories in SA1D
echo "Creating SAIT integration directories..."
mkdir -p "${SA1D_ROOT}/src/sait"
mkdir -p "${SA1D_ROOT}/include/sait"

# Copy core SAIT files (start with essential ones)
echo "Copying core SAIT files..."

# Core data structures
if [ -f "${SAIT_SOURCE_DIR}/hypergraph.hpp" ]; then
    cp "${SAIT_SOURCE_DIR}/hypergraph.hpp" "${SA1D_ROOT}/include/sait/"
    echo "✓ Copied hypergraph.hpp"
fi

if [ -f "${SAIT_SOURCE_DIR}/hypergraph.cpp" ]; then
    cp "${SAIT_SOURCE_DIR}/hypergraph.cpp" "${SA1D_ROOT}/src/sait/"
    echo "✓ Copied hypergraph.cpp"
fi

# Basic algorithms  
if [ -f "${SAIT_SOURCE_DIR}/fiedler_ordering.hpp" ]; then
    cp "${SAIT_SOURCE_DIR}/fiedler_ordering.hpp" "${SA1D_ROOT}/include/sait/"
    echo "✓ Copied fiedler_ordering.hpp"
fi

if [ -f "${SAIT_SOURCE_DIR}/fiedler_ordering.cpp" ]; then
    cp "${SAIT_SOURCE_DIR}/fiedler_ordering.cpp" "${SA1D_ROOT}/src/sait/"
    echo "✓ Copied fiedler_ordering.cpp"
fi

if [ -f "${SAIT_SOURCE_DIR}/rcm_ordering.hpp" ]; then
    cp "${SAIT_SOURCE_DIR}/rcm_ordering.hpp" "${SA1D_ROOT}/include/sait/"
    echo "✓ Copied rcm_ordering.hpp"
fi

if [ -f "${SAIT_SOURCE_DIR}/rcm_ordering.cpp" ]; then
    cp "${SAIT_SOURCE_DIR}/rcm_ordering.cpp" "${SA1D_ROOT}/src/sait/"
    echo "✓ Copied rcm_ordering.cpp"
fi

# Support files
if [ -f "${SAIT_SOURCE_DIR}/graph_conversion.hpp" ]; then
    cp "${SAIT_SOURCE_DIR}/graph_conversion.hpp" "${SA1D_ROOT}/include/sait/"
    echo "✓ Copied graph_conversion.hpp"
fi

if [ -f "${SAIT_SOURCE_DIR}/graph_conversion.cpp" ]; then
    cp "${SAIT_SOURCE_DIR}/graph_conversion.cpp" "${SA1D_ROOT}/src/sait/"
    echo "✓ Copied graph_conversion.cpp"
fi

if [ -f "${SAIT_SOURCE_DIR}/random_ordering.hpp" ]; then
    cp "${SAIT_SOURCE_DIR}/random_ordering.hpp" "${SA1D_ROOT}/include/sait/"
    echo "✓ Copied random_ordering.hpp"
fi

if [ -f "${SAIT_SOURCE_DIR}/random_ordering.cpp" ]; then
    cp "${SAIT_SOURCE_DIR}/random_ordering.cpp" "${SA1D_ROOT}/src/sait/"
    echo "✓ Copied random_ordering.cpp"
fi

# IO utilities
if [ -f "${SAIT_SOURCE_DIR}/io_utils.hpp" ]; then
    cp "${SAIT_SOURCE_DIR}/io_utils.hpp" "${SA1D_ROOT}/include/sait/"
    echo "✓ Copied io_utils.hpp"
fi

if [ -f "${SAIT_SOURCE_DIR}/io_utils.cpp" ]; then
    cp "${SAIT_SOURCE_DIR}/io_utils.cpp" "${SA1D_ROOT}/src/sait/"
    echo "✓ Copied io_utils.cpp"
fi

echo "Core files copied successfully"

# Update include paths in copied files
echo "Updating include paths..."
find "${SA1D_ROOT}/src/sait" -name "*.cpp" -exec sed -i 's/#include "\([^"]*\.hpp\)"/#include "sait\/\1"/g' {} \;
find "${SA1D_ROOT}/include/sait" -name "*.hpp" -exec sed -i 's/#include "\([^"]*\.hpp\)"/#include "sait\/\1"/g' {} \;

echo "Include paths updated"

# List what was actually copied
echo ""
echo "=== Integration Summary ==="
echo "SAIT headers copied:"
ls -la "${SA1D_ROOT}/include/sait/" 2>/dev/null || echo "No headers directory"

echo ""
echo "SAIT sources copied:"
ls -la "${SA1D_ROOT}/src/sait/" 2>/dev/null || echo "No sources directory"

echo ""
echo "=== SAIT core integration complete! ==="
echo "Next steps:"
echo "1. Update CMakeLists.txt to include SAIT sources"
echo "2. Implement VertexOrdering interface"
echo "3. Test compilation" 