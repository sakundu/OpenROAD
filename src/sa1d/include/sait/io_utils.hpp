#pragma once

#include "hypergraph.hpp"
#include <string>

/**
 * @brief Read a hypergraph from hMetis format file
 * @param filename Path to the hMetis file
 * @return Hypergraph object
 */
Hypergraph readHMetisFile(const std::string& filename);

/**
 * @brief Write vertex ordering to file
 * @param ordering Vector of vertex IDs in the desired order
 * @param filename Output file path
 */
void writeOrdering(const std::vector<int>& ordering, const std::string& filename);

/**
 * @brief Read vertex ordering from file
 * @param filename Input file path containing vertex ordering
 * @return Vector of vertex IDs in order (0-indexed)
 */
std::vector<int> readOrdering(const std::string& filename); 