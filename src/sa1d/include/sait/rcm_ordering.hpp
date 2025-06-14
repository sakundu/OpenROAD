#pragma once

#include "hypergraph.hpp"
#include <vector>

/**
 * @brief Compute Reverse Cuthill-McKee ordering using custom implementation
 * @param hg Input hypergraph
 * @return Vector of vertex IDs in RCM ordering
 */
std::vector<int> computeRCMOrdering(const Hypergraph& hg);

/**
 * @brief Compute Reverse Cuthill-McKee ordering using Boost Graph Library
 * @param hg Input hypergraph  
 * @return Vector of vertex IDs in RCM ordering
 */
std::vector<int> computeRCMOrderingBoost(const Hypergraph& hg);

/**
 * @brief Compare custom and Boost RCM implementations
 * @param hg Input hypergraph
 * @return Pair of orderings (custom, boost) for comparison
 */
std::pair<std::vector<int>, std::vector<int>> compareRCMImplementations(const Hypergraph& hg); 