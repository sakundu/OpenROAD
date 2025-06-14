#pragma once

#include "sait/hypergraph.hpp"
#include <vector>

/**
 * @brief Compute DFS ordering
 * @param hg Input hypergraph
 * @param start_vertex Starting vertex (-1 for automatic selection)
 * @return Vector of vertex IDs in DFS ordering
 */
std::vector<int> computeDFSOrdering(const Hypergraph& hg, int start_vertex = -1); 