#pragma once

#include "sait/hypergraph.hpp"
#include <vector>

/**
 * @brief Compute BFS ordering
 * @param hg Input hypergraph
 * @param start_vertex Starting vertex (-1 for automatic selection)
 * @return Vector of vertex IDs in BFS ordering
 */
std::vector<int> computeBFSOrdering(const Hypergraph& hg, int start_vertex = -1); 