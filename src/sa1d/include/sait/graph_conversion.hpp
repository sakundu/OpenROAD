#pragma once

#include "hypergraph.hpp"
#include <vector>

/**
 * @brief Convert hypergraph to adjacency list representation using clique expansion
 * @param hg Input hypergraph
 * @return Adjacency list where adj_list[v] contains neighbors of vertex v
 */
std::vector<std::vector<int>> hypergraphToAdjacencyList(const Hypergraph& hg); 