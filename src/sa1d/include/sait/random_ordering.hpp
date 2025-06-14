#pragma once

#include "hypergraph.hpp"
#include <vector>

/**
 * @brief Compute random ordering
 * @param hg Input hypergraph
 * @return Vector of vertex IDs in random order
 */
std::vector<int> computeRandomOrdering(const Hypergraph& hg); 