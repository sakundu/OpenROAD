#pragma once

#include "hypergraph.hpp"
#include <vector>

/**
 * @brief Compute Fiedler vector ordering (spectral ordering)
 * @param hg Input hypergraph
 * @return Vector of vertex IDs in Fiedler ordering
 */
std::vector<int> computeFiedlerOrdering(const Hypergraph& hg); 