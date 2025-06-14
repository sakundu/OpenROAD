#include "../../include/sait/random_ordering.hpp"
#include <algorithm>
#include <random>
#include <vector>
#include <iostream>

std::vector<int> computeRandomOrdering(const Hypergraph& hg) {
    // std::cout << "Computing random ordering..." << std::endl;
    std::vector<int> ordering(hg.num_vertices);
    std::iota(ordering.begin(), ordering.end(), 0);
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(ordering.begin(), ordering.end(), g);
    return ordering;
} 