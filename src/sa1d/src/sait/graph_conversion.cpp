#include "../../include/sait/graph_conversion.hpp"
#include <unordered_set>
#include <iostream>
#include <omp.h>
#include <algorithm>

std::vector<std::vector<int>> hypergraphToAdjacencyList(const Hypergraph& hg) {
    // std::cout << "Converting hypergraph to adjacency list using clique expansion..." << std::endl;
    
    // Initialize adjacency list
    std::vector<std::vector<int>> adj_list(hg.num_vertices);
    
    // Use sets to avoid duplicate edges
    std::vector<std::unordered_set<int>> adj_sets(hg.num_vertices);
    
    int total_edges = 0;
    
    // For each hyperedge, create a clique (all vertices connected to each other)
    // Use thread-local storage to avoid race conditions
    std::vector<std::vector<std::unordered_set<int>>> thread_adj_sets(omp_get_max_threads(), 
                                                                     std::vector<std::unordered_set<int>>(hg.num_vertices));
    
    #pragma omp parallel reduction(+:total_edges)
    {
        int thread_id = omp_get_thread_num();
        auto& local_adj_sets = thread_adj_sets[thread_id];
        
        #pragma omp for
        for (int he = 0; he < hg.num_hyperedges; ++he) {
            const auto& vertices = hg.hyperedges[he];
            
            // Skip hyperedges with fewer than 2 vertices
            if (vertices.size() < 2) {
                continue;
            }
            
            // Connect every pair of vertices in this hyperedge
            for (size_t i = 0; i < vertices.size(); ++i) {
                for (size_t j = i + 1; j < vertices.size(); ++j) {
                    int v1 = vertices[i];
                    int v2 = vertices[j];
                    
                    // Validate vertex indices
                    if (v1 < 0 || v1 >= hg.num_vertices || v2 < 0 || v2 >= hg.num_vertices) {
                        continue;
                    }
                    
                    // Add edge in both directions (undirected graph)
                    if (local_adj_sets[v1].find(v2) == local_adj_sets[v1].end()) {
                        local_adj_sets[v1].insert(v2);
                        local_adj_sets[v2].insert(v1);
                        total_edges++;
                    }
                }
            }
        }
    }
    
    // Merge thread-local adjacency sets
    // std::cout << "Merging thread-local adjacency sets..." << std::endl;
    for (int t = 0; t < omp_get_max_threads(); ++t) {
        for (int v = 0; v < hg.num_vertices; ++v) {
            adj_sets[v].insert(thread_adj_sets[t][v].begin(), thread_adj_sets[t][v].end());
        }
    }
    
    // Convert sets to vectors for final adjacency list in parallel
    #pragma omp parallel for
    for (int v = 0; v < hg.num_vertices; ++v) {
        adj_list[v].reserve(adj_sets[v].size());
        for (int neighbor : adj_sets[v]) {
            adj_list[v].push_back(neighbor);
        }
        
        // Sort neighbors for consistent ordering
        std::sort(adj_list[v].begin(), adj_list[v].end());
    }
    
    // std::cout << "Graph conversion complete: " << total_edges << " edges created" << std::endl;
    
    return adj_list;
} 