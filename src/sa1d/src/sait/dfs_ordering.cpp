#include "sait/dfs_ordering.hpp"
#include "sait/graph_conversion.hpp"
#include <vector>
#include <iostream>
#include <algorithm>

// Recursive DFS helper function
void dfsVisit(int vertex, const std::vector<std::vector<int>>& adj_list, 
              std::vector<bool>& visited, std::vector<int>& ordering) {
    visited[vertex] = true;
    ordering.push_back(vertex);
    
    // Visit neighbors in sorted order for consistent results
    std::vector<int> neighbors = adj_list[vertex];
    std::sort(neighbors.begin(), neighbors.end());
    
    for (int neighbor : neighbors) {
        if (!visited[neighbor]) {
            dfsVisit(neighbor, adj_list, visited, ordering);
        }
    }
}

std::vector<int> computeDFSOrdering(const Hypergraph& hg, int start_vertex) {
    // std::cout << "Computing DFS ordering..." << std::endl;
    
    // Convert hypergraph to adjacency list
    auto adj_list = hypergraphToAdjacencyList(hg);
    
    std::vector<int> ordering;
    std::vector<bool> visited(hg.num_vertices, false);
    
    // Choose starting vertex if not specified
    if (start_vertex == -1) {
        // Find vertex with minimum degree as starting point
        int min_degree = hg.num_vertices + 1;
        start_vertex = 0;
        
        for (int v = 0; v < hg.num_vertices; ++v) {
            int degree = adj_list[v].size();
            if (degree < min_degree) {
                min_degree = degree;
                start_vertex = v;
            }
        }
        
        // std::cout << "Selected starting vertex: " << start_vertex << " (degree: " << adj_list[start_vertex].size() << ")" << std::endl;
    }
    
    // Validate starting vertex
    if (start_vertex < 0 || start_vertex >= hg.num_vertices) {
        std::cerr << "Warning: Invalid start vertex " << start_vertex 
                  << ", using vertex 0" << std::endl;
        start_vertex = 0;
    }
    
    // DFS from each connected component
    for (int component_start = 0; component_start < hg.num_vertices; ++component_start) {
        // Use the specified start vertex for the first component
        int current_start = (component_start == 0) ? start_vertex : component_start;
        
        // Skip if already visited
        if (visited[current_start]) continue;
        
        // Perform DFS from this component
        dfsVisit(current_start, adj_list, visited, ordering);
    }
    
    // Handle isolated vertices (vertices with no edges)
    for (int v = 0; v < hg.num_vertices; ++v) {
        if (!visited[v]) {
            ordering.push_back(v);
        }
    }
    
    // std::cout << "DFS ordering complete. Vertices processed: " << ordering.size() << std::endl;
    
    if (static_cast<int>(ordering.size()) != hg.num_vertices) {
        std::cerr << "Warning: DFS ordering size mismatch. Expected: " 
                  << hg.num_vertices << ", Got: " << ordering.size() << std::endl;
    }
    
    return ordering;
} 