#include "../../include/sait/rcm_ordering.hpp"
#include "../../include/sait/graph_conversion.hpp"
#include <queue>
#include <vector>
#include <iostream>
#include <algorithm>
#include <limits>
#include <chrono>

// Boost Graph Library includes
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/cuthill_mckee_ordering.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/bandwidth.hpp>

// Boost graph type definitions
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
                             boost::property<boost::vertex_color_t, boost::default_color_type,
                             boost::property<boost::vertex_degree_t, int> > > BoostGraph;
typedef boost::graph_traits<BoostGraph>::vertex_descriptor BoostVertex;
typedef boost::graph_traits<BoostGraph>::vertices_size_type BoostVerticesSizeType;

/**
 * Find a peripheral vertex (vertex with minimum eccentricity)
 * Uses multiple BFS traversals to find vertices far from each other
 */
int findPeripheralVertex(const std::vector<std::vector<int>>& adj_list) {
    int n = adj_list.size();
    if (n == 0) return 0;
    
    // Start with vertex 0
    int current_vertex = 0;
    int max_distance = 0;
    
    // Perform a few iterations to find a good peripheral vertex
    for (int iter = 0; iter < 3; ++iter) {
        std::vector<int> distance(n, -1);
        std::queue<int> bfs_queue;
        
        // BFS from current vertex
        bfs_queue.push(current_vertex);
        distance[current_vertex] = 0;
        
        int farthest_vertex = current_vertex;
        int farthest_distance = 0;
        
        while (!bfs_queue.empty()) {
            int v = bfs_queue.front();
            bfs_queue.pop();
            
            for (int neighbor : adj_list[v]) {
                if (distance[neighbor] == -1) {
                    distance[neighbor] = distance[v] + 1;
                    bfs_queue.push(neighbor);
                    
                    if (distance[neighbor] > farthest_distance) {
                        farthest_distance = distance[neighbor];
                        farthest_vertex = neighbor;
                    }
                }
            }
        }
        
        // Update for next iteration
        if (farthest_distance > max_distance) {
            max_distance = farthest_distance;
            current_vertex = farthest_vertex;
        } else {
            break; // No improvement, stop
        }
    }
    
    return current_vertex;
}

/**
 * Compute vertex degrees for sorting
 */
std::vector<int> computeVertexDegrees(const std::vector<std::vector<int>>& adj_list) {
    std::vector<int> degrees(adj_list.size());
    for (size_t i = 0; i < adj_list.size(); ++i) {
        degrees[i] = adj_list[i].size();
    }
    return degrees;
}

/**
 * Custom RCM ordering implementation
 */
std::vector<int> computeRCMOrdering(const Hypergraph& hg) {
    // std::cout << "Computing RCM (Reverse Cuthill-McKee) ordering using custom implementation..." << std::endl;
    
    // Convert hypergraph to adjacency list
    auto adj_list = hypergraphToAdjacencyList(hg);
    
    // Compute vertex degrees
    auto degrees = computeVertexDegrees(adj_list);
    
    std::vector<int> ordering;
    std::vector<bool> visited(hg.num_vertices, false);
    
    // Process each connected component
    for (int component_start = 0; component_start < hg.num_vertices; ++component_start) {
        if (visited[component_start]) continue;
        
        // Find peripheral vertex for this component
        int peripheral_vertex = findPeripheralVertex(adj_list);
        
        // If the peripheral vertex is already visited, find an unvisited one in this component
        if (visited[peripheral_vertex]) {
            // Find an unvisited vertex in this component using BFS
            std::queue<int> component_queue;
            component_queue.push(component_start);
            std::vector<bool> component_visited(hg.num_vertices, false);
            component_visited[component_start] = true;
            
            peripheral_vertex = component_start;
            while (!component_queue.empty()) {
                int v = component_queue.front();
                component_queue.pop();
                
                if (!visited[v]) {
                    peripheral_vertex = v;
                    break;
                }
                
                for (int neighbor : adj_list[v]) {
                    if (!component_visited[neighbor]) {
                        component_visited[neighbor] = true;
                        component_queue.push(neighbor);
                    }
                }
            }
        }
        
        // If this peripheral vertex is already processed, skip
        if (visited[peripheral_vertex]) continue;
        
        // Modified BFS with degree-based ordering (Cuthill-McKee)
        std::queue<int> cm_queue;
        cm_queue.push(peripheral_vertex);
        visited[peripheral_vertex] = true;
        
        while (!cm_queue.empty()) {
            int current = cm_queue.front();
            cm_queue.pop();
            ordering.push_back(current);
            
            // Collect unvisited neighbors
            std::vector<int> unvisited_neighbors;
            for (int neighbor : adj_list[current]) {
                if (!visited[neighbor]) {
                    unvisited_neighbors.push_back(neighbor);
                }
            }
            
            // Sort neighbors by degree (ascending order for CM)
            std::sort(unvisited_neighbors.begin(), unvisited_neighbors.end(),
                     [&degrees](int a, int b) {
                         if (degrees[a] != degrees[b]) {
                             return degrees[a] < degrees[b]; // Ascending degree order
                         }
                         return a < b; // Tie-breaking by vertex ID
                     });
            
            // Add sorted neighbors to queue
            for (int neighbor : unvisited_neighbors) {
                if (!visited[neighbor]) {
                    visited[neighbor] = true;
                    cm_queue.push(neighbor);
                }
            }
        }
    }
    
    // Handle isolated vertices
    for (int v = 0; v < hg.num_vertices; ++v) {
        if (!visited[v]) {
            ordering.push_back(v);
        }
    }
    
    // Reverse the ordering (Cuthill-McKee → Reverse Cuthill-McKee)
    std::reverse(ordering.begin(), ordering.end());
    
    // std::cout << "Custom RCM ordering complete. Vertices processed: " << ordering.size() << std::endl;
    
    if (static_cast<int>(ordering.size()) != hg.num_vertices) {
        std::cerr << "Warning: RCM ordering size mismatch. Expected: " 
                  << hg.num_vertices << ", Got: " << ordering.size() << std::endl;
    }
    
    return ordering;
}

/**
 * Boost Graph Library RCM implementation (simplified and robust)
 */
std::vector<int> computeRCMOrderingBoost(const Hypergraph& hg) {
    // std::cout << "Computing RCM ordering using Boost Graph Library..." << std::endl;
    
    // Convert hypergraph to adjacency list
    auto adj_list = hypergraphToAdjacencyList(hg);
    
    // Check for edge cases
    if (hg.num_vertices <= 1) {
        std::vector<int> ordering(hg.num_vertices);
        for (int i = 0; i < hg.num_vertices; ++i) {
            ordering[i] = i;
        }
        // std::cout << "Boost RCM ordering complete (trivial case). Vertices processed: " << ordering.size() << std::endl;
        return ordering;
    }
    
    try {
        // Create Boost graph
        BoostGraph boost_graph(hg.num_vertices);
        
        // Add edges to Boost graph
        int edge_count = 0;
        for (int v = 0; v < hg.num_vertices; ++v) {
            for (int neighbor : adj_list[v]) {
                if (v < neighbor) { // Add each edge only once
                    boost::add_edge(v, neighbor, boost_graph);
                    edge_count++;
                }
            }
        }
        
        // std::cout << "Created Boost graph with " << hg.num_vertices 
        //           << " vertices and " << edge_count << " edges" << std::endl;
        
        // Check if graph has any edges
        if (edge_count == 0) {
            std::cout << "Graph has no edges, using trivial ordering" << std::endl;
            std::vector<int> ordering(hg.num_vertices);
            for (int i = 0; i < hg.num_vertices; ++i) {
                ordering[i] = i;
            }
            return ordering;
        }
        
        // Compute Cuthill-McKee ordering
        std::vector<BoostVerticesSizeType> cm_ordering;
        cm_ordering.reserve(hg.num_vertices);
        
        // Create property maps for the algorithm
        typedef boost::property_map<BoostGraph, boost::vertex_index_t>::type IndexMap;
        IndexMap index_map = boost::get(boost::vertex_index, boost_graph);
        
        typedef boost::iterator_property_map<std::vector<boost::default_color_type>::iterator, IndexMap> ColorMap;
        std::vector<boost::default_color_type> colors(hg.num_vertices);
        ColorMap color_map(colors.begin(), index_map);
        
        // Use Boost's Cuthill-McKee ordering with proper error checking
        try {
            boost::cuthill_mckee_ordering(boost_graph, 
                                         std::back_inserter(cm_ordering),
                                         color_map,
                                         boost::make_degree_map(boost_graph));
            
            // Convert to our format and reverse for RCM
            std::vector<int> ordering;
            ordering.reserve(cm_ordering.size());
            
            // Reverse the CM ordering to get RCM
            for (auto it = cm_ordering.rbegin(); it != cm_ordering.rend(); ++it) {
                ordering.push_back(static_cast<int>(*it));
            }
            
            // Handle any vertices not included in the ordering (isolated vertices)
            if (static_cast<int>(ordering.size()) < hg.num_vertices) {
                std::vector<bool> included(hg.num_vertices, false);
                for (int v : ordering) {
                    included[v] = true;
                }
                
                for (int v = 0; v < hg.num_vertices; ++v) {
                    if (!included[v]) {
                        ordering.push_back(v);
                    }
                }
            }
            
            // std::cout << "Boost RCM ordering complete. Vertices processed: " << ordering.size() << std::endl;
            return ordering;
            
        } catch (const std::exception& e) {
            std::cerr << "Error in Boost cuthill_mckee_ordering: " << e.what() << std::endl;
            throw;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error in Boost RCM computation: " << e.what() << std::endl;
        std::cerr << "Falling back to custom implementation..." << std::endl;
        return computeRCMOrdering(hg);
    }
}

/**
 * Calculate bandwidth of an ordering
 */
int calculateBandwidth(const std::vector<int>& ordering, const std::vector<std::vector<int>>& adj_list) {
    int max_bandwidth = 0;
    std::vector<int> position(adj_list.size());
    
    // Create position mapping
    for (size_t i = 0; i < ordering.size(); ++i) {
        position[ordering[i]] = i;
    }
    
    // Calculate maximum bandwidth
    for (int v = 0; v < static_cast<int>(adj_list.size()); ++v) {
        for (int neighbor : adj_list[v]) {
            int bandwidth = std::abs(position[v] - position[neighbor]);
            max_bandwidth = std::max(max_bandwidth, bandwidth);
        }
    }
    
    return max_bandwidth;
}

/**
 * Compare custom and Boost RCM implementations
 */
std::pair<std::vector<int>, std::vector<int>> compareRCMImplementations(const Hypergraph& hg) {
    std::cout << "\n=== Comparing RCM Implementations ===\n";
    
    auto adj_list = hypergraphToAdjacencyList(hg);
    
    // Time custom implementation
    auto start_time = std::chrono::high_resolution_clock::now();
    auto custom_ordering = computeRCMOrdering(hg);
    auto custom_time = std::chrono::high_resolution_clock::now();
    auto custom_duration = std::chrono::duration_cast<std::chrono::milliseconds>(custom_time - start_time);
    
    std::cout << std::endl;
    
    // Time Boost implementation
    start_time = std::chrono::high_resolution_clock::now();
    auto boost_ordering = computeRCMOrderingBoost(hg);
    auto boost_time = std::chrono::high_resolution_clock::now();
    auto boost_duration = std::chrono::duration_cast<std::chrono::milliseconds>(boost_time - start_time);
    
    // Calculate bandwidths
    int custom_bandwidth = calculateBandwidth(custom_ordering, adj_list);
    int boost_bandwidth = calculateBandwidth(boost_ordering, adj_list);
    
    // Compare results
    std::cout << "\n=== Performance Comparison ===\n";
    std::cout << "Custom RCM time: " << custom_duration.count() << " ms\n";
    std::cout << "Boost RCM time:  " << boost_duration.count() << " ms\n";
    std::cout << "Speedup: " << static_cast<double>(custom_duration.count()) / boost_duration.count() << "x\n\n";
    
    std::cout << "=== Bandwidth Comparison ===\n";
    std::cout << "Custom RCM bandwidth: " << custom_bandwidth << "\n";
    std::cout << "Boost RCM bandwidth:  " << boost_bandwidth << "\n";
    
    if (custom_bandwidth < boost_bandwidth) {
        std::cout << "Custom implementation achieved better bandwidth reduction!\n";
    } else if (boost_bandwidth < custom_bandwidth) {
        std::cout << "Boost implementation achieved better bandwidth reduction!\n";
    } else {
        std::cout << "Both implementations achieved the same bandwidth!\n";
    }
    
    // Check if orderings are identical
    bool identical = (custom_ordering.size() == boost_ordering.size());
    if (identical) {
        for (size_t i = 0; i < custom_ordering.size(); ++i) {
            if (custom_ordering[i] != boost_ordering[i]) {
                identical = false;
                break;
            }
        }
    }
    
    if (identical) {
        std::cout << "Orderings are identical!\n";
    } else {
        std::cout << "Orderings are different (expected for heuristic algorithms)\n";
    }
    
    std::cout << "==========================================\n\n";
    
    return std::make_pair(custom_ordering, boost_ordering);
} 