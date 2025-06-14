#pragma once

#include <vector>
#include <string>
#include <iostream>

/**
 * @brief Hypergraph data structure for storing hypergraphs in memory
 */
struct Hypergraph {
    int num_vertices;
    int num_hyperedges;
    
    // hyperedges[i] = list of vertices in hyperedge i (0-indexed)
    std::vector<std::vector<int>> hyperedges;
    
    // vertex_to_hyperedges[v] = list of hyperedges containing vertex v
    std::vector<std::vector<int>> vertex_to_hyperedges;
    
    /**
     * @brief Default constructor
     */
    Hypergraph() : num_vertices(0), num_hyperedges(0) {}
    
    /**
     * @brief Constructor with dimensions
     */
    Hypergraph(int vertices, int hyperedges) 
        : num_vertices(vertices), num_hyperedges(hyperedges) {
        this->hyperedges.resize(hyperedges);
        vertex_to_hyperedges.resize(vertices);
    }
    
    /**
     * @brief Add a hyperedge to the hypergraph
     * @param vertices List of vertices in the hyperedge (0-indexed)
     */
    void addHyperedge(const std::vector<int>& vertices);
    
    /**
     * @brief Build the vertex-to-hyperedges mapping
     */
    void buildVertexToHyperedgeMapping();
    
    /**
     * @brief Get degree of a vertex (number of hyperedges it belongs to)
     */
    int getVertexDegree(int vertex) const;
    
    /**
     * @brief Get size of a hyperedge (number of vertices in it)
     */
    int getHyperedgeSize(int hyperedge) const;
    
    /**
     * @brief Print basic statistics about the hypergraph
     */
    void printStats() const;
    
    /**
     * @brief Validate the hypergraph structure
     */
    bool isValid() const;
}; 