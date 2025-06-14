#include "../../include/sait/io_utils.hpp"
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

Hypergraph readHMetisFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open file: " + filename);
    }
    
    std::string line;
    
    // Read the header line
    if (!std::getline(file, line)) {
        throw std::runtime_error("Cannot read header from file: " + filename);
    }
    
    std::istringstream header_stream(line);
    int num_hyperedges, num_vertices;
    
    if (!(header_stream >> num_hyperedges >> num_vertices)) {
        throw std::runtime_error("Invalid header format in file: " + filename);
    }
    
    std::cout << "Reading hypergraph: " << num_vertices << " vertices, " 
              << num_hyperedges << " hyperedges" << std::endl;
    
    // Create hypergraph
    Hypergraph hg(num_vertices, num_hyperedges);
    
    // Read hyperedges
    int hyperedge_id = 0;
    while (std::getline(file, line) && hyperedge_id < num_hyperedges) {
        if (line.empty()) continue; // Skip empty lines
        
        std::istringstream line_stream(line);
        std::vector<int> vertices;
        int vertex;
        
        // Parse vertices in this hyperedge (convert from 1-indexed to 0-indexed)
        while (line_stream >> vertex) {
            if (vertex < 1 || vertex > num_vertices) {
                throw std::runtime_error("Invalid vertex ID in hyperedge " + 
                                       std::to_string(hyperedge_id) + ": " + std::to_string(vertex));
            }
            vertices.push_back(vertex - 1); // Convert to 0-indexed
        }
        
        if (!vertices.empty()) {
            hg.hyperedges[hyperedge_id] = vertices;
            hyperedge_id++;
        }
    }
    
    if (hyperedge_id != num_hyperedges) {
        std::cerr << "Warning: Expected " << num_hyperedges << " hyperedges, but read " 
                  << hyperedge_id << std::endl;
        hg.num_hyperedges = hyperedge_id;
        hg.hyperedges.resize(hyperedge_id);
    }
    
    // Build vertex-to-hyperedges mapping
    hg.buildVertexToHyperedgeMapping();
    
    // Validate the hypergraph
    if (!hg.isValid()) {
        throw std::runtime_error("Invalid hypergraph structure after reading file: " + filename);
    }
    
    std::cout << "Successfully loaded hypergraph from " << filename << std::endl;
    
    return hg;
}

void writeOrdering(const std::vector<int>& ordering, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot create output file: " + filename);
    }
    
    // Write ordering in single column format (convert back to 1-indexed for output)
    for (size_t i = 0; i < ordering.size(); ++i) {
        file << (ordering[i] + 1) << std::endl; // Convert to 1-indexed, one per line
    }
    
    std::cout << "Ordering written to " << filename << std::endl;
}

std::vector<int> readOrdering(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open ordering file: " + filename);
    }
    
    std::vector<int> ordering;
    std::string line;
    
    // Read all lines and parse vertices (supports both single column and single row formats)
    while (std::getline(file, line)) {
        if (line.empty()) continue; // Skip empty lines
        
        std::istringstream line_stream(line);
        int vertex;
        
        // Parse vertices from the line (assume they are 1-indexed in file)
        while (line_stream >> vertex) {
            if (vertex < 1) {
                throw std::runtime_error("Invalid vertex ID in ordering file: " + std::to_string(vertex));
            }
            ordering.push_back(vertex - 1); // Convert to 0-indexed
        }
    }
    
    if (ordering.empty()) {
        throw std::runtime_error("No vertices found in ordering file: " + filename);
    }
    
    std::cout << "Read ordering with " << ordering.size() << " vertices from " << filename << std::endl;
    
    return ordering;
} 