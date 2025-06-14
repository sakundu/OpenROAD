#include "../../include/sait/hypergraph.hpp"
#include <algorithm>
#include <unordered_set>

void Hypergraph::addHyperedge(const std::vector<int>& vertices) {
    if (hyperedges.size() >= static_cast<size_t>(num_hyperedges)) {
        hyperedges.resize(num_hyperedges + 1);
        num_hyperedges++;
    }
    
    int hyperedge_id = hyperedges.size() - 1;
    hyperedges[hyperedge_id] = vertices;
    
    // Update vertex-to-hyperedges mapping
    for (int vertex : vertices) {
        if (vertex >= 0 && vertex < num_vertices) {
            vertex_to_hyperedges[vertex].push_back(hyperedge_id);
        }
    }
}

void Hypergraph::buildVertexToHyperedgeMapping() {
    // Clear existing mapping
    for (auto& vec : vertex_to_hyperedges) {
        vec.clear();
    }
    
    // Rebuild mapping
    for (int he = 0; he < num_hyperedges; ++he) {
        for (int vertex : hyperedges[he]) {
            if (vertex >= 0 && vertex < num_vertices) {
                vertex_to_hyperedges[vertex].push_back(he);
            }
        }
    }
}

int Hypergraph::getVertexDegree(int vertex) const {
    if (vertex < 0 || vertex >= num_vertices) {
        return 0;
    }
    return vertex_to_hyperedges[vertex].size();
}

int Hypergraph::getHyperedgeSize(int hyperedge) const {
    if (hyperedge < 0 || hyperedge >= num_hyperedges) {
        return 0;
    }
    return hyperedges[hyperedge].size();
}

void Hypergraph::printStats() const {
    std::cout << "Hypergraph Statistics:\n";
    std::cout << "  Vertices: " << num_vertices << "\n";
    std::cout << "  Hyperedges: " << num_hyperedges << "\n";
    
    if (num_hyperedges > 0) {
        // Calculate average hyperedge size
        int total_size = 0;
        int min_size = hyperedges[0].size();
        int max_size = hyperedges[0].size();
        
        for (const auto& he : hyperedges) {
            total_size += he.size();
            min_size = std::min(min_size, static_cast<int>(he.size()));
            max_size = std::max(max_size, static_cast<int>(he.size()));
        }
        
        std::cout << "  Average hyperedge size: " << static_cast<double>(total_size) / num_hyperedges << "\n";
        std::cout << "  Min hyperedge size: " << min_size << "\n";
        std::cout << "  Max hyperedge size: " << max_size << "\n";
    }
    
    if (num_vertices > 0) {
        // Calculate average vertex degree
        int total_degree = 0;
        int min_degree = getVertexDegree(0);
        int max_degree = getVertexDegree(0);
        
        for (int v = 0; v < num_vertices; ++v) {
            int degree = getVertexDegree(v);
            total_degree += degree;
            min_degree = std::min(min_degree, degree);
            max_degree = std::max(max_degree, degree);
        }
        
        std::cout << "  Average vertex degree: " << static_cast<double>(total_degree) / num_vertices << "\n";
        std::cout << "  Min vertex degree: " << min_degree << "\n";
        std::cout << "  Max vertex degree: " << max_degree << "\n";
    }
}

bool Hypergraph::isValid() const {
    // Check basic dimensions
    if (num_vertices <= 0 || num_hyperedges <= 0) {
        return false;
    }
    
    if (static_cast<int>(hyperedges.size()) != num_hyperedges) {
        return false;
    }
    
    if (static_cast<int>(vertex_to_hyperedges.size()) != num_vertices) {
        return false;
    }
    
    // Check that all vertices in hyperedges are valid
    for (const auto& he : hyperedges) {
        for (int vertex : he) {
            if (vertex < 0 || vertex >= num_vertices) {
                return false;
            }
        }
    }
    
    // Check consistency of vertex-to-hyperedges mapping
    for (int v = 0; v < num_vertices; ++v) {
        for (int he : vertex_to_hyperedges[v]) {
            if (he < 0 || he >= num_hyperedges) {
                return false;
            }
            
            // Check that vertex v is actually in hyperedge he
            const auto& vertices_in_he = hyperedges[he];
            if (std::find(vertices_in_he.begin(), vertices_in_he.end(), v) == vertices_in_he.end()) {
                return false;
            }
        }
    }
    
    return true;
} 