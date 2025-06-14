#include "../../include/sait/fiedler_ordering.hpp"
#include "../../include/sait/graph_conversion.hpp"
#include <Eigen/Sparse>
#include <Eigen/Eigenvalues>
#include <Eigen/SparseLU>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <numeric>

// For large matrices, we'll use iterative methods
#ifdef EIGEN_USE_SPECTRA
#include <Spectra/GenEigsSolver.h>
#include <Spectra/MatOp/SparseGenMatProd.h>
#endif

/**
 * Build the Laplacian matrix from adjacency list
 * L = D - A where D is degree matrix and A is adjacency matrix
 */
Eigen::SparseMatrix<double> buildLaplacianMatrix(const std::vector<std::vector<int>>& adj_list) {
    int n = adj_list.size();
    Eigen::SparseMatrix<double> laplacian(n, n);
    
    // Reserve space for efficient insertion
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(2 * n + 2 * std::accumulate(adj_list.begin(), adj_list.end(), 0,
                                                [](int sum, const std::vector<int>& neighbors) {
                                                    return sum + neighbors.size();
                                                }));
    
    // Build Laplacian: L = D - A
    for (int i = 0; i < n; ++i) {
        int degree = adj_list[i].size();
        
        // Diagonal entry: degree
        if (degree > 0) {
            triplets.emplace_back(i, i, static_cast<double>(degree));
        }
        
        // Off-diagonal entries: -1 for each edge
        for (int neighbor : adj_list[i]) {
            if (neighbor != i) { // Avoid self-loops
                triplets.emplace_back(i, neighbor, -1.0);
            }
        }
    }
    
    laplacian.setFromTriplets(triplets.begin(), triplets.end());
    return laplacian;
}

/**
 * Find connected components for handling disconnected graphs
 */
std::vector<std::vector<int>> findConnectedComponents(const std::vector<std::vector<int>>& adj_list) {
    int n = adj_list.size();
    std::vector<bool> visited(n, false);
    std::vector<std::vector<int>> components;
    
    for (int start = 0; start < n; ++start) {
        if (visited[start]) continue;
        
        // BFS to find component
        std::vector<int> component;
        std::queue<int> queue;
        queue.push(start);
        visited[start] = true;
        
        while (!queue.empty()) {
            int v = queue.front();
            queue.pop();
            component.push_back(v);
            
            for (int neighbor : adj_list[v]) {
                if (!visited[neighbor]) {
                    visited[neighbor] = true;
                    queue.push(neighbor);
                }
            }
        }
        
        components.push_back(component);
    }
    
    return components;
}

/**
 * Power iteration method for computing Fiedler vector (approximate but fast)
 */
Eigen::VectorXd computeFiedlerVectorPowerIteration(const Eigen::SparseMatrix<double>& laplacian) {
    int n = laplacian.rows();
    if (n <= 2) {
        Eigen::VectorXd fiedler(n);
        for (int i = 0; i < n; ++i) {
            fiedler(i) = static_cast<double>(i) - static_cast<double>(n-1) / 2.0;
        }
        return fiedler;
    }
    
    // Initialize random vector orthogonal to the constant vector
    Eigen::VectorXd v = Eigen::VectorXd::Random(n);
    
    // Make orthogonal to constant vector (nullspace of Laplacian)
    double mean = v.mean();
    v.array() -= mean;
    
    // Power iteration with shift to target second smallest eigenvalue
    // Use shift-and-invert: solve (L + sigma*I)^(-1) * v
    double sigma = 0.1; // Small shift to avoid the zero eigenvalue
    Eigen::SparseMatrix<double> shifted_laplacian = laplacian;
    for (int i = 0; i < n; ++i) {
        shifted_laplacian.coeffRef(i, i) += sigma;
    }
    
    // Use iterative solver
    Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
    solver.compute(shifted_laplacian);
    
    if (solver.info() != Eigen::Success) {
        std::cerr << "  Warning: LU decomposition failed, using simple approximation" << std::endl;
        // Fallback: use vertex coordinates as approximation
        for (int i = 0; i < n; ++i) {
            v(i) = static_cast<double>(i);
        }
        return v;
    }
    
    // Power iteration
    const int max_iterations = 50;
    const double tolerance = 1e-6;
    
    for (int iter = 0; iter < max_iterations; ++iter) {
        Eigen::VectorXd v_new = solver.solve(v);
        
        // Orthogonalize against constant vector
        double mean_new = v_new.mean();
        v_new.array() -= mean_new;
        
        // Normalize
        double norm = v_new.norm();
        if (norm > 1e-12) {
            v_new /= norm;
        }
        
        // Check convergence
        double change = (v_new - v).norm();
        v = v_new;
        
        if (change < tolerance) {
            std::cout << "  Power iteration converged in " << (iter + 1) << " iterations" << std::endl;
            break;
        }
    }
    
    return v;
}

/**
 * Compute Fiedler ordering for a single connected component with efficient methods
 */
std::vector<std::pair<double, int>> computeComponentFiedlerVector(
    const std::vector<int>& component,
    const std::vector<std::vector<int>>& adj_list) {
    
    int comp_size = component.size();
    if (comp_size <= 1) {
        // Single vertex component
        return {{0.0, component[0]}};
    }
    
    if (comp_size == 2) {
        // Two vertex component - arbitrary ordering
        return {{-1.0, component[0]}, {1.0, component[1]}};
    }
    
    // Create subgraph adjacency list
    std::unordered_map<int, int> vertex_to_index;
    for (int i = 0; i < comp_size; ++i) {
        vertex_to_index[component[i]] = i;
    }
    
    std::vector<std::vector<int>> sub_adj_list(comp_size);
    for (int i = 0; i < comp_size; ++i) {
        int original_vertex = component[i];
        for (int neighbor : adj_list[original_vertex]) {
            if (vertex_to_index.find(neighbor) != vertex_to_index.end()) {
                sub_adj_list[i].push_back(vertex_to_index[neighbor]);
            }
        }
    }
    
    // Build Laplacian for subgraph
    Eigen::SparseMatrix<double> laplacian = buildLaplacianMatrix(sub_adj_list);
    
    Eigen::VectorXd fiedler_vector;
    
    // Choose method based on component size
    if (comp_size <= 1000) {
        // For small components, use dense solver
        try {
            Eigen::MatrixXd dense_laplacian = laplacian.toDense();
            Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(dense_laplacian);
            
            if (solver.info() == Eigen::Success) {
                // Find the second smallest eigenvalue
                Eigen::VectorXd eigenvalues = solver.eigenvalues();
                Eigen::MatrixXd eigenvectors = solver.eigenvectors();
                
                int fiedler_index = 1; // Usually index 1 for connected graphs
                if (comp_size > 2) {
                    // Verify we have the correct Fiedler vector
                    for (int i = 1; i < std::min(3, comp_size); ++i) {
                        if (eigenvalues(i) > 1e-10) { // Skip near-zero eigenvalues
                            fiedler_index = i;
                            break;
                        }
                    }
                }
                
                fiedler_vector = eigenvectors.col(fiedler_index);
                std::cout << "  Component size: " << comp_size 
                          << ", Fiedler eigenvalue: " << eigenvalues(fiedler_index) 
                          << " (dense solver)" << std::endl;
            } else {
                throw std::runtime_error("Dense eigenvalue computation failed");
            }
        } catch (const std::exception& e) {
            std::cout << "  Dense solver failed, using power iteration: " << e.what() << std::endl;
            fiedler_vector = computeFiedlerVectorPowerIteration(laplacian);
        }
    } else {
        // For large components, use power iteration
        std::cout << "  Component size: " << comp_size << " (using power iteration)" << std::endl;
        fiedler_vector = computeFiedlerVectorPowerIteration(laplacian);
    }
    
    // Create result pairs (fiedler_value, original_vertex)
    std::vector<std::pair<double, int>> result;
    for (int i = 0; i < comp_size; ++i) {
        result.emplace_back(fiedler_vector(i), component[i]);
    }
    
    return result;
}

std::vector<int> computeFiedlerOrdering(const Hypergraph& hg) {
    // std::cout << "Computing Fiedler vector ordering (spectral ordering)..." << std::endl;
    
    // Convert hypergraph to adjacency list
    auto adj_list = hypergraphToAdjacencyList(hg);
    
    // Find connected components
    auto components = findConnectedComponents(adj_list);
    std::cout << "Found " << components.size() << " connected components" << std::endl;
    
    // Collect all (fiedler_value, vertex) pairs
    std::vector<std::pair<double, int>> all_fiedler_pairs;
    
    // Process each component separately
    for (size_t comp_idx = 0; comp_idx < components.size(); ++comp_idx) {
        const auto& component = components[comp_idx];
        std::cout << "Processing component " << comp_idx + 1 
                  << " with " << component.size() << " vertices..." << std::endl;
        
        auto comp_fiedler_pairs = computeComponentFiedlerVector(component, adj_list);
        
        // Add component offset for consistent ordering across components
        double component_offset = static_cast<double>(comp_idx) * 1000.0;
        for (auto& pair : comp_fiedler_pairs) {
            pair.first += component_offset;
        }
        
        all_fiedler_pairs.insert(all_fiedler_pairs.end(), 
                                comp_fiedler_pairs.begin(), 
                                comp_fiedler_pairs.end());
    }
    
    // Sort vertices by Fiedler vector values
    std::sort(all_fiedler_pairs.begin(), all_fiedler_pairs.end());
    
    // Extract the ordering
    std::vector<int> ordering;
    ordering.reserve(hg.num_vertices);
    for (const auto& pair : all_fiedler_pairs) {
        ordering.push_back(pair.second);
    }
    
    // std::cout << "Fiedler ordering complete. Vertices processed: " << ordering.size() << std::endl;
    
    if (static_cast<int>(ordering.size()) != hg.num_vertices) {
        std::cerr << "Warning: Fiedler ordering size mismatch. Expected: " 
                  << hg.num_vertices << ", Got: " << ordering.size() << std::endl;
    }
    
    return ordering;
} 