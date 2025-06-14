#include "sait/sfc_ordering.hpp"
#include "sait/graph_conversion.hpp"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <numeric>
#include <random>
#include <omp.h>

// Utility functions for space-filling curves
namespace {

// Morton encoding for Z-order curve
uint64_t morton2D(uint32_t x, uint32_t y) {
    auto expandBits = [](uint32_t v) -> uint64_t {
        uint64_t x = v;
        x = (x | (x << 16)) & 0x0000FFFF0000FFFF;
        x = (x | (x << 8))  & 0x00FF00FF00FF00FF;
        x = (x | (x << 4))  & 0x0F0F0F0F0F0F0F0F;
        x = (x | (x << 2))  & 0x3333333333333333;
        x = (x | (x << 1))  & 0x5555555555555555;
        return x;
    };
    return expandBits(x) | (expandBits(y) << 1);
}

uint64_t morton3D(uint32_t x, uint32_t y, uint32_t z) {
    auto expandBits = [](uint32_t v) -> uint64_t {
        uint64_t x = v & 0x1fffff; // Only use lower 21 bits
        x = (x | x << 32) & 0x1f00000000ffff;
        x = (x | x << 16) & 0x1f0000ff0000ff;
        x = (x | x << 8)  & 0x100f00f00f00f00f;
        x = (x | x << 4)  & 0x10c30c30c30c30c3;
        x = (x | x << 2)  & 0x1249249249249249;
        return x;
    };
    return expandBits(x) | (expandBits(y) << 1) | (expandBits(z) << 2);
}

// Hilbert curve implementation (simplified)
uint64_t hilbert2D(uint32_t x, uint32_t y, int order) {
    uint64_t d = 0;
    uint32_t n = 1U << order;
    
    for (uint32_t s = n / 2; s > 0; s /= 2) {
        uint32_t rx = (x & s) > 0;
        uint32_t ry = (y & s) > 0;
        d += s * s * ((3 * rx) ^ ry);
        
        // Rotate
        if (ry == 0) {
            if (rx == 1) {
                x = n - 1 - x;
                y = n - 1 - y;
            }
            std::swap(x, y);
        }
    }
    return d;
}

// Simplified 3D Hilbert curve using recursive approach
uint64_t hilbert3D(uint32_t x, uint32_t y, uint32_t z, int order) {
    // For 3D Hilbert, we use a simpler approximation by combining 2D Hilbert with Z coordinate
    uint64_t h2d = hilbert2D(x, y, order);
    return h2d + (static_cast<uint64_t>(z) << (2 * order));
}

uint32_t coordinateToInt(double coord, int order) {
    uint32_t max_val = (1U << order) - 1;
    if (coord <= 0.0) return 0;
    if (coord >= 1.0) return max_val;
    return static_cast<uint32_t>(coord * max_val);
}

// Connected components using DFS
std::vector<std::vector<int>> findConnectedComponents(const std::vector<std::vector<int>>& adjacency_list) {
    int num_vertices = adjacency_list.size();
    std::vector<bool> visited(num_vertices, false);
    std::vector<std::vector<int>> components;
    
    for (int start = 0; start < num_vertices; ++start) {
        if (!visited[start]) {
            std::vector<int> component;
            std::vector<int> stack = {start};
            
            while (!stack.empty()) {
                int v = stack.back();
                stack.pop_back();
                
                if (!visited[v]) {
                    visited[v] = true;
                    component.push_back(v);
                    
                    for (int neighbor : adjacency_list[v]) {
                        if (!visited[neighbor]) {
                            stack.push_back(neighbor);
                        }
                    }
                }
            }
            
            if (!component.empty()) {
                std::sort(component.begin(), component.end());
                components.push_back(component);
            }
        }
    }
    
    return components;
}

// Extract subgraph Laplacian for a given set of vertices
typedef Eigen::SparseMatrix<double> SpMat;
typedef Eigen::Triplet<double> Triplet;

SpMat extractSubgraphLaplacian(const SpMat& full_laplacian, const std::vector<int>& vertices) {
    int sub_size = vertices.size();
    std::unordered_map<int, int> vertex_map;
    for (int i = 0; i < sub_size; ++i) {
        vertex_map[vertices[i]] = i;
    }
    
    std::vector<Triplet> triplets;
    
    for (int k = 0; k < full_laplacian.outerSize(); ++k) {
        for (SpMat::InnerIterator it(full_laplacian, k); it; ++it) {
            int row = it.row();
            int col = it.col();
            
            auto row_it = vertex_map.find(row);
            auto col_it = vertex_map.find(col);
            
            if (row_it != vertex_map.end() && col_it != vertex_map.end()) {
                triplets.emplace_back(row_it->second, col_it->second, it.value());
            }
        }
    }
    
    SpMat sub_laplacian(sub_size, sub_size);
    sub_laplacian.setFromTriplets(triplets.begin(), triplets.end());
    return sub_laplacian;
}

// Deflated power iteration to compute multiple eigenvectors
std::pair<double, Eigen::VectorXd> computeDeflatedEigenvector(
    const SpMat& laplacian, 
    const std::vector<Eigen::VectorXd>& previous_eigenvectors,
    int max_iterations = 100,
    double tolerance = 1e-6) {
    
    int n = laplacian.rows();
    
    // Initialize random vector
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> dist(0.0, 1.0);
    
    Eigen::VectorXd v(n);
    for (int i = 0; i < n; ++i) {
        v[i] = dist(gen);
    }
    v.normalize();
    
    // Deflate against previous eigenvectors
    for (const auto& prev : previous_eigenvectors) {
        v -= v.dot(prev) * prev;
    }
    if (v.norm() < 1e-12) {
        // Generate a new random vector
        for (int i = 0; i < n; ++i) {
            v[i] = dist(gen);
        }
    }
    v.normalize();
    
    // Use shift-and-invert for better convergence to small eigenvalues
    double shift = 0.1; // Shift to focus on small eigenvalues
    
    // Create shifted matrix: A_shifted = L + shift * I
    SpMat shifted_laplacian = laplacian;
    for (int i = 0; i < n; ++i) {
        shifted_laplacian.coeffRef(i, i) += shift;
    }
    
    // Set up sparse LU solver for shifted system
    Eigen::SparseLU<SpMat> solver;
    solver.compute(shifted_laplacian);
    
    if (solver.info() != Eigen::Success) {
        // Fallback to simple power iteration without shift
        std::cout << "    Warning: LU factorization failed, using simple power iteration" << std::endl;
        
        double eigenvalue = 0.0;
        for (int iter = 0; iter < max_iterations; ++iter) {
            // Apply Laplacian
            Eigen::VectorXd Lv = laplacian * v;
            
            // Deflate against previous eigenvectors
            for (const auto& prev : previous_eigenvectors) {
                double proj = Lv.dot(prev);
                Lv -= proj * prev;
            }
            
            // Normalize
            double norm = Lv.norm();
            if (norm < 1e-12) {
                break;
            }
            Lv /= norm;
            
            // Check convergence
            double new_eigenvalue = v.dot(laplacian * Lv);
            if (iter > 0 && std::abs(new_eigenvalue - eigenvalue) < tolerance) {
                eigenvalue = new_eigenvalue;
                v = Lv;
                break;
            }
            
            eigenvalue = new_eigenvalue;
            v = Lv;
        }
        
        return std::make_pair(eigenvalue, v);
    }
    
    double eigenvalue = 0.0;
    
    for (int iter = 0; iter < max_iterations; ++iter) {
        // Solve (L + shift*I) * u = v for u
        Eigen::VectorXd u = solver.solve(v);
        
        if (solver.info() != Eigen::Success) {
            std::cout << "    Warning: Linear solve failed at iteration " << iter << std::endl;
            break;
        }
        
        // Deflate against previous eigenvectors
        for (const auto& prev : previous_eigenvectors) {
            double dot_product = u.dot(prev);
            #pragma omp parallel for
            for (int i = 0; i < u.size(); ++i) {
                u[i] -= dot_product * prev[i];
            }
        }
        
        // Normalize
        double norm = u.norm();
        if (norm < 1e-12) {
            // Restart with random vector
            for (int i = 0; i < n; ++i) {
                u[i] = dist(gen);
            }
            u.normalize();
            continue;
        }
        u /= norm;
        
        // Compute Rayleigh quotient for original matrix
        Eigen::VectorXd Lu = laplacian * u;
        double new_eigenvalue = u.dot(Lu);
        
        // Check convergence
        if (iter > 0 && std::abs(new_eigenvalue - eigenvalue) < tolerance) {
            eigenvalue = new_eigenvalue;
            v = u;
            break;
        }
        
        eigenvalue = new_eigenvalue;
        v = u;
    }
    
    // Final deflation and normalization
    for (const auto& prev : previous_eigenvectors) {
        double dot_product = v.dot(prev);
        #pragma omp parallel for
        for (int i = 0; i < v.size(); ++i) {
            v[i] -= dot_product * prev[i];
        }
    }
    v.normalize();
    
    // Recompute eigenvalue
    Eigen::VectorXd Lv = laplacian * v;
    eigenvalue = v.dot(Lv);
    
    return std::make_pair(eigenvalue, v);
}

} // anonymous namespace

std::string sfcStrategyToString(SFCStrategy strategy) {
    switch (strategy) {
        case SFCStrategy::HILBERT_2D: return "Hilbert-2D";
        case SFCStrategy::HILBERT_3D: return "Hilbert-3D";
        case SFCStrategy::ZORDER_2D: return "Z-Order-2D";
        case SFCStrategy::ZORDER_3D: return "Z-Order-3D";
        case SFCStrategy::LEXICOGRAPHIC_2D: return "Lexicographic-2D";
        case SFCStrategy::LEXICOGRAPHIC_3D: return "Lexicographic-3D";
        default: return "Unknown";
    }
}

void normalizeCoordinates2D(std::vector<Point2D>& points) {
    if (points.empty()) return;
    
    double min_x = points[0].x, max_x = points[0].x;
    double min_y = points[0].y, max_y = points[0].y;
    
    // Parallel reduction to find min/max values
    #pragma omp parallel for reduction(min:min_x,min_y) reduction(max:max_x,max_y)
    for (size_t i = 0; i < points.size(); ++i) {
        const auto& p = points[i];
        min_x = std::min(min_x, p.x);
        max_x = std::max(max_x, p.x);
        min_y = std::min(min_y, p.y);
        max_y = std::max(max_y, p.y);
    }
    
    double range_x = max_x - min_x;
    double range_y = max_y - min_y;
    
    // Avoid division by zero
    if (range_x < 1e-10) range_x = 1.0;
    if (range_y < 1e-10) range_y = 1.0;
    
    // Parallel normalization
    #pragma omp parallel for
    for (size_t i = 0; i < points.size(); ++i) {
        points[i].x = (points[i].x - min_x) / range_x;
        points[i].y = (points[i].y - min_y) / range_y;
    }
}

void normalizeCoordinates3D(std::vector<Point3D>& points) {
    if (points.empty()) return;
    
    double min_x = points[0].x, max_x = points[0].x;
    double min_y = points[0].y, max_y = points[0].y;
    double min_z = points[0].z, max_z = points[0].z;
    
    // Parallel reduction to find min/max values
    #pragma omp parallel for reduction(min:min_x,min_y,min_z) reduction(max:max_x,max_y,max_z)
    for (size_t i = 0; i < points.size(); ++i) {
        const auto& p = points[i];
        min_x = std::min(min_x, p.x);
        max_x = std::max(max_x, p.x);
        min_y = std::min(min_y, p.y);
        max_y = std::max(max_y, p.y);
        min_z = std::min(min_z, p.z);
        max_z = std::max(max_z, p.z);
    }
    
    double range_x = max_x - min_x;
    double range_y = max_y - min_y;
    double range_z = max_z - min_z;
    
    // Avoid division by zero
    if (range_x < 1e-10) range_x = 1.0;
    if (range_y < 1e-10) range_y = 1.0;
    if (range_z < 1e-10) range_z = 1.0;
    
    // Parallel normalization
    #pragma omp parallel for
    for (size_t i = 0; i < points.size(); ++i) {
        points[i].x = (points[i].x - min_x) / range_x;
        points[i].y = (points[i].y - min_y) / range_y;
        points[i].z = (points[i].z - min_z) / range_z;
    }
}

std::vector<int> lexicographicOrder2D(const std::vector<Point2D>& points) {
    std::vector<int> indices(points.size());
    std::iota(indices.begin(), indices.end(), 0);
    
    std::sort(indices.begin(), indices.end(), [&points](int a, int b) {
        const auto& pa = points[a];
        const auto& pb = points[b];
        if (std::abs(pa.x - pb.x) > 1e-10) {
            return pa.x < pb.x;
        }
        return pa.y < pb.y;
    });
    
    std::vector<int> ordering(points.size());
    for (size_t i = 0; i < indices.size(); ++i) {
        ordering[i] = points[indices[i]].vertex_id;
    }
    return ordering;
}

std::vector<int> lexicographicOrder3D(const std::vector<Point3D>& points) {
    std::vector<int> indices(points.size());
    std::iota(indices.begin(), indices.end(), 0);
    
    std::sort(indices.begin(), indices.end(), [&points](int a, int b) {
        const auto& pa = points[a];
        const auto& pb = points[b];
        if (std::abs(pa.x - pb.x) > 1e-10) {
            return pa.x < pb.x;
        }
        if (std::abs(pa.y - pb.y) > 1e-10) {
            return pa.y < pb.y;
        }
        return pa.z < pb.z;
    });
    
    std::vector<int> ordering(points.size());
    for (size_t i = 0; i < indices.size(); ++i) {
        ordering[i] = points[indices[i]].vertex_id;
    }
    return ordering;
}

std::vector<int> zOrder2D(const std::vector<Point2D>& points) {
    std::vector<std::pair<uint64_t, int>> morton_values;
    morton_values.resize(points.size());
    
    // Parallel computation of Morton codes
    #pragma omp parallel for
    for (size_t i = 0; i < points.size(); ++i) {
        const auto& p = points[i];
        uint32_t x = coordinateToInt(p.x, 16);
        uint32_t y = coordinateToInt(p.y, 16);
        uint64_t morton = morton2D(x, y);
        morton_values[i] = std::make_pair(morton, p.vertex_id);
    }
    
    std::sort(morton_values.begin(), morton_values.end());
    
    std::vector<int> ordering;
    ordering.reserve(points.size());
    for (const auto& mv : morton_values) {
        ordering.push_back(mv.second);
    }
    return ordering;
}

std::vector<int> zOrder3D(const std::vector<Point3D>& points) {
    std::vector<std::pair<uint64_t, int>> morton_values;
    morton_values.resize(points.size());
    
    // Parallel computation of Morton codes
    #pragma omp parallel for
    for (size_t i = 0; i < points.size(); ++i) {
        const auto& p = points[i];
        uint32_t x = coordinateToInt(p.x, 16);
        uint32_t y = coordinateToInt(p.y, 16);
        uint32_t z = coordinateToInt(p.z, 16);
        uint64_t morton = morton3D(x, y, z);
        morton_values[i] = std::make_pair(morton, p.vertex_id);
    }
    
    std::sort(morton_values.begin(), morton_values.end());
    
    std::vector<int> ordering;
    ordering.reserve(points.size());
    for (const auto& mv : morton_values) {
        ordering.push_back(mv.second);
    }
    return ordering;
}

std::vector<int> hilbertOrder2D(const std::vector<Point2D>& points, int order) {
    std::vector<std::pair<uint64_t, int>> hilbert_values;
    hilbert_values.resize(points.size());
    
    // Parallel computation of Hilbert codes
    #pragma omp parallel for
    for (size_t i = 0; i < points.size(); ++i) {
        const auto& p = points[i];
        uint32_t x = coordinateToInt(p.x, order);
        uint32_t y = coordinateToInt(p.y, order);
        uint64_t hilbert = hilbert2D(x, y, order);
        hilbert_values[i] = std::make_pair(hilbert, p.vertex_id);
    }
    
    std::sort(hilbert_values.begin(), hilbert_values.end());
    
    std::vector<int> ordering;
    ordering.reserve(points.size());
    for (const auto& hv : hilbert_values) {
        ordering.push_back(hv.second);
    }
    return ordering;
}

std::vector<int> hilbertOrder3D(const std::vector<Point3D>& points, int order) {
    std::vector<std::pair<uint64_t, int>> hilbert_values;
    hilbert_values.resize(points.size());
    
    // Parallel computation of Hilbert codes
    #pragma omp parallel for
    for (size_t i = 0; i < points.size(); ++i) {
        const auto& p = points[i];
        uint32_t x = coordinateToInt(p.x, order);
        uint32_t y = coordinateToInt(p.y, order);
        uint32_t z = coordinateToInt(p.z, order);
        uint64_t hilbert = hilbert3D(x, y, z, order);
        hilbert_values[i] = std::make_pair(hilbert, p.vertex_id);
    }
    
    std::sort(hilbert_values.begin(), hilbert_values.end());
    
    std::vector<int> ordering;
    ordering.reserve(points.size());
    for (const auto& hv : hilbert_values) {
        ordering.push_back(hv.second);
    }
    return ordering;
}

SFCOrderingResult computeSpectralSFCOrdering(const Hypergraph& hg, const SFCOrderingParams& params) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // OpenMP diagnostics
    // std::cout << "OpenMP parallelization enabled with " << omp_get_num_threads() << " threads" << std::endl;

    // Validate input

    SFCOrderingResult result;
    result.strategy_used = params.sfc_strategy;
    result.dimensions_used = params.num_eigenvectors;
    
    // std::cout << "Computing Spectral + SFC ordering using " << sfcStrategyToString(params.sfc_strategy) << "..." << std::endl;
    
    try {
        // Step 1: Build 2-section graph (clique expansion)
        // std::cout << "Building 2-section graph using clique expansion..." << std::endl;
        auto adjacency_list = hypergraphToAdjacencyList(hg);
        
        int num_vertices = hg.num_vertices;
        // std::cout << "2-section graph built with " << num_vertices << " vertices" << std::endl;
        
        // Step 2: Build Laplacian matrix L = D - A
        // std::cout << "Constructing Laplacian matrix..." << std::endl;
        
        // Use sparse matrix for efficiency
        typedef Eigen::SparseMatrix<double> SpMat;
        typedef Eigen::Triplet<double> Triplet;
        
        std::vector<Triplet> triplets;
        std::vector<int> degrees(num_vertices, 0);
        
        // Count degrees and build adjacency triplets in parallel
        int total_edges = 0;
        
        // First pass: count degrees in parallel
        #pragma omp parallel for reduction(+:total_edges)
        for (int v = 0; v < num_vertices; ++v) {
            for (int neighbor : adjacency_list[v]) {
                if (neighbor > v) { // Avoid double counting
                    #pragma omp atomic
                    degrees[v]++;
                    #pragma omp atomic 
                    degrees[neighbor]++;
                    total_edges++;
                }
            }
        }
        
        // Reserve space for triplets
        triplets.reserve(2 * total_edges + num_vertices);
        
        // Second pass: build triplets (sequential to avoid race conditions)
        for (int v = 0; v < num_vertices; ++v) {
            for (int neighbor : adjacency_list[v]) {
                if (neighbor > v) { // Avoid double counting
                    triplets.emplace_back(v, neighbor, -1.0);
                    triplets.emplace_back(neighbor, v, -1.0);
                }
            }
        }
        
        // Add diagonal entries (degree matrix)
        for (int v = 0; v < num_vertices; ++v) {
            triplets.emplace_back(v, v, static_cast<double>(degrees[v]));
        }
        
        SpMat laplacian(num_vertices, num_vertices);
        laplacian.setFromTriplets(triplets.begin(), triplets.end());
        
        // std::cout << "Laplacian matrix constructed: " << num_vertices << "x" << num_vertices 
        //           << " with " << total_edges << " edges" << std::endl;
        
        // Step 3: Compute eigenvectors using advanced sparse solvers
        // std::cout << "Computing top " << params.num_eigenvectors << " non-trivial eigenvectors..." << std::endl;
        
        Eigen::VectorXd eigenvalues;
        Eigen::MatrixXd eigenvectors;
        
        // Choose solver strategy based on graph size and connectivity
        if (num_vertices <= 1000) {
            // Small graphs: use dense solver for reliability
            // std::cout << "Using dense eigenvalue solver for small graph..." << std::endl;
            Eigen::MatrixXd dense_laplacian = Eigen::MatrixXd(laplacian);
            Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(dense_laplacian);
            
            if (solver.info() != Eigen::Success) {
                throw std::runtime_error("Dense eigenvalue computation failed");
            }
            
            eigenvalues = solver.eigenvalues();
            eigenvectors = solver.eigenvectors();
            
        } else {
            // Large graphs: use optimized sparse iterative methods
            // std::cout << "Using sparse iterative solver with preconditioning..." << std::endl;
            
            // Find connected components for better numerical stability
            std::vector<std::vector<int>> components = findConnectedComponents(adjacency_list);
            // std::cout << "Found " << components.size() << " connected components" << std::endl;
            
            if (components.size() == 1 && static_cast<int>(components[0].size()) == num_vertices) {
                // Single large connected component: use power iteration with deflation
                eigenvalues.resize(num_vertices);
                eigenvectors.resize(num_vertices, num_vertices);
                
                // Initialize with zero eigenvalue and constant eigenvector
                eigenvalues[0] = 0.0;
                eigenvectors.col(0) = Eigen::VectorXd::Ones(num_vertices) / std::sqrt(num_vertices);
                
                // Compute subsequent eigenvectors using deflated power iteration
                std::vector<Eigen::VectorXd> computed_eigenvectors;
                computed_eigenvectors.push_back(eigenvectors.col(0));
                
                for (int k = 1; k <= params.num_eigenvectors; ++k) {
                    // std::cout << "  Computing eigenvector " << k << "..." << std::endl;
                    
                    auto result = computeDeflatedEigenvector(laplacian, computed_eigenvectors, 100, 1e-6);
                    eigenvalues[k] = result.first;
                    eigenvectors.col(k) = result.second;
                    computed_eigenvectors.push_back(result.second);
                    
                    // std::cout << "    Eigenvalue " << k << ": " << eigenvalues[k] << std::endl;
                }
                
            } else {
                // Multiple components: use dense solver on largest component
                // Find largest component
                auto largest_comp = *std::max_element(components.begin(), components.end(),
                    [](const std::vector<int>& a, const std::vector<int>& b) {
                        return a.size() < b.size();
                    });
                
                // std::cout << "Processing largest component with " << largest_comp.size() << " vertices..." << std::endl;
                
                // Extract subgraph Laplacian for largest component
                auto sub_laplacian = extractSubgraphLaplacian(laplacian, largest_comp);
                
                if (largest_comp.size() <= 1000) {
                    // Use dense solver on component
                    Eigen::MatrixXd dense_sub = Eigen::MatrixXd(sub_laplacian);
                    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(dense_sub);
                    
                    if (solver.info() != Eigen::Success) {
                        throw std::runtime_error("Component eigenvalue computation failed");
                    }
                    
                    // Map back to full graph
                    eigenvalues = Eigen::VectorXd::Zero(num_vertices);
                    eigenvectors = Eigen::MatrixXd::Zero(num_vertices, num_vertices);
                    
                    auto comp_eigenvalues = solver.eigenvalues();
                    auto comp_eigenvectors = solver.eigenvectors();
                    
                    for (int k = 0; k < std::min(static_cast<int>(largest_comp.size()), params.num_eigenvectors + 1); ++k) {
                        eigenvalues[k] = comp_eigenvalues[k];
                        for (size_t i = 0; i < largest_comp.size(); ++i) {
                            eigenvectors(largest_comp[i], k) = comp_eigenvectors(i, k);
                        }
                    }
                } else {
                    // Use power iteration on large component
                    eigenvalues.resize(num_vertices);
                    eigenvectors.resize(num_vertices, num_vertices);
                    eigenvectors.setZero();
                    
                    // Zero eigenvalue with constant eigenvector on component
                    eigenvalues[0] = 0.0;
                    for (int v : largest_comp) {
                        eigenvectors(v, 0) = 1.0 / std::sqrt(largest_comp.size());
                    }
                    
                    // Compute additional eigenvectors on component
                    std::vector<Eigen::VectorXd> comp_eigenvectors;
                    Eigen::VectorXd comp_constant = Eigen::VectorXd::Ones(largest_comp.size()) / std::sqrt(largest_comp.size());
                    comp_eigenvectors.push_back(comp_constant);
                    
                    for (int k = 1; k <= params.num_eigenvectors; ++k) {
                        auto result = computeDeflatedEigenvector(sub_laplacian, comp_eigenvectors, 100, 1e-6);
                        eigenvalues[k] = result.first;
                        
                        // Map back to full graph
                        for (size_t i = 0; i < largest_comp.size(); ++i) {
                            eigenvectors(largest_comp[i], k) = result.second[i];
                        }
                        comp_eigenvectors.push_back(result.second);
                    }
                }
            }
        }
        
        // Store eigenvalues for debugging
        result.eigenvalues.resize(eigenvalues.size());
        for (int i = 0; i < eigenvalues.size(); ++i) {
            result.eigenvalues[i] = eigenvalues[i];
        }
        
        // std::cout << "Eigenvalue computation complete. First few eigenvalues: ";
        // for (int i = 0; i < std::min(5, static_cast<int>(eigenvalues.size())); ++i) {
        //     std::cout << eigenvalues[i] << " ";
        // }
        // std::cout << std::endl;
        
        // Step 4: Embed vertices using selected eigenvectors (skip the first zero eigenvector)
        int start_eigenvector = 1; // Skip the trivial zero eigenvector
        
        if (params.num_eigenvectors == 2) {
            // std::cout << "Creating 2D embedding..." << std::endl;
            result.embedding_2d.resize(num_vertices);
            
            // Parallel embedding creation
            #pragma omp parallel for
            for (int v = 0; v < num_vertices; ++v) {
                double x = eigenvectors(v, start_eigenvector);
                double y = eigenvectors(v, start_eigenvector + 1);
                result.embedding_2d[v] = Point2D(x, y, v);
            }
            
            if (params.normalize_coordinates) {
                normalizeCoordinates2D(result.embedding_2d);
            }
            
        } else if (params.num_eigenvectors == 3) {
            // std::cout << "Creating 3D embedding..." << std::endl;
            result.embedding_3d.resize(num_vertices);
            
            // Parallel embedding creation
            #pragma omp parallel for
            for (int v = 0; v < num_vertices; ++v) {
                double x = eigenvectors(v, start_eigenvector);
                double y = eigenvectors(v, start_eigenvector + 1);
                double z = eigenvectors(v, start_eigenvector + 2);
                result.embedding_3d[v] = Point3D(x, y, z, v);
            }
            
            if (params.normalize_coordinates) {
                normalizeCoordinates3D(result.embedding_3d);
            }
        }
        
        // Step 5: Apply Space-Filling Curve ordering
        // std::cout << "Applying " << sfcStrategyToString(params.sfc_strategy) << " ordering..." << std::endl;
        
        switch (params.sfc_strategy) {
            case SFCStrategy::HILBERT_2D:
                result.ordering = hilbertOrder2D(result.embedding_2d, params.hilbert_order);
                break;
            case SFCStrategy::HILBERT_3D:
                result.ordering = hilbertOrder3D(result.embedding_3d, params.hilbert_order);
                break;
            case SFCStrategy::ZORDER_2D:
                result.ordering = zOrder2D(result.embedding_2d);
                break;
            case SFCStrategy::ZORDER_3D:
                result.ordering = zOrder3D(result.embedding_3d);
                break;
            case SFCStrategy::LEXICOGRAPHIC_2D:
                result.ordering = lexicographicOrder2D(result.embedding_2d);
                break;
            case SFCStrategy::LEXICOGRAPHIC_3D:
                result.ordering = lexicographicOrder3D(result.embedding_3d);
                break;
        }
        
        auto end_time = std::chrono::high_resolution_clock::now();
        result.computation_time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
        
        // std::cout << "Spectral + SFC ordering complete. Vertices processed: " << result.ordering.size() << std::endl;
        // std::cout << "Total computation time: " << result.computation_time_ms << " ms" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error in Spectral + SFC ordering computation: " << e.what() << std::endl;
        
        // Fallback: return identity ordering
        result.ordering.resize(hg.num_vertices);
        std::iota(result.ordering.begin(), result.ordering.end(), 0);
        
        auto end_time = std::chrono::high_resolution_clock::now();
        result.computation_time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    }
    
    return result;
}

std::vector<int> computeSFCOrdering(const Hypergraph& hg, SFCStrategy strategy) {
    SFCOrderingParams params;
    params.sfc_strategy = strategy;
    
    // Set appropriate number of eigenvectors based on strategy
    if (strategy == SFCStrategy::HILBERT_3D || strategy == SFCStrategy::ZORDER_3D || 
        strategy == SFCStrategy::LEXICOGRAPHIC_3D) {
        params.num_eigenvectors = 3;
    } else {
        params.num_eigenvectors = 2;
    }
    
    SFCOrderingResult result = computeSpectralSFCOrdering(hg, params);
    return result.ordering;
} 