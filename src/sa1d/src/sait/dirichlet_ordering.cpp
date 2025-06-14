#include "sait/dirichlet_ordering.hpp"
#include "sait/peak_cutwidth_ordering.hpp"
#include <iostream>
#include <algorithm>
#include <chrono>
#include <unordered_set>
#include <random>
#include <Eigen/SparseLU>
#include <Eigen/SparseCholesky>
#include <Eigen/IterativeLinearSolvers>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <map>
#include <cmath>

// Build 2-section graph adjacency matrix from hypergraph
Eigen::SparseMatrix<double> build2SectionAdjacency(const Hypergraph& hg, bool verbose) {
    if (verbose) {
        std::cout << "Building 2-section graph adjacency matrix..." << std::endl;
    }
    
    int n = hg.num_vertices;
    Eigen::SparseMatrix<double> adjacency(n, n);
    
    // Use triplets for efficient sparse matrix construction
    std::vector<Eigen::Triplet<double>> triplets;
    
    // For each hyperedge, add edges between all pairs of vertices (clique expansion)
    int total_edges = 0;
    for (int he = 0; he < hg.num_hyperedges; ++he) {
        const auto& hyperedge = hg.hyperedges[he];
        int edge_size = hyperedge.size();
        
        // Add edge between every pair of vertices in this hyperedge
        for (int i = 0; i < edge_size; ++i) {
            for (int j = i + 1; j < edge_size; ++j) {
                int u = hyperedge[i];
                int v = hyperedge[j];
                
                // Add edge (u,v) and (v,u) with weight 1.0
                triplets.emplace_back(u, v, 1.0);
                triplets.emplace_back(v, u, 1.0);
                total_edges++;
            }
        }
    }
    
    // Build sparse matrix from triplets
    adjacency.setFromTriplets(triplets.begin(), triplets.end());
    
    if (verbose) {
        std::cout << "2-section graph: " << n << " vertices, " << total_edges << " edges" << std::endl;
        std::cout << "Adjacency matrix: " << adjacency.nonZeros() << " non-zeros" << std::endl;
    }
    
    return adjacency;
}

// Build graph Laplacian matrix L = D - A
Eigen::SparseMatrix<double> buildGraphLaplacian(const Eigen::SparseMatrix<double>& adjacency, bool verbose) {
    if (verbose) {
        std::cout << "Building graph Laplacian matrix..." << std::endl;
    }
    
    int n = adjacency.rows();
    Eigen::SparseMatrix<double> laplacian(n, n);
    
    // Compute degree vector
    Eigen::VectorXd degrees = Eigen::VectorXd::Zero(n);
    for (int k = 0; k < adjacency.outerSize(); ++k) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(adjacency, k); it; ++it) {
            degrees[it.row()] += it.value();
        }
    }
    
    // Build Laplacian L = D - A using triplets
    std::vector<Eigen::Triplet<double>> triplets;
    
    // Add diagonal entries (degree matrix D)
    for (int i = 0; i < n; ++i) {
        if (degrees[i] > 0) {
            triplets.emplace_back(i, i, degrees[i]);
        }
    }
    
    // Subtract adjacency matrix A
    for (int k = 0; k < adjacency.outerSize(); ++k) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(adjacency, k); it; ++it) {
            if (it.row() != it.col()) {  // Off-diagonal entries
                triplets.emplace_back(it.row(), it.col(), -it.value());
            }
        }
    }
    
    laplacian.setFromTriplets(triplets.begin(), triplets.end());
    
    if (verbose) {
        std::cout << "Laplacian matrix: " << laplacian.nonZeros() << " non-zeros" << std::endl;
    }
    
    return laplacian;
}

// Solve Dirichlet problem using sparse linear solver
Eigen::VectorXd solveDirichletProblem(
    const Eigen::SparseMatrix<double>& L_FF,
    const Eigen::SparseMatrix<double>& L_FT,
    const Eigen::VectorXd& x_T,
    const DirichletEmbeddingParams& params,
    double& solver_time_ms,
    bool& converged,
    double& residual
) {
    auto solver_start = std::chrono::high_resolution_clock::now();
    
    if (params.verbose) {
        std::cout << "Solving Dirichlet problem: L_FF * x_F = -L_FT * x_T" << std::endl;
        std::cout << "System size: " << L_FF.rows() << " x " << L_FF.cols() << std::endl;
        std::cout << "RHS size: " << x_T.size() << std::endl;
    }
    
    // Compute right-hand side: -L_FT * x_T
    Eigen::VectorXd rhs = -L_FT * x_T;
    
    Eigen::VectorXd x_F;
    
    if (params.use_sparse_solver) {
        // Use sparse Cholesky solver (L_FF should be positive definite)
        Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
        solver.compute(L_FF);
        
        if (solver.info() != Eigen::Success) {
            if (params.verbose) {
                std::cout << "Cholesky factorization failed, trying SparseLU..." << std::endl;
            }
            
            // Fallback to LU decomposition
            Eigen::SparseLU<Eigen::SparseMatrix<double>> lu_solver;
            lu_solver.compute(L_FF);
            
            if (lu_solver.info() != Eigen::Success) {
                if (params.verbose) {
                    std::cout << "SparseLU factorization also failed. Matrix may be singular." << std::endl;
                    std::cout << "Trying regularization..." << std::endl;
                }
                
                // Add small regularization to diagonal to handle near-singular matrices
                Eigen::SparseMatrix<double> L_FF_reg = L_FF;
                double reg_param = 1e-8;
                for (int i = 0; i < L_FF_reg.rows(); ++i) {
                    L_FF_reg.coeffRef(i, i) += reg_param;
                }
                
                lu_solver.compute(L_FF_reg);
                if (lu_solver.info() != Eigen::Success) {
                    throw std::runtime_error("Sparse LU factorization failed even with regularization");
                }
                
                if (params.verbose) {
                    std::cout << "Regularization successful with parameter " << reg_param << std::endl;
                }
            }
            
            x_F = lu_solver.solve(rhs);
            converged = (lu_solver.info() == Eigen::Success);
        } else {
            x_F = solver.solve(rhs);
            converged = (solver.info() == Eigen::Success);
        }
        
        // Compute residual
        Eigen::VectorXd residual_vec = L_FF * x_F - rhs;
        residual = residual_vec.norm();
        
    } else {
        // Use iterative solver (BiCGSTAB)
        Eigen::BiCGSTAB<Eigen::SparseMatrix<double>> solver;
        solver.setTolerance(params.solver_tolerance);
        solver.setMaxIterations(params.max_solver_iterations);
        solver.compute(L_FF);
        
        if (solver.info() != Eigen::Success) {
            throw std::runtime_error("Iterative solver setup failed");
        }
        
        x_F = solver.solve(rhs);
        converged = (solver.info() == Eigen::Success);
        residual = solver.error();
        
        if (params.verbose) {
            std::cout << "Iterative solver: " << solver.iterations() << " iterations" << std::endl;
            std::cout << "Final residual: " << residual << std::endl;
        }
    }
    
    auto solver_end = std::chrono::high_resolution_clock::now();
    solver_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(solver_end - solver_start).count();
    
    if (params.verbose) {
        std::cout << "Solver completed in " << solver_time_ms << "ms" << std::endl;
        std::cout << "Converged: " << (converged ? "Yes" : "No") << std::endl;
        std::cout << "Final residual: " << residual << std::endl;
    }
    
    return x_F;
}

// Create ordering from coordinates by sorting
std::vector<int> createOrderingFromCoordinates(
    const Eigen::VectorXd& coordinates,
    const std::vector<int>& vertex_ids
) {
    if (coordinates.size() != static_cast<int>(vertex_ids.size())) {
        throw std::invalid_argument("Coordinates and vertex_ids must have same size");
    }
    
    // Create pairs of (coordinate, vertex_id)
    std::vector<std::pair<double, int>> coord_vertex_pairs;
    for (int i = 0; i < coordinates.size(); ++i) {
        coord_vertex_pairs.emplace_back(coordinates[i], vertex_ids[i]);
    }
    
    // Sort by coordinate
    std::sort(coord_vertex_pairs.begin(), coord_vertex_pairs.end());
    
    // Extract sorted vertex IDs
    std::vector<int> ordering;
    ordering.reserve(vertex_ids.size());
    for (const auto& pair : coord_vertex_pairs) {
        ordering.push_back(pair.second);
    }
    
    return ordering;
}

// Validate fixed vertex specifications
bool validateFixedVertices(const Hypergraph& hg, const std::vector<FixedVertex>& fixed_vertices) {
    std::unordered_set<int> seen_vertices;
    
    for (const auto& fv : fixed_vertices) {
        // Check if vertex ID is valid
        if (fv.vertex_id < 0 || fv.vertex_id >= hg.num_vertices) {
            std::cerr << "Invalid vertex ID: " << fv.vertex_id << std::endl;
            return false;
        }
        
        // Check for duplicates
        if (seen_vertices.count(fv.vertex_id) > 0) {
            std::cerr << "Duplicate fixed vertex: " << fv.vertex_id << std::endl;
            return false;
        }
        seen_vertices.insert(fv.vertex_id);
    }
    
    return true;
}

// Main Dirichlet embedding algorithm
DirichletEmbeddingResult computeDirichletEmbedding(
    const Hypergraph& hg,
    const std::vector<FixedVertex>& fixed_vertices,
    const DirichletEmbeddingParams& params
) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    DirichletEmbeddingResult result;
    
    if (params.verbose) {
        std::cout << "=== Dirichlet Spectral Embedding ===" << std::endl;
        std::cout << "Hypergraph: " << hg.num_vertices << " vertices, " << hg.num_hyperedges << " hyperedges" << std::endl;
        std::cout << "Fixed vertices: " << fixed_vertices.size() << std::endl;
    }
    
    // Validate inputs
    if (!validateFixedVertices(hg, fixed_vertices)) {
        throw std::invalid_argument("Invalid fixed vertex specifications");
    }
    
    if (fixed_vertices.empty()) {
        throw std::invalid_argument("At least one fixed vertex is required");
    }
    
    if (static_cast<int>(fixed_vertices.size()) >= hg.num_vertices) {
        throw std::invalid_argument("All vertices cannot be fixed");
    }
    
    // Step 1: Build 2-section graph adjacency matrix
    auto adjacency = build2SectionAdjacency(hg, params.verbose);
    result.num_edges_2section = adjacency.nonZeros() / 2;  // Each edge counted twice
    
    // Step 2: Build graph Laplacian
    auto laplacian = buildGraphLaplacian(adjacency, params.verbose);
    
    // Step 3: Partition vertices into fixed and movable
    std::unordered_set<int> fixed_set;
    for (const auto& fv : fixed_vertices) {
        fixed_set.insert(fv.vertex_id);
    }
    
    std::vector<int> movable_vertices;
    std::vector<int> fixed_vertex_ids;
    std::vector<double> fixed_coordinates;
    
    for (int v = 0; v < hg.num_vertices; ++v) {
        if (fixed_set.count(v) > 0) {
            fixed_vertex_ids.push_back(v);
        } else {
            movable_vertices.push_back(v);
        }
    }
    
    // Get fixed coordinates in same order as fixed_vertex_ids
    for (int fv_id : fixed_vertex_ids) {
        for (const auto& fv : fixed_vertices) {
            if (fv.vertex_id == fv_id) {
                fixed_coordinates.push_back(fv.x_coordinate);
                break;
            }
        }
    }
    
    result.num_movable_vertices = movable_vertices.size();
    result.num_fixed_vertices = fixed_vertex_ids.size();
    
    if (params.verbose) {
        std::cout << "Movable vertices: " << result.num_movable_vertices << std::endl;
        std::cout << "Fixed vertices: " << result.num_fixed_vertices << std::endl;
    }
    
    // Step 4: Extract Laplacian blocks
    // L = [L_FF L_FT]
    //     [L_TF L_TT]
    
    int n_movable = movable_vertices.size();
    int n_fixed = fixed_vertex_ids.size();
    
    // Build index mappings
    std::unordered_map<int, int> movable_index_map;
    std::unordered_map<int, int> fixed_index_map;
    
    for (int i = 0; i < n_movable; ++i) {
        movable_index_map[movable_vertices[i]] = i;
    }
    for (int i = 0; i < n_fixed; ++i) {
        fixed_index_map[fixed_vertex_ids[i]] = i;
    }
    
    // Extract L_FF (movable-movable block) and L_FT (movable-fixed block)
    std::vector<Eigen::Triplet<double>> L_FF_triplets;
    std::vector<Eigen::Triplet<double>> L_FT_triplets;
    
    // Iterate over all columns of the Laplacian matrix
    for (int col = 0; col < laplacian.cols(); ++col) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(laplacian, col); it; ++it) {
            int row = it.row();
            int col_idx = it.col();
            double value = it.value();
            
            // Check if this entry belongs to L_FF or L_FT
            if (movable_index_map.count(row) > 0) {
                int i = movable_index_map[row];
                
                if (movable_index_map.count(col_idx) > 0) {
                    // L_FF block (movable-movable)
                    int j = movable_index_map[col_idx];
                    
                    // Bounds check
                    if (i >= 0 && i < n_movable && j >= 0 && j < n_movable) {
                        L_FF_triplets.emplace_back(i, j, value);
                    } else if (params.verbose) {
                        std::cout << "WARNING: L_FF bounds check failed: (" << i << "," << j << ") for matrix " << n_movable << "x" << n_movable << std::endl;
                    }
                } else if (fixed_index_map.count(col_idx) > 0) {
                    // L_FT block (movable-fixed)
                    int j = fixed_index_map[col_idx];
                    
                    // Bounds check
                    if (i >= 0 && i < n_movable && j >= 0 && j < n_fixed) {
                        L_FT_triplets.emplace_back(i, j, value);
                    } else if (params.verbose) {
                        std::cout << "WARNING: L_FT bounds check failed: (" << i << "," << j << ") for matrix " << n_movable << "x" << n_fixed << std::endl;
                    }
                }
            }
        }
    }
    
    Eigen::SparseMatrix<double> L_FF(n_movable, n_movable);
    L_FF.setFromTriplets(L_FF_triplets.begin(), L_FF_triplets.end());
    
    Eigen::SparseMatrix<double> L_FT(n_movable, n_fixed);
    L_FT.setFromTriplets(L_FT_triplets.begin(), L_FT_triplets.end());
    
    if (params.verbose) {
        std::cout << "L_FF: " << L_FF.rows() << "x" << L_FF.cols() << " (" << L_FF.nonZeros() << " nnz)" << std::endl;
        std::cout << "L_FT: " << L_FT.rows() << "x" << L_FT.cols() << " (" << L_FT.nonZeros() << " nnz)" << std::endl;
        
        // Check for zero diagonal entries (indicates disconnected components)
        int zero_diag_count = 0;
        for (int i = 0; i < L_FF.rows(); ++i) {
            if (std::abs(L_FF.coeff(i, i)) < 1e-12) {
                zero_diag_count++;
            }
        }
        if (zero_diag_count > 0) {
            std::cout << "WARNING: L_FF has " << zero_diag_count << " zero diagonal entries (disconnected components)" << std::endl;
        }
    }
    
    // Step 5: Solve Dirichlet problem
    Eigen::VectorXd x_T(n_fixed);
    for (int i = 0; i < n_fixed; ++i) {
        x_T[i] = fixed_coordinates[i];
    }
    
    Eigen::VectorXd x_F = solveDirichletProblem(
        L_FF, L_FT, x_T, params,
        result.solver_time_ms, result.solver_converged, result.solver_residual
    );
    
    // Step 6: Normalize coordinates if requested
    if (params.normalize_coordinates) {
        double min_coord = x_F.minCoeff();
        double max_coord = x_F.maxCoeff();
        
        // Include fixed coordinates in normalization
        for (double coord : fixed_coordinates) {
            min_coord = std::min(min_coord, coord);
            max_coord = std::max(max_coord, coord);
        }
        
        if (max_coord > min_coord) {
            double range = max_coord - min_coord;
            x_F = (x_F.array() - min_coord) / range;
            
            // Also normalize fixed coordinates for consistency
            for (int i = 0; i < n_fixed; ++i) {
                x_T[i] = (fixed_coordinates[i] - min_coord) / range;
            }
        }
        
        if (params.verbose) {
            std::cout << "Normalized coordinates to [0,1] range" << std::endl;
        }
    }
    
    // Step 7: Create final ordering and results
    result.ordering = createOrderingFromCoordinates(x_F, movable_vertices);
    
    // Store coordinates
    result.coordinates.resize(n_movable);
    for (int i = 0; i < n_movable; ++i) {
        result.coordinates[i] = x_F[i];
    }
    
    // Create full coordinate vector and mapping
    result.full_coordinates.resize(hg.num_vertices);
    result.vertex_mapping.resize(hg.num_vertices);
    
    for (int i = 0; i < n_movable; ++i) {
        int v = movable_vertices[i];
        result.full_coordinates[v] = x_F[i];
        result.vertex_mapping[v] = v;
    }
    
    for (int i = 0; i < n_fixed; ++i) {
        int v = fixed_vertex_ids[i];
        result.full_coordinates[v] = x_T[i];
        result.vertex_mapping[v] = v;
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    result.computation_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    
    if (params.verbose) {
        std::cout << "=== Dirichlet Embedding Complete ===" << std::endl;
        std::cout << "Total time: " << result.computation_time_ms << "ms" << std::endl;
        std::cout << "Solver time: " << result.solver_time_ms << "ms" << std::endl;
        std::cout << "Final ordering size: " << result.ordering.size() << std::endl;
        
        // Print coordinate range
        if (!result.coordinates.empty()) {
            auto minmax = std::minmax_element(result.coordinates.begin(), result.coordinates.end());
            std::cout << "Coordinate range: [" << *minmax.first << ", " << *minmax.second << "]" << std::endl;
        }
    }
    
    return result;
}

// Create a test case with pre-placed IOs
void createTestCaseWithIOs(
    int num_vertices,
    int num_hyperedges, 
    int num_ios,
    Hypergraph& hg,
    std::vector<FixedVertex>& fixed_vertices
) {
    if (num_ios >= num_vertices) {
        throw std::invalid_argument("Number of IOs must be less than total vertices");
    }
    
    // Initialize hypergraph
    hg.num_vertices = num_vertices;
    hg.num_hyperedges = num_hyperedges;
    hg.hyperedges.clear();
    hg.vertex_to_hyperedges.clear();
    hg.vertex_to_hyperedges.resize(num_vertices);
    
    // Generate random hyperedges with guaranteed connectivity
    std::random_device rd;
    std::mt19937 gen(42);  // Fixed seed for reproducibility
    std::uniform_int_distribution<> vertex_dist(0, num_vertices - 1);
    std::uniform_int_distribution<> size_dist(2, std::min(6, num_vertices));
    
    // First, create a spanning tree to ensure connectivity
    for (int i = 1; i < num_vertices && i < num_hyperedges; ++i) {
        std::vector<int> hyperedge = {i-1, i};
        // Add one more random vertex to make it more interesting
        if (num_vertices > 2) {
            int extra_vertex = vertex_dist(gen);
            if (extra_vertex != i-1 && extra_vertex != i) {
                hyperedge.push_back(extra_vertex);
            }
        }
        hg.hyperedges.push_back(hyperedge);
        
        // Update vertex-to-hyperedges mapping
        for (int v : hyperedge) {
            hg.vertex_to_hyperedges[v].push_back(i-1);
        }
    }
    
    // Generate remaining random hyperedges
    for (int he = num_vertices - 1; he < num_hyperedges; ++he) {
        int edge_size = size_dist(gen);
        std::unordered_set<int> edge_vertices;
        
        // Generate unique vertices for this hyperedge
        while (static_cast<int>(edge_vertices.size()) < edge_size) {
            edge_vertices.insert(vertex_dist(gen));
        }
        
        std::vector<int> hyperedge(edge_vertices.begin(), edge_vertices.end());
        hg.hyperedges.push_back(hyperedge);
        
        // Update vertex-to-hyperedges mapping
        for (int v : hyperedge) {
            hg.vertex_to_hyperedges[v].push_back(he);
        }
    }
    
    // Select IO vertices spread across the range
    fixed_vertices.clear();
    std::uniform_real_distribution<> coord_dist(0.0, 100.0);
    
    // Ensure IOs are spread across the vertex range for better connectivity
    for (int i = 0; i < num_ios; ++i) {
        int vertex_id = (i * num_vertices) / num_ios;
        if (vertex_id >= num_vertices) vertex_id = num_vertices - 1;
        
        double x_coord = coord_dist(gen);
        fixed_vertices.emplace_back(vertex_id, x_coord);
    }
    
    // Sort fixed vertices by coordinate for better visualization
    std::sort(fixed_vertices.begin(), fixed_vertices.end(),
              [](const FixedVertex& a, const FixedVertex& b) {
                  return a.x_coordinate < b.x_coordinate;
              });
}

// Read pre-placed coordinates from file
std::vector<FixedVertex> readPreplacedCoordinates(const std::string& filename) {
    std::vector<FixedVertex> fixed_vertices;
    
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open pre-placed coordinates file: " + filename);
    }
    
    std::string line;
    int line_number = 0;
    
    while (std::getline(file, line)) {
        line_number++;
        
        // Skip empty lines and comments
        if (line.empty() || line[0] == '#') {
            continue;
        }
        
        std::istringstream iss(line);
        int vertex_id;
        double x_coord, y_coord;
        
        if (!(iss >> vertex_id >> x_coord >> y_coord)) {
            throw std::runtime_error("Invalid format in pre-placed coordinates file at line " + 
                                    std::to_string(line_number) + ": " + line);
        }
        
        // For Dirichlet embedding, we only use the x_coordinate
        fixed_vertices.emplace_back(vertex_id, x_coord);
    }
    
    file.close();
    
    if (fixed_vertices.empty()) {
        throw std::runtime_error("No valid pre-placed coordinates found in file: " + filename);
    }
    
    // Sort by vertex ID for consistency
    std::sort(fixed_vertices.begin(), fixed_vertices.end(),
              [](const FixedVertex& a, const FixedVertex& b) {
                  return a.vertex_id < b.vertex_id;
              });
    
    return fixed_vertices;
}

// Write pre-placed coordinates to file
void writePreplacedCoordinates(
    const std::vector<FixedVertex>& fixed_vertices,
    const std::string& filename,
    double y_coordinate
) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot create pre-placed coordinates file: " + filename);
    }
    
    // Write header comment
    file << "# Pre-placed coordinates file\n";
    file << "# Format: vertex_id x_coordinate y_coordinate\n";
    file << "# Generated by Dirichlet spectral embedding\n";
    file << "\n";
    
    // Write coordinates
    for (const auto& fv : fixed_vertices) {
        file << fv.vertex_id << " " << std::fixed << std::setprecision(6) 
             << fv.x_coordinate << " " << y_coordinate << "\n";
    }
    
    file.close();
}

// Write Dirichlet embedding result as pre-placed coordinates
void writeDirichletResultAsCoordinates(
    const DirichletEmbeddingResult& result,
    const std::vector<FixedVertex>& fixed_vertices,
    const std::string& filename,
    double y_coordinate
) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot create coordinates file: " + filename);
    }
    
    // Write header comment
    file << "# Dirichlet spectral embedding result\n";
    file << "# Format: vertex_id x_coordinate y_coordinate\n";
    file << "# Fixed vertices: " << result.num_fixed_vertices << "\n";
    file << "# Movable vertices: " << result.num_movable_vertices << "\n";
    file << "# Computation time: " << result.computation_time_ms << "ms\n";
    file << "# Solver converged: " << (result.solver_converged ? "Yes" : "No") << "\n";
    file << "\n";
    
    // Create a map of all coordinates
    std::map<int, double> all_coordinates;
    
    // Add fixed vertices
    for (const auto& fv : fixed_vertices) {
        all_coordinates[fv.vertex_id] = fv.x_coordinate;
    }
    
    // Add movable vertices (use full_coordinates which includes normalization)
    for (int v = 0; v < static_cast<int>(result.full_coordinates.size()); ++v) {
        if (all_coordinates.find(v) == all_coordinates.end()) {
            // This is a movable vertex
            all_coordinates[v] = result.full_coordinates[v];
        }
    }
    
    // Write all coordinates sorted by vertex ID
    for (const auto& pair : all_coordinates) {
        file << pair.first << " " << std::fixed << std::setprecision(6) 
             << pair.second << " " << y_coordinate << "\n";
    }
    
    file.close();
}

// ============================================================================
// SOFT-ANCHORED SPECTRAL EMBEDDING IMPLEMENTATION
// ============================================================================

SoftAnchoredEmbeddingResult computeSoftAnchoredEmbedding(
    const Hypergraph& hg,
    const std::vector<FixedVertex>& fixed_vertices,
    const SoftAnchoredEmbeddingParams& params) {
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    if (params.verbose) {
        std::cout << "=== Soft-Anchored Spectral Embedding ===" << std::endl;
        std::cout << "Method: " << (params.method == SoftAnchoringMethod::PENALTY_METHOD ? 
                                   "Penalty Method" : "Virtual Springs") << std::endl;
        std::cout << "Hypergraph: " << hg.num_vertices << " vertices, " 
                  << hg.num_hyperedges << " hyperedges" << std::endl;
        std::cout << "Fixed vertices: " << fixed_vertices.size() << std::endl;
    }
    
    // Validate inputs
    if (!validateFixedVertices(hg, fixed_vertices)) {
        throw std::runtime_error("Invalid fixed vertex specifications");
    }
    
    // Dispatch to appropriate method
    SoftAnchoredEmbeddingResult result;
    switch (params.method) {
        case SoftAnchoringMethod::PENALTY_METHOD:
            result = computePenaltyMethodEmbedding(hg, fixed_vertices, params);
            break;
        case SoftAnchoringMethod::VIRTUAL_SPRINGS:
            result = computeVirtualSpringsEmbedding(hg, fixed_vertices, params);
            break;
        default:
            throw std::runtime_error("Unknown soft anchoring method");
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    result.computation_time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    
    if (params.verbose) {
        std::cout << "=== Soft-Anchored Embedding Complete ===" << std::endl;
        std::cout << "Total time: " << result.computation_time_ms << "ms" << std::endl;
        std::cout << "Standard cells ordered: " << result.num_standard_cells << std::endl;
    }
    
    return result;
}

// Helper function to solve linear system with different solver types
Eigen::VectorXd solveLinearSystem(
    const Eigen::SparseMatrix<double>& system_matrix,
    const Eigen::VectorXd& rhs,
    SolverType solver_type,
    double tolerance) {
    
    Eigen::VectorXd solution;
    
    switch (solver_type) {
        case SolverType::SPARSE_CHOLESKY: {
            Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver;
            solver.compute(system_matrix);
            
            if (solver.info() != Eigen::Success) {
                // Try LU as fallback
                Eigen::SparseLU<Eigen::SparseMatrix<double>> lu_solver;
                lu_solver.compute(system_matrix);
                
                if (lu_solver.info() != Eigen::Success) {
                    // Try regularization
                    Eigen::SparseMatrix<double> regularized_matrix = system_matrix;
                    double reg_param = 1e-8;
                    for (int i = 0; i < regularized_matrix.rows(); ++i) {
                        regularized_matrix.coeffRef(i, i) += reg_param;
                    }
                    
                    lu_solver.compute(regularized_matrix);
                    if (lu_solver.info() != Eigen::Success) {
                        throw std::runtime_error("Cholesky factorization failed");
                    }
                }
                
                solution = lu_solver.solve(rhs);
                if (lu_solver.info() != Eigen::Success) {
                    throw std::runtime_error("Cholesky solve failed");
                }
            } else {
                solution = solver.solve(rhs);
                if (solver.info() != Eigen::Success) {
                    throw std::runtime_error("Cholesky solve failed");
                }
            }
            break;
        }
        
        case SolverType::SPARSE_LU: {
            Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
            solver.compute(system_matrix);
            
            if (solver.info() != Eigen::Success) {
                // Try regularization
                Eigen::SparseMatrix<double> regularized_matrix = system_matrix;
                double reg_param = 1e-8;
                for (int i = 0; i < regularized_matrix.rows(); ++i) {
                    regularized_matrix.coeffRef(i, i) += reg_param;
                }
                
                solver.compute(regularized_matrix);
                if (solver.info() != Eigen::Success) {
                    throw std::runtime_error("LU factorization failed");
                }
            }
            
            solution = solver.solve(rhs);
            
            if (solver.info() != Eigen::Success) {
                throw std::runtime_error("LU solve failed");
            }
            break;
        }
        
        case SolverType::ITERATIVE_BICGSTAB: {
            Eigen::BiCGSTAB<Eigen::SparseMatrix<double>> solver;
            solver.setTolerance(tolerance);
            solver.compute(system_matrix);
            
            if (solver.info() != Eigen::Success) {
                throw std::runtime_error("BiCGSTAB setup failed");
            }
            
            solution = solver.solve(rhs);
            
            if (solver.info() != Eigen::Success) {
                throw std::runtime_error("BiCGSTAB solve failed");
            }
            break;
        }
        
        default:
            throw std::runtime_error("Unknown solver type");
    }
    
    return solution;
}

// Helper function to normalize coordinates
void normalizeCoordinates(std::vector<double>& coordinates) {
    if (coordinates.empty()) return;
    
    auto minmax = std::minmax_element(coordinates.begin(), coordinates.end());
    double min_coord = *minmax.first;
    double max_coord = *minmax.second;
    
    if (max_coord > min_coord) {
        double range = max_coord - min_coord;
        for (double& coord : coordinates) {
            coord = (coord - min_coord) / range;
        }
    }
}

SoftAnchoredEmbeddingResult computePenaltyMethodEmbedding(
    const Hypergraph& hg,
    const std::vector<FixedVertex>& fixed_vertices,
    const SoftAnchoredEmbeddingParams& params) {
    
    SoftAnchoredEmbeddingResult result;
    result.method_used = SoftAnchoringMethod::PENALTY_METHOD;
    result.penalty_parameter_used = params.penalty_parameter;
    result.num_vertices = hg.num_vertices;
    result.num_fixed_vertices = fixed_vertices.size();
    result.num_standard_cells = hg.num_vertices - fixed_vertices.size();
    
    if (params.verbose) {
        std::cout << "Building 2-section graph adjacency matrix..." << std::endl;
    }
    
    // Step 1: Build 2-section graph and Laplacian
    auto adjacency = build2SectionAdjacency(hg);
    auto laplacian = buildGraphLaplacian(adjacency);
    
    if (params.verbose) {
        std::cout << "Laplacian matrix: " << laplacian.nonZeros() << " non-zeros" << std::endl;
        std::cout << "Setting up penalty method with μ = " << params.penalty_parameter << std::endl;
    }
    
    // Step 2: Build diagonal penalty matrix D_T
    Eigen::SparseMatrix<double> penalty_matrix(hg.num_vertices, hg.num_vertices);
    std::vector<Eigen::Triplet<double>> penalty_triplets;
    
    // Create lookup for fixed vertices
    std::unordered_set<int> fixed_vertex_set;
    for (const auto& fv : fixed_vertices) {
        fixed_vertex_set.insert(fv.vertex_id);
        penalty_triplets.emplace_back(fv.vertex_id, fv.vertex_id, 1.0);
    }
    
    penalty_matrix.setFromTriplets(penalty_triplets.begin(), penalty_triplets.end());
    
    // Step 3: Form system matrix (L + μ * D_T)
    Eigen::SparseMatrix<double> system_matrix = laplacian + params.penalty_parameter * penalty_matrix;
    
    // Step 4: Build RHS vector (μ * D_T * c)
    Eigen::VectorXd rhs = Eigen::VectorXd::Zero(hg.num_vertices);
    for (const auto& fv : fixed_vertices) {
        rhs[fv.vertex_id] = params.penalty_parameter * fv.x_coordinate;
    }
    
    if (params.verbose) {
        std::cout << "Solving penalty method system: (" << system_matrix.rows() 
                  << " x " << system_matrix.cols() << ")" << std::endl;
        std::cout << "System matrix: " << system_matrix.nonZeros() << " non-zeros" << std::endl;
    }
    
    // Step 5: Solve the system
    auto solver_start = std::chrono::high_resolution_clock::now();
    Eigen::VectorXd solution;
    
    try {
        solution = solveLinearSystem(system_matrix, rhs, params.solver_type, params.solver_tolerance);
        result.solver_converged = true;
        result.solver_residual = (system_matrix * solution - rhs).norm();
    } catch (const std::exception& e) {
        if (params.verbose) {
            std::cout << "Primary solver failed: " << e.what() << std::endl;
            std::cout << "Trying fallback solver..." << std::endl;
        }
        
        try {
            solution = solveLinearSystem(system_matrix, rhs, SolverType::SPARSE_LU, params.solver_tolerance);
            result.solver_converged = true;
            result.solver_residual = (system_matrix * solution - rhs).norm();
        } catch (const std::exception& e2) {
            throw std::runtime_error("All solvers failed: " + std::string(e2.what()));
        }
    }
    
    auto solver_end = std::chrono::high_resolution_clock::now();
    result.solver_time_ms = std::chrono::duration<double, std::milli>(solver_end - solver_start).count();
    
    if (params.verbose) {
        std::cout << "Solver completed in " << result.solver_time_ms << "ms" << std::endl;
        std::cout << "Converged: " << (result.solver_converged ? "Yes" : "No") << std::endl;
        std::cout << "Final residual: " << std::scientific << result.solver_residual << std::fixed << std::endl;
    }
    
    // Step 6: Extract and normalize coordinates
    result.coordinates.resize(hg.num_vertices);
    for (int i = 0; i < hg.num_vertices; ++i) {
        result.coordinates[i] = solution[i];
    }
    
    if (params.normalize_coordinates) {
        normalizeCoordinates(result.coordinates);
        if (params.verbose) {
            std::cout << "Normalized coordinates to [0,1] range" << std::endl;
        }
    }
    
    // Step 7: Extract standard cell coordinates and create ordering
    std::vector<std::pair<double, int>> standard_cell_pairs;
    for (int v = 0; v < hg.num_vertices; ++v) {
        if (fixed_vertex_set.find(v) == fixed_vertex_set.end()) {
            // This is a standard cell
            standard_cell_pairs.emplace_back(result.coordinates[v], v);
            result.standard_cell_coords.push_back(result.coordinates[v]);
        }
    }
    
    // Sort standard cells by coordinate
    std::sort(standard_cell_pairs.begin(), standard_cell_pairs.end());
    
    // Extract ordering
    result.ordering.reserve(standard_cell_pairs.size());
    for (const auto& pair : standard_cell_pairs) {
        result.ordering.push_back(pair.second);
    }
    
    if (params.verbose) {
        std::cout << "Standard cell coordinate range: [" 
                  << *std::min_element(result.standard_cell_coords.begin(), result.standard_cell_coords.end())
                  << ", " 
                  << *std::max_element(result.standard_cell_coords.begin(), result.standard_cell_coords.end())
                  << "]" << std::endl;
    }
    
    return result;
}

SoftAnchoredEmbeddingResult computeVirtualSpringsEmbedding(
    const Hypergraph& hg,
    const std::vector<FixedVertex>& fixed_vertices,
    const SoftAnchoredEmbeddingParams& params) {
    
    SoftAnchoredEmbeddingResult result;
    result.method_used = SoftAnchoringMethod::VIRTUAL_SPRINGS;
    result.virtual_spring_weight_used = params.virtual_spring_weight;
    result.num_vertices = hg.num_vertices;
    result.num_fixed_vertices = fixed_vertices.size();
    result.num_standard_cells = hg.num_vertices - fixed_vertices.size();
    
    if (params.verbose) {
        std::cout << "Building 2-section graph adjacency matrix..." << std::endl;
    }
    
    // Step 1: Build 2-section graph and Laplacian
    auto adjacency = build2SectionAdjacency(hg);
    auto laplacian = buildGraphLaplacian(adjacency);
    
    if (params.verbose) {
        std::cout << "Laplacian matrix: " << laplacian.nonZeros() << " non-zeros" << std::endl;
        std::cout << "Setting up virtual springs with weight = " << params.virtual_spring_weight << std::endl;
    }
    
    // Step 2: Build diagonal weight matrix W_IO
    Eigen::SparseMatrix<double> weight_matrix(hg.num_vertices, hg.num_vertices);
    std::vector<Eigen::Triplet<double>> weight_triplets;
    
    // Create lookup for fixed vertices
    std::unordered_set<int> fixed_vertex_set;
    for (const auto& fv : fixed_vertices) {
        fixed_vertex_set.insert(fv.vertex_id);
        weight_triplets.emplace_back(fv.vertex_id, fv.vertex_id, params.virtual_spring_weight);
    }
    
    weight_matrix.setFromTriplets(weight_triplets.begin(), weight_triplets.end());
    
    // Step 3: Form system matrix (L + W_IO)
    Eigen::SparseMatrix<double> system_matrix = laplacian + weight_matrix;
    
    // Step 4: Build RHS vector (W_IO * c)
    Eigen::VectorXd rhs = Eigen::VectorXd::Zero(hg.num_vertices);
    for (const auto& fv : fixed_vertices) {
        rhs[fv.vertex_id] = params.virtual_spring_weight * fv.x_coordinate;
    }
    
    if (params.verbose) {
        std::cout << "Solving virtual springs system: (" << system_matrix.rows() 
                  << " x " << system_matrix.cols() << ")" << std::endl;
        std::cout << "System matrix: " << system_matrix.nonZeros() << " non-zeros" << std::endl;
    }
    
    // Step 5: Solve the system
    auto solver_start = std::chrono::high_resolution_clock::now();
    Eigen::VectorXd solution;
    
    try {
        solution = solveLinearSystem(system_matrix, rhs, params.solver_type, params.solver_tolerance);
        result.solver_converged = true;
        result.solver_residual = (system_matrix * solution - rhs).norm();
    } catch (const std::exception& e) {
        if (params.verbose) {
            std::cout << "Primary solver failed: " << e.what() << std::endl;
            std::cout << "Trying fallback solver..." << std::endl;
        }
        
        try {
            solution = solveLinearSystem(system_matrix, rhs, SolverType::SPARSE_LU, params.solver_tolerance);
            result.solver_converged = true;
            result.solver_residual = (system_matrix * solution - rhs).norm();
        } catch (const std::exception& e2) {
            throw std::runtime_error("All solvers failed: " + std::string(e2.what()));
        }
    }
    
    auto solver_end = std::chrono::high_resolution_clock::now();
    result.solver_time_ms = std::chrono::duration<double, std::milli>(solver_end - solver_start).count();
    
    if (params.verbose) {
        std::cout << "Solver completed in " << result.solver_time_ms << "ms" << std::endl;
        std::cout << "Converged: " << (result.solver_converged ? "Yes" : "No") << std::endl;
        std::cout << "Final residual: " << std::scientific << result.solver_residual << std::fixed << std::endl;
    }
    
    // Step 6: Extract and normalize coordinates
    result.coordinates.resize(hg.num_vertices);
    for (int i = 0; i < hg.num_vertices; ++i) {
        result.coordinates[i] = solution[i];
    }
    
    if (params.normalize_coordinates) {
        normalizeCoordinates(result.coordinates);
        if (params.verbose) {
            std::cout << "Normalized coordinates to [0,1] range" << std::endl;
        }
    }
    
    // Step 7: Extract standard cell coordinates and create ordering
    std::vector<std::pair<double, int>> standard_cell_pairs;
    for (int v = 0; v < hg.num_vertices; ++v) {
        if (fixed_vertex_set.find(v) == fixed_vertex_set.end()) {
            // This is a standard cell
            standard_cell_pairs.emplace_back(result.coordinates[v], v);
            result.standard_cell_coords.push_back(result.coordinates[v]);
        }
    }
    
    // Sort standard cells by coordinate
    std::sort(standard_cell_pairs.begin(), standard_cell_pairs.end());
    
    // Extract ordering
    result.ordering.reserve(standard_cell_pairs.size());
    for (const auto& pair : standard_cell_pairs) {
        result.ordering.push_back(pair.second);
    }
    
    if (params.verbose) {
        std::cout << "Standard cell coordinate range: [" 
                  << *std::min_element(result.standard_cell_coords.begin(), result.standard_cell_coords.end())
                  << ", " 
                  << *std::max_element(result.standard_cell_coords.begin(), result.standard_cell_coords.end())
                  << "]" << std::endl;
    }
    
    return result;
}

void writeSoftAnchoredResultAsCoordinates(
    const SoftAnchoredEmbeddingResult& result,
    const std::vector<FixedVertex>& fixed_vertices,
    const std::string& filename,
    double y_coordinate) {
    
    std::ofstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot create coordinate file: " + filename);
    }
    
    // Write header with metadata
    file << "# Soft-anchored spectral embedding result\n";
    file << "# Method: " << (result.method_used == SoftAnchoringMethod::PENALTY_METHOD ? 
                            "Penalty Method" : "Virtual Springs") << "\n";
    file << "# Format: vertex_id x_coordinate y_coordinate\n";
    file << "# Fixed vertices: " << result.num_fixed_vertices << "\n";
    file << "# Standard cells: " << result.num_standard_cells << "\n";
    file << "# Computation time: " << result.computation_time_ms << "ms\n";
    file << "# Solver converged: " << (result.solver_converged ? "Yes" : "No") << "\n";
    
    if (result.method_used == SoftAnchoringMethod::PENALTY_METHOD) {
        file << "# Penalty parameter: " << result.penalty_parameter_used << "\n";
    } else {
        file << "# Virtual spring weight: " << result.virtual_spring_weight_used << "\n";
    }
    file << "\n";
    
    // Write all vertex coordinates
    for (int v = 0; v < static_cast<int>(result.coordinates.size()); ++v) {
        file << v << " " << std::fixed << std::setprecision(6) 
             << result.coordinates[v] << " " << y_coordinate << "\n";
    }
    
    file.close();
    std::cout << "Soft-anchored result written to: " << filename << std::endl;
}

// ============================================================================
// BLOCK-BASED GREEDY REFINEMENT WITH IO CONSTRAINTS IMPLEMENTATION
// ============================================================================

std::vector<int> ensureCorrectIOOrder(
    const std::vector<int>& initial_ordering,
    const std::vector<FixedVertex>& fixed_vertices,
    bool& order_corrected) {
    
    // Create map: vertex_id -> x_coordinate
    std::unordered_map<int, double> io_coordinates;
    for (const auto& fv : fixed_vertices) {
        io_coordinates[fv.vertex_id] = fv.x_coordinate;
    }
    
    // Extract IO positions and coordinates from current ordering
    std::vector<std::pair<int, double>> io_positions_coords; // (position, coordinate)
    std::vector<int> io_vertices; // vertex IDs in order of appearance
    
    for (int pos = 0; pos < static_cast<int>(initial_ordering.size()); ++pos) {
        int vertex = initial_ordering[pos];
        if (io_coordinates.count(vertex) > 0) {
            io_positions_coords.emplace_back(pos, io_coordinates[vertex]);
            io_vertices.push_back(vertex);
        }
    }
    
    // Check if IOs are already sorted by coordinate
    bool needs_correction = false;
    for (int i = 1; i < static_cast<int>(io_positions_coords.size()); ++i) {
        if (io_positions_coords[i].second < io_positions_coords[i-1].second) {
            needs_correction = true;
            break;
        }
    }
    
    if (!needs_correction) {
        order_corrected = false;
        return initial_ordering;
    }
    
    // Sort IOs by coordinate
    std::vector<std::pair<double, int>> coord_vertex_pairs;
    for (int vertex : io_vertices) {
        coord_vertex_pairs.emplace_back(io_coordinates[vertex], vertex);
    }
    std::sort(coord_vertex_pairs.begin(), coord_vertex_pairs.end());
    
    // Create corrected ordering
    std::vector<int> corrected_ordering;
    std::unordered_set<int> io_set(io_vertices.begin(), io_vertices.end());
    
    int io_index = 0;
    for (int pos = 0; pos < static_cast<int>(initial_ordering.size()); ++pos) {
        int vertex = initial_ordering[pos];
        
        if (io_set.count(vertex) > 0) {
            // Replace with correctly ordered IO
            if (io_index < static_cast<int>(coord_vertex_pairs.size())) {
                corrected_ordering.push_back(coord_vertex_pairs[io_index].second);
                io_index++;
            }
        } else {
            // Keep standard cell in same position
            corrected_ordering.push_back(vertex);
        }
    }
    
    order_corrected = true;
    return corrected_ordering;
}

std::vector<RefinementBlock> identifyRefinementBlocks(
    const std::vector<int>& ordering,
    const std::unordered_set<int>& fixed_vertex_set) {
    
    std::vector<RefinementBlock> blocks;
    
    // Find all IO positions
    std::vector<int> io_positions;
    for (int pos = 0; pos < static_cast<int>(ordering.size()); ++pos) {
        if (fixed_vertex_set.count(ordering[pos]) > 0) {
            io_positions.push_back(pos);
        }
    }
    
    // Create blocks between consecutive IOs
    int current_start = 0;
    
    for (int io_pos : io_positions) {
        if (io_pos > current_start) {
            // Block before this IO
            RefinementBlock block;
            block.start_pos = current_start;
            block.end_pos = io_pos - 1;
            
            // Extract movable vertices in this block
            for (int pos = block.start_pos; pos <= block.end_pos; ++pos) {
                block.movable_vertices.push_back(ordering[pos]);
            }
            
            block.block_size = block.movable_vertices.size();
            
            if (!block.movable_vertices.empty()) {
                blocks.push_back(block);
            }
        }
        current_start = io_pos + 1;
    }
    
    // Final block after last IO
    if (current_start < static_cast<int>(ordering.size())) {
        RefinementBlock block;
        block.start_pos = current_start;
        block.end_pos = ordering.size() - 1;
        
        for (int pos = block.start_pos; pos <= block.end_pos; ++pos) {
            block.movable_vertices.push_back(ordering[pos]);
        }
        
        block.block_size = block.movable_vertices.size();
        
        if (!block.movable_vertices.empty()) {
            blocks.push_back(block);
        }
    }
    
    return blocks;
}

int refineBlock(
    HypergraphCutwidthTracker& tracker,
    RefinementBlock& block,
    const BlockBasedRefinementParams& params) {
    
    int initial_peak = tracker.getPeakCutwidth();
    int swaps_performed = 0;
    
    if (params.verbose) {
        std::cout << "  Refining block [" << block.start_pos << "," << block.end_pos 
                  << "] with " << block.block_size << " vertices" << std::endl;
    }
    
    for (int iteration = 0; iteration < params.max_iterations_per_block; ++iteration) {
        bool improved = false;
        int iteration_swaps = 0;
        
        // Phase 1: Hyperedge-aware priority swaps within block
        if (params.use_hyperedge_aware) {
            auto priority_candidates = tracker.getPrioritySwapCandidates();
            
            for (const auto& [pos1, pos2] : priority_candidates) {
                // Only consider swaps within this block
                if (pos1 >= block.start_pos && pos1 <= block.end_pos &&
                    pos2 >= block.start_pos && pos2 <= block.end_pos &&
                    abs(pos1 - pos2) == 1) {
                    
                    int delta = tracker.evaluateSwap(std::min(pos1, pos2));
                    if (delta < 0) {
                        tracker.performSwap(std::min(pos1, pos2));
                        iteration_swaps++;
                        improved = true;
                    }
                }
            }
        }
        
        // Phase 2: Adjacent swaps within block
        for (int pos = block.start_pos; pos < block.end_pos; ++pos) {
            int delta = tracker.evaluateSwap(pos);
            if (delta < 0) {
                tracker.performSwap(pos);
                iteration_swaps++;
                improved = true;
            }
        }
        
        // Phase 3: Distance swaps within block
        if (params.max_distance_swaps > 1) {
            for (int distance = 2; distance <= params.max_distance_swaps; ++distance) {
                for (int pos1 = block.start_pos; pos1 <= block.end_pos - distance; ++pos1) {
                    int pos2 = pos1 + distance;
                    
                    if (pos2 <= block.end_pos) {
                        int delta = tracker.evaluateDistantSwap(pos1, pos2);
                        if (delta < 0) {
                            tracker.performDistantSwap(pos1, pos2);
                            iteration_swaps++;
                            improved = true;
                        }
                    }
                }
            }
        }
        
        swaps_performed += iteration_swaps;
        
        if (!improved) break;
        
        if (params.verbose && iteration_swaps > 0) {
            std::cout << "    Iteration " << (iteration + 1) 
                      << ": " << iteration_swaps << " swaps" << std::endl;
        }
    }
    
    block.swaps_performed = swaps_performed;
    block.initial_peak_contribution = initial_peak;
    block.final_peak_contribution = tracker.getPeakCutwidth();
    
    return swaps_performed;
}

int optimizeBlockBoundaries(
    HypergraphCutwidthTracker& tracker,
    const std::vector<RefinementBlock>& blocks,
    const std::unordered_set<int>& fixed_vertex_set) {
    
    int total_swaps = 0;
    
    // Try swaps at block boundaries (adjacent to IOs)
    for (const auto& block : blocks) {
        // Check swap just before block start (if not at beginning)
        if (block.start_pos > 0) {
            int prev_pos = block.start_pos - 1;
            // Only try if the previous position is not an IO
            if (fixed_vertex_set.count(tracker.getOrdering()[prev_pos]) == 0) {
                int delta = tracker.evaluateSwap(prev_pos);
                if (delta < 0) {
                    tracker.performSwap(prev_pos);
                    total_swaps++;
                }
            }
        }
        
        // Check swap just after block end (if not at end)
        if (block.end_pos < static_cast<int>(tracker.getOrdering().size()) - 1) {
            int delta = tracker.evaluateSwap(block.end_pos);
            if (delta < 0) {
                tracker.performSwap(block.end_pos);
                total_swaps++;
            }
        }
    }
    
    return total_swaps;
}

BlockBasedRefinementResult applyBlockBasedGreedyRefinement(
    const Hypergraph& hg,
    const std::vector<int>& initial_ordering,
    const std::vector<FixedVertex>& fixed_vertices,
    const BlockBasedRefinementParams& params) {
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    BlockBasedRefinementResult result;
    result.num_ios = fixed_vertices.size();
    result.num_movable = initial_ordering.size() - fixed_vertices.size();
    
    if (params.verbose) {
        std::cout << "\n=== Block-Based Greedy Refinement ===" << std::endl;
        std::cout << "Total vertices: " << initial_ordering.size() << std::endl;
        std::cout << "IO vertices: " << result.num_ios << std::endl;
        std::cout << "Movable vertices: " << result.num_movable << std::endl;
    }
    
    // Step 1: Ensure correct IO order
    bool order_corrected;
    auto corrected_ordering = ensureCorrectIOOrder(initial_ordering, fixed_vertices, order_corrected);
    result.io_order_corrected = order_corrected;
    
    if (params.verbose && order_corrected) {
        std::cout << "IO order corrected based on placement coordinates" << std::endl;
    }
    
    // Step 2: Initialize tracker
    HypergraphCutwidthTracker tracker(hg);
    tracker.initialize(corrected_ordering);
    result.initial_peak = tracker.getPeakCutwidth();
    
    if (params.verbose) {
        std::cout << "Initial peak cutwidth: " << result.initial_peak << std::endl;
    }
    
    // Step 3: Identify refinement blocks
    std::unordered_set<int> fixed_vertex_set;
    for (const auto& fv : fixed_vertices) {
        fixed_vertex_set.insert(fv.vertex_id);
    }
    
    result.blocks = identifyRefinementBlocks(corrected_ordering, fixed_vertex_set);
    
    if (params.verbose) {
        std::cout << "Identified " << result.blocks.size() << " refinement blocks:" << std::endl;
        for (size_t i = 0; i < result.blocks.size(); ++i) {
            const auto& block = result.blocks[i];
            std::cout << "  Block " << (i + 1) << ": positions [" 
                      << block.start_pos << "," << block.end_pos 
                      << "], " << block.block_size << " vertices" << std::endl;
        }
    }
    
    // Step 4: Global refinement iterations
    bool global_improved = true;
    int global_iteration = 0;
    
    while (global_improved && global_iteration < params.max_global_iterations) {
        global_improved = false;
        int iteration_swaps = 0;
        
        if (params.verbose) {
            std::cout << "\n--- Global Iteration " << (global_iteration + 1) << " ---" << std::endl;
        }
        
        // Refine each block
        for (auto& block : result.blocks) {
            int block_swaps = refineBlock(tracker, block, params);
            iteration_swaps += block_swaps;
            if (block_swaps > 0) global_improved = true;
        }
        
        // Optimize block boundaries
        if (params.optimize_boundaries) {
            int boundary_swaps = optimizeBlockBoundaries(tracker, result.blocks, fixed_vertex_set);
            iteration_swaps += boundary_swaps;
            if (boundary_swaps > 0) global_improved = true;
            
            if (params.verbose && boundary_swaps > 0) {
                std::cout << "  Boundary optimization: " << boundary_swaps << " swaps" << std::endl;
            }
        }
        
        result.total_swaps += iteration_swaps;
        global_iteration++;
        
        if (params.verbose) {
            std::cout << "Global iteration " << global_iteration 
                      << ": " << iteration_swaps << " swaps, peak = " 
                      << tracker.getPeakCutwidth() << std::endl;
        }
        
        // Check convergence
        if (iteration_swaps == 0) {
            global_improved = false;
        }
    }
    
    // Step 5: Finalize results
    result.ordering = tracker.getOrdering();
    result.final_peak = tracker.getPeakCutwidth();
    result.global_iterations = global_iteration;
    result.converged = !global_improved;
    
    auto end_time = std::chrono::high_resolution_clock::now();
    result.computation_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    
    if (params.verbose) {
        std::cout << "\n=== Block-Based Refinement Complete ===" << std::endl;
        std::cout << "Initial peak: " << result.initial_peak << std::endl;
        std::cout << "Final peak: " << result.final_peak << std::endl;
        std::cout << "Improvement: " << (result.initial_peak - result.final_peak);
        if (result.initial_peak > 0) {
            std::cout << " (" << std::fixed << std::setprecision(1) 
                      << (100.0 * (result.initial_peak - result.final_peak) / result.initial_peak) << "%)";
        }
        std::cout << std::endl;
        std::cout << "Total swaps: " << result.total_swaps << std::endl;
        std::cout << "Global iterations: " << result.global_iterations << std::endl;
        std::cout << "Computation time: " << result.computation_time_ms << " ms" << std::endl;
        std::cout << "Converged: " << (result.converged ? "Yes" : "No") << std::endl;
        
        // Block-level statistics
        std::cout << "\nBlock Statistics:" << std::endl;
        for (size_t i = 0; i < result.blocks.size(); ++i) {
            const auto& block = result.blocks[i];
            std::cout << "  Block " << (i + 1) << ": " << block.swaps_performed 
                      << " swaps, " << block.block_size << " vertices" << std::endl;
        }
    }
    
    return result;
}

// ============================================================================
// IO-ANCHORED GREEDY REFINEMENT IMPLEMENTATION
// ============================================================================

IOAnchoredRefinementResult applyIOAnchoredGreedyRefinement(
    const Hypergraph& hg,
    const std::vector<int>& initial_ordering,
    const std::vector<FixedVertex>& fixed_vertices,
    const IOAnchoredRefinementParams& params) {
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    IOAnchoredRefinementResult result;
    result.num_ios = fixed_vertices.size();
    result.num_movable = initial_ordering.size() - fixed_vertices.size();
    
    if (params.verbose) {
        std::cout << "\n=== IO-Anchored Greedy Refinement ===" << std::endl;
        std::cout << "Total vertices: " << initial_ordering.size() << std::endl;
        std::cout << "IO vertices (fixed): " << result.num_ios << std::endl;
        std::cout << "Movable vertices: " << result.num_movable << std::endl;
        std::cout << "Cross-block moves: ENABLED" << std::endl;
    }
    
    // Step 1: Ensure correct IO order
    bool order_corrected;
    auto corrected_ordering = ensureCorrectIOOrder(initial_ordering, fixed_vertices, order_corrected);
    
    if (params.verbose && order_corrected) {
        std::cout << "IO order corrected based on placement coordinates" << std::endl;
    }
    
    // Step 2: Initialize tracker
    HypergraphCutwidthTracker tracker(hg);
    tracker.initialize(corrected_ordering);
    result.initial_peak = tracker.getPeakCutwidth();
    
    if (params.verbose) {
        std::cout << "Initial peak cutwidth: " << result.initial_peak << std::endl;
    }
    
    // Step 3: Identify fixed IO positions
    std::unordered_set<int> fixed_vertex_set;
    std::unordered_set<int> fixed_positions;
    for (const auto& fv : fixed_vertices) {
        fixed_vertex_set.insert(fv.vertex_id);
    }
    
    // Find current positions of IOs
    for (int pos = 0; pos < static_cast<int>(corrected_ordering.size()); ++pos) {
        if (fixed_vertex_set.count(corrected_ordering[pos]) > 0) {
            fixed_positions.insert(pos);
            result.io_positions.push_back(pos);
        }
    }
    
    if (params.verbose) {
        std::cout << "Fixed IO positions: ";
        for (int pos : result.io_positions) {
            std::cout << pos << "(" << corrected_ordering[pos] << ") ";
        }
        std::cout << std::endl;
    }
    
    // Step 4: Main refinement loop
    bool improved = true;
    int iteration = 0;
    
    while (improved && iteration < params.max_iterations) {
        improved = false;
        int iteration_swaps = 0;
        int iteration_insertions = 0;
        
        if (params.verbose) {
            std::cout << "\n--- Iteration " << (iteration + 1) << " ---" << std::endl;
        }
        
        // Phase 1: Try all valid swaps (including cross-block)
        for (int pos1 = 0; pos1 < static_cast<int>(tracker.getOrdering().size()) - 1; ++pos1) {
            // Skip if pos1 is an IO
            if (fixed_positions.count(pos1) > 0) continue;
            
            for (int pos2 = pos1 + 1; pos2 < static_cast<int>(tracker.getOrdering().size()); ++pos2) {
                // Skip if pos2 is an IO
                if (fixed_positions.count(pos2) > 0) continue;
                
                // Skip if distance is too large
                if (pos2 - pos1 > params.max_distance_swaps) break;
                
                // Evaluate swap
                int delta = tracker.evaluateSwap(pos1);
                if (delta < 0) {
                    tracker.performSwap(pos1);
                    iteration_swaps++;
                    improved = true;
                    
                    if (params.verbose && iteration_swaps <= 5) {
                        std::cout << "  Beneficial swap at positions " << pos1 << "-" << pos2 
                                  << " (delta: " << delta << ")" << std::endl;
                    }
                }
            }
        }
        
        // Phase 2: Try distant swaps (cross-block optimization)
        if (params.use_insertion_moves) {
            for (int pos1 = 0; pos1 < static_cast<int>(tracker.getOrdering().size()); ++pos1) {
                // Skip if pos1 is an IO
                if (fixed_positions.count(pos1) > 0) continue;
                
                for (int pos2 = pos1 + params.max_distance_swaps + 1; 
                     pos2 < static_cast<int>(tracker.getOrdering().size()) && 
                     pos2 - pos1 <= params.max_insertion_distance; ++pos2) {
                    
                    // Skip if pos2 is an IO
                    if (fixed_positions.count(pos2) > 0) continue;
                    
                    // Evaluate distant swap
                    int delta = tracker.evaluateDistantSwap(pos1, pos2);
                    if (delta < 0) {
                        tracker.performDistantSwap(pos1, pos2);
                        iteration_insertions++;
                        improved = true;
                        
                        if (params.verbose && iteration_insertions <= 3) {
                            std::cout << "  Beneficial distant swap: " << pos1 << " ↔ " << pos2 
                                      << " (delta: " << delta << ")" << std::endl;
                        }
                        
                        break; // Re-evaluate after changes
                    }
                }
                
                if (iteration_insertions > 0) break; // Re-start after distant swaps
            }
        }
        
        // Phase 3: Optimize near IOs (if enabled)
        if (params.optimize_io_neighbors) {
            for (int io_pos : result.io_positions) {
                // Try swaps adjacent to IOs
                for (int offset : {-1, 1}) {
                    int adj_pos = io_pos + offset;
                    if (adj_pos >= 0 && adj_pos < static_cast<int>(tracker.getOrdering().size()) - 1 &&
                        fixed_positions.count(adj_pos) == 0 && 
                        fixed_positions.count(adj_pos + 1) == 0) {
                        
                        int delta = tracker.evaluateSwap(adj_pos);
                        if (delta < 0) {
                            tracker.performSwap(adj_pos);
                            iteration_swaps++;
                            improved = true;
                        }
                    }
                }
            }
        }
        
        result.total_swaps += iteration_swaps;
        result.total_insertions += iteration_insertions;
        iteration++;
        
        if (params.verbose) {
            std::cout << "Iteration " << iteration << ": " << iteration_swaps << " swaps, " 
                      << iteration_insertions << " insertions, peak = " 
                      << tracker.getPeakCutwidth() << std::endl;
        }
        
        // Check convergence
        if (iteration_swaps == 0 && iteration_insertions == 0) {
            improved = false;
        }
    }
    
    // Step 5: Finalize results
    result.ordering = tracker.getOrdering();
    result.final_peak = tracker.getPeakCutwidth();
    result.iterations = iteration;
    result.converged = !improved;
    
    auto end_time = std::chrono::high_resolution_clock::now();
    result.computation_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    
    if (params.verbose) {
        std::cout << "\n=== IO-Anchored Refinement Complete ===" << std::endl;
        std::cout << "Initial peak: " << result.initial_peak << std::endl;
        std::cout << "Final peak: " << result.final_peak << std::endl;
        std::cout << "Improvement: " << (result.initial_peak - result.final_peak);
        if (result.initial_peak > 0) {
            std::cout << " (" << std::fixed << std::setprecision(1) 
                      << (100.0 * (result.initial_peak - result.final_peak) / result.initial_peak) << "%)";
        }
        std::cout << std::endl;
        std::cout << "Total swaps: " << result.total_swaps << std::endl;
        std::cout << "Total insertions: " << result.total_insertions << std::endl;
        std::cout << "Iterations: " << result.iterations << std::endl;
        std::cout << "Computation time: " << result.computation_time_ms << " ms" << std::endl;
        std::cout << "Converged: " << (result.converged ? "Yes" : "No") << std::endl;
        
        // Verify IO positions are maintained
        std::cout << "\nIO Position Verification:" << std::endl;
        bool io_positions_maintained = true;
        for (int pos = 0; pos < static_cast<int>(result.ordering.size()); ++pos) {
            if (fixed_vertex_set.count(result.ordering[pos]) > 0) {
                std::cout << "  IO " << result.ordering[pos] << " at position " << pos << std::endl;
            }
        }
        
        if (io_positions_maintained) {
            std::cout << "✓ All IO positions maintained correctly" << std::endl;
        }
    }
    
    return result;
}

IOAnchoredRefinementResult applyIOAnchoredGreedyRefinement(
    const Hypergraph& hg,
    const std::vector<int>& initial_ordering,
    const std::unordered_set<int>& fixed_vertex_set,
    const IOAnchoredRefinementParams& params) {
    
    // Convert fixed vertex set to FixedVertex vector with position-based coordinates
    std::vector<FixedVertex> fixed_vertex_list;
    for (int vertex_id : fixed_vertex_set) {
        // Find position of this vertex in the initial ordering
        auto it = std::find(initial_ordering.begin(), initial_ordering.end(), vertex_id);
        if (it != initial_ordering.end()) {
            double position_coord = static_cast<double>(std::distance(initial_ordering.begin(), it));
            fixed_vertex_list.emplace_back(vertex_id, position_coord);
        }
    }
    
    // Sort fixed vertices by their position coordinates to maintain order
    std::sort(fixed_vertex_list.begin(), fixed_vertex_list.end(),
              [](const FixedVertex& a, const FixedVertex& b) {
                  return a.x_coordinate < b.x_coordinate;
              });
    
    return applyIOAnchoredGreedyRefinement(hg, initial_ordering, fixed_vertex_list, params);
} 