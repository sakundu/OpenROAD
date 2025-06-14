#pragma once

#include "sait/hypergraph.hpp"
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <Eigen/Dense>
#include <Eigen/Sparse>

/**
 * @brief Solver types for linear systems
 */
enum class SolverType {
    SPARSE_CHOLESKY,     // Sparse Cholesky factorization
    SPARSE_LU,           // Sparse LU factorization
    ITERATIVE_BICGSTAB   // Iterative BiCGSTAB solver
};

/**
 * @brief Parameters for Dirichlet spectral embedding
 */
struct DirichletEmbeddingParams {
    bool verbose = false;                    // Print progress information
    bool use_sparse_solver = true;          // Use sparse vs dense linear solver
    double solver_tolerance = 1e-8;         // Solver convergence tolerance
    int max_solver_iterations = 1000;       // Maximum solver iterations
    bool normalize_coordinates = true;      // Normalize final coordinates to [0,1]
    
    DirichletEmbeddingParams() = default;
};

/**
 * @brief Soft anchoring methods for spectral embedding
 */
enum class SoftAnchoringMethod {
    PENALTY_METHOD,      // Variant 1: Penalty method with μ parameter
    VIRTUAL_SPRINGS      // Variant 2: Virtual springs method with weights
};

/**
 * @brief Parameters for soft-anchored spectral embedding
 */
struct SoftAnchoredEmbeddingParams {
    SoftAnchoringMethod method = SoftAnchoringMethod::PENALTY_METHOD;
    double penalty_parameter = 1.0;        // μ for penalty method
    double virtual_spring_weight = 1.0;    // w_virtual for springs method
    bool normalize_coordinates = true;
    bool verbose = false;
    SolverType solver_type = SolverType::SPARSE_CHOLESKY;
    double solver_tolerance = 1e-10;
};

/**
 * @brief Result of soft-anchored spectral embedding
 */
struct SoftAnchoredEmbeddingResult {
    std::vector<int> ordering;              // Final ordering of standard cells V_F
    std::vector<double> coordinates;        // Computed coordinates for all vertices
    std::vector<double> standard_cell_coords; // Coordinates for V_F only
    
    // Algorithm metadata
    int num_vertices;
    int num_fixed_vertices;
    int num_standard_cells;
    bool solver_converged;
    double solver_residual;
    double computation_time_ms;
    double solver_time_ms;
    
    // Soft anchoring specific
    SoftAnchoringMethod method_used;
    double penalty_parameter_used;
    double virtual_spring_weight_used;
    
    SoftAnchoredEmbeddingResult() : num_vertices(0), num_fixed_vertices(0), 
                                   num_standard_cells(0), solver_converged(false),
                                   solver_residual(0.0), computation_time_ms(0.0),
                                   solver_time_ms(0.0), method_used(SoftAnchoringMethod::PENALTY_METHOD),
                                   penalty_parameter_used(0.0), virtual_spring_weight_used(0.0) {}
};

/**
 * @brief Result of Dirichlet spectral embedding
 */
struct DirichletEmbeddingResult {
    std::vector<int> ordering;               // Final ordering of movable vertices
    std::vector<double> coordinates;         // Computed coordinates for movable vertices
    std::vector<double> full_coordinates;    // All coordinates [movable, fixed]
    std::vector<int> vertex_mapping;         // Maps original vertex ID to position in full_coordinates
    
    // Algorithm statistics
    int num_movable_vertices;                // Number of movable vertices
    int num_fixed_vertices;                  // Number of fixed vertices
    int num_edges_2section;                  // Number of edges in 2-section graph
    double computation_time_ms;              // Total computation time
    double solver_time_ms;                   // Time spent in linear solver
    bool solver_converged;                   // Whether linear solver converged
    double solver_residual;                  // Final solver residual
    
    DirichletEmbeddingResult() : num_movable_vertices(0), num_fixed_vertices(0), 
                                num_edges_2section(0), computation_time_ms(0.0), 
                                solver_time_ms(0.0), solver_converged(false), 
                                solver_residual(0.0) {}
};

/**
 * @brief Specification for pre-placed (fixed) vertices
 */
struct FixedVertex {
    int vertex_id;                           // Original vertex ID in hypergraph
    double x_coordinate;                     // Fixed X-coordinate
    
    FixedVertex(int id, double x) : vertex_id(id), x_coordinate(x) {}
};

/**
 * @brief Compute Dirichlet spectral embedding for hypergraph ordering
 * 
 * This function implements the Dirichlet spectral embedding algorithm:
 * 1. Build 2-section graph from hypergraph
 * 2. Construct graph Laplacian L
 * 3. Partition L into fixed/movable blocks: L = [L_FF L_FT; L_TF L_TT]
 * 4. Solve Dirichlet problem: L_FF * x_F = -L_FT * x_T
 * 5. Sort movable vertices by computed coordinates
 * 
 * @param hg Input hypergraph representing the netlist
 * @param fixed_vertices List of pre-placed vertices with fixed X-coordinates
 * @param params Algorithm parameters
 * @return Complete embedding result with ordering and coordinates
 */
DirichletEmbeddingResult computeDirichletEmbedding(
    const Hypergraph& hg,
    const std::vector<FixedVertex>& fixed_vertices,
    const DirichletEmbeddingParams& params = DirichletEmbeddingParams()
);

/**
 * @brief Build 2-section graph adjacency matrix from hypergraph
 * 
 * For each hyperedge, adds edges between all pairs of vertices (clique expansion).
 * Returns sparse adjacency matrix for efficiency.
 * 
 * @param hg Input hypergraph
 * @param verbose Print progress information
 * @return Sparse adjacency matrix of 2-section graph
 */
Eigen::SparseMatrix<double> build2SectionAdjacency(const Hypergraph& hg, bool verbose = false);

/**
 * @brief Build graph Laplacian matrix L = D - A
 * 
 * @param adjacency Adjacency matrix A
 * @param verbose Print progress information
 * @return Sparse Laplacian matrix L
 */
Eigen::SparseMatrix<double> buildGraphLaplacian(const Eigen::SparseMatrix<double>& adjacency, bool verbose = false);

/**
 * @brief Solve Dirichlet problem using sparse linear solver
 * 
 * Solves L_FF * x_F = -L_FT * x_T for movable vertex coordinates.
 * 
 * @param L_FF Laplacian block for movable vertices
 * @param L_FT Laplacian block coupling movable and fixed vertices
 * @param x_T Fixed vertex coordinates
 * @param params Solver parameters
 * @param solver_time_ms Output: time spent in solver
 * @param converged Output: whether solver converged
 * @param residual Output: final solver residual
 * @return Computed coordinates for movable vertices
 */
Eigen::VectorXd solveDirichletProblem(
    const Eigen::SparseMatrix<double>& L_FF,
    const Eigen::SparseMatrix<double>& L_FT,
    const Eigen::VectorXd& x_T,
    const DirichletEmbeddingParams& params,
    double& solver_time_ms,
    bool& converged,
    double& residual
);

/**
 * @brief Create ordering from coordinates by sorting
 * 
 * @param coordinates Computed coordinates for vertices
 * @param vertex_ids Original vertex IDs corresponding to coordinates
 * @return Ordering of vertex IDs sorted by coordinate
 */
std::vector<int> createOrderingFromCoordinates(
    const Eigen::VectorXd& coordinates,
    const std::vector<int>& vertex_ids
);

/**
 * @brief Utility function to validate fixed vertex specifications
 * 
 * @param hg Input hypergraph
 * @param fixed_vertices Fixed vertex specifications
 * @return True if all fixed vertices are valid
 */
bool validateFixedVertices(const Hypergraph& hg, const std::vector<FixedVertex>& fixed_vertices);

/**
 * @brief Create a simple test case with pre-placed IOs
 * 
 * Generates a test hypergraph with some vertices designated as IOs with fixed coordinates.
 * Useful for testing and demonstration.
 * 
 * @param num_vertices Total number of vertices
 * @param num_hyperedges Number of hyperedges
 * @param num_ios Number of IO vertices to fix
 * @param hg Output: generated hypergraph
 * @param fixed_vertices Output: generated fixed vertex specifications
 */
void createTestCaseWithIOs(
    int num_vertices,
    int num_hyperedges, 
    int num_ios,
    Hypergraph& hg,
    std::vector<FixedVertex>& fixed_vertices
);

/**
 * @brief Read pre-placed coordinates from file
 * 
 * Reads a file where each line contains: vertex_id x_coordinate y_coordinate
 * For Dirichlet embedding, only the x_coordinate is used.
 * 
 * @param filename Path to the pre-placed coordinates file
 * @return Vector of FixedVertex specifications
 */
std::vector<FixedVertex> readPreplacedCoordinates(const std::string& filename);

/**
 * @brief Write pre-placed coordinates to file
 * 
 * Writes coordinates in the standard format: vertex_id x_coordinate y_coordinate
 * For vertices without y_coordinates, writes 0.0 as placeholder.
 * 
 * @param fixed_vertices Vector of fixed vertex specifications
 * @param filename Output file path
 * @param y_coordinate Default Y coordinate to write (default: 0.0)
 */
void writePreplacedCoordinates(
    const std::vector<FixedVertex>& fixed_vertices,
    const std::string& filename,
    double y_coordinate = 0.0
);

/**
 * @brief Write Dirichlet embedding result as pre-placed coordinates
 * 
 * Writes both fixed and computed movable vertex coordinates to file.
 * Useful for saving the result for further processing or visualization.
 * 
 * @param result Dirichlet embedding result
 * @param fixed_vertices Original fixed vertex specifications
 * @param filename Output file path
 * @param y_coordinate Default Y coordinate to write (default: 0.0)
 */
void writeDirichletResultAsCoordinates(
    const DirichletEmbeddingResult& result,
    const std::vector<FixedVertex>& fixed_vertices,
    const std::string& filename,
    double y_coordinate = 0.0
);

/**
 * @brief Compute soft-anchored spectral embedding using penalty method or virtual springs
 * @param hg Input hypergraph
 * @param fixed_vertices IO vertices with fixed coordinates
 * @param params Embedding parameters
 * @return Complete embedding result
 */
SoftAnchoredEmbeddingResult computeSoftAnchoredEmbedding(
    const Hypergraph& hg,
    const std::vector<FixedVertex>& fixed_vertices,
    const SoftAnchoredEmbeddingParams& params = SoftAnchoredEmbeddingParams()
);

/**
 * @brief Penalty method implementation (Variant 1)
 * @param hg Input hypergraph
 * @param fixed_vertices IO vertices with fixed coordinates
 * @param params Embedding parameters
 * @return Complete embedding result
 */
SoftAnchoredEmbeddingResult computePenaltyMethodEmbedding(
    const Hypergraph& hg,
    const std::vector<FixedVertex>& fixed_vertices,
    const SoftAnchoredEmbeddingParams& params
);

/**
 * @brief Virtual springs method implementation (Variant 2)
 * @param hg Input hypergraph
 * @param fixed_vertices IO vertices with fixed coordinates
 * @param params Embedding parameters
 * @return Complete embedding result
 */
SoftAnchoredEmbeddingResult computeVirtualSpringsEmbedding(
    const Hypergraph& hg,
    const std::vector<FixedVertex>& fixed_vertices,
    const SoftAnchoredEmbeddingParams& params
);

/**
 * @brief Write soft-anchored embedding result as coordinate file
 * @param result Soft-anchored embedding result
 * @param fixed_vertices Original fixed vertices
 * @param filename Output filename
 * @param y_coordinate Y-coordinate for all vertices (default 0.0)
 */
void writeSoftAnchoredResultAsCoordinates(
    const SoftAnchoredEmbeddingResult& result,
    const std::vector<FixedVertex>& fixed_vertices,
    const std::string& filename,
    double y_coordinate = 0.0
);

// ============================================================================
// BLOCK-BASED GREEDY REFINEMENT WITH IO CONSTRAINTS
// ============================================================================

/**
 * @brief Parameters for block-based refinement with IO constraints
 */
struct BlockBasedRefinementParams {
    int max_iterations_per_block = 10;      // Iterations within each block
    int max_global_iterations = 5;          // Global passes over all blocks
    bool use_hyperedge_aware = true;        // Use priority swap candidates
    bool verbose = false;                   // Debug output
    double convergence_threshold = 0.01;    // Stop if improvement < threshold
    bool verify_io_order = true;            // Check IO coordinate ordering
    int max_distance_swaps = 3;             // Maximum distance for intra-block swaps
    bool optimize_boundaries = true;        // Optimize at block boundaries
};

/**
 * @brief Information about a refinement block between IOs
 */
struct RefinementBlock {
    int start_pos;                          // Start position in ordering (inclusive)
    int end_pos;                            // End position in ordering (inclusive)
    std::vector<int> movable_vertices;      // Vertex IDs in this block
    int initial_peak_contribution;          // Peak cutwidth contribution before refinement
    int final_peak_contribution;            // Peak cutwidth contribution after refinement
    int swaps_performed;                    // Number of beneficial swaps in this block
    int block_size;                         // Number of movable vertices in block
    
    RefinementBlock() : start_pos(-1), end_pos(-1), initial_peak_contribution(0),
                       final_peak_contribution(0), swaps_performed(0), block_size(0) {}
};

/**
 * @brief Result of block-based refinement with IO constraints
 */
struct BlockBasedRefinementResult {
    std::vector<int> ordering;              // Final refined ordering
    std::vector<RefinementBlock> blocks;    // Information about each block
    int initial_peak;                       // Initial peak cutwidth
    int final_peak;                         // Final peak cutwidth
    int total_swaps;                        // Total beneficial swaps across all blocks
    int global_iterations;                  // Number of global iterations performed
    double computation_time_ms;             // Total computation time
    bool converged;                         // Whether algorithm converged
    bool io_order_corrected;                // Whether IO order was corrected
    int num_ios;                            // Number of IO vertices
    int num_movable;                        // Number of movable vertices
    
    BlockBasedRefinementResult() : initial_peak(0), final_peak(0), total_swaps(0),
                                  global_iterations(0), computation_time_ms(0.0),
                                  converged(false), io_order_corrected(false),
                                  num_ios(0), num_movable(0) {}
};

/**
 * @brief Enhanced greedy refinement with IO constraints and block-based optimization
 * 
 * This function implements an advanced refinement algorithm that:
 * 1. Ensures IOs are ordered by their placement coordinates
 * 2. Partitions the ordering into blocks between consecutive IOs
 * 3. Performs intensive greedy refinement within each block independently
 * 4. Maintains IO positions as hard barriers throughout refinement
 * 5. Optimizes block boundaries and applies distance swaps within blocks
 * 
 * @param hg Input hypergraph
 * @param initial_ordering Initial vertex ordering
 * @param fixed_vertices IO vertices with placement coordinates
 * @param params Refinement parameters
 * @return Complete refinement result with block statistics
 */
BlockBasedRefinementResult applyBlockBasedGreedyRefinement(
    const Hypergraph& hg,
    const std::vector<int>& initial_ordering,
    const std::vector<FixedVertex>& fixed_vertices,
    const BlockBasedRefinementParams& params = BlockBasedRefinementParams()
);

/**
 * @brief Ensure IOs are ordered by their placement coordinates
 * 
 * @param initial_ordering Current vertex ordering
 * @param fixed_vertices IO vertices with coordinates
 * @param order_corrected Output: whether order was corrected
 * @return Corrected ordering with IOs sorted by coordinate
 */
std::vector<int> ensureCorrectIOOrder(
    const std::vector<int>& initial_ordering,
    const std::vector<FixedVertex>& fixed_vertices,
    bool& order_corrected
);

/**
 * @brief Identify refinement blocks between consecutive IOs
 * 
 * @param ordering Current vertex ordering
 * @param fixed_vertex_set Set of IO vertex IDs
 * @return Vector of refinement blocks
 */
std::vector<RefinementBlock> identifyRefinementBlocks(
    const std::vector<int>& ordering,
    const std::unordered_set<int>& fixed_vertex_set
);

/**
 * @brief Parameters for IO-anchored refinement (allows cross-block moves)
 */
struct IOAnchoredRefinementParams {
    int max_iterations = 50;                // Maximum refinement iterations
    bool use_hyperedge_aware = true;        // Use priority swap candidates
    bool verbose = false;                   // Debug output
    double convergence_threshold = 0.01;    // Stop if improvement < threshold
    int max_distance_swaps = 5;             // Maximum distance for swaps
    bool use_insertion_moves = true;        // Allow insertion moves (not just swaps)
    int max_insertion_distance = 10;       // Maximum distance for insertion moves
    bool optimize_io_neighbors = true;     // Special optimization near IOs
};

/**
 * @brief Result of IO-anchored refinement
 */
struct IOAnchoredRefinementResult {
    std::vector<int> ordering;              // Final refined ordering
    int initial_peak;                       // Initial peak cutwidth
    int final_peak;                         // Final peak cutwidth
    int total_swaps;                        // Total beneficial swaps performed
    int total_insertions;                   // Total beneficial insertions performed
    int iterations;                         // Number of iterations performed
    double computation_time_ms;             // Total computation time
    bool converged;                         // Whether algorithm converged
    std::vector<int> io_positions;          // Fixed IO positions (for verification)
    int num_ios;                            // Number of IO vertices
    int num_movable;                        // Number of movable vertices
    
    IOAnchoredRefinementResult() : initial_peak(0), final_peak(0), total_swaps(0),
                                  total_insertions(0), iterations(0), computation_time_ms(0.0),
                                  converged(false), num_ios(0), num_movable(0) {}
};

/**
 * @brief Enhanced IO-anchored greedy refinement allowing cross-block moves
 * 
 * This function implements an advanced refinement algorithm that:
 * 1. Keeps IO vertices fixed at their current positions (no IO reordering)
 * 2. Allows standard cells to move freely across the entire ordering
 * 3. Uses both swap and insertion operations for optimization
 * 4. Provides much larger search space than block-based approach
 * 5. Maintains IO ordering constraints while maximizing optimization potential
 * 
 * @param hg Input hypergraph
 * @param initial_ordering Initial vertex ordering
 * @param fixed_vertices IO vertices with placement coordinates
 * @param params Refinement parameters
 * @return Complete refinement result with statistics
 */
IOAnchoredRefinementResult applyIOAnchoredGreedyRefinement(
    const Hypergraph& hg,
    const std::vector<int>& initial_ordering,
    const std::vector<FixedVertex>& fixed_vertices,
    const IOAnchoredRefinementParams& params = IOAnchoredRefinementParams()
);

/**
 * @brief Enhanced IO-anchored refinement using vertex set interface
 * 
 * @param hg Input hypergraph
 * @param initial_ordering Initial vertex ordering
 * @param fixed_vertex_set Set of IO vertex IDs
 * @param params Refinement parameters
 * @return Complete refinement result with statistics
 */
IOAnchoredRefinementResult applyIOAnchoredGreedyRefinement(
    const Hypergraph& hg,
    const std::vector<int>& initial_ordering,
    const std::unordered_set<int>& fixed_vertex_set,
    const IOAnchoredRefinementParams& params = IOAnchoredRefinementParams()
); 