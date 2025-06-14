#pragma once

#include "sait/hypergraph.hpp"
#include <vector>
#include <string>

/**
 * @brief Space-Filling Curve strategies for ordering
 */
enum class SFCStrategy {
    HILBERT_2D,      // Hilbert curve in 2D
    HILBERT_3D,      // Hilbert curve in 3D  
    ZORDER_2D,       // Z-order (Morton code) in 2D
    ZORDER_3D,       // Z-order (Morton code) in 3D
    LEXICOGRAPHIC_2D, // Lexicographic sort of (x,y)
    LEXICOGRAPHIC_3D  // Lexicographic sort of (x,y,z)
};

/**
 * @brief 2D point for vertex embedding
 */
struct Point2D {
    double x, y;
    int vertex_id;
    
    Point2D() : x(0.0), y(0.0), vertex_id(-1) {}
    Point2D(double x, double y, int id) : x(x), y(y), vertex_id(id) {}
};

/**
 * @brief 3D point for vertex embedding
 */
struct Point3D {
    double x, y, z;
    int vertex_id;
    
    Point3D() : x(0.0), y(0.0), z(0.0), vertex_id(-1) {}
    Point3D(double x, double y, double z, int id) : x(x), y(y), z(z), vertex_id(id) {}
};

/**
 * @brief Spectral + SFC ordering parameters
 */
struct SFCOrderingParams {
    int num_eigenvectors;        // 2 or 3 eigenvectors to use
    SFCStrategy sfc_strategy;    // Space-filling curve strategy
    int hilbert_order;           // Order parameter for Hilbert curve (typically 10-16)
    bool normalize_coordinates;  // Whether to normalize coordinates to [0,1]
    
    // Default parameters
    SFCOrderingParams() 
        : num_eigenvectors(2), sfc_strategy(SFCStrategy::HILBERT_2D), 
          hilbert_order(12), normalize_coordinates(true) {}
};

/**
 * @brief Result of spectral + SFC ordering computation
 */
struct SFCOrderingResult {
    std::vector<int> ordering;           // Vertex ordering
    std::vector<Point2D> embedding_2d;   // 2D embedding (if applicable)
    std::vector<Point3D> embedding_3d;   // 3D embedding (if applicable)
    std::vector<double> eigenvalues;     // Computed eigenvalues
    int dimensions_used;                 // 2 or 3
    SFCStrategy strategy_used;           // Strategy that was applied
    double computation_time_ms;          // Total computation time
    
    SFCOrderingResult() : dimensions_used(0), strategy_used(SFCStrategy::LEXICOGRAPHIC_2D), 
                         computation_time_ms(0.0) {}
};

/**
 * @brief Compute Spectral + Space-Filling Curve ordering
 * @param hg Input hypergraph
 * @param params Ordering parameters
 * @return Complete SFC ordering result with embeddings and metadata
 */
SFCOrderingResult computeSpectralSFCOrdering(const Hypergraph& hg, 
                                            const SFCOrderingParams& params = SFCOrderingParams());

/**
 * @brief Compute Spectral + SFC ordering (simple interface)
 * @param hg Input hypergraph
 * @param strategy Space-filling curve strategy to use
 * @return Vertex ordering vector
 */
std::vector<int> computeSFCOrdering(const Hypergraph& hg, 
                                   SFCStrategy strategy = SFCStrategy::HILBERT_2D);

/**
 * @brief Convert SFC strategy to string for display
 */
std::string sfcStrategyToString(SFCStrategy strategy);

/**
 * @brief Hilbert curve ordering in 2D
 * @param points 2D points to order
 * @param order Hilbert curve order parameter
 * @return Ordered vertex IDs
 */
std::vector<int> hilbertOrder2D(const std::vector<Point2D>& points, int order);

/**
 * @brief Hilbert curve ordering in 3D
 * @param points 3D points to order
 * @param order Hilbert curve order parameter
 * @return Ordered vertex IDs
 */
std::vector<int> hilbertOrder3D(const std::vector<Point3D>& points, int order);

/**
 * @brief Z-order (Morton code) ordering in 2D
 * @param points 2D points to order
 * @return Ordered vertex IDs
 */
std::vector<int> zOrder2D(const std::vector<Point2D>& points);

/**
 * @brief Z-order (Morton code) ordering in 3D
 * @param points 3D points to order
 * @return Ordered vertex IDs
 */
std::vector<int> zOrder3D(const std::vector<Point3D>& points);

/**
 * @brief Lexicographic ordering in 2D
 * @param points 2D points to order
 * @return Ordered vertex IDs
 */
std::vector<int> lexicographicOrder2D(const std::vector<Point2D>& points);

/**
 * @brief Lexicographic ordering in 3D
 * @param points 3D points to order
 * @return Ordered vertex IDs
 */
std::vector<int> lexicographicOrder3D(const std::vector<Point3D>& points);

/**
 * @brief Normalize coordinates to [0,1] range
 * @param points 2D points to normalize (in-place)
 */
void normalizeCoordinates2D(std::vector<Point2D>& points);

/**
 * @brief Normalize coordinates to [0,1] range
 * @param points 3D points to normalize (in-place)
 */
void normalizeCoordinates3D(std::vector<Point3D>& points); 