#pragma once

#include <vector>
#include <string>
#include <memory>
#include <unordered_map>
#include "odb/db.h"
#include "sa1d/NetOverlap.h"

// Include SAIT headers
#include "sait/hypergraph.hpp"
#include "sait/fiedler_ordering.hpp"
#include "sait/rcm_ordering.hpp"
#include "sait/random_ordering.hpp"

namespace utl {
class Logger;
}

namespace sa1d {

// Forward declaration
class OptSA;

/**
 * @brief Vertex ordering methods available
 */
enum class OrderingMethod {
    RANDOM,              // Default SA1D random initialization
    SIZE_BASED,          // Default SA1D size-based initialization  
    FIEDLER,            // SAIT Fiedler vector ordering
    RCM,                // SAIT Reverse Cuthill-McKee
    RCM_BOOST           // SAIT RCM using Boost Graph Library
};

/**
 * @brief Parameters for vertex ordering
 */
struct VertexOrderingParams {
    OrderingMethod method = OrderingMethod::RANDOM;
    bool verbose = false;                   // Debug output
    
    VertexOrderingParams() = default;
};

/**
 * @brief Result of vertex ordering computation
 */
struct VertexOrderingResult {
    std::vector<int> cell_ordering;         // Ordered cell IDs (SA1D format)
    std::vector<odb::dbOrientType> orientations; // Cell orientations
    double initial_hpwl = 0.0;             // HPWL before ordering
    double final_hpwl = 0.0;               // HPWL after ordering
    double computation_time_ms = 0.0;       // Time spent in ordering
    bool success = false;                   // Whether ordering succeeded
    std::string algorithm_used = "";        // Algorithm that was used
    std::string error_message = "";         // Error details if failed
    
    // SAIT-specific metrics
    int vertices_processed = 0;             // Number of vertices in hypergraph
    int hyperedges_processed = 0;           // Number of hyperedges
    
    // Overlap/Cutwidth metrics
    int initial_peak_cutwidth = 0;          // Peak cutwidth before ordering
    int final_peak_cutwidth = 0;           // Peak cutwidth after ordering
    int initial_overlap = 0;                // Peak overlap before ordering
    int final_overlap = 0;                  // Peak overlap after ordering
    
    VertexOrderingResult() = default;
};

/**
 * @brief Main vertex ordering interface for SA1D integration
 */
class VertexOrderingInterface {
public:
    VertexOrderingInterface(OptSA* opt_sa);
    ~VertexOrderingInterface();
    
    /**
     * @brief Compute vertex ordering for SA1D initialization
     */
    VertexOrderingResult computeOrdering(const VertexOrderingParams& params);
    
    /**
     * @brief Get string representation of ordering method
     */
    static std::string orderingMethodToString(OrderingMethod method);
    
private:
    OptSA* opt_sa_;
    odb::dbDatabase* db_;
    odb::dbBlock* block_;
    utl::Logger* logger_;
    
    // Simple SA1D methods
    VertexOrderingResult computeRandomOrdering();
    VertexOrderingResult computeSizeBasedOrdering();
    
    // SAIT integration methods
    std::unique_ptr<Hypergraph> convertToSAITHypergraph();
    VertexOrderingResult convertFromSAITOrdering(const std::vector<int>& vertex_ordering);
    VertexOrderingResult runSAITAlgorithm(const Hypergraph& hg, const VertexOrderingParams& params);
    
    // ID mapping between SA1D cells and SAIT vertices
    std::unordered_map<int, int> cell_to_vertex_map_;
    std::unordered_map<int, int> vertex_to_cell_map_;
    
    // I/O Terminal tracking
    std::unordered_map<size_t, int> io_net_to_vertex_map_;  // net_id -> vertex_id for I/O terminals
    std::unordered_map<int, size_t> vertex_to_io_net_map_;  // vertex_id -> net_id for I/O terminals
    
    // Helper methods
    double computeCurrentHPWL();
    double computeOrderingHPWL(const std::vector<int>& cell_ordering);
    
    // Overlap computation methods
    int computeCurrentOverlap();
    int computeOrderingOverlap(const std::vector<int>& cell_ordering);
    
    // Error handling
    bool validateHypergraph(const Hypergraph& hg);
    
    // Helper methods for I/O terminal handling
    bool hasIOTerminals() const;
};

} // namespace sa1d 