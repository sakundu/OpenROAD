#pragma once

#include <vector>
#include <string>
#include <memory>
#include <functional>
#include "odb/db.h"
#include "sa1d/VertexOrdering.h"

// Include SAIT best-orderings headers
#include "sait/hypergraph.hpp"
#include "sait/dirichlet_ordering.hpp"

namespace utl {
class Logger;
}

namespace sa1d {

// Forward declarations
class OptSA;

/**
 * @brief Result from best-orderings computation
 */
struct BestOrderingsResult {
    struct OrderingInfo {
        std::string algorithm_name;
        std::vector<int> cell_ordering;
        std::vector<odb::dbOrientType> orientations;
        double initial_hpwl = 0.0;
        double final_hpwl = 0.0;
        double computation_time_ms = 0.0;
        int initial_peak_cutwidth = 0;
        int final_peak_cutwidth = 0;
        double improvement_percentage = 0.0;
        
        OrderingInfo() = default;
    };
    
    std::vector<OrderingInfo> top_orderings;  // Top 6 orderings
    bool success = false;
    std::string error_message = "";
    double total_computation_time_ms = 0.0;
    int algorithms_tested = 0;
    
    // Convenience accessors  
    const OrderingInfo& getBest() const { return top_orderings[0]; }
    const OrderingInfo& getSecondBest() const { return top_orderings.size() > 1 ? top_orderings[1] : top_orderings[0]; }
    const OrderingInfo& getThirdBest() const { return top_orderings.size() > 2 ? top_orderings[2] : top_orderings[0]; }
    const OrderingInfo& get(size_t index) const { return index < top_orderings.size() ? top_orderings[index] : top_orderings[0]; }
    
    size_t getCount() const { return top_orderings.size(); }
    bool hasMultiple() const { return top_orderings.size() > 1; }
    
    BestOrderingsResult() = default;
};

/**
 * @brief Parameters for best-orderings computation
 */
struct BestOrderingsParams {
    bool verbose = false;
    bool use_parallel = true;
    int max_threads = 0;  // 0 = auto-detect
    int top_count = 6;    // Number of top orderings to return
    bool include_advanced_methods = true;  // Include Dirichlet, etc. if terminal file exists
    bool apply_refinement = true;          // Apply greedy refinement to each
    bool use_constrained_refinement = false;  // Use IO-constrained refinement if terminal file exists
    
    BestOrderingsParams() = default;
};

/**
 * @brief Interface for SAIT best-orderings functionality
 */
class BestOrderingsInterface {
public:
    BestOrderingsInterface(OptSA* opt_sa);
    ~BestOrderingsInterface();
    
    /**
     * @brief Compute best orderings using all available SAIT algorithms
     */
    BestOrderingsResult computeBestOrderings(const BestOrderingsParams& params = BestOrderingsParams());
    
    /**
     * @brief Initialize SA1D workers with best orderings
     * @param result The result from computeBestOrderings
     * @param num_workers Total number of SA workers
     * @return Vector of worker initialization data
     */
    struct WorkerInitData {
        std::vector<int> cell_ordering;
        std::vector<odb::dbOrientType> orientations;
        std::string algorithm_name;
        bool use_ordering;  // If false, worker should use random initialization
    };
    
    std::vector<WorkerInitData> prepareWorkerInitialization(
        const BestOrderingsResult& result, 
        int num_workers);
    
private:
    OptSA* opt_sa_;
    odb::dbDatabase* db_;
    odb::dbBlock* block_;
    utl::Logger* logger_;
    
    // SAIT algorithm implementations (using function pointers for flexibility)
    struct AlgorithmInfo {
        std::string name;
        std::function<std::vector<int>(const ::Hypergraph&)> compute_func;
        bool requires_terminal_file = false;
    };
    
    std::vector<AlgorithmInfo> getAvailableAlgorithms(bool include_advanced);
    
    // Hypergraph conversion (shared with VertexOrdering)
    std::unique_ptr<::Hypergraph> convertToSAITHypergraph();
    
    // Helper methods
    std::vector<int> applyRefinement(const ::Hypergraph& hg, 
                                   const std::vector<int>& vertex_ordering,
                                   const std::string& algorithm_name,
                                   bool use_constrained);
    
    BestOrderingsResult::OrderingInfo convertToOrderingInfo(
        const std::string& algorithm_name,
        const std::vector<int>& vertex_ordering,
        const ::Hypergraph& hg,
        double computation_time_ms);
    
    double computeHPWL(const std::vector<int>& cell_ordering);
    bool validateOrdering(const std::vector<int>& ordering);
    
    // ID mapping (shared with VertexOrdering)
    std::unordered_map<int, int> cell_to_vertex_map_;
    std::unordered_map<int, int> vertex_to_cell_map_;
    
    // I/O terminal mappings
    std::unordered_map<size_t, int> io_net_to_vertex_map_;  // net_id -> vertex_id
    std::unordered_map<int, size_t> vertex_to_io_net_map_;  // vertex_id -> net_id
    
    // Helper methods for I/O integration
    bool hasIOTerminals() const;
    std::vector<int> filterCellVertices(const std::vector<int>& vertex_ordering, int num_cells) const;
    std::vector<::FixedVertex> convertToSAITFixedVertices() const;
    std::vector<::FixedVertex> extractIOCoordinates() const;
};

} // namespace sa1d 