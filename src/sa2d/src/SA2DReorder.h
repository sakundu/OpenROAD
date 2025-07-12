#pragma once

#include <vector>
#include <unordered_map>
#include <memory>

#include "infrastructure/Coordinates.h"
#include "infrastructure/network.h"
#include "infrastructure/architecture.h"
#include "ThreadSafeGrid.h"

namespace utl {
class Logger;
}

namespace odb {
class dbBlock;
}

namespace sa2d {

// Forward declarations
class ImmutableGridInfo;
class WorkerGrid;

class SA2DReorderer {
public:
    SA2DReorderer(const dpl::Architecture* arch, 
                  dpl::Network* network,  // Non-const to allow calling getNode()
                  const ImmutableGridInfo* grid_info,
                  utl::Logger* logger,
                  odb::dbBlock* block,
                  WorkerGrid* grid = nullptr,  // Optional grid to keep synchronized
                  int max_displacement_x = 500,  // Default from DPL
                  int max_displacement_y = 100);  // Default from DPL
    
    // Run reordering on the current DPL network state
    // Returns true if any improvement was made
    bool reorderPlacement(int passes = 1, double tolerance = 0.01, int window_size = 3);
    
    // Get statistics from the last reordering run
    struct ReorderStats {
        int64_t initial_hpwl;
        int64_t final_hpwl;
        double improvement_percent;
        int windows_processed;
        int permutations_tried;
        int improvements_found;
    };
    
    const ReorderStats& getStats() const { return stats_; }
    
private:
    // Core reordering logic
    void reorderPass();
    void processRowWindows(const std::vector<int>& row_cells, int row_id);
    void reorderSegment(int seg_start, int seg_end, 
                       DbuX left_limit, DbuX right_limit);
    
    // Cost evaluation (matching DPL's approach)
    double calculateCost(const std::vector<int>& cell_ids, 
                        int start_idx, int end_idx) const;
    
    // Permutation handling
    bool tryAllPermutations(const std::vector<int>& cell_ids,
                           int start_idx, int end_idx,
                           DbuX left_limit, DbuX right_limit);
    
    // Utility functions
    bool isValidForReordering(int cell_id) const;
    std::vector<int> getCellsInRow(int row_id) const;
    std::vector<int> getAffectedNets(int cell_id) const;
    int64_t calculateTotalHPWL() const;
    
    // Position validation
    bool validatePositions(const std::vector<int>& cell_ids,
                          const std::vector<DbuX>& positions) const;
    
    // Apply the best found permutation
    void applyBestPermutation(const std::vector<int>& cell_ids,
                             const std::vector<DbuX>& best_positions);
    
    // References to shared infrastructure
    const dpl::Architecture* arch_;
    dpl::Network* network_;  // Non-const to allow calling getNode()
    const ImmutableGridInfo* grid_info_;
    utl::Logger* logger_;
    odb::dbBlock* block_;
    WorkerGrid* grid_;  // Optional grid to keep synchronized with DPL moves
    
    // Displacement limits (in sites)
    int max_displacement_x_;
    int max_displacement_y_;
    
    // Reordering parameters
    int window_size_;
    double tolerance_;
    int skip_nets_larger_than_;
    
    // Statistics tracking
    ReorderStats stats_;
    
    // Edge mask for cost calculation (following DPL pattern)
    mutable std::vector<int> edge_mask_;
    mutable int traversal_;
};

} // namespace sa2d 