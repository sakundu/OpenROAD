// SA2D Reordering Implementation
// ==============================
// This implementation follows DPL's reordering algorithm with these key features:
// 
// MATCHES DPL:
// ✓ Window sizes 2-4 (default 3)
// ✓ Skips multi-height cells
// ✓ Finds continuous runs of single-height cells
// ✓ Calculates left/right limits based on neighboring cells
// ✓ Distributes extra space evenly among cells
// ✓ Tries all permutations
// ✓ Evaluates cost using X-direction HPWL only
// ✓ Checks displacement limits
// ✓ Performs site alignment after reordering
// ✓ Uses edge mask to avoid double-counting nets
// ✓ Skips nets larger than 100 pins
// ✓ Reverts if no improvement after site alignment
//
// DIFFERENCES FROM DPL:
// - Processes entire rows instead of segments (between blockages)
// - Maintains SA2D grid state in addition to DPL network
// - No placement violation checking (DPL's hasPlacementViolation)
//
// TODO: Add segment-based processing to handle blockages properly

#include "SA2DReorder.h"
#include "ThreadSafeGrid.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "utl/Logger.h"
#include "infrastructure/architecture.h"
#include "infrastructure/network.h"

using utl::SA2D;

namespace sa2d {

SA2DReorderer::SA2DReorderer(const dpl::Architecture* arch,
                             dpl::Network* network,
                             const ImmutableGridInfo* grid_info,
                             utl::Logger* logger,
                             odb::dbBlock* block,
                             WorkerGrid* grid,
                             int max_displacement_x,
                             int max_displacement_y)
    : arch_(arch), 
      network_(network), 
      grid_info_(grid_info), 
      logger_(logger),
      block_(block),
      grid_(grid),
      max_displacement_x_(max_displacement_x),
      max_displacement_y_(max_displacement_y),
      window_size_(3),
      tolerance_(0.01),
      skip_nets_larger_than_(100),
      traversal_(0)
{
    // Initialize edge mask for cost calculation
    edge_mask_.resize(network_->getNumEdges());
    std::fill(edge_mask_.begin(), edge_mask_.end(), 0);
    
    // Initialize stats
    stats_ = {};
}

bool SA2DReorderer::reorderPlacement(int passes, double tolerance, int window_size)
{
    // FIXED: Grid synchronization implemented - reorderer now updates both DPL network 
    // and SA2D grid state to prevent overlaps
    
    window_size_ = std::min(4, std::max(2, window_size));
    tolerance_ = std::max(tolerance, 0.01);
    
    logger_->info(SA2D, 600, "Starting SA2D reordering with window size {}, {} passes, tolerance {:.3f}",
                  window_size_, passes, tolerance_);
    
    // Calculate initial HPWL 
    stats_.initial_hpwl = calculateTotalHPWL();
    int64_t curr_hpwl = stats_.initial_hpwl;
    
    if (curr_hpwl == 0) {
        logger_->info(SA2D, 601, "Skipping reordering - HPWL is zero");
        return false;
    }
    
    // Initialize statistics
    stats_.windows_processed = 0;
    stats_.permutations_tried = 0;
    stats_.improvements_found = 0;
    
    // Run multiple passes
    for (int pass = 1; pass <= passes; pass++) {
        const int64_t last_hpwl = curr_hpwl;
        
        // Perform one reordering pass
        reorderPass();
        
        // Calculate new HPWL
        curr_hpwl = calculateTotalHPWL();
        
                 logger_->info(SA2D, 602, "Pass {} of reordering; HPWL is {:.1f} u",
                       pass, block_->dbuToMicrons(curr_hpwl));
        
        // Check for early termination
        if (last_hpwl == 0 || 
            std::abs(curr_hpwl - last_hpwl) / (double)last_hpwl <= tolerance_) {
            break;
        }
    }
    
    // Calculate final statistics
    stats_.final_hpwl = curr_hpwl;
    stats_.improvement_percent = ((stats_.initial_hpwl - stats_.final_hpwl) / 
                                 (double)stats_.initial_hpwl) * 100.0;
    
         logger_->info(SA2D, 603, "End of SA2D reordering; HPWL is {:.1f} u, improvement is {:.2f}%",
                   block_->dbuToMicrons(stats_.final_hpwl),
                   stats_.improvement_percent);
    
    return stats_.improvement_percent > 0.01;  // Return true if significant improvement
}

void SA2DReorderer::reorderPass()
{
    traversal_ = 0;
    std::fill(edge_mask_.begin(), edge_mask_.end(), traversal_);
    
    // NOTE: Key difference from DPL - SA2D processes entire rows, not segments
    // DPL processes segments (portions of rows between blockages/fixed cells)
    // This means SA2D might try to reorder across fixed cells, which could
    // be problematic. A future improvement would be to segment rows properly.
    
    // Process each row
    for (int row_id = 0; row_id < arch_->getNumRows(); row_id++) {
        std::vector<int> row_cells = getCellsInRow(row_id);
        
        if (row_cells.size() < 2) {
            continue;  // Need at least 2 cells for reordering
        }
        
        // Sort cells by position (left to right)
        std::sort(row_cells.begin(), row_cells.end(), 
                  [this](int a, int b) {
                      const dpl::Node* node_a = network_->getNode(a);
                      const dpl::Node* node_b = network_->getNode(b);
                      return node_a->getLeft() < node_b->getLeft();
                  });
        
        // Process windows within the row
        processRowWindows(row_cells, row_id);
    }
}

void SA2DReorderer::processRowWindows(const std::vector<int>& row_cells, int row_id)
{
    const auto& row = arch_->getRow(row_id);
    const DbuX row_left = row->getLeft();
    const DbuX row_right = row->getRight();
    
    int n = row_cells.size();
    int j = 0;
    
    while (j < n) {
        // Skip multi-height cells (like DPL)
        while (j < n && arch_->isMultiHeightCell(network_->getNode(row_cells[j]))) {
            ++j;
        }
        
        const int jstart = j;
        
        // Find continuous run of single-height cells
        while (j < n && arch_->isSingleHeightCell(network_->getNode(row_cells[j]))) {
            ++j;
        }
        
        const int jstop = j - 1;
        
        // Process windows in this run of single-height cells
        for (int i = jstart; i + window_size_ <= jstop; ++i) {
            int window_start = i;
            int window_end = std::min(jstop, window_start + window_size_ - 1);
            
            // Adjust window to end if we're near the end
            if (window_end == jstop) {
                window_start = std::max(jstart, window_end - window_size_ + 1);
            }
            
            // Calculate limits for this window
            DbuX left_limit = row_left;
            DbuX right_limit = row_right;
            
            // Adjust limits based on neighboring cells
            if (window_start > 0) {
                const dpl::Node* prev_node = network_->getNode(row_cells[window_start - 1]);
                int left_padding, right_padding;
                arch_->getCellPadding(prev_node, left_padding, right_padding);
                left_limit = std::max(left_limit, prev_node->getRight() + right_padding);
            }
            
            if (window_end < n - 1) {
                const dpl::Node* next_node = network_->getNode(row_cells[window_end + 1]);
                int left_padding, right_padding;
                arch_->getCellPadding(next_node, left_padding, right_padding);
                right_limit = std::min(right_limit, next_node->getLeft() - left_padding);
            }
            
            // Extract cells for this window
            std::vector<int> window_cells;
            for (int k = window_start; k <= window_end; k++) {
                window_cells.push_back(row_cells[k]);
            }
            
            // Try to reorder this window
            tryAllPermutations(window_cells, 0, window_cells.size() - 1, 
                              left_limit, right_limit);
            
            stats_.windows_processed++;
        }
    }
}

bool SA2DReorderer::tryAllPermutations(const std::vector<int>& cell_ids,
                                       int start_idx, int end_idx,
                                       DbuX left_limit, DbuX right_limit)
{
    const int size = end_idx - start_idx + 1;
    if (size <= 1) return false;
    
    // Store original positions
    std::unordered_map<int, DbuX> original_positions;
    for (int i = start_idx; i <= end_idx; i++) {
        original_positions[cell_ids[i]] = network_->getNode(cell_ids[i])->getLeft();
    }
    
    // Calculate cell widths and padding
    std::vector<DbuX> cell_widths(size);
    std::vector<DbuX> left_padding(size);
    std::vector<DbuX> right_padding(size);
    DbuX total_width{0};
    DbuX total_padding{0};
    
    for (int i = 0; i < size; i++) {
        const dpl::Node* node = network_->getNode(cell_ids[start_idx + i]);
        cell_widths[i] = node->getWidth();
        
        int left_pad, right_pad;
        arch_->getCellPadding(node, left_pad, right_pad);
        left_padding[i] = DbuX{left_pad};
        right_padding[i] = DbuX{right_pad};
        
        total_width += cell_widths[i];
        total_padding += left_padding[i] + right_padding[i];
    }
    
    // Check if we have enough space
    if (right_limit - left_limit < total_width + total_padding) {
        return false;  // Not enough space
    }
    
    // DPL FEATURE: Distribute extra space evenly among cells
    const DbuX extra_space = (right_limit - left_limit) - (total_width + total_padding);
    const DbuX space_per_cell = extra_space / size;
    const DbuX site_width = arch_->getRow(0)->getSiteWidth();
    const int sites_per_cell_total = (space_per_cell / site_width).v;
    const int sites_per_cell_right = sites_per_cell_total >> 1;
    const int sites_per_cell_left = sites_per_cell_total - sites_per_cell_right;
    
    // Add extra spacing to padding (like DPL)
    for (int i = 0; i < size; i++) {
        if (total_width + total_padding + sites_per_cell_right * site_width < right_limit - left_limit) {
            total_padding += sites_per_cell_right * site_width;
            right_padding[i] += sites_per_cell_right * site_width;
        }
        if (total_width + total_padding + sites_per_cell_left * site_width < right_limit - left_limit) {
            total_padding += sites_per_cell_left * site_width;
            left_padding[i] += sites_per_cell_left * site_width;
        }
    }
    
    // Final check after adding extra spacing
    if (right_limit - left_limit < total_width + total_padding) {
        return false;  // Not enough space
    }
    
    // Calculate current cost
    double best_cost = calculateCost(cell_ids, start_idx, end_idx);
    const double original_cost = best_cost;
    
    std::vector<DbuX> best_positions(size);
    std::vector<int> order(size);
    for (int i = 0; i < size; i++) {
        order[i] = i;
    }
    
    bool found_improvement = false;
    
    // Try all permutations
    do {
        stats_.permutations_tried++;
        
        // Calculate positions for this permutation
        std::vector<DbuX> positions(size);
        DbuX x = left_limit;
        bool displacement_ok = true;
        
        for (int i = 0; i < size; i++) {
            const int cell_idx = order[i];
            x += left_padding[cell_idx];
            positions[cell_idx] = x;
            x += cell_widths[cell_idx];
            x += right_padding[cell_idx];
        }
        
        // Apply positions temporarily and check displacement
        for (int i = 0; i < size; i++) {
            dpl::Node* node = const_cast<dpl::Node*>(network_->getNode(cell_ids[start_idx + i]));
            
            // Update grid if provided
            if (grid_) {
                grid_->removeCell(cell_ids[start_idx + i]);
            }
            
            node->setLeft(positions[i]);
            
            // Update grid if provided
            if (grid_) {
                GridX new_gx = grid_info_->gridX(positions[i]);
                GridY new_gy = grid_info_->gridSnapDownY(node->getBottom());
                GridX gw = grid_info_->gridPaddedWidth(node);
                GridY gh = grid_info_->gridHeight(node);
                grid_->placeCell(cell_ids[start_idx + i], new_gx, new_gy, gw, gh);
            }
            
            // DPL FEATURE: Check displacement limits
            const DbuX dx = abs(node->getLeft() - node->getOrigLeft());
            const int dx_sites = dx.v / grid_info_->getSiteWidth();
            if (dx_sites > max_displacement_x_) {
                displacement_ok = false;
            }
        }
        
        if (displacement_ok) {
            // Calculate cost for this permutation
            double cost = calculateCost(cell_ids, start_idx, end_idx);
            
            if (cost < best_cost) {
                best_cost = cost;
                best_positions = positions;
                found_improvement = true;
            }
        }
        
    } while (std::next_permutation(order.begin(), order.end()));
    
    if (found_improvement) {
        // Apply best permutation to both DPL network and grid
        bool placement_ok = true;
        bool shifted = false;
        
        // First, place cells at best positions
        for (int i = 0; i < size; i++) {
            dpl::Node* node = const_cast<dpl::Node*>(network_->getNode(cell_ids[start_idx + i]));
            
            // Update grid if provided
            if (grid_) {
                grid_->removeCell(cell_ids[start_idx + i]);
            }
            
            node->setLeft(best_positions[i]);
            
            if (grid_) {
                GridX new_gx = grid_info_->gridX(best_positions[i]);
                GridY new_gy = grid_info_->gridSnapDownY(node->getBottom());
                GridX gw = grid_info_->gridPaddedWidth(node);
                GridY gh = grid_info_->gridHeight(node);
                
                if (grid_info_->isMultiHeightCell(node)) {
                    grid_->placeMultiHeightCell(cell_ids[start_idx + i], new_gx, new_gy, gw, gh);
                } else {
                    grid_->placeCell(cell_ids[start_idx + i], new_gx, new_gy, gw, gh);
                }
            }
        }
        
        // DPL FEATURE: Site alignment check
        // Sort cells by position after reordering
        std::vector<int> sorted_cells(cell_ids.begin() + start_idx, cell_ids.begin() + end_idx + 1);
        std::sort(sorted_cells.begin(), sorted_cells.end(), 
                  [this](int a, int b) {
                      return network_->getNode(a)->getLeft() < network_->getNode(b)->getLeft();
                  });
        
        // Check site alignment and fix if needed
        DbuX left = left_limit;
        for (int i = 0; i < size; i++) {
            dpl::Node* node = const_cast<dpl::Node*>(network_->getNode(sorted_cells[i]));
            
            DbuX x = node->getLeft();
            DbuX aligned_x = x;
            
            // Align to site grid
            const DbuX site_width = arch_->getRow(0)->getSiteWidth();
            const DbuX offset = (x - left_limit) % site_width;
            if (offset.v != 0) {
                // Snap to nearest site
                if (offset.v < site_width.v / 2) {
                    aligned_x = x - offset;
                } else {
                    aligned_x = x + (site_width - offset);
                }
                
                // Make sure we don't go outside bounds
                aligned_x = std::max(left, aligned_x);
                aligned_x = std::min(aligned_x, right_limit - node->getWidth());
            }
            
            if (abs(aligned_x - x) != 0) {
                shifted = true;
                
                // Update position
                if (grid_) {
                    grid_->removeCell(sorted_cells[i]);
                }
                
                node->setLeft(aligned_x);
                
                if (grid_) {
                    GridX new_gx = grid_info_->gridX(aligned_x);
                    GridY new_gy = grid_info_->gridSnapDownY(node->getBottom());
                    GridX gw = grid_info_->gridPaddedWidth(node);
                    GridY gh = grid_info_->gridHeight(node);
                    grid_->placeCell(sorted_cells[i], new_gx, new_gy, gw, gh);
                }
            }
            
            left = node->getRight();
            
            // Check displacement after alignment
            const DbuX dx = abs(node->getLeft() - node->getOrigLeft());
            const int dx_sites = dx.v / grid_info_->getSiteWidth();
            if (dx_sites > max_displacement_x_) {
                placement_ok = false;
                break;
            }
        }
        
        // If we shifted cells, recalculate cost
        if (placement_ok && shifted) {
            double new_cost = calculateCost(cell_ids, start_idx, end_idx);
            if (new_cost >= original_cost) {
                placement_ok = false;  // No improvement after alignment
            }
        }
        
        if (placement_ok) {
            stats_.improvements_found++;
            logger_->info(SA2D, 604, "Found improvement in window: {:.1f} -> {:.1f}",
                          original_cost, best_cost);
        } else {
            // Restore original positions if placement failed
            found_improvement = false;
        }
    }
    
    if (!found_improvement) {
        // Restore original positions in both DPL network and grid
        for (const auto& [cell_id, original_pos] : original_positions) {
            dpl::Node* node = const_cast<dpl::Node*>(network_->getNode(cell_id));
            
            // Update grid if provided - remove from current and place at original
            if (grid_) {
                grid_->removeCell(cell_id);
                
                GridX orig_gx = grid_info_->gridX(original_pos);
                GridY orig_gy = grid_info_->gridSnapDownY(node->getBottom());
                GridX gw = grid_info_->gridPaddedWidth(node);
                GridY gh = grid_info_->gridHeight(node);
                
                if (grid_info_->isMultiHeightCell(node)) {
                    grid_->placeMultiHeightCell(cell_id, orig_gx, orig_gy, gw, gh);
                } else {
                    grid_->placeCell(cell_id, orig_gx, orig_gy, gw, gh);
                }
            }
            
            // Restore DPL network
            node->setLeft(original_pos);
        }
    }
    
    return found_improvement;
}

double SA2DReorderer::calculateCost(const std::vector<int>& cell_ids, 
                                   int start_idx, int end_idx) const
{
    // NOTE: This matches DPL's cost calculation exactly:
    // - Only considers X-direction HPWL (not Y)
    // - Only for nets connected to cells in the window
    // - Uses edge mask to avoid double counting
    // - Skips nets with >100 pins
    
    ++traversal_;
    double cost = 0.0;
    
    for (int i = start_idx; i <= end_idx; i++) {
        const dpl::Node* node = network_->getNode(cell_ids[i]);
        
        // Process each pin of the cell
        for (int pin_idx = 0; pin_idx < node->getNumPins(); pin_idx++) {
            const dpl::Pin* pin = node->getPins()[pin_idx];
            const dpl::Edge* edge = pin->getEdge();
            
            const int num_pins = edge->getNumPins();
            if (num_pins <= 1 || num_pins >= skip_nets_larger_than_) {
                continue;
            }
            
            // Use edge mask to avoid double counting
            if (edge_mask_[edge->getId()] == traversal_) {
                continue;
            }
            edge_mask_[edge->getId()] = traversal_;
            
            // Calculate HPWL for this net (X-direction only, matching DPL)
            DbuX xmin = std::numeric_limits<DbuX>::max();
            DbuX xmax = std::numeric_limits<DbuX>::min();
            
            for (int j = 0; j < edge->getNumPins(); j++) {
                const dpl::Pin* net_pin = edge->getPins()[j];
                const dpl::Node* net_node = net_pin->getNode();
                
                const DbuX pin_x = net_node->getCenterX() + net_pin->getOffsetX();
                xmin = std::min(xmin, pin_x);
                xmax = std::max(xmax, pin_x);
            }
            
            cost += (xmax - xmin).v;
        }
    }
    
    return cost;
}

int64_t SA2DReorderer::calculateTotalHPWL() const
{
    int64_t total_hpwl = 0;
    
    for (int edge_idx = 0; edge_idx < network_->getNumEdges(); edge_idx++) {
        const dpl::Edge* edge = network_->getEdge(edge_idx);
        
        const int num_pins = edge->getNumPins();
        if (num_pins <= 1) continue;
        
        // Calculate bounding box
        DbuX xmin = std::numeric_limits<DbuX>::max();
        DbuX xmax = std::numeric_limits<DbuX>::min();
        DbuY ymin = std::numeric_limits<DbuY>::max();
        DbuY ymax = std::numeric_limits<DbuY>::min();
        
        for (int pin_idx = 0; pin_idx < num_pins; pin_idx++) {
            const dpl::Pin* pin = edge->getPins()[pin_idx];
            const dpl::Node* node = pin->getNode();
            
            const DbuX pin_x = node->getCenterX() + pin->getOffsetX();
            const DbuY pin_y = node->getCenterY() + pin->getOffsetY();
            
            xmin = std::min(xmin, pin_x);
            xmax = std::max(xmax, pin_x);
            ymin = std::min(ymin, pin_y);
            ymax = std::max(ymax, pin_y);
        }
        
        total_hpwl += (xmax - xmin).v + (ymax - ymin).v;
    }
    
    return total_hpwl;
}

std::vector<int> SA2DReorderer::getCellsInRow(int row_id) const
{
    std::vector<int> cells;
    
    for (int i = 0; i < network_->getNumNodes(); i++) {
        const dpl::Node* node = network_->getNode(i);
        
        if (node->getType() != dpl::Node::CELL || node->isFixed()) {
            continue;
        }
        
        // Check if cell is in this row
        const GridY cell_row = grid_info_->gridSnapDownY(node->getBottom());
        if (cell_row.v == row_id) {
            cells.push_back(i);
        }
    }
    
    return cells;
}

bool SA2DReorderer::isValidForReordering(int cell_id) const
{
    const dpl::Node* node = network_->getNode(cell_id);
    
    // Only process movable standard cells
    if (node->getType() != dpl::Node::CELL || node->isFixed()) {
        return false;
    }
    
    // Only single-height cells for now
    if (arch_->isMultiHeightCell(node)) {
        return false;
    }
    
    return true;
}

} // namespace sa2d 