/////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2024, Precision Innovations Inc.
// All rights reserved.
//
// BSD 3-Clause License
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
///////////////////////////////////////////////////////////////////////////////

#include "Worker.h"
#include "WorkerManager.h"  // For SimpleBarrier definition
#include "sa2d/SA2D.h"
#include "utl/Logger.h"
#include "infrastructure/Objects.h"  // Now we can access this!
#include "infrastructure/network.h"  // For Network class
#include "infrastructure/Coordinates.h"  // For coordinate conversions
#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map> // Added for overlap checking

namespace sa2d {

SAWorker::SAWorker(SA2D* sa2d, int worker_id)
    : sa2d_(sa2d),
      worker_id_(worker_id),
      network_(nullptr),
      arch_(nullptr),
      grid_info_(nullptr),
      temp_(100.0),
      cooling_rate_(0.95),
      max_iter_(1000),
      move_budget_(100000),
      max_displacement_x_(500),
      max_displacement_y_(100),
      distribution_(0.0, 1.0)
{
}

void SAWorker::setSeed(int seed)
{
    rng_.seed(seed + worker_id_);  // Add worker_id to ensure different seeds
}

void SAWorker::setMaxDisplacement(int max_x, int max_y)
{
    max_displacement_x_ = max_x;
    max_displacement_y_ = max_y;
}

void SAWorker::initFromDPL(dpl::Network* network,
                          const dpl::Architecture* arch,
                          const ImmutableGridInfo* grid_info)
{
    network_ = network;
    arch_ = arch;
    grid_info_ = grid_info;
    
    // Initialize cell states
    state_.cells.resize(network->getNumNodes());
    
    // Initialize worker grid
    grid_ = std::make_unique<WorkerGrid>(grid_info_);
    best_grid_ = std::make_unique<WorkerGrid>(grid_info_);
    
    // Copy current placement
    int misaligned_initial = 0;
    utl::Logger* logger = sa2d_->getLogger();
    for (int i = 0; i < network_->getNumNodes(); i++) {
        dpl::Node* node = const_cast<dpl::Node*>(network_->getNode(i));
        if (node->getType() == dpl::Node::CELL && node->isPlaced()) {
            state_.cells[i].x = node->getLeft();
            state_.cells[i].y = node->getBottom();
            state_.cells[i].orient = node->getOrient();
            
            // Check if initial position is site-aligned
            if (node->getLeft().v % grid_info_->getSiteWidth() != 0) {
                misaligned_initial++;
                
                // Get absolute position for debugging
                odb::dbInst* inst = node->getDbInst();
                int abs_x, abs_y;
                inst->getLocation(abs_x, abs_y);
                
                logger->warn(utl::SA2D, 206, "Cell {} has misaligned initial X position: core-rel={}, absolute={}, site_width={}, remainder={}",
                            node->name(), node->getLeft().v, abs_x, grid_info_->getSiteWidth(), abs_x % grid_info_->getSiteWidth());
            }
            
            // Place in grid
            GridX gx = grid_info_->gridX(node->getLeft());
            GridY gy = grid_info_->gridSnapDownY(node->getBottom());
            GridX gw = grid_info_->gridPaddedWidth(node);
            GridY gh = grid_info_->gridHeight(node);
            
            // Debug specific cells
            /*if (std::string(node->name()) == "_27965_" || std::string(node->name()) == "_27947_") {
                logger->info(utl::SA2D, 227, "Initial placement: {} at grid ({}, {}), size={}x{}, dbu=({}, {})",
                            node->name(), gx.v, gy.v, gw.v, gh.v,
                            node->getLeft().v, node->getBottom().v);
            }*/
            
            grid_->placeCell(i, gx, gy, gw, gh);
        }
    }
    
    if (misaligned_initial > 0) {
        utl::Logger* logger = sa2d_->getLogger();
        logger->warn(utl::SA2D, 203, "Found {} cells with misaligned initial positions!", misaligned_initial);
    }
    
    // Initialize net HPWL cache
    state_.net_hpwl_cache.resize(network_->getNumEdges());
    
    // Calculate initial HPWL
    state_.total_hpwl = calcInitialHPWL();
    
    // Initialize best state
    best_state_ = state_;
    best_grid_->copyFrom(*grid_);
}

int64_t SAWorker::calcInitialHPWL()
{
    int64_t total_hpwl = 0;
    
    for (int net_id = 0; net_id < network_->getNumEdges(); ++net_id) {
        dpl::Edge* net = const_cast<dpl::Edge*>(network_->getEdge(net_id));
        
        // Skip single-pin nets (following DPL convention)
        if (net->getNumPins() <= 1) {
            state_.net_hpwl_cache[net_id] = 0;
            continue;
        }
        
        DbuX min_x = DbuX{std::numeric_limits<int>::max()};
        DbuX max_x = DbuX{std::numeric_limits<int>::min()};
        DbuY min_y = DbuY{std::numeric_limits<int>::max()};
        DbuY max_y = DbuY{std::numeric_limits<int>::min()};
        
        // Get bounding box of all pins
        for (const dpl::Pin* pin : net->getPins()) {
            int node_id = pin->getNode()->getId();
            const dpl::Node* node = network_->getNode(node_id);
            
            // Use center positions like DPL does
            DbuX pin_x = node->getCenterX() + pin->getOffsetX();
            DbuY pin_y = node->getCenterY() + pin->getOffsetY();
            
            min_x = std::min(min_x, pin_x);
            max_x = std::max(max_x, pin_x);
            min_y = std::min(min_y, pin_y);
            max_y = std::max(max_y, pin_y);
        }
        
        uint64_t hpwl = (max_x - min_x).v + (max_y - min_y).v;
        state_.net_hpwl_cache[net_id] = hpwl;
        total_hpwl += hpwl;
    }
    
    return total_hpwl;
}

GridPt SAWorker::generateRandomPosition(int cell_id)
{
    dpl::Node* node = const_cast<dpl::Node*>(network_->getNode(cell_id));
    
    // Current position in grid coordinates
    GridX curr_x = grid_info_->gridX(state_.cells[cell_id].x);
    GridY curr_y = grid_info_->gridSnapDownY(state_.cells[cell_id].y);
    
    // Generate random displacement within limits
    std::uniform_int_distribution<int> dist_x(-max_displacement_x_, max_displacement_x_);
    std::uniform_int_distribution<int> dist_y(-max_displacement_y_, max_displacement_y_);
    
    int dx = dist_x(rng_);
    int dy = dist_y(rng_);
    
    // New position
    GridX new_x = curr_x + dx;
    GridY new_y = curr_y + dy;
    
    // Clip to grid bounds
    new_x = std::max(GridX{0}, new_x);
    new_y = std::max(GridY{0}, new_y);
    
    GridX width = grid_info_->gridPaddedWidth(node);
    GridY height = grid_info_->gridHeight(node);
    
    new_x = std::min(new_x, GridX{grid_info_->getRowSiteCount() - width.v});
    new_y = std::min(new_y, GridY{grid_info_->getRowCount() - height.v});
    
    // Ensure new_y is valid (corresponds to an actual row)
    new_y = grid_info_->nearestValidGridY(new_y);
    
    // Restrict to group boundary if cell is in a group
    if (node->inGroup()) {
        const auto group_rect = grid_info_->gridWithin(node->getGroup()->getBBox());
        new_x = std::max(new_x, group_rect.xlo);
        new_y = std::max(new_y, group_rect.ylo);
        new_x = std::min(new_x, GridX{group_rect.xhi.v - width.v});
        new_y = std::min(new_y, GridY{group_rect.yhi.v - height.v});
    }
    
    return GridPt{new_x, new_y};
}

bool SAWorker::canPlaceCell(int cell_id, GridX x, GridY y)
{
    const dpl::Node* node = network_->getNode(cell_id);
    
    // 1. Check row alignment (y must be valid row)
    if (y.v < 0 || y.v >= grid_info_->getRowCount()) {
        return false;
    }
    
    // 2. Check site alignment (x must be valid site)
    GridX width = grid_info_->gridPaddedWidth(node);
    if (x.v < 0 || x.v + width.v > grid_info_->getRowSiteCount()) {
        return false;
    }
    
    // 3. Check site compatibility
    odb::dbSite* cell_site = node->getSite();
    const auto& available_sites = grid_info_->getSitesAt(x, y);
    if (available_sites.find(cell_site) == available_sites.end()) {
        return false;  // Site type not available at this location
    }
    
    // 4. Check no overlaps
    GridY height = grid_info_->gridHeight(node);
    for (GridY yi = y; yi < y + height; yi++) {
        for (GridX xi = x; xi < x + width; xi++) {
            if (grid_->isOccupied(xi, yi)) {
                int other_id = grid_->getCellAt(xi, yi);
                if (other_id != cell_id) {  // Not self
                    // Debug overlap
                    //4 || cell_id == 11906 || other_id == 11924 || other_id == 11906) {
                        // const dpl::Node* other_node = network_->getNode(other_id);
                        // utl::Logger* logger = sa2d_->getLogger();
                        /*logger->info(utl::SA2D, 228, "canPlaceCell: {} (id={}) blocked by {} (id={}) at grid ({}, {})",
                                    node->name(), cell_id, other_node->name(), other_id, xi.v, yi.v);*/
                    //}
                    return false;
                }
            }
        }
    }
    
    return true;
}

odb::dbOrientType SAWorker::getCellOrientation(int cell_id, GridX x, GridY y)
{
    const dpl::Node* node = network_->getNode(cell_id);
    odb::dbSite* cell_site = node->getSite();
    
    // Get the orientation from the pixel's site information
    return grid_info_->getValidOrientation(cell_site, x, y);
}

std::vector<int> SAWorker::getAffectedNets(int cell_id)
{
    std::vector<int> affected_nets;
    const dpl::Node* node = network_->getNode(cell_id);
    
    for (const dpl::Pin* pin : node->getPins()) {
        affected_nets.push_back(pin->getEdge()->getId());
    }
    
    return affected_nets;
}

int64_t SAWorker::calcDeltaHPWL(const std::vector<int>& affected_nets)
{
    int64_t delta = 0;
    
    for (int net_id : affected_nets) {
        dpl::Edge* net = const_cast<dpl::Edge*>(network_->getEdge(net_id));
        
        // Skip single-pin nets
        if (net->getNumPins() <= 1) {
            continue;
        }
        
        DbuX min_x = DbuX{std::numeric_limits<int>::max()};
        DbuX max_x = DbuX{std::numeric_limits<int>::min()};
        DbuY min_y = DbuY{std::numeric_limits<int>::max()};
        DbuY max_y = DbuY{std::numeric_limits<int>::min()};
        
        // Calculate new HPWL
        for (const dpl::Pin* pin : net->getPins()) {
            int node_id = pin->getNode()->getId();
            const dpl::Node* node = network_->getNode(node_id);
            
            DbuX pin_x;
            DbuY pin_y;
            if (node->getType() == dpl::Node::CELL) {
                // Movable cell - use updated center position
                DbuX center_x = state_.cells[node_id].x + node->getWidth() / 2;
                DbuY center_y = state_.cells[node_id].y + node->getHeight() / 2;
                pin_x = center_x + pin->getOffsetX();
                pin_y = center_y + pin->getOffsetY();
            } else {
                // Fixed node - use original center position
                pin_x = node->getCenterX() + pin->getOffsetX();
                pin_y = node->getCenterY() + pin->getOffsetY();
            }
            
            min_x = std::min(min_x, pin_x);
            max_x = std::max(max_x, pin_x);
            min_y = std::min(min_y, pin_y);
            max_y = std::max(max_y, pin_y);
        }
        
        uint64_t new_hpwl = (max_x - min_x).v + (max_y - min_y).v;
        delta += new_hpwl - state_.net_hpwl_cache[net_id];
    }
    
    return delta;
}

void SAWorker::updateHPWLCache(const std::vector<int>& affected_nets)
{
    for (int net_id : affected_nets) {
        dpl::Edge* net = const_cast<dpl::Edge*>(network_->getEdge(net_id));
        
        // Skip single-pin nets
        if (net->getNumPins() <= 1) {
            state_.net_hpwl_cache[net_id] = 0;
            continue;
        }
        
        DbuX min_x = DbuX{std::numeric_limits<int>::max()};
        DbuX max_x = DbuX{std::numeric_limits<int>::min()};
        DbuY min_y = DbuY{std::numeric_limits<int>::max()};
        DbuY max_y = DbuY{std::numeric_limits<int>::min()};
        
        // Calculate new HPWL
        for (const dpl::Pin* pin : net->getPins()) {
            int node_id = pin->getNode()->getId();
            const dpl::Node* node = network_->getNode(node_id);
            
            DbuX pin_x;
            DbuY pin_y;
            if (node->getType() == dpl::Node::CELL) {
                // Movable cell - use updated center position
                DbuX center_x = state_.cells[node_id].x + node->getWidth() / 2;
                DbuY center_y = state_.cells[node_id].y + node->getHeight() / 2;
                pin_x = center_x + pin->getOffsetX();
                pin_y = center_y + pin->getOffsetY();
            } else {
                // Fixed node - use original center position
                pin_x = node->getCenterX() + pin->getOffsetX();
                pin_y = node->getCenterY() + pin->getOffsetY();
            }
            
            min_x = std::min(min_x, pin_x);
            max_x = std::max(max_x, pin_x);
            min_y = std::min(min_y, pin_y);
            max_y = std::max(max_y, pin_y);
        }
        
        state_.net_hpwl_cache[net_id] = (max_x - min_x).v + (max_y - min_y).v;
    }
}

bool SAWorker::acceptMove(int64_t delta_cost, float temp)
{
    if (delta_cost <= 0) {
        return true;  // Always accept improving moves
    }
    
    // Metropolis criterion
    float prob = std::exp(-delta_cost / temp);
    return distribution_(rng_) < prob;
}

bool SAWorker::tryMove(int cell_id)
{
    dpl::Node* node = const_cast<dpl::Node*>(network_->getNode(cell_id));
    
    // Skip non-movable cells
    if (node->getType() != dpl::Node::CELL || node->isFixed()) {
        return false;
    }
    
    // Save current state
    state_.cells[cell_id].prev_x = state_.cells[cell_id].x;
    state_.cells[cell_id].prev_y = state_.cells[cell_id].y;
    state_.cells[cell_id].prev_orient = state_.cells[cell_id].orient;
    
    // Generate new position within displacement limits
    GridPt new_pos = generateRandomPosition(cell_id);
    GridX grid_x = new_pos.x;
    GridY grid_y = new_pos.y;
    
    // Full legality check with grid (including site compatibility)
    if (!canPlaceCell(cell_id, grid_x, grid_y)) {
        illegal_moves_++;
        return false;
    }
    
    // Get the correct orientation for this position
    odb::dbOrientType new_orient = getCellOrientation(cell_id, grid_x, grid_y);
    
    // Update position temporarily
    state_.cells[cell_id].x = grid_info_->gridToDbuX(grid_x);
    state_.cells[cell_id].y = grid_info_->gridYToDbu(grid_y);
    state_.cells[cell_id].orient = new_orient;
    
    // Calculate cost change
    std::vector<int> affected_nets = getAffectedNets(cell_id);
    int64_t delta = calcDeltaHPWL(affected_nets);
    
    if (acceptMove(delta, temp_)) {
        // Update grid occupancy
        grid_->removeCell(cell_id);
        GridX gw = grid_info_->gridPaddedWidth(node);
        GridY gh = grid_info_->gridHeight(node);
        grid_->placeCell(cell_id, grid_x, grid_y, gw, gh);
        
        // Debug specific cells
        /*if (std::string(node->name()) == "_27965_" || std::string(node->name()) == "_27947_") {
            utl::Logger* logger = sa2d_->getLogger();
            logger->info(utl::SA2D, 223, "Accepted move of {} to grid ({}, {}), dbu ({}, {})",
                        node->name(), grid_x.v, grid_y.v, 
                        state_.cells[cell_id].x.v, state_.cells[cell_id].y.v);
        }*/
        
        // Update HPWL cache and total
        updateHPWLCache(affected_nets);
        state_.total_hpwl += delta;
        
        accepted_moves_++;
        return true;
    } else {
        // Restore previous state
        state_.cells[cell_id].x = state_.cells[cell_id].prev_x;
        state_.cells[cell_id].y = state_.cells[cell_id].prev_y;
        state_.cells[cell_id].orient = state_.cells[cell_id].prev_orient;
        
        rejected_moves_++;
        return false;
    }
}

bool SAWorker::trySwap(int cell1_id, int cell2_id)
{
    dpl::Node* node1 = const_cast<dpl::Node*>(network_->getNode(cell1_id));
    dpl::Node* node2 = const_cast<dpl::Node*>(network_->getNode(cell2_id));
    
    // Skip non-movable cells
    if (node1->getType() != dpl::Node::CELL || node1->isFixed() ||
        node2->getType() != dpl::Node::CELL || node2->isFixed()) {
        return false;
    }
    
    // Skip if cells have different widths or heights (swap would be complex)
    GridX gw1 = grid_info_->gridPaddedWidth(node1);
    GridY gh1 = grid_info_->gridHeight(node1);
    GridX gw2 = grid_info_->gridPaddedWidth(node2);
    GridY gh2 = grid_info_->gridHeight(node2);
    
    if (gw1 != gw2 || gh1 != gh2) {
        // Can't simply swap cells of different sizes
        return false;
    }
    
    // Save current states
    state_.cells[cell1_id].prev_x = state_.cells[cell1_id].x;
    state_.cells[cell1_id].prev_y = state_.cells[cell1_id].y;
    state_.cells[cell1_id].prev_orient = state_.cells[cell1_id].orient;
    
    state_.cells[cell2_id].prev_x = state_.cells[cell2_id].x;
    state_.cells[cell2_id].prev_y = state_.cells[cell2_id].y;
    state_.cells[cell2_id].prev_orient = state_.cells[cell2_id].orient;
    
    // Get grid positions
    GridX gx1 = grid_info_->gridX(state_.cells[cell1_id].x);
    GridY gy1 = grid_info_->gridSnapDownY(state_.cells[cell1_id].y);
    
    GridX gx2 = grid_info_->gridX(state_.cells[cell2_id].x);
    GridY gy2 = grid_info_->gridSnapDownY(state_.cells[cell2_id].y);
    
    // Remove both cells from grid
    grid_->removeCell(cell1_id);
    grid_->removeCell(cell2_id);
    
    // Check if swap is legal
    bool legal1 = canPlaceCell(cell1_id, gx2, gy2);
    bool legal2 = canPlaceCell(cell2_id, gx1, gy1);
    
    // Debug specific cells
    if ((std::string(node1->name()) == "_27965_" || std::string(node1->name()) == "_27947_") &&
        (std::string(node2->name()) == "_27965_" || std::string(node2->name()) == "_27947_")) {
        utl::Logger* logger = sa2d_->getLogger();
        logger->info(utl::SA2D, 225, "Swap check: {} (width={}) to grid ({}, {}), {} (width={}) to grid ({}, {})",
                    node1->name(), grid_info_->gridPaddedWidth(node1).v, gx2.v, gy2.v,
                    node2->name(), grid_info_->gridPaddedWidth(node2).v, gx1.v, gy1.v);
        logger->info(utl::SA2D, 226, "  Legal1={}, Legal2={}", legal1, legal2);
    }
    
    if (!legal1 || !legal2) {
        // Restore grid state
        grid_->placeCell(cell1_id, gx1, gy1, gw1, gh1);
        grid_->placeCell(cell2_id, gx2, gy2, gw2, gh2);
        
        illegal_moves_++;
        return false;
    }
    
    // Get orientations for swapped positions
    odb::dbOrientType orient1_new = getCellOrientation(cell1_id, gx2, gy2);
    odb::dbOrientType orient2_new = getCellOrientation(cell2_id, gx1, gy1);
    
    // Perform swap
    // Convert grid positions back to DBU to ensure site alignment
    DbuX new_x1 = grid_info_->gridToDbuX(gx2);
    DbuX new_x2 = grid_info_->gridToDbuX(gx1);
    
    // Debug check
    if (new_x1.v % grid_info_->getSiteWidth() != 0 || new_x2.v % grid_info_->getSiteWidth() != 0) {
        utl::Logger* logger = sa2d_->getLogger();
        logger->error(utl::SA2D, 209, "Swap producing misaligned positions! gx1={}, gx2={}, new_x1={}, new_x2={}, site_width={}",
                     gx1.v, gx2.v, new_x1.v, new_x2.v, grid_info_->getSiteWidth());
    }
    
    state_.cells[cell1_id].x = new_x1;
    state_.cells[cell1_id].y = grid_info_->gridYToDbu(gy2);
    state_.cells[cell1_id].orient = orient1_new;
    
    state_.cells[cell2_id].x = new_x2;
    state_.cells[cell2_id].y = grid_info_->gridYToDbu(gy1);
    state_.cells[cell2_id].orient = orient2_new;
    
    // Calculate cost change
    std::vector<int> affected_nets = getAffectedNets(cell1_id);
    std::vector<int> nets2 = getAffectedNets(cell2_id);
    affected_nets.insert(affected_nets.end(), nets2.begin(), nets2.end());
    
    // Remove duplicates
    std::sort(affected_nets.begin(), affected_nets.end());
    affected_nets.erase(std::unique(affected_nets.begin(), affected_nets.end()), 
                       affected_nets.end());
    
    int64_t delta = calcDeltaHPWL(affected_nets);
    
    if (acceptMove(delta, temp_)) {
        // Update grid with new positions
        grid_->placeCell(cell1_id, gx2, gy2, gw1, gh1);
        grid_->placeCell(cell2_id, gx1, gy1, gw2, gh2);
        
        // Debug specific cells
        if (std::string(node1->name()) == "_27965_" || std::string(node1->name()) == "_27947_" ||
            std::string(node2->name()) == "_27965_" || std::string(node2->name()) == "_27947_") {
            utl::Logger* logger = sa2d_->getLogger();
            logger->info(utl::SA2D, 224, "Accepted swap: {} to grid ({}, {}), {} to grid ({}, {})",
                        node1->name(), gx2.v, gy2.v, node2->name(), gx1.v, gy1.v);
        }
        
        // Update HPWL cache and total
        updateHPWLCache(affected_nets);
        state_.total_hpwl += delta;
        
        accepted_moves_++;
        return true;
    } else {
        // Restore previous state
        state_.cells[cell1_id].x = state_.cells[cell1_id].prev_x;
        state_.cells[cell1_id].y = state_.cells[cell1_id].prev_y;
        state_.cells[cell1_id].orient = state_.cells[cell1_id].prev_orient;
        
        state_.cells[cell2_id].x = state_.cells[cell2_id].prev_x;
        state_.cells[cell2_id].y = state_.cells[cell2_id].prev_y;
        state_.cells[cell2_id].orient = state_.cells[cell2_id].prev_orient;
        
        // Restore grid state
        grid_->placeCell(cell1_id, gx1, gy1, gw1, gh1);
        grid_->placeCell(cell2_id, gx2, gy2, gw2, gh2);
        
        rejected_moves_++;
        return false;
    }
}

void SAWorker::updateBestSolution()
{
    if (state_.total_hpwl < best_state_.total_hpwl) {
        best_state_ = state_;
        best_grid_->copyFrom(*grid_);
    }
}

void SAWorker::run()
{
    utl::Logger* logger = sa2d_->getLogger();
    
    logger->info(utl::SA2D, 101, "Worker {} starting SA with temp={}, cooling_rate={}, max_iter={}",
                 worker_id_, temp_, cooling_rate_, max_iter_);
    
    // Reset statistics
    accepted_moves_ = 0;
    rejected_moves_ = 0;
    illegal_moves_ = 0;
    
    float current_temp = temp_;
    int moves_tried = 0;
    
    // Get movable cells
    std::vector<int> movable_cells;
    for (int i = 0; i < network_->getNumNodes(); ++i) {
        dpl::Node* node = const_cast<dpl::Node*>(network_->getNode(i));
        if (node->getType() == dpl::Node::CELL && !node->isFixed()) {
            movable_cells.push_back(i);
        }
    }
    
    if (movable_cells.empty()) {
        logger->warn(utl::SA2D, 102, "No movable cells found!");
        return;
    }
    
    // Main SA loop
    for (int iter = 0; iter < max_iter_ && moves_tried < move_budget_; ++iter) {
        int moves_per_temp = movable_cells.size();
        
        for (int move = 0; move < moves_per_temp && moves_tried < move_budget_; ++move) {
            // Randomly choose between single move and swap
            bool do_swap = (distribution_(rng_) < 0.3);  // 30% swaps
            
            if (do_swap && movable_cells.size() > 1) {
                // Random swap
                std::uniform_int_distribution<int> cell_dist(0, movable_cells.size() - 1);
                int idx1 = cell_dist(rng_);
                int idx2 = cell_dist(rng_);
                while (idx2 == idx1) {
                    idx2 = cell_dist(rng_);
                }
                
                trySwap(movable_cells[idx1], movable_cells[idx2]);
            } else {
                // Random single move
                std::uniform_int_distribution<int> cell_dist(0, movable_cells.size() - 1);
                int idx = cell_dist(rng_);
                tryMove(movable_cells[idx]);
            }
            
            moves_tried++;
        }
        
        // Update best solution
        updateBestSolution();
        
        // Cool down
        current_temp *= cooling_rate_;
        
        // Log progress every 10%
        if ((iter + 1) % std::max(1, max_iter_ / 10) == 0) {
            logger->info(utl::SA2D, 103, "Worker {} - Iter {}/{}: temp={:.2f}, HPWL={:.1f} u, best_HPWL={:.1f} u, accept_rate={:.2f}%",
                        worker_id_, iter + 1, max_iter_, current_temp, 
                        sa2d_->getBlock()->dbuToMicrons(state_.total_hpwl), 
                        sa2d_->getBlock()->dbuToMicrons(best_state_.total_hpwl),
                        100.0 * accepted_moves_ / (accepted_moves_ + rejected_moves_ + 1));
        }
    }
    
    logger->info(utl::SA2D, 104, "Worker {} completed: moves_tried={}, accepted={}, rejected={}, illegal={}",
                worker_id_, moves_tried, accepted_moves_, rejected_moves_, illegal_moves_);
    logger->info(utl::SA2D, 105, "Worker {} final HPWL: {:.1f} u -> {:.1f} u (improvement: {:.2f}%)",
                worker_id_, 
                sa2d_->getBlock()->dbuToMicrons(state_.total_hpwl), 
                sa2d_->getBlock()->dbuToMicrons(best_state_.total_hpwl),
                100.0 * (1.0 - (double)best_state_.total_hpwl / state_.total_hpwl));
}

void SAWorker::runParallel(int iterations, SimpleBarrier& sync_barrier, 
                          const std::atomic<bool>& should_stop)
{
    // Reset statistics for this round
    accepted_moves_ = 0;
    rejected_moves_ = 0;
    illegal_moves_ = 0;
    
    // Get movable cells
    std::vector<int> movable_cells;
    for (int i = 0; i < network_->getNumNodes(); ++i) {
        dpl::Node* node = const_cast<dpl::Node*>(network_->getNode(i));
        if (node->getType() == dpl::Node::CELL && !node->isFixed()) {
            movable_cells.push_back(i);
        }
    }
    
    if (movable_cells.empty()) {
        sync_barrier.arrive_and_wait();
        return;
    }
    
    int moves_per_iter = movable_cells.size();
    
    // Run SA for specified iterations
    for (int iter = 0; iter < iterations && !should_stop.load(); ++iter) {
        for (int move = 0; move < moves_per_iter; ++move) {
            // Randomly choose between single move and swap
            bool do_swap = (distribution_(rng_) < 0.3);  // 30% swaps
            
            if (do_swap && movable_cells.size() > 1) {
                // Random swap
                std::uniform_int_distribution<int> cell_dist(0, movable_cells.size() - 1);
                int idx1 = cell_dist(rng_);
                int idx2 = cell_dist(rng_);
                while (idx2 == idx1) {
                    idx2 = cell_dist(rng_);
                }
                
                trySwap(movable_cells[idx1], movable_cells[idx2]);
            } else {
                // Random single move
                std::uniform_int_distribution<int> cell_dist(0, movable_cells.size() - 1);
                int idx = cell_dist(rng_);
                tryMove(movable_cells[idx]);
            }
        }
        
        // Update best solution
        updateBestSolution();
        
        // Cool down
        temp_ *= cooling_rate_;
    }
    
    // Wait for all workers to finish this round
    sync_barrier.arrive_and_wait();
}

void SAWorker::copyStateFrom(const SAWorker& other)
{
    // Copy cell positions and orientations
    state_ = other.state_;
    
    // Copy grid occupancy
    grid_->copyFrom(*other.grid_);
    
    // Note: Don't copy RNG state - maintain diversity
    // Note: Don't copy best_state_ - let worker find its own best
    // Note: Don't copy temperature - each worker manages its own cooling
    
    // Reset winner status
    is_winner_ = false;
    
    // Reset move statistics for the new round
    accepted_moves_ = 0;
    rejected_moves_ = 0;
    illegal_moves_ = 0;
}

void SAWorker::applyToDPL(dpl::Network* network)
{
    utl::Logger* logger = sa2d_->getLogger();
    int misaligned_count = 0;
    int moved_count = 0;
    int overlap_count = 0;
    
    // First, check for overlaps in the best solution
    struct GridPtHash {
        std::size_t operator()(const std::pair<GridX, GridY>& p) const {
            return std::hash<int>()(p.first.v) ^ (std::hash<int>()(p.second.v) << 1);
        }
    };
    std::unordered_map<std::pair<GridX, GridY>, int, GridPtHash> position_map;
    
    for (int i = 0; i < network_->getNumNodes(); i++) {
        dpl::Node* node = const_cast<dpl::Node*>(network_->getNode(i));
        if (node->getType() == dpl::Node::CELL && !node->isFixed()) {
            GridX gx = grid_info_->gridX(best_state_.cells[i].x);
            GridY gy = grid_info_->gridSnapDownY(best_state_.cells[i].y);
            GridX gw = grid_info_->gridPaddedWidth(node);
            GridY gh = grid_info_->gridHeight(node);
            
            // Check all pixels this cell would occupy
            for (GridY yi = gy; yi < gy + gh; yi++) {
                for (GridX xi = gx; xi < gx + gw; xi++) {
                    auto key = std::make_pair(xi, yi);
                    auto it = position_map.find(key);
                    if (it != position_map.end() && it->second != i) {
                        // Overlap detected
                        dpl::Node* other = const_cast<dpl::Node*>(network_->getNode(it->second));
                        logger->warn(utl::SA2D, 218, "Overlap detected: {} and {} at grid ({}, {})",
                                    node->name(), other->name(), xi.v, yi.v);
                        overlap_count++;
                        
                        // Detailed debugging for the overlapping cells
                        if (overlap_count == 1) {  // Only print details for first overlap
                            logger->warn(utl::SA2D, 220, "  Cell {} (id={}): pos=({}, {}), grid=({}, {}), size={}x{}",
                                        node->name(), i, 
                                        best_state_.cells[i].x.v, best_state_.cells[i].y.v,
                                        gx.v, gy.v, gw.v, gh.v);
                            logger->warn(utl::SA2D, 221, "  Cell {} (id={}): pos=({}, {})",
                                        other->name(), it->second,
                                        best_state_.cells[it->second].x.v, best_state_.cells[it->second].y.v);
                            
                            // Check if these cells overlap in the best_grid_
                            int grid_cell_at_pos = best_grid_->getCellAt(xi, yi);
                            logger->warn(utl::SA2D, 222, "  best_grid_ at ({}, {}) contains cell_id={}",
                                        xi.v, yi.v, grid_cell_at_pos);
                        }
                    }
                    position_map[key] = i;
                }
            }
        }
    }
    
    if (overlap_count > 0) {
        logger->error(utl::SA2D, 219, "Found {} overlapping cells in best solution!", overlap_count);
    }
    
    // Apply best solution back to DPL network
    for (int i = 0; i < network_->getNumNodes(); i++) {
        dpl::Node* node = const_cast<dpl::Node*>(network_->getNode(i));
        if (node->getType() == dpl::Node::CELL) {
            // Check if position changed
            if (node->getLeft() != best_state_.cells[i].x || 
                node->getBottom() != best_state_.cells[i].y ||
                node->getOrient() != best_state_.cells[i].orient) {
                
                moved_count++;
                
                // Debug: Check grid conversion roundtrip
                GridX gx = grid_info_->gridX(best_state_.cells[i].x);
                DbuX roundtrip_x = grid_info_->gridToDbuX(gx);
                if (roundtrip_x != best_state_.cells[i].x) {
                    logger->warn(utl::SA2D, 207, "Cell {} position {} doesn't roundtrip through grid (got {})",
                                node->name(), best_state_.cells[i].x.v, roundtrip_x.v);
                }
                
                // Log the move for debugging
                /*logger->info(utl::SA2D, 204, "Moving cell {} from ({}, {}) to ({}, {})",
                            node->name(), 
                            node->getLeft().v, node->getBottom().v,
                            best_state_.cells[i].x.v, best_state_.cells[i].y.v);*/
                            
                // Special debugging for _53274_
                if (std::string(node->name()) == "_53274_") {
                    GridY gy = grid_info_->gridSnapDownY(best_state_.cells[i].y);
                    DbuY dbu_y = grid_info_->gridYToDbu(gy);
                    logger->warn(utl::SA2D, 214, "Cell _53274_ debug: best_y={}, grid_y={}, back_to_dbu={}",
                                best_state_.cells[i].y.v, gy.v, dbu_y.v);
                }
            }
            
            // Check if position is site-aligned
            DbuX x = best_state_.cells[i].x;
            
            if (x.v % grid_info_->getSiteWidth() != 0) {
                // Get absolute position for complete debugging
                odb::dbInst* inst = node->getDbInst();
                int old_abs_x, old_abs_y;
                inst->getLocation(old_abs_x, old_abs_y);
                
                // Calculate what the new absolute position will be
                odb::dbBlock* block = inst->getBlock();
                odb::Rect core = block->getCoreArea();
                int new_abs_x = core.xMin() + x.v;
                
                logger->warn(utl::SA2D, 201, "Cell {} (id={}) X position {} is not site-aligned (site width = {})",
                            node->name(), i, x.v, grid_info_->getSiteWidth());
                logger->warn(utl::SA2D, 208, "  Cell width = {}, was at grid ({}, {})",
                            node->getWidth().v,
                            grid_info_->gridX(best_state_.cells[i].x).v,
                            grid_info_->gridSnapDownY(best_state_.cells[i].y).v);
                logger->warn(utl::SA2D, 210, "  Old absolute X = {}, new absolute X = {}, new_abs % site_width = {}",
                            old_abs_x, new_abs_x, new_abs_x % grid_info_->getSiteWidth());
                misaligned_count++;
            }
            
            // Move to best position
            node->setLeft(best_state_.cells[i].x);
            node->setBottom(best_state_.cells[i].y);
            node->setOrient(best_state_.cells[i].orient);
        }
    }
    
    logger->info(utl::SA2D, 205, "Moved {} cells total", moved_count);
    
    if (misaligned_count > 0) {
        logger->warn(utl::SA2D, 202, "Found {} cells with site alignment issues", misaligned_count);
    }
    
    // DPL can run additional legalization if needed
}

}  // namespace sa2d 