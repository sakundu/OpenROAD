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
#include "dpl/Opendp.h"
#include "utl/Logger.h"
#include "infrastructure/Objects.h"  // Now we can access this!
#include "infrastructure/network.h"  // For Network class
#include "infrastructure/Coordinates.h"  // For coordinate conversions
#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map> // Added for overlap checking
#include <unordered_set> // Added for chain validation
#include <map>
#include <set>

namespace {
// Transform pin offset based on cell orientation
void transformPinOffset(odb::dbOrientType orient, 
                       dpl::DbuX& offset_x, 
                       dpl::DbuY& offset_y,
                       dpl::DbuX cell_width,
                       dpl::DbuY cell_height)
{
    // Based on DPL's implementation in Objects.cpp
    int dx = offset_x.v;
    int dy = offset_y.v;
    
    switch (orient.getValue()) {
        case odb::dbOrientType::Value::R0:
            // No transformation needed
            break;
        case odb::dbOrientType::Value::R90:
            offset_x = dpl::DbuX{dy};
            offset_y = dpl::DbuY{-dx};
            break;
        case odb::dbOrientType::Value::R180:
            offset_x = dpl::DbuX{-dx};
            offset_y = dpl::DbuY{-dy};
            break;
        case odb::dbOrientType::Value::R270:
            offset_x = dpl::DbuX{-dy};
            offset_y = dpl::DbuY{dx};
            break;
        case odb::dbOrientType::Value::MY:
            offset_x = dpl::DbuX{-dx};
            offset_y = dpl::DbuY{dy};
            break;
        case odb::dbOrientType::Value::MX:
            offset_x = dpl::DbuX{dx};
            offset_y = dpl::DbuY{-dy};
            break;
        case odb::dbOrientType::Value::MYR90:
            offset_x = dpl::DbuX{-dy};
            offset_y = dpl::DbuY{-dx};
            break;
        case odb::dbOrientType::Value::MXR90:
            offset_x = dpl::DbuX{dy};
            offset_y = dpl::DbuY{dx};
            break;
    }
}
}  // anonymous namespace

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
      moves_per_iter_(10000),  // Default: 10k moves per iteration
      max_displacement_x_(500),
      max_displacement_y_(100),
      kick_interval_(100),
      kick_threshold_(0.05f),
      kick_strength_(10),
      kick_temp_multiplier_(1.5f),
      enable_kicks_(true),
      is_winner_(false),
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
    
    // Pre-populate affected nets cache for all cells
    affected_nets_cache_.clear();
    for (int i = 0; i < network_->getNumNodes(); ++i) {
        dpl::Node* node = const_cast<dpl::Node*>(network_->getNode(i));
        if (node->getType() == dpl::Node::CELL) {
            getAffectedNets(i);  // This will cache the result
        }
    }
    
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
            
            DbuX pin_x;
            DbuY pin_y;
            if (node->getType() == dpl::Node::CELL) {
                // Movable cell - use center position
                DbuX center_x = state_.cells[node_id].x + node->getWidth() / 2;
                DbuY center_y = state_.cells[node_id].y + node->getHeight() / 2;
                
                // Get pin offset and transform based on current orientation
                DbuX offset_x = pin->getOffsetX();
                DbuY offset_y = pin->getOffsetY();
                transformPinOffset(state_.cells[node_id].orient, 
                                 offset_x, offset_y,
                                 node->getWidth(), node->getHeight());
                
                pin_x = center_x + offset_x;
                pin_y = center_y + offset_y;
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
    // Check cache first
    auto it = affected_nets_cache_.find(cell_id);
    if (it != affected_nets_cache_.end()) {
        return it->second;
    }
    
    // Not in cache, compute and store
    std::vector<int> affected_nets;
    const dpl::Node* node = network_->getNode(cell_id);
    
    for (const dpl::Pin* pin : node->getPins()) {
        affected_nets.push_back(pin->getEdge()->getId());
    }
    
    // Cache the result
    affected_nets_cache_[cell_id] = affected_nets;
    
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
                
                // Get pin offset and transform based on current orientation
                DbuX offset_x = pin->getOffsetX();
                DbuY offset_y = pin->getOffsetY();
                transformPinOffset(state_.cells[node_id].orient, 
                                 offset_x, offset_y,
                                 node->getWidth(), node->getHeight());
                
                pin_x = center_x + offset_x;
                pin_y = center_y + offset_y;
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
                
                // Get pin offset and transform based on current orientation
                DbuX offset_x = pin->getOffsetX();
                DbuY offset_y = pin->getOffsetY();
                transformPinOffset(state_.cells[node_id].orient, 
                                 offset_x, offset_y,
                                 node->getWidth(), node->getHeight());
                
                pin_x = center_x + offset_x;
                pin_y = center_y + offset_y;
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

int64_t SAWorker::calcNetHPWL(int net_id)
{
    dpl::Edge* net = const_cast<dpl::Edge*>(network_->getEdge(net_id));
    
    // Skip single-pin nets
    if (net->getNumPins() <= 1) {
        return 0;
    }
    
    DbuX min_x = DbuX{std::numeric_limits<int>::max()};
    DbuX max_x = DbuX{std::numeric_limits<int>::min()};
    DbuY min_y = DbuY{std::numeric_limits<int>::max()};
    DbuY max_y = DbuY{std::numeric_limits<int>::min()};
    
    // Calculate HPWL
    for (const dpl::Pin* pin : net->getPins()) {
        int node_id = pin->getNode()->getId();
        const dpl::Node* node = network_->getNode(node_id);
        
        DbuX pin_x;
        DbuY pin_y;
        if (node->getType() == dpl::Node::CELL) {
            // Movable cell - use updated center position
            DbuX center_x = state_.cells[node_id].x + node->getWidth() / 2;
            DbuY center_y = state_.cells[node_id].y + node->getHeight() / 2;
            
            // Get pin offset and transform based on current orientation
            DbuX offset_x = pin->getOffsetX();
            DbuY offset_y = pin->getOffsetY();
            transformPinOffset(state_.cells[node_id].orient, 
                             offset_x, offset_y,
                             node->getWidth(), node->getHeight());
            
            pin_x = center_x + offset_x;
            pin_y = center_y + offset_y;
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
    
    return (max_x - min_x).v + (max_y - min_y).v;
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
        
        // Update HPWL cache and total
        updateHPWLCache(affected_nets);
        state_.total_hpwl += delta;
        
        accepted_moves_++;
        accepted_single_moves_++;  // Track single moves separately
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
        
        // Update HPWL cache and total
        updateHPWLCache(affected_nets);
        state_.total_hpwl += delta;
        
        accepted_moves_++;
        accepted_swaps_++;  // Track swaps separately
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

bool SAWorker::tryFlip(int cell_id)
{
    dpl::Node* node = const_cast<dpl::Node*>(network_->getNode(cell_id));
    
    // Skip non-movable cells
    if (node->getType() != dpl::Node::CELL || node->isFixed()) {
        return false;
    }
    
    // Only flip single-height cells
    if (!grid_info_->isSingleHeightCell(node)) {
        return false;
    }
    
    // Get current grid position
    GridY gy = grid_info_->gridSnapDownY(state_.cells[cell_id].y);
    
    // Check if row supports Y symmetry
    if (!grid_info_->isRowYSymmetric(gy)) {
        return false;
    }
    
    // Save current state
    state_.cells[cell_id].prev_x = state_.cells[cell_id].x;
    state_.cells[cell_id].prev_y = state_.cells[cell_id].y;
    state_.cells[cell_id].prev_orient = state_.cells[cell_id].orient;
    
    // Calculate current and flipped orientations
    odb::dbOrientType current_orient = state_.cells[cell_id].orient;
    odb::dbOrientType flipped_orient;
    
    switch (current_orient.getValue()) {
        case odb::dbOrientType::Value::R0:
            flipped_orient = odb::dbOrientType::MY;
            break;
        case odb::dbOrientType::Value::R180:
            flipped_orient = odb::dbOrientType::MX;
            break;
        case odb::dbOrientType::Value::MY:
            flipped_orient = odb::dbOrientType::R0;
            break;
        case odb::dbOrientType::Value::MX:
            flipped_orient = odb::dbOrientType::R180;
            break;
        default:
            // Can't flip other orientations
            return false;
    }
    
    // Calculate HPWL before flip
    std::vector<int> affected_nets = getAffectedNets(cell_id);
    int64_t hpwl_before = 0;
    for (int net_id : affected_nets) {
        hpwl_before += state_.net_hpwl_cache[net_id];
    }
    
    // Temporarily apply flip to calculate new HPWL
    state_.cells[cell_id].orient = flipped_orient;
    
    // Calculate HPWL after flip
    int64_t hpwl_after = 0;
    for (int net_id : affected_nets) {
        hpwl_after += calcNetHPWL(net_id);
    }
    
    int64_t delta = hpwl_after - hpwl_before;
    
    // Only accept flips that improve HPWL (like DPL)
    if (delta < 0) {
        // Check legality - flipping doesn't change position, just orientation
        // But we need to check edge spacing/padding if that's implemented
        
        // Accept the flip
        updateHPWLCache(affected_nets);
        state_.total_hpwl += delta;
        
        accepted_moves_++;
        accepted_flips_++;  // Track flip separately
        return true;
    } else {
        // Restore previous orientation
        state_.cells[cell_id].orient = state_.cells[cell_id].prev_orient;
        
        rejected_moves_++;
        return false;
    }
}

bool SAWorker::tryChainMove(int cell_id)
{
    dpl::Node* node = const_cast<dpl::Node*>(network_->getNode(cell_id));
    
    // Skip non-movable cells
    if (node->getType() != dpl::Node::CELL || node->isFixed()) {
        return false;
    }
    
    // Temperature-based early termination: skip chain moves at low temperatures
    if (temp_ < 1.0) {
        // At low temperatures, just try regular moves instead
        return tryMove(cell_id);
    }
    
    attempted_chain_moves_++;
    
    // Generate target position
    GridPt target_pos = generateRandomPosition(cell_id);
    
    // Check if target position is already occupied
    GridX width = grid_info_->gridPaddedWidth(node);
    GridY height = grid_info_->gridHeight(node);
    
    // Quick check: is any part of the target area free?
    bool has_occupied = false;
    for (GridY y = target_pos.y; y.v < target_pos.y.v + height.v && !has_occupied; ++y) {
        for (GridX x = target_pos.x; x.v < target_pos.x.v + width.v; ++x) {
            if (grid_->isOccupied(x, y)) {
                has_occupied = true;
                break;
            }
        }
    }
    
    if (!has_occupied) {
        // Position is completely free, just do a regular move
        attempted_chain_moves_--;
        attempted_single_moves_++;
        return tryMove(cell_id);
    }
    
    // Adaptive chain length based on temperature
    int max_chain_length = temp_ > 10.0 ? 3 : 2;
    
    // Try ripple in both directions, but with reduced chain length
    bool left_success = tryRippleLeft(cell_id, target_pos, max_chain_length);
    if (left_success) return true;
    
    bool right_success = tryRippleRight(cell_id, target_pos, max_chain_length);
    return right_success;
}

bool SAWorker::tryRippleLeft(int cell_id, GridPt target_pos, int max_chain_length)
{
    std::vector<ChainedMove> chain;
    const dpl::Node* seed_node = network_->getNode(cell_id);
    
    // Add the seed cell move
    ChainedMove seed_move;
    seed_move.cell_id = cell_id;
    seed_move.old_pos.x = grid_info_->gridX(state_.cells[cell_id].x);
    seed_move.old_pos.y = grid_info_->gridSnapDownY(state_.cells[cell_id].y);
    seed_move.new_pos = target_pos;
    seed_move.old_orient = state_.cells[cell_id].orient;
    seed_move.new_orient = getCellOrientation(cell_id, target_pos.x, target_pos.y);
    chain.push_back(seed_move);
    
    // Build chain by shifting cells left
    GridX curr_x = target_pos.x;
    GridY curr_y = target_pos.y;
    GridX seed_width = grid_info_->gridPaddedWidth(seed_node);
    
    for (int i = 0; i < max_chain_length; ++i) {
        // Check what's occupying the current position
        int blocking_cell = grid_->getCellAt(curr_x, curr_y);
        if (blocking_cell == -1 || blocking_cell == cell_id) {
            // Found empty space or self, chain complete
            break;
        }
        
        const dpl::Node* blocking_node = network_->getNode(blocking_cell);
        if (blocking_node->isFixed()) {
            // Can't move fixed cells, chain fails
            return false;
        }
        
        // Check if this is a multi-height cell
        GridY blocking_height = grid_info_->gridHeight(blocking_node);
        GridY seed_height = grid_info_->gridHeight(seed_node);
        if (blocking_height.v > seed_height.v) {
            // Can't handle multi-height cells in chain for now
            return false;
        }
        
        // Calculate where to shift this cell
        GridX blocking_width = grid_info_->gridPaddedWidth(blocking_node);
        GridX new_x = curr_x.v > 0 ? GridX{curr_x.v - blocking_width.v} : GridX{0};
        
        // Check if we'd go out of bounds
        if (new_x.v < 0) {
            return false;
        }
        
        // Add to chain
        ChainedMove move;
        move.cell_id = blocking_cell;
        move.old_pos.x = grid_info_->gridX(state_.cells[blocking_cell].x);
        move.old_pos.y = grid_info_->gridSnapDownY(state_.cells[blocking_cell].y);
        move.new_pos.x = new_x;
        move.new_pos.y = curr_y;
        move.old_orient = state_.cells[blocking_cell].orient;
        move.new_orient = getCellOrientation(blocking_cell, new_x, curr_y);
        chain.push_back(move);
        
        // Move to next position
        curr_x = new_x;
    }
    
    // Validate the entire chain
    if (!validateChain(chain)) {
        return false;
    }
    
    // Calculate total cost change
    int64_t total_delta = calculateChainDelta(chain);
    
    // Accept/reject based on SA criterion
    if (acceptMove(total_delta, temp_)) {
        executeChain(chain);
        accepted_chain_moves_++;
        total_cells_shifted_ += chain.size();
        max_chain_length_ = std::max(max_chain_length_, (int)chain.size());
        return true;
    }
    
    return false;
}

bool SAWorker::tryRippleRight(int cell_id, GridPt target_pos, int max_chain_length)
{
    std::vector<ChainedMove> chain;
    const dpl::Node* seed_node = network_->getNode(cell_id);
    
    // Add the seed cell move
    ChainedMove seed_move;
    seed_move.cell_id = cell_id;
    seed_move.old_pos.x = grid_info_->gridX(state_.cells[cell_id].x);
    seed_move.old_pos.y = grid_info_->gridSnapDownY(state_.cells[cell_id].y);
    seed_move.new_pos = target_pos;
    seed_move.old_orient = state_.cells[cell_id].orient;
    seed_move.new_orient = getCellOrientation(cell_id, target_pos.x, target_pos.y);
    chain.push_back(seed_move);
    
    // Build chain by shifting cells right
    GridX curr_x = target_pos.x;
    GridY curr_y = target_pos.y;
    GridX seed_width = grid_info_->gridPaddedWidth(seed_node);
    
    for (int i = 0; i < max_chain_length; ++i) {
        // Check cells that would be displaced
        bool found_blocking = false;
        int blocking_cell = -1;
        
        // Check all positions occupied by the seed cell at target
        for (GridX x = curr_x; x.v < curr_x.v + seed_width.v; ++x) {
            int cell_at_pos = grid_->getCellAt(x, curr_y);
            if (cell_at_pos != -1 && cell_at_pos != cell_id) {
                blocking_cell = cell_at_pos;
                found_blocking = true;
                break;
            }
        }
        
        if (!found_blocking) {
            // Found enough empty space, chain complete
            break;
        }
        
        const dpl::Node* blocking_node = network_->getNode(blocking_cell);
        if (blocking_node->isFixed()) {
            // Can't move fixed cells, chain fails
            return false;
        }
        
        // Check if this is a multi-height cell
        GridY blocking_height = grid_info_->gridHeight(blocking_node);
        GridY seed_height = grid_info_->gridHeight(seed_node);
        if (blocking_height.v > seed_height.v) {
            // Can't handle multi-height cells in chain for now
            return false;
        }
        
        // Calculate where to shift this cell (to the right)
        GridX blocking_width = grid_info_->gridPaddedWidth(blocking_node);
        GridX blocking_curr_x = grid_info_->gridX(state_.cells[blocking_cell].x);
        GridX new_x{curr_x.v + seed_width.v};
        
        // Check if we'd go out of bounds
        if (new_x.v + blocking_width.v > grid_info_->getRowSiteCount()) {
            return false;
        }
        
        // Add to chain
        ChainedMove move;
        move.cell_id = blocking_cell;
        move.old_pos.x = blocking_curr_x;
        move.old_pos.y = grid_info_->gridSnapDownY(state_.cells[blocking_cell].y);
        move.new_pos.x = new_x;
        move.new_pos.y = curr_y;
        move.old_orient = state_.cells[blocking_cell].orient;
        move.new_orient = getCellOrientation(blocking_cell, new_x, curr_y);
        chain.push_back(move);
        
        // Move to next position
        curr_x = new_x;
        seed_width = blocking_width;  // For next iteration
    }
    
    // Validate the entire chain
    if (!validateChain(chain)) {
        return false;
    }
    
    // Calculate total cost change
    int64_t total_delta = calculateChainDelta(chain);
    
    // Accept/reject based on SA criterion
    if (acceptMove(total_delta, temp_)) {
        executeChain(chain);
        accepted_chain_moves_++;
        total_cells_shifted_ += chain.size();
        max_chain_length_ = std::max(max_chain_length_, (int)chain.size());
        return true;
    }
    
    return false;
}

bool SAWorker::validateChain(const std::vector<ChainedMove>& chain)
{
    // Quick sanity check
    if (chain.empty()) return false;
    
    // Check displacement limits for all cells in chain
    for (const auto& move : chain) {
        const dpl::Node* node = network_->getNode(move.cell_id);
        
        // Convert grid coordinates to DBU
        DbuX new_x = grid_info_->gridToDbuX(move.new_pos.x);
        DbuY new_y = grid_info_->gridYToDbu(move.new_pos.y);
        
        // Check displacement from original position
        DbuX orig_x = node->getOrigLeft();
        DbuY orig_y = node->getOrigBottom();
        
        int dx_sites = abs((new_x.v - orig_x.v) / grid_info_->getSiteWidth());
        // Get row height for displacement calculation
        std::optional<int> uniform_height = grid_info_->getUniformRowHeight();
        int row_height = uniform_height.has_value() ? 
                        uniform_height.value() : 
                        grid_info_->getSiteWidth();  // Fallback to site width for single-height
        int dy_sites = abs((new_y.v - orig_y.v) / row_height);
        
        if (dx_sites > max_displacement_x_ || dy_sites > max_displacement_y_) {
            return false;
        }
        
        // Check if cell belongs to a group and respects group boundaries
        if (node->inGroup()) {
            const auto group_rect = grid_info_->gridWithin(node->getGroup()->getBBox());
            GridX width = grid_info_->gridPaddedWidth(node);
            GridY height = grid_info_->gridHeight(node);
            
            if (move.new_pos.x < group_rect.xlo || 
                move.new_pos.x + width > group_rect.xhi ||
                move.new_pos.y < group_rect.ylo || 
                move.new_pos.y + height > group_rect.yhi) {
                return false;
            }
        }
    }
    
    // Create a temporary grid state to check for overlaps
    // This is necessary to ensure no conflicts with existing cells
    WorkerGrid temp_grid(grid_info_);
    temp_grid.copyFrom(*grid_);
    
    // Build a set of cells that are part of the chain for quick lookup
    std::unordered_set<int> chain_cells;
    for (const auto& move : chain) {
        chain_cells.insert(move.cell_id);
    }
    
    // Apply all moves to temp grid and check for conflicts
    for (const auto& move : chain) {
        const dpl::Node* node = network_->getNode(move.cell_id);
        GridX width = grid_info_->gridPaddedWidth(node);
        GridY height = grid_info_->gridHeight(node);
        
        // Remove from old position
        temp_grid.removeCell(move.cell_id);
        
        // Check if new position is free or only occupied by other chain cells
        for (GridY y = move.new_pos.y; y.v < move.new_pos.y.v + height.v; ++y) {
            for (GridX x = move.new_pos.x; x.v < move.new_pos.x.v + width.v; ++x) {
                int occupying_cell = temp_grid.getCellAt(x, y);
                if (occupying_cell != -1 && chain_cells.count(occupying_cell) == 0) {
                    return false;  // Occupied by a cell not in the chain
                }
            }
        }
        
        // Place at new position
        temp_grid.placeCell(move.cell_id, move.new_pos.x, move.new_pos.y, width, height);
    }
    
    return true;
}

int64_t SAWorker::calculateChainDelta(const std::vector<ChainedMove>& chain)
{
    // Collect all affected nets
    std::set<int> affected_nets_set;
    
    for (const auto& move : chain) {
        auto nets = getAffectedNets(move.cell_id);
        affected_nets_set.insert(nets.begin(), nets.end());
    }
    
    std::vector<int> affected_nets(affected_nets_set.begin(), affected_nets_set.end());
    
    // Save current positions
    std::vector<CellState> saved_states;
    for (const auto& move : chain) {
        saved_states.push_back(state_.cells[move.cell_id]);
    }
    
    // Temporarily apply moves
    for (const auto& move : chain) {
        state_.cells[move.cell_id].x = grid_info_->gridToDbuX(move.new_pos.x);
        state_.cells[move.cell_id].y = grid_info_->gridYToDbu(move.new_pos.y);
        state_.cells[move.cell_id].orient = move.new_orient;
    }
    
    // Calculate delta
    int64_t delta = calcDeltaHPWL(affected_nets);
    
    // Restore positions
    for (size_t i = 0; i < chain.size(); ++i) {
        state_.cells[chain[i].cell_id] = saved_states[i];
    }
    
    return delta;
}

bool SAWorker::executeChain(const std::vector<ChainedMove>& chain)
{
    // Apply all moves in the chain
    for (const auto& move : chain) {
        const dpl::Node* node = network_->getNode(move.cell_id);
        GridX width = grid_info_->gridPaddedWidth(node);
        GridY height = grid_info_->gridHeight(node);
        
        // Update grid
        grid_->removeCell(move.cell_id);
        grid_->placeCell(move.cell_id, move.new_pos.x, move.new_pos.y, width, height);
        
        // Update state
        state_.cells[move.cell_id].x = grid_info_->gridToDbuX(move.new_pos.x);
        state_.cells[move.cell_id].y = grid_info_->gridYToDbu(move.new_pos.y);
        state_.cells[move.cell_id].orient = move.new_orient;
    }
    
    // Update HPWL cache for affected nets
    std::set<int> affected_nets_set;
    for (const auto& move : chain) {
        auto nets = getAffectedNets(move.cell_id);
        affected_nets_set.insert(nets.begin(), nets.end());
    }
    
    std::vector<int> affected_nets(affected_nets_set.begin(), affected_nets_set.end());
    updateHPWLCache(affected_nets);
    
    // Update best solution if this is better
    updateBestSolution();
    
    return true;
}

void SAWorker::revertChain(const std::vector<ChainedMove>& chain)
{
    // This function is not currently used but could be useful for debugging
    // or more complex move strategies
    for (const auto& move : chain) {
        const dpl::Node* node = network_->getNode(move.cell_id);
        GridX width = grid_info_->gridPaddedWidth(node);
        GridY height = grid_info_->gridHeight(node);
        
        // Restore grid
        grid_->removeCell(move.cell_id);
        grid_->placeCell(move.cell_id, move.old_pos.x, move.old_pos.y, width, height);
        
        // Restore state
        state_.cells[move.cell_id].x = grid_info_->gridToDbuX(move.old_pos.x);
        state_.cells[move.cell_id].y = grid_info_->gridYToDbu(move.old_pos.y);
        state_.cells[move.cell_id].orient = move.old_orient;
    }
}

void SAWorker::updateBestSolution()
{
    if (state_.total_hpwl < best_state_.total_hpwl) {
        best_state_ = state_;
        best_grid_->copyFrom(*grid_);
    }
}

bool SAWorker::shouldPerformKick(int iteration)
{
    // Don't kick if disabled
    if (!enable_kicks_) {
        return false;
    }
    
    // Check if rolling acceptance rate is too low
    if (rolling_accept_rate_ < kick_threshold_) {
        // Make sure we don't kick too frequently
        if (iteration - last_kick_iteration_ >= kick_interval_ / 2) {
            return true;
        }
    }
    
    // Check if we're stagnating (no improvement for many iterations)
    if (stagnation_counter_ > kick_interval_) {
        return true;
    }
    
    // Periodic kicks
    if (iteration > 0 && iteration % kick_interval_ == 0) {
        return true;
    }
    
    return false;
}

bool SAWorker::shouldPerformChainMoves(int iteration)
{
    // Don't perform if disabled
    if (!enable_chain_moves_) {
        return false;
    }
    
    // Don't perform at very low temperatures (less likely to accept)
    if (temp_ < 1.0) {
        return false;
    }
    
    // Perform periodically
    if (iteration > 0 && (iteration - last_chain_iteration_) >= chain_move_interval_) {
        return true;
    }
    
    return false;
}

void SAWorker::run()
{
    utl::Logger* logger = sa2d_->getLogger();
    
    // Reset statistics
    accepted_moves_ = 0;
    rejected_moves_ = 0;
    illegal_moves_ = 0;
    accepted_flips_ = 0;
    accepted_swaps_ = 0;
    accepted_single_moves_ = 0;
    attempted_flips_ = 0;
    attempted_swaps_ = 0;
    attempted_single_moves_ = 0;
    
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
    
    // Print table header
    logger->info(utl::SA2D, 106, "");
    logger->info(utl::SA2D, 107, "Iteration    Progress    Temperature    Current HPWL    Best HPWL    Accept Rate");
    logger->info(utl::SA2D, 108, "---------    --------    -----------    ------------    ---------    -----------");
    
    // Print initial state
    logger->info(utl::SA2D, 109, "{:>9}    {:>7}%    {:>11.2e}    {:>12.1f}    {:>9.1f}    {:>10.1f}%",
                0,
                0,
                current_temp,
                sa2d_->getBlock()->dbuToMicrons(state_.total_hpwl),
                sa2d_->getBlock()->dbuToMicrons(best_state_.total_hpwl),
                100.0);  // Initial accept rate is 100%
    
    // Main SA loop
    for (int iter = 0; iter < max_iter_ && moves_tried < move_budget_; ++iter) {
        int moves_per_temp = moves_per_iter_;  // Use fixed moves per iteration
        int moves_accepted_this_iter = 0;
        int moves_attempted_this_iter = 0;
        
        // Check if we should perform a kick move
        if (shouldPerformKick(iter)) {
            bool kick_success = tryRegionShuffle(kick_strength_);
            if (kick_success) {
                last_kick_iteration_ = iter;
                stagnation_counter_ = 0;  // Reset stagnation
            }
        }
        
        // Check if we should perform chain moves
        if (shouldPerformChainMoves(iter)) {
            last_chain_iteration_ = iter;
            // Try a few chain moves
            for (int i = 0; i < chain_moves_per_round_; ++i) {
                std::uniform_int_distribution<int> cell_dist(0, movable_cells.size() - 1);
                int idx = cell_dist(rng_);
                tryChainMove(movable_cells[idx]);
            }
        }
        
        for (int move = 0; move < moves_per_temp && moves_tried < move_budget_; ++move) {
            int old_accepted = accepted_moves_;
            moves_attempted_this_iter++;
            
            // Randomly choose between single move, swap, and flip
            float rand_val = distribution_(rng_);
            
            if (rand_val < 0.15) {
                // 15% flips
                std::uniform_int_distribution<int> cell_dist(0, movable_cells.size() - 1);
                int idx = cell_dist(rng_);
                attempted_flips_++;
                tryFlip(movable_cells[idx]);
            } else if (rand_val < 0.30 && movable_cells.size() > 1) {
                // 15% swaps
                std::uniform_int_distribution<int> cell_dist(0, movable_cells.size() - 1);
                int idx1 = cell_dist(rng_);
                int idx2 = cell_dist(rng_);
                while (idx2 == idx1) {
                    idx2 = cell_dist(rng_);
                }
                
                attempted_swaps_++;
                trySwap(movable_cells[idx1], movable_cells[idx2]);
            } else {
                // 70% single moves
                std::uniform_int_distribution<int> cell_dist(0, movable_cells.size() - 1);
                int idx = cell_dist(rng_);
                attempted_single_moves_++;
                tryMove(movable_cells[idx]);
            }
            
            if (accepted_moves_ > old_accepted) {
                moves_accepted_this_iter++;
            }
            
            moves_tried++;
        }
        
        // Update rolling acceptance rate (exponential moving average)
        double iter_accept_rate = (double)moves_accepted_this_iter / (moves_attempted_this_iter + 1);
        rolling_accept_rate_ = 0.9 * rolling_accept_rate_ + 0.1 * iter_accept_rate;
        
        // Update best solution and stagnation counter
        int64_t old_best = best_state_.total_hpwl;
        updateBestSolution();
        
        if (best_state_.total_hpwl < old_best) {
            stagnation_counter_ = 0;
            best_hpwl_at_last_improvement_ = best_state_.total_hpwl;
        } else {
            stagnation_counter_++;
        }
        
        // Cool down
        current_temp *= cooling_rate_;
        
        // Report progress at regular intervals
        int report_interval = std::min(100, std::max(1, max_iter_ / 10));
        if ((iter > 0 && iter % report_interval == 0) || iter == max_iter_ - 1) {
            int progress = (iter * 100) / max_iter_;
            double accept_rate = getAcceptRate();
            
            logger->info(utl::SA2D, 109, "{:>9}    {:>7}%    {:>11.2e}    {:>12.1f}    {:>9.1f}    {:>10.1f}%",
                        iter,
                        progress,
                        current_temp,
                        sa2d_->getBlock()->dbuToMicrons(state_.total_hpwl),
                        sa2d_->getBlock()->dbuToMicrons(best_state_.total_hpwl),
                        accept_rate * 100.0);
        }
    }
    
    logger->info(utl::SA2D, 105, "");
    logger->info(utl::SA2D, 110, "Worker {} completed. Final HPWL: {:.1f} u (improvement: {:.2f}%)",
                worker_id_, 
                sa2d_->getBlock()->dbuToMicrons(best_state_.total_hpwl),
                100.0 * (1.0 - (double)best_state_.total_hpwl / state_.total_hpwl));
    
    // Report move statistics
    logger->info(utl::SA2D, 340, "Worker {} move statistics:", worker_id_);
    logger->info(utl::SA2D, 341, "  Single moves: {} attempted, {} accepted ({:.1f}%)", 
                attempted_single_moves_, accepted_single_moves_, 
                attempted_single_moves_ > 0 ? 100.0 * accepted_single_moves_ / attempted_single_moves_ : 0.0);
    logger->info(utl::SA2D, 342, "  Swaps: {} attempted, {} accepted ({:.1f}%)", 
                attempted_swaps_, accepted_swaps_, 
                attempted_swaps_ > 0 ? 100.0 * accepted_swaps_ / attempted_swaps_ : 0.0);
    logger->info(utl::SA2D, 343, "  Flips: {} attempted, {} accepted ({:.1f}%)", 
                attempted_flips_, accepted_flips_, 
                attempted_flips_ > 0 ? 100.0 * accepted_flips_ / attempted_flips_ : 0.0);
    if (attempted_chain_moves_ > 0) {
        logger->info(utl::SA2D, 344, "  Chain moves: {} attempted, {} accepted ({:.1f}%), max chain length: {}, total cells shifted: {}",
                    attempted_chain_moves_, accepted_chain_moves_,
                    100.0 * accepted_chain_moves_ / attempted_chain_moves_,
                    max_chain_length_, total_cells_shifted_);
    }
    
    // Report kick statistics if kicks were enabled
    if (enable_kicks_ && kick_attempts_ > 0) {
        logger->info(utl::SA2D, 330, "Worker {} LSMC kicks: {} attempted, {} accepted ({:.1f}% success), {} total swaps",
                    worker_id_, kick_attempts_, kick_accepted_, 
                    100.0 * kick_accepted_ / kick_attempts_,
                    total_swaps_applied_);
    }
}

void SAWorker::runParallel(int iterations, SimpleBarrier& sync_barrier, 
                          const std::atomic<bool>& should_stop)
{
    // Reset statistics for this round
    accepted_moves_ = 0;
    rejected_moves_ = 0;
    illegal_moves_ = 0;
    accepted_flips_ = 0;
    accepted_swaps_ = 0;
    accepted_single_moves_ = 0;
    attempted_flips_ = 0;
    attempted_swaps_ = 0;
    attempted_single_moves_ = 0;
    // Note: Don't clear affected_nets_cache_ - it remains valid
    
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
    
    int moves_per_iter = moves_per_iter_;  // Use fixed moves per iteration
    
    // Run SA for specified iterations
    for (int iter = 0; iter < iterations && !should_stop.load(); ++iter) {
        int moves_accepted_this_iter = 0;
        int moves_attempted_this_iter = 0;
        
        // Check if we should perform a kick move
        if (shouldPerformKick(iter)) {
            bool kick_success = tryRegionShuffle(kick_strength_);
            if (kick_success) {
                last_kick_iteration_ = iter;
                stagnation_counter_ = 0;  // Reset stagnation
            }
        }
        
        // Check if we should perform chain moves
        if (shouldPerformChainMoves(iter)) {
            last_chain_iteration_ = iter;
            // Try a few chain moves
            for (int i = 0; i < chain_moves_per_round_; ++i) {
                std::uniform_int_distribution<int> cell_dist(0, movable_cells.size() - 1);
                int idx = cell_dist(rng_);
                tryChainMove(movable_cells[idx]);
            }
        }
        
        for (int move = 0; move < moves_per_iter; ++move) {
            int old_accepted = accepted_moves_;
            moves_attempted_this_iter++;
            
            // Randomly choose between single move, swap, and flip
            float rand_val = distribution_(rng_);
            
            if (rand_val < 0.15) {
                // 15% flips
                std::uniform_int_distribution<int> cell_dist(0, movable_cells.size() - 1);
                int idx = cell_dist(rng_);
                attempted_flips_++;
                tryFlip(movable_cells[idx]);
            } else if (rand_val < 0.30 && movable_cells.size() > 1) {
                // 15% swaps
                std::uniform_int_distribution<int> cell_dist(0, movable_cells.size() - 1);
                int idx1 = cell_dist(rng_);
                int idx2 = cell_dist(rng_);
                while (idx2 == idx1) {
                    idx2 = cell_dist(rng_);
                }
                
                attempted_swaps_++;
                trySwap(movable_cells[idx1], movable_cells[idx2]);
            } else {
                // 70% single moves
                std::uniform_int_distribution<int> cell_dist(0, movable_cells.size() - 1);
                int idx = cell_dist(rng_);
                attempted_single_moves_++;
                tryMove(movable_cells[idx]);
            }
            
            if (accepted_moves_ > old_accepted) {
                moves_accepted_this_iter++;
            }
        }
        
        // Update rolling acceptance rate (exponential moving average)
        double iter_accept_rate = (double)moves_accepted_this_iter / (moves_attempted_this_iter + 1);
        rolling_accept_rate_ = 0.9 * rolling_accept_rate_ + 0.1 * iter_accept_rate;
        
        // Update best solution and stagnation counter
        int64_t old_best = best_state_.total_hpwl;
        updateBestSolution();
        
        if (best_state_.total_hpwl < old_best) {
            stagnation_counter_ = 0;
            best_hpwl_at_last_improvement_ = best_state_.total_hpwl;
        } else {
            stagnation_counter_++;
        }
        
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
    // Apply best solution back to DPL network
    for (int i = 0; i < network_->getNumNodes(); i++) {
        dpl::Node* node = const_cast<dpl::Node*>(network_->getNode(i));
        if (node->getType() == dpl::Node::CELL) {
            // Move to best position
            node->setLeft(best_state_.cells[i].x);
            node->setBottom(best_state_.cells[i].y);
            node->setOrient(best_state_.cells[i].orient);
        }
    }
}

bool SAWorker::tryRegionShuffle(int region_size)
{
    
    // Get movable cells (cache this if called frequently)
    std::vector<int> movable_cells;
    for (int i = 0; i < network_->getNumNodes(); ++i) {
        dpl::Node* node = const_cast<dpl::Node*>(network_->getNode(i));
        if (node->getType() == dpl::Node::CELL && !node->isFixed()) {
            movable_cells.push_back(i);
        }
    }
    
    if (movable_cells.size() < 2) {
        return false;  // Need at least 2 cells to shuffle
    }
    
    // 1. Select random center point for region
    int max_x_coord = grid_info_->getRowSiteCount();
    int max_y_coord = grid_info_->getRowCount();
    
    std::uniform_int_distribution<int> x_dist(region_size/2, 
                                             std::max(region_size/2, max_x_coord - region_size/2));
    std::uniform_int_distribution<int> y_dist(region_size/2, 
                                             std::max(region_size/2, max_y_coord - region_size/2));
    GridX center_x{x_dist(rng_)};
    GridY center_y{y_dist(rng_)};
    
    // 2. Define region bounds
    GridX x_min{std::max(0, center_x.v - region_size/2)};
    GridX x_max{std::min(max_x_coord, center_x.v + region_size/2)};
    GridY y_min{std::max(0, center_y.v - region_size/2)};
    GridY y_max{std::min(max_y_coord, center_y.v + region_size/2)};
    
    // 3. Collect movable cells in region, grouped by size
    std::map<std::pair<GridX, GridY>, std::vector<int>> size_groups;
    
    for (int cell_id : movable_cells) {
        GridX gx = grid_info_->gridX(state_.cells[cell_id].x);
        GridY gy = grid_info_->gridSnapDownY(state_.cells[cell_id].y);
        
        if (gx >= x_min && gx < x_max && gy >= y_min && gy < y_max) {
            const dpl::Node* node = network_->getNode(cell_id);
            GridX width = grid_info_->gridPaddedWidth(node);
            GridY height = grid_info_->gridHeight(node);
            
            size_groups[{width, height}].push_back(cell_id);
        }
    }
    
    // 4. For each size group with 2+ cells, create random swaps
    std::vector<std::pair<int, int>> swap_pairs;
    
    for (auto& [size, cells] : size_groups) {
        if (cells.size() < 2) continue;
        
        // Shuffle the cell IDs to create random pairings
        std::vector<int> shuffled = cells;
        std::shuffle(shuffled.begin(), shuffled.end(), rng_);
        
        // Create swap pairs (each cell swaps with its shuffled counterpart)
        // Only create swaps where cells actually change positions
        for (size_t i = 0; i < cells.size(); ++i) {
            if (cells[i] != shuffled[i]) {
                // Only add each swap once (avoid duplicates)
                bool already_added = false;
                for (const auto& [c1, c2] : swap_pairs) {
                    if ((c1 == cells[i] && c2 == shuffled[i]) ||
                        (c1 == shuffled[i] && c2 == cells[i])) {
                        already_added = true;
                        break;
                    }
                }
                if (!already_added) {
                    swap_pairs.push_back({cells[i], shuffled[i]});
                }
            }
        }
    }
    
    if (swap_pairs.empty()) {
        return false;  // No swaps possible in this region
    }
    
    kick_attempts_++;
    
    // 5. Save original states and calculate total cost change
    std::vector<CellState> saved_states;
    std::set<int> all_affected_nets;
    
    // Save states and perform swaps temporarily
    for (const auto& [cell1, cell2] : swap_pairs) {
        saved_states.push_back(state_.cells[cell1]);
        saved_states.push_back(state_.cells[cell2]);
        
        // Perform swap in state
        performSwapInState(cell1, cell2);
        
        // Collect affected nets
        auto nets1 = getAffectedNets(cell1);
        auto nets2 = getAffectedNets(cell2);
        all_affected_nets.insert(nets1.begin(), nets1.end());
        all_affected_nets.insert(nets2.begin(), nets2.end());
    }
    
    // 6. Calculate total delta
    std::vector<int> affected_nets_vec(all_affected_nets.begin(), all_affected_nets.end());
    int64_t total_delta = calcDeltaHPWL(affected_nets_vec);
    
    // 7. Accept/reject based on kick temperature
    float kick_temp = temp_ * kick_temp_multiplier_;
    
    if (acceptMove(total_delta, kick_temp)) {
        // Apply all swaps to grid
        for (const auto& [cell1, cell2] : swap_pairs) {
            applySwapToGrid(cell1, cell2);
        }
        
        // Update HPWL cache
        updateHPWLCache(affected_nets_vec);
        state_.total_hpwl += total_delta;
        
        kick_accepted_++;
        total_swaps_applied_ += swap_pairs.size();
        
        // Update best solution if this is better
        updateBestSolution();
        
        /*logger->info(utl::SA2D, 320, "Region shuffle accepted: {} swaps, delta={:.1f} u, region=({},{}) size={}",
                    swap_pairs.size(), 
                    sa2d_->getBlock()->dbuToMicrons(total_delta),
                    center_x.v, center_y.v, region_size);*/
        return true;
    } else {
        // Restore all original states
        int idx = 0;
        for (const auto& [cell1, cell2] : swap_pairs) {
            state_.cells[cell1] = saved_states[idx++];
            state_.cells[cell2] = saved_states[idx++];
        }
        
        kick_rejected_++;
        return false;
    }
}

void SAWorker::performSwapInState(int cell1_id, int cell2_id)
{
    // Swap positions
    std::swap(state_.cells[cell1_id].x, state_.cells[cell2_id].x);
    std::swap(state_.cells[cell1_id].y, state_.cells[cell2_id].y);
    
    // Update orientations based on new positions
    GridX gx1 = grid_info_->gridX(state_.cells[cell1_id].x);
    GridY gy1 = grid_info_->gridSnapDownY(state_.cells[cell1_id].y);
    GridX gx2 = grid_info_->gridX(state_.cells[cell2_id].x);
    GridY gy2 = grid_info_->gridSnapDownY(state_.cells[cell2_id].y);
    
    state_.cells[cell1_id].orient = getCellOrientation(cell1_id, gx1, gy1);
    state_.cells[cell2_id].orient = getCellOrientation(cell2_id, gx2, gy2);
}

void SAWorker::applySwapToGrid(int cell1_id, int cell2_id)
{
    // Remove from grid
    grid_->removeCell(cell1_id);
    grid_->removeCell(cell2_id);
    
    // Get sizes (same for both since we only swap same-sized cells)
    const dpl::Node* node1 = network_->getNode(cell1_id);
    GridX gw = grid_info_->gridPaddedWidth(node1);
    GridY gh = grid_info_->gridHeight(node1);
    
    // Place in swapped positions
    GridX gx1 = grid_info_->gridX(state_.cells[cell1_id].x);
    GridY gy1 = grid_info_->gridSnapDownY(state_.cells[cell1_id].y);
    GridX gx2 = grid_info_->gridX(state_.cells[cell2_id].x);
    GridY gy2 = grid_info_->gridSnapDownY(state_.cells[cell2_id].y);
    
    grid_->placeCell(cell1_id, gx1, gy1, gw, gh);
    grid_->placeCell(cell2_id, gx2, gy2, gw, gh);
}

}  // namespace sa2d 