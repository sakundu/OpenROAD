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
#include "SA2DReorder.h"  // For reordering functionality
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
#include <chrono>
#include <numeric>

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
      enable_slides_(true),
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
            
            // Use appropriate placement method based on cell height
            if (grid_info_->isMultiHeightCell(node)) {
                grid_->placeMultiHeightCell(i, gx, gy, gw, gh);
            } else {
                grid_->placeCell(i, gx, gy, gw, gh);
            }
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

double SAWorker::calcCellHPWLContribution(int cell_id)
{
    // Get all nets connected to this cell
    std::vector<int> affected_nets = getAffectedNets(cell_id);
    
    double total_contribution = 0.0;
    for (int net_id : affected_nets) {
        // For each net, calculate its HPWL
        int64_t net_hpwl = calcNetHPWL(net_id);
        // Add to total contribution
        total_contribution += static_cast<double>(net_hpwl);
    }
    
    return total_contribution;
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
    
    // Check if multi-height and delegate to specialized handler
    if (grid_info_->isMultiHeightCell(node)) {
        return tryMoveMultiHeight(cell_id);
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
    
    // Get cell dimensions
    GridX gw1 = grid_info_->gridPaddedWidth(node1);
    GridY gh1 = grid_info_->gridHeight(node1);
    GridX gw2 = grid_info_->gridPaddedWidth(node2);
    GridY gh2 = grid_info_->gridHeight(node2);
    
    // Check if either is multi-height 
    bool is_multi_height = grid_info_->isMultiHeightCell(node1) || grid_info_->isMultiHeightCell(node2);
    
    // For multi-height cells, require same dimensions and use specialized handler
    if (is_multi_height) {
        if (gw1 != gw2 || gh1 != gh2) {
            return false;
        }
        // Use specialized multi-height swap function
        return trySwapMultiHeight(cell1_id, cell2_id);
    }
    
    attempted_swaps_++;
    
    // Save current state
    state_.cells[cell1_id].prev_x = state_.cells[cell1_id].x;
    state_.cells[cell1_id].prev_y = state_.cells[cell1_id].y;
    state_.cells[cell1_id].prev_orient = state_.cells[cell1_id].orient;
    
    state_.cells[cell2_id].prev_x = state_.cells[cell2_id].x;
    state_.cells[cell2_id].prev_y = state_.cells[cell2_id].y;
    state_.cells[cell2_id].prev_orient = state_.cells[cell2_id].orient;
    
    // Get current positions
    GridX gx1 = grid_info_->gridX(state_.cells[cell1_id].x);
    GridY gy1 = grid_info_->gridSnapDownY(state_.cells[cell1_id].y);
    GridX gx2 = grid_info_->gridX(state_.cells[cell2_id].x);
    GridY gy2 = grid_info_->gridSnapDownY(state_.cells[cell2_id].y);
    
    // For different-sized cells, check if there's enough space BEFORE removing cells
    bool legal1 = false;
    bool legal2 = false;
    bool is_diff_size = (gw1 != gw2 || gh1 != gh2);
    
    if (is_diff_size) {
        // Different sizes - Phase 1: Simple gap-fitting approach
        attempted_diff_size_swaps_++;
        
        // Debug logging for different-size swaps (only log every 100th attempt)
        /*if (attempted_diff_size_swaps_ % 100 == 0) {
            utl::Logger* logger = sa2d_->getLogger();
            logger->info(utl::SA2D, 460, "Worker {} attempting different-size swap #{}: {} ({}x{}) <-> {} ({}x{})",
                        worker_id_, attempted_diff_size_swaps_,
                        network_->getNode(cell1_id)->name(), gw1.v, gh1.v,
                        network_->getNode(cell2_id)->name(), gw2.v, gh2.v);
        }*/
        
        // Check if cell1 can fit at cell2's position
        // We need to check if any part of where cell1 would go is occupied by cells OTHER than cell1 and cell2
        legal1 = true;
        for (GridY y = gy2; y.v < gy2.v + gh1.v && legal1; ++y) {
            for (GridX x = gx2; x.v < gx2.v + gw1.v && legal1; ++x) {
                // Check bounds
                if (x >= grid_info_->getRowSiteCount() || y >= grid_info_->getRowCount()) {
                    legal1 = false;
                    break;
                }
                // Check if occupied by another cell (not cell1 or cell2)
                if (grid_->isOccupied(x, y)) {
                    int occupant = grid_->getCellAt(x, y);
                    if (occupant != cell1_id && occupant != cell2_id) {
                        legal1 = false;
                        break;
                    }
                }
            }
        }
        
        // Check if cell2 can fit at cell1's position
        legal2 = true;
        for (GridY y = gy1; y.v < gy1.v + gh2.v && legal2; ++y) {
            for (GridX x = gx1; x.v < gx1.v + gw2.v && legal2; ++x) {
                // Check bounds
                if (x >= grid_info_->getRowSiteCount() || y >= grid_info_->getRowCount()) {
                    legal2 = false;
                    break;
                }
                // Check if occupied by another cell (not cell1 or cell2)
                if (grid_->isOccupied(x, y)) {
                    int occupant = grid_->getCellAt(x, y);
                    if (occupant != cell1_id && occupant != cell2_id) {
                        legal2 = false;
                        break;
                    }
                }
            }
        }
        
        // Now check additional legality constraints if space is available
        if (legal1) {
            // Temporarily remove cells to check full legality
            grid_->removeCell(cell1_id);
            grid_->removeCell(cell2_id);
            legal1 = canPlaceCell(cell1_id, gx2, gy2);
            // Restore cells using appropriate method
            if (grid_info_->isMultiHeightCell(node1)) {
                grid_->placeMultiHeightCell(cell1_id, gx1, gy1, gw1, gh1);
            } else {
                grid_->placeCell(cell1_id, gx1, gy1, gw1, gh1);
            }
            if (grid_info_->isMultiHeightCell(node2)) {
                grid_->placeMultiHeightCell(cell2_id, gx2, gy2, gw2, gh2);
            } else {
                grid_->placeCell(cell2_id, gx2, gy2, gw2, gh2);
            }
        }
        
        if (legal2) {
            // Temporarily remove cells to check full legality
            grid_->removeCell(cell1_id);
            grid_->removeCell(cell2_id);
            legal2 = canPlaceCell(cell2_id, gx1, gy1);
            // Restore cells using appropriate method
            if (grid_info_->isMultiHeightCell(node1)) {
                grid_->placeMultiHeightCell(cell1_id, gx1, gy1, gw1, gh1);
            } else {
                grid_->placeCell(cell1_id, gx1, gy1, gw1, gh1);
            }
            if (grid_info_->isMultiHeightCell(node2)) {
                grid_->placeMultiHeightCell(cell2_id, gx2, gy2, gw2, gh2);
            } else {
                grid_->placeCell(cell2_id, gx2, gy2, gw2, gh2);
            }
        }
    } else {
        // Same size - use existing simple check
        grid_->removeCell(cell1_id);
        grid_->removeCell(cell2_id);
        legal1 = canPlaceCell(cell1_id, gx2, gy2);
        legal2 = canPlaceCell(cell2_id, gx1, gy1);
        
        if (!legal1 || !legal2) {
            // Restore grid state using appropriate method
            if (grid_info_->isMultiHeightCell(node1)) {
                grid_->placeMultiHeightCell(cell1_id, gx1, gy1, gw1, gh1);
            } else {
                grid_->placeCell(cell1_id, gx1, gy1, gw1, gh1);
            }
            if (grid_info_->isMultiHeightCell(node2)) {
                grid_->placeMultiHeightCell(cell2_id, gx2, gy2, gw2, gh2);
            } else {
                grid_->placeCell(cell2_id, gx2, gy2, gw2, gh2);
            }
        }
    }
    
    if (!legal1 || !legal2) {
        illegal_moves_++;
        return false;
    }
    
    // CRITICAL: For different-sized cells, we need to check that they won't overlap
    // AFTER the swap. This can happen if cells were adjacent before the swap.
    if (is_diff_size) {
        // Check if the swapped positions would overlap
        bool would_overlap = false;
        
        // Check each position that cell1 would occupy at cell2's location
        for (GridY y1 = gy2; y1.v < gy2.v + gh1.v && !would_overlap; ++y1) {
            for (GridX x1 = gx2; x1.v < gx2.v + gw1.v && !would_overlap; ++x1) {
                // Check against each position that cell2 would occupy at cell1's location
                for (GridY y2 = gy1; y2.v < gy1.v + gh2.v && !would_overlap; ++y2) {
                    for (GridX x2 = gx1; x2.v < gx1.v + gw2.v && !would_overlap; ++x2) {
                        if (x1.v == x2.v && y1.v == y2.v) {
                            would_overlap = true;
                            
                            // Debug log (only log every 100th rejection)
                            static int overlap_rejection_count = 0;
                            overlap_rejection_count++;
                            /*if (overlap_rejection_count % 100 == 0) {
                                utl::Logger* logger = sa2d_->getLogger();
                                logger->info(utl::SA2D, 473, "Different-size swap rejected (#{}) - would create overlap at ({},{})! {} and {} were adjacent",
                                            overlap_rejection_count, x1.v, y1.v,
                                            network_->getNode(cell1_id)->name(),
                                            network_->getNode(cell2_id)->name());
                            }*/
                        }
                    }
                }
            }
        }
        
        if (would_overlap) {
            illegal_moves_++;
            return false;
        }
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
        // Update grid with new positions using each cell's actual dimensions
        // For different-sized swaps, we need to remove and re-place both cells
        if (is_diff_size) {
            grid_->removeCell(cell1_id);
            grid_->removeCell(cell2_id);
        }
        
        // Place cells using the appropriate method based on height
        if (grid_info_->isMultiHeightCell(node1)) {
            grid_->placeMultiHeightCell(cell1_id, gx2, gy2, gw1, gh1);
        } else {
            grid_->placeCell(cell1_id, gx2, gy2, gw1, gh1);
        }
        
        if (grid_info_->isMultiHeightCell(node2)) {
            grid_->placeMultiHeightCell(cell2_id, gx1, gy1, gw2, gh2);
        } else {
            grid_->placeCell(cell2_id, gx1, gy1, gw2, gh2);
        }
        
        // Update HPWL cache and total
        updateHPWLCache(affected_nets);
        state_.total_hpwl += delta;
        
        accepted_moves_++;
        accepted_swaps_++;  // Track swaps separately
        if (is_diff_size) {
            accepted_diff_size_swaps_++;  // Track different-size swaps
            
            // Debug log successful different-size swap (only log every 10th acceptance)
            /*if (accepted_diff_size_swaps_ % 10 == 0) {
                utl::Logger* logger = sa2d_->getLogger();
                logger->info(utl::SA2D, 461, "Worker {} ACCEPTED different-size swap #{}: {} -> ({},{}) and {} -> ({},{})",
                            worker_id_, accepted_diff_size_swaps_,
                            network_->getNode(cell1_id)->name(), gx2.v, gy2.v,
                            network_->getNode(cell2_id)->name(), gx1.v, gy1.v);
            }*/
            
            // Overlap verification removed - adjacent cell issue fixed
        }
        return true;
    } else {
        // Restore previous state
        state_.cells[cell1_id].x = state_.cells[cell1_id].prev_x;
        state_.cells[cell1_id].y = state_.cells[cell1_id].prev_y;
        state_.cells[cell1_id].orient = state_.cells[cell1_id].prev_orient;
        
        state_.cells[cell2_id].x = state_.cells[cell2_id].prev_x;
        state_.cells[cell2_id].y = state_.cells[cell2_id].prev_y;
        state_.cells[cell2_id].orient = state_.cells[cell2_id].prev_orient;
        
        // Restore grid state with original dimensions if we had removed them
        if (!is_diff_size) {
            // Place cells using appropriate method based on height
            if (grid_info_->isMultiHeightCell(node1)) {
                grid_->placeMultiHeightCell(cell1_id, gx1, gy1, gw1, gh1);
            } else {
                grid_->placeCell(cell1_id, gx1, gy1, gw1, gh1);
            }
            
            if (grid_info_->isMultiHeightCell(node2)) {
                grid_->placeMultiHeightCell(cell2_id, gx2, gy2, gw2, gh2);
            } else {
                grid_->placeCell(cell2_id, gx2, gy2, gw2, gh2);
            }
        }
        
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
    
    // Get current grid position
    GridY gy = grid_info_->gridSnapDownY(state_.cells[cell_id].y);
    GridY cell_height = grid_info_->gridHeight(node);
    
    // For multi-height cells, check Y symmetry for ALL spanned rows
    if (grid_info_->isMultiHeightCell(node)) {
        for (GridY row = gy; row.v < gy.v + cell_height.v; ++row) {
            if (!grid_info_->isRowYSymmetric(row)) {
                return false;  // At least one row doesn't support Y flipping
            }
        }
    } else {
        // Single-height cell - just check the one row
        if (!grid_info_->isRowYSymmetric(gy)) {
            return false;
        }
    }
    
    // Save current state
    state_.cells[cell_id].prev_x = state_.cells[cell_id].x;
    state_.cells[cell_id].prev_y = state_.cells[cell_id].y;
    state_.cells[cell_id].prev_orient = state_.cells[cell_id].orient;
    
    // Calculate current and flipped orientations
    odb::dbOrientType current_orient = state_.cells[cell_id].orient;
    odb::dbOrientType flipped_orient;
    
    // Y-axis flipping is safe for multi-height cells because power/ground
    // connections remain at the same vertical positions (top/bottom of cell)
    switch (current_orient.getValue()) {
        case odb::dbOrientType::Value::R0:
            flipped_orient = odb::dbOrientType::MY;  // Y-axis flip
            break;
        case odb::dbOrientType::Value::R180:
            flipped_orient = odb::dbOrientType::MX;  // Y-axis flip
            break;
        case odb::dbOrientType::Value::MY:
            flipped_orient = odb::dbOrientType::R0;  // Y-axis flip back
            break;
        case odb::dbOrientType::Value::MX:
            flipped_orient = odb::dbOrientType::R180;  // Y-axis flip back
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
    
    // Skip multi-height cells in chain moves for v0
    if (grid_info_->isMultiHeightCell(node)) {
        // Fall back to simple move for multi-height cells
        return tryMove(cell_id);
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
        if (blocking_height.v > 1 || seed_height.v > 1) {
            // Can't handle multi-height cells in chain for v0
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
        if (blocking_height.v > 1 || seed_height.v > 1) {
            // Can't handle multi-height cells in chain for v0
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
        // Validate current solution before saving as best
        WorkerGrid validation_grid(grid_info_);
        int overlap_count = 0;
        
        for (int i = 0; i < network_->getNumNodes(); i++) {
            const dpl::Node* node = network_->getNode(i);
            if (node->getType() == dpl::Node::CELL) {
                GridX gx = grid_info_->gridX(state_.cells[i].x);
                GridY gy = grid_info_->gridSnapDownY(state_.cells[i].y);
                GridX gw = grid_info_->gridPaddedWidth(node);
                GridY gh = grid_info_->gridHeight(node);
                
                // Check for overlaps
                for (GridY y = gy; y.v < gy.v + gh.v; ++y) {
                    for (GridX x = gx; x.v < gx.v + gw.v; ++x) {
                        if (validation_grid.isOccupied(x, y)) {
                            overlap_count++;
                        }
                    }
                }
                
                // Place the cell
                if (grid_info_->isMultiHeightCell(node)) {
                    validation_grid.placeMultiHeightCell(i, gx, gy, gw, gh);
                } else {
                    validation_grid.placeCell(i, gx, gy, gw, gh);
                }
            }
        }
        
        if (overlap_count > 0) {
            utl::Logger* logger = sa2d_->getLogger();
            /*logger->warn(utl::SA2D, 466, "Worker {} updating best solution WITH {} OVERLAPS! HPWL: {} -> {}",
                        worker_id_, overlap_count, 
                        sa2d_->getBlock()->dbuToMicrons(best_state_.total_hpwl),
                        sa2d_->getBlock()->dbuToMicrons(state_.total_hpwl));*/
            
            // Let's find out which cells are overlapping
            validation_grid.clear();
            std::vector<std::pair<int, int>> overlapping_pairs;
            
            for (int i = 0; i < network_->getNumNodes(); i++) {
                const dpl::Node* node = network_->getNode(i);
                if (node->getType() == dpl::Node::CELL) {
                    GridX gx = grid_info_->gridX(state_.cells[i].x);
                    GridY gy = grid_info_->gridSnapDownY(state_.cells[i].y);
                    GridX gw = grid_info_->gridPaddedWidth(node);
                    GridY gh = grid_info_->gridHeight(node);
                    
                    // Check for overlaps with specific cells
                    for (GridY y = gy; y.v < gy.v + gh.v; ++y) {
                        for (GridX x = gx; x.v < gx.v + gw.v; ++x) {
                            if (validation_grid.isOccupied(x, y)) {
                                int other_cell = validation_grid.getCellAt(x, y);
                                overlapping_pairs.push_back({i, other_cell});
                                /*logger->warn(utl::SA2D, 469, "  Overlap: {} ({}x{}) at ({},{}) overlaps with {} at pixel ({},{})",
                                           node->name(), gw.v, gh.v, gx.v, gy.v,
                                           network_->getNode(other_cell)->name(), x.v, y.v);*/
                            }
                        }
                    }
                    
                    // Place the cell
                    if (grid_info_->isMultiHeightCell(node)) {
                        validation_grid.placeMultiHeightCell(i, gx, gy, gw, gh);
                    } else {
                        validation_grid.placeCell(i, gx, gy, gw, gh);
                    }
                }
            }
            
            // Check grid consistency
            checkGridStateConsistency();
            
            // CRITICAL: Don't update best solution if it has overlaps!
            //logger->warn(utl::SA2D, 470, "REJECTING best solution update due to overlaps!");
            return;  // Don't update best_state_
        }
        
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
    attempted_diff_size_swaps_ = 0;
    accepted_diff_size_swaps_ = 0;
    
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
    
    // Detect single-row scenario and initialize SA1D mode if enabled
    if (use_sa1d_operators_) {
        detectSingleRowMode();
        if (single_row_mode_) {
            logger->info(utl::SA2D, 702, "Using SA1D operators with move probabilities: [{:.2f}, {:.2f}, {:.2f}]",
                        sa1d_move_probs_[0], sa1d_move_probs_[1], sa1d_move_probs_[2]);
        }
    }
    
    // Main SA loop
    for (int iter = 0; iter < max_iter_ && moves_tried < move_budget_; ++iter) {
        int moves_per_temp = moves_per_iter_;  // Use fixed moves per iteration
        int moves_accepted_this_iter = 0;
        int moves_attempted_this_iter = 0;
        
        // Check if we should perform a kick move
        if (shouldPerformKick(iter)) {
            auto start = std::chrono::high_resolution_clock::now();
            bool kick_success = tryRegionShuffle(kick_strength_);
            auto end = std::chrono::high_resolution_clock::now();
            if (kick_success) {
                kick_time_ += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
            }
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
                auto start = std::chrono::high_resolution_clock::now();
                bool chain_accepted = tryChainMove(movable_cells[idx]);
                auto end = std::chrono::high_resolution_clock::now();
                if (chain_accepted) {
                    chain_move_time_ += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
                }
            }
        }
        
        for (int move = 0; move < moves_per_temp && moves_tried < move_budget_; ++move) {
            int old_accepted = accepted_moves_;
            moves_attempted_this_iter++;
            
            bool move_success = false;
            
            if (single_row_mode_) {
                // Use SA1D-style operators
                int move_type = selectMoveType1D();
                std::uniform_int_distribution<int> cell_dist(0, movable_cells.size() - 1);
                int cell_idx = cell_dist(rng_);
                
                switch (move_type) {
                    case 0: {  // Swap
                        int cell_idx2 = cell_dist(rng_);
                        while (cell_idx2 == cell_idx) {
                            cell_idx2 = cell_dist(rng_);
                        }
                        attempted_swaps_1d_++;
                        auto start = std::chrono::high_resolution_clock::now();
                        move_success = trySwapCells1D(movable_cells[cell_idx], movable_cells[cell_idx2]);
                        auto end = std::chrono::high_resolution_clock::now();
                        if (move_success) {
                            swap_time_ += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
                        }
                        break;
                    }
                    case 1: {  // Move
                        int target_pos = selectRandomPosition1D();
                        attempted_moves_1d_++;
                        auto start = std::chrono::high_resolution_clock::now();
                        move_success = tryMoveCell1D(movable_cells[cell_idx], target_pos);
                        auto end = std::chrono::high_resolution_clock::now();
                        if (move_success) {
                            single_move_time_ += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
                        }
                        break;
                    }
                    case 2: {  // Flip
                        attempted_flips_1d_++;
                        auto start = std::chrono::high_resolution_clock::now();
                        move_success = tryFlip(movable_cells[cell_idx]);
                        auto end = std::chrono::high_resolution_clock::now();
                        if (move_success) {
                            flip_time_ += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
                            accepted_flips_1d_++;
                        }
                        break;
                    }
                }
            } else {
                // Use regular SA2D operators
                float rand_val = distribution_(rng_);
                
                if (rand_val < 0.10) {
                    // 10% flips
                    std::uniform_int_distribution<int> cell_dist(0, movable_cells.size() - 1);
                    int idx = cell_dist(rng_);
                    attempted_flips_++;
                    auto start = std::chrono::high_resolution_clock::now();
                    move_success = tryFlip(movable_cells[idx]);
                    auto end = std::chrono::high_resolution_clock::now();
                    if (move_success) {
                        flip_time_ += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
                    }
                } else if (rand_val < 0.20) {
                    // 10% swaps
                    std::uniform_int_distribution<int> cell_dist(0, movable_cells.size() - 1);
                    int idx1 = cell_dist(rng_);
                    int idx2 = cell_dist(rng_);
                    while (idx2 == idx1) {
                        idx2 = cell_dist(rng_);
                    }
                    
                    attempted_swaps_++;
                    auto start = std::chrono::high_resolution_clock::now();
                    move_success = trySwap(movable_cells[idx1], movable_cells[idx2]);
                    auto end = std::chrono::high_resolution_clock::now();
                    if (move_success) {
                        swap_time_ += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
                    }
                } else if (rand_val < 0.30 && enable_slides_) {
                    // 10% slides (if enabled)
                    std::uniform_int_distribution<int> cell_dist(0, movable_cells.size() - 1);
                    int idx = cell_dist(rng_);
                    auto start = std::chrono::high_resolution_clock::now();
                    move_success = trySlide(movable_cells[idx]);
                    auto end = std::chrono::high_resolution_clock::now();
                    if (move_success) {
                        slide_time_ += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
                    }
                } else {
                    // 70% single moves
                    std::uniform_int_distribution<int> cell_dist(0, movable_cells.size() - 1);
                    int idx = cell_dist(rng_);
                    attempted_single_moves_++;
                    auto start = std::chrono::high_resolution_clock::now();
                    move_success = tryMove(movable_cells[idx]);
                    auto end = std::chrono::high_resolution_clock::now();
                    if (move_success) {
                        single_move_time_ += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
                    }
                }
            }
            
            if (move_success) {
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
        
        // Periodically check if we're still in single-row mode
        if (use_sa1d_operators_ && iter % 100 == 0) {
            detectSingleRowMode();
        }
        
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
    
    if (single_row_mode_ && (attempted_swaps_1d_ > 0 || attempted_moves_1d_ > 0 || attempted_flips_1d_ > 0)) {
        logger->info(utl::SA2D, 703, "  SA1D mode statistics:");
        logger->info(utl::SA2D, 704, "    Swaps (1D): {} attempted, {} accepted ({:.1f}%)", 
                    attempted_swaps_1d_, accepted_swaps_1d_, 
                    attempted_swaps_1d_ > 0 ? 100.0 * accepted_swaps_1d_ / attempted_swaps_1d_ : 0.0);
        logger->info(utl::SA2D, 705, "    Moves (1D): {} attempted, {} accepted ({:.1f}%)", 
                    attempted_moves_1d_, accepted_moves_1d_, 
                    attempted_moves_1d_ > 0 ? 100.0 * accepted_moves_1d_ / attempted_moves_1d_ : 0.0);
        logger->info(utl::SA2D, 706, "    Flips (1D): {} attempted, {} accepted ({:.1f}%)", 
                    attempted_flips_1d_, accepted_flips_1d_, 
                    attempted_flips_1d_ > 0 ? 100.0 * accepted_flips_1d_ / attempted_flips_1d_ : 0.0);
    } else {
        logger->info(utl::SA2D, 341, "  Single moves: {} attempted, {} accepted ({:.1f}%)", 
                    attempted_single_moves_, accepted_single_moves_, 
                    attempted_single_moves_ > 0 ? 100.0 * accepted_single_moves_ / attempted_single_moves_ : 0.0);
        logger->info(utl::SA2D, 342, "  Swaps: {} attempted, {} accepted ({:.1f}%)", 
                    attempted_swaps_, accepted_swaps_, 
                    attempted_swaps_ > 0 ? 100.0 * accepted_swaps_ / attempted_swaps_ : 0.0);
        if (attempted_diff_size_swaps_ > 0) {
            logger->info(utl::SA2D, 345, "    Different-size swaps: {} attempted, {} accepted ({:.1f}%)",
                        attempted_diff_size_swaps_, accepted_diff_size_swaps_,
                        100.0 * accepted_diff_size_swaps_ / attempted_diff_size_swaps_);
        }
        logger->info(utl::SA2D, 343, "  Flips: {} attempted, {} accepted ({:.1f}%)", 
                    attempted_flips_, accepted_flips_, 
                    attempted_flips_ > 0 ? 100.0 * accepted_flips_ / attempted_flips_ : 0.0);
        logger->info(utl::SA2D, 346, "  Slides: {} attempted, {} accepted ({:.1f}%)", 
                    attempted_slides_, accepted_slides_, 
                    attempted_slides_ > 0 ? 100.0 * accepted_slides_ / attempted_slides_ : 0.0);
        if (attempted_chain_moves_ > 0) {
            logger->info(utl::SA2D, 344, "  Chain moves: {} attempted, {} accepted ({:.1f}%), max chain length: {}, total cells shifted: {}",
                        attempted_chain_moves_, accepted_chain_moves_,
                        100.0 * accepted_chain_moves_ / attempted_chain_moves_,
                        max_chain_length_, total_cells_shifted_);
        }
    }
    
    // Report kick statistics if kicks were enabled
    if (enable_kicks_ && kick_attempts_ > 0) {
        logger->info(utl::SA2D, 330, "Worker {} LSMC kicks: {} attempted, {} accepted ({:.1f}% success), {} total swaps",
                    worker_id_, kick_attempts_, kick_accepted_, 
                    100.0 * kick_accepted_ / kick_attempts_,
                    total_swaps_applied_);
    }
    
    // Report runtime statistics
    //reportRuntimeStatistics();
    
    logger->info(utl::SA2D, 403, "Worker {} starting SA optimization...", worker_id_);
    logger->info(utl::SA2D, 404, "Parameters: max_temp={}, cooling_rate={}, max_iter={}", 
                 temp_, cooling_rate_, max_iter_);
    logger->info(utl::SA2D, 405, "Move budget: {}, moves per iteration: {}",
                 move_budget_, moves_per_iter_);
    
    // Check if this is a low row design
    if (isLowRowDesign()) {
        logger->info(utl::SA2D, 485, "Low row design detected ({} rows) - using specialized kick moves", 
                     grid_info_->getRowCount());
        logger->info(utl::SA2D, 486, "Kick strategies: horizontal chain swap, row compression, inter-row transfer, sliding window");
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
    accepted_slides_ = 0;
    attempted_flips_ = 0;
    attempted_swaps_ = 0;
    attempted_single_moves_ = 0;
    attempted_slides_ = 0;
    attempted_diff_size_swaps_ = 0;
    accepted_diff_size_swaps_ = 0;
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
            auto start = std::chrono::high_resolution_clock::now();
            bool kick_success = tryRegionShuffle(kick_strength_);
            auto end = std::chrono::high_resolution_clock::now();
            if (kick_success) {
                kick_time_ += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
            }
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
                auto start = std::chrono::high_resolution_clock::now();
                bool chain_accepted = tryChainMove(movable_cells[idx]);
                auto end = std::chrono::high_resolution_clock::now();
                if (chain_accepted) {
                    chain_move_time_ += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
                }
            }
        }
        
        for (int move = 0; move < moves_per_iter; ++move) {
            int old_accepted = accepted_moves_;
            moves_attempted_this_iter++;
            
            // Randomly choose between single move, swap, flip, and slide
            float rand_val = distribution_(rng_);
            
            if (rand_val < 0.10) {
                // 10% flips
                std::uniform_int_distribution<int> cell_dist(0, movable_cells.size() - 1);
                int idx = cell_dist(rng_);
                attempted_flips_++;
                auto start = std::chrono::high_resolution_clock::now();
                bool flip_accepted = tryFlip(movable_cells[idx]);
                auto end = std::chrono::high_resolution_clock::now();
                if (flip_accepted) {
                    flip_time_ += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
                }
            } else if (rand_val < 0.20) {
                // 10% swaps
                std::uniform_int_distribution<int> cell_dist(0, movable_cells.size() - 1);
                int idx1 = cell_dist(rng_);
                int idx2 = cell_dist(rng_);
                while (idx2 == idx1) {
                    idx2 = cell_dist(rng_);
                }
                
                attempted_swaps_++;
                auto start = std::chrono::high_resolution_clock::now();
                bool swap_accepted = trySwap(movable_cells[idx1], movable_cells[idx2]);
                auto end = std::chrono::high_resolution_clock::now();
                if (swap_accepted) {
                    swap_time_ += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
                }
            } else if (rand_val < 0.30 && enable_slides_) {
                // 10% slides (if enabled)
                std::uniform_int_distribution<int> cell_dist(0, movable_cells.size() - 1);
                int idx = cell_dist(rng_);
                auto start = std::chrono::high_resolution_clock::now();
                bool slide_accepted = trySlide(movable_cells[idx]);
                auto end = std::chrono::high_resolution_clock::now();
                if (slide_accepted) {
                    slide_time_ += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
                }
            } else {
                // 70% single moves
                std::uniform_int_distribution<int> cell_dist(0, movable_cells.size() - 1);
                int idx = cell_dist(rng_);
                attempted_single_moves_++;
                auto start = std::chrono::high_resolution_clock::now();
                bool move_accepted = tryMove(movable_cells[idx]);
                auto end = std::chrono::high_resolution_clock::now();
                if (move_accepted) {
                    single_move_time_ += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
                }
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
    
    checkGridStateConsistency();
}

void SAWorker::applyToDPL(dpl::Network* network)
{
    utl::Logger* logger = sa2d_->getLogger();
    logger->info(utl::SA2D, 350, "Worker {} applying best solution with HPWL: {:.1f} u",
                worker_id_, 
                sa2d_->getBlock()->dbuToMicrons(best_state_.total_hpwl));
    
    // First, validate the best solution for overlaps
    logger->info(utl::SA2D, 463, "Validating best solution for overlaps...");
    
    // Create a temporary grid to check for overlaps
    WorkerGrid validation_grid(grid_info_);
    std::vector<std::pair<int, int>> overlapping_cells;
    
    for (int i = 0; i < network_->getNumNodes(); i++) {
        dpl::Node* node = const_cast<dpl::Node*>(network_->getNode(i));
        if (node->getType() == dpl::Node::CELL) {
            GridX gx = grid_info_->gridX(best_state_.cells[i].x);
            GridY gy = grid_info_->gridSnapDownY(best_state_.cells[i].y);
            GridX gw = grid_info_->gridPaddedWidth(node);
            GridY gh = grid_info_->gridHeight(node);
            
            // Check if this placement would overlap
            bool has_overlap = false;
            for (GridY y = gy; y.v < gy.v + gh.v && !has_overlap; ++y) {
                for (GridX x = gx; x.v < gx.v + gw.v && !has_overlap; ++x) {
                    if (validation_grid.isOccupied(x, y)) {
                        int other_id = validation_grid.getCellAt(x, y);
                        overlapping_cells.push_back({i, other_id});
                        has_overlap = true;
                        
                        logger->warn(utl::SA2D, 464, "OVERLAP DETECTED: {} at ({},{}) overlaps with {} at pixel ({},{})",
                                    node->name(), gx.v, gy.v,
                                    network_->getNode(other_id)->name(), x.v, y.v);
                    }
                }
            }
            
            // Place the cell
            if (grid_info_->isMultiHeightCell(node)) {
                validation_grid.placeMultiHeightCell(i, gx, gy, gw, gh);
            } else {
                validation_grid.placeCell(i, gx, gy, gw, gh);
            }
        }
    }
    
    if (!overlapping_cells.empty()) {
        logger->warn(utl::SA2D, 465, "Found {} overlaps in best solution before applying to DPL!", 
                     overlapping_cells.size());
    }
    
    // Apply best solution back to DPL network
    int cells_updated = 0;
    for (int i = 0; i < network_->getNumNodes(); i++) {
        dpl::Node* node = const_cast<dpl::Node*>(network_->getNode(i));
        if (node->getType() == dpl::Node::CELL) {
            // Move to best position
            node->setLeft(best_state_.cells[i].x);
            node->setBottom(best_state_.cells[i].y);
            node->setOrient(best_state_.cells[i].orient);
            cells_updated++;
        }
    }
    
    logger->info(utl::SA2D, 351, "Updated {} cells in DPL network", cells_updated);
    
    // Optional: Apply reordering to the best solution
    if (sa2d_->shouldPerformReordering()) {
        // Check if we should use DPL's reordering instead
        if (sa2d_->getUseDPLReordering()) {
            logger->info(utl::SA2D, 605, "Skipping SA2D reordering - will use DPL reordering after SA completes");
        } else {
            logger->warn(utl::SA2D, 605, "SA2D's built-in reordering is disabled due to overlap issues.");
            logger->info(utl::SA2D, 606, "Please use DPL's native reordering instead:");
            logger->info(utl::SA2D, 607, "  sa2d_set_use_dpl_reordering 1");
            logger->info(utl::SA2D, 608, "  sa2d_set_enable_reordering 1");
            logger->info(utl::SA2D, 609, "This will run DPL's proven reordering after SA completes.");
            
            // DISABLED: SA2D's reordering implementation causes overlaps
            // Use DPL's native reordering instead which is guaranteed to be safe
        }
    }
}

// SA1D Operators Implementation

bool SAWorker::isSingleRowPlacement()
{
    // Check if all movable cells are in the same row
    GridY first_row = GridY{-1};
    for (int i = 0; i < network_->getNumNodes(); i++) {
        const dpl::Node* node = network_->getNode(i);
        if (node->getType() == dpl::Node::CELL && !node->isFixed()) {
            GridY row = grid_info_->gridSnapDownY(state_.cells[i].y);
            if (first_row.v == -1) {
                first_row = row;
            } else if (row.v != first_row.v) {
                return false;  // Multiple rows found
            }
        }
    }
    return first_row.v != -1;  // At least one movable cell found
}

void SAWorker::detectSingleRowMode()
{
    bool was_single_row = single_row_mode_;
    single_row_mode_ = isSingleRowPlacement();
    
    if (single_row_mode_ && !was_single_row) {
        utl::Logger* logger = sa2d_->getLogger();
        logger->info(utl::SA2D, 700, "Single-row placement detected, switching to SA1D operators");
        initializeOrdering1D();
    } else if (!single_row_mode_ && was_single_row) {
        utl::Logger* logger = sa2d_->getLogger();
        logger->info(utl::SA2D, 701, "Multi-row placement detected, switching back to SA2D operators");
        cell_ordering_1d_.clear();
    }
}

void SAWorker::initializeOrdering1D()
{
    if (use_best_orderings_1d_) {
        // TODO: Implement best orderings integration
        // For now, use current placement ordering
        buildOrdering1DFromCurrentPlacement();
    } else {
        // Default: maintain current X-coordinate ordering
        buildOrdering1DFromCurrentPlacement();
    }
}

void SAWorker::buildOrdering1DFromCurrentPlacement()
{
    cell_ordering_1d_.clear();
    
    // Collect all movable cells with their X positions
    std::vector<std::pair<DbuX, int>> cell_positions;
    for (int i = 0; i < network_->getNumNodes(); i++) {
        const dpl::Node* node = network_->getNode(i);
        if (node->getType() == dpl::Node::CELL && !node->isFixed()) {
            cell_positions.push_back({state_.cells[i].x, i});
        }
    }
    
    // Sort by X position
    std::sort(cell_positions.begin(), cell_positions.end(),
              [](const std::pair<DbuX, int>& a, const std::pair<DbuX, int>& b) {
                  return a.first.v < b.first.v;
              });
    
    // Build ordering
    for (const auto& pair : cell_positions) {
        cell_ordering_1d_.push_back(pair.second);
    }
}

void SAWorker::updateOrdering1D()
{
    // Rebuild ordering from current placement
    buildOrdering1DFromCurrentPlacement();
}

int SAWorker::getCellPosition1D(int cell_id)
{
    auto it = std::find(cell_ordering_1d_.begin(), cell_ordering_1d_.end(), cell_id);
    if (it != cell_ordering_1d_.end()) {
        return std::distance(cell_ordering_1d_.begin(), it);
    }
    return -1;  // Not found
}

int SAWorker::getCellAt1DPosition(int pos)
{
    if (pos >= 0 && pos < static_cast<int>(cell_ordering_1d_.size())) {
        return cell_ordering_1d_[pos];
    }
    return -1;  // Invalid position
}

void SAWorker::updateGrid1D(const std::vector<int>& affected_cells)
{
    // Remove all affected cells from grid
    for (int cell_id : affected_cells) {
        grid_->removeCell(cell_id);
    }
    
    // Re-place all affected cells
    for (int cell_id : affected_cells) {
        const dpl::Node* node = network_->getNode(cell_id);
        GridX gx = grid_info_->gridX(state_.cells[cell_id].x);
        GridY gy = grid_info_->gridSnapDownY(state_.cells[cell_id].y);
        GridX gw = grid_info_->gridPaddedWidth(node);
        GridY gh = grid_info_->gridHeight(node);
        
        if (grid_info_->isMultiHeightCell(node)) {
            grid_->placeMultiHeightCell(cell_id, gx, gy, gw, gh);
        } else {
            grid_->placeCell(cell_id, gx, gy, gw, gh);
        }
    }
}

int SAWorker::computeOverlap1D()
{
    if (!single_row_mode_) return 0;
    
    int total_overlap = 0;
    
    // Sort cells by X position
    std::vector<std::pair<DbuX, int>> cell_positions;
    for (int cell_id : cell_ordering_1d_) {
        cell_positions.push_back({state_.cells[cell_id].x, cell_id});
    }
    
    std::sort(cell_positions.begin(), cell_positions.end(),
              [](const std::pair<DbuX, int>& a, const std::pair<DbuX, int>& b) {
                  return a.first.v < b.first.v;
              });
    
    // Calculate overlaps
    for (size_t i = 0; i < cell_positions.size() - 1; i++) {
        int curr_cell = cell_positions[i].second;
        int next_cell = cell_positions[i + 1].second;
        
        const dpl::Node* curr_node = network_->getNode(curr_cell);
        DbuX curr_right = state_.cells[curr_cell].x + curr_node->getWidth();
        DbuX next_left = state_.cells[next_cell].x;
        
        if (curr_right.v > next_left.v) {
            total_overlap += curr_right.v - next_left.v;
        }
    }
    
    return total_overlap;
}

int SAWorker::selectMoveType1D()
{
    float rand_num = distribution_(rng_);
    float cumulative = 0.0f;
    
    for (size_t i = 0; i < sa1d_move_probs_.size(); ++i) {
        cumulative += sa1d_move_probs_[i];
        if (rand_num <= cumulative) {
            return static_cast<int>(i);
        }
    }
    
    return 0;  // Default to swap
}

int SAWorker::selectRandomPosition1D()
{
    if (cell_ordering_1d_.empty()) return 0;
    
    std::uniform_int_distribution<int> pos_dist(0, cell_ordering_1d_.size() - 1);
    return pos_dist(rng_);
}

bool SAWorker::trySwapCells1D(int cell_id1, int cell_id2)
{
    if (cell_id1 == cell_id2) return false;
    
    // Get positions in the 1D ordering
    int pos1 = getCellPosition1D(cell_id1);
    int pos2 = getCellPosition1D(cell_id2);
    
    if (pos1 == -1 || pos2 == -1) return false;  // Cell not found in ordering
    
    if (pos1 > pos2) {
        std::swap(pos1, pos2);
        std::swap(cell_id1, cell_id2);
    }
    
    const dpl::Node* node1 = network_->getNode(cell_id1);
    const dpl::Node* node2 = network_->getNode(cell_id2);
    
    GridX width1 = grid_info_->gridPaddedWidth(node1);
    GridX width2 = grid_info_->gridPaddedWidth(node2);
    bool same_width = (width1.v == width2.v);
    
    // Store original positions
    DbuX orig_x1 = state_.cells[cell_id1].x;
    DbuX orig_x2 = state_.cells[cell_id2].x;
    
    // Calculate new positions
    DbuX new_x1 = orig_x2;
    DbuX new_x2 = orig_x1;
    
    std::vector<int> affected_cells = {cell_id1, cell_id2};
    std::vector<DbuX> original_positions = {orig_x1, orig_x2};
    
    // Handle width differences (like SA1D)
    if (!same_width) {
        int delta_width = width1.v - width2.v;
        new_x1 = DbuX{orig_x2.v - delta_width};
        
        // Shift intermediate cells
        for (int pos = pos1 + 1; pos < pos2; pos++) {
            int intermediate_cell = getCellAt1DPosition(pos);
            if (intermediate_cell != -1) {
                original_positions.push_back(state_.cells[intermediate_cell].x);
                state_.cells[intermediate_cell].x = DbuX{state_.cells[intermediate_cell].x.v - delta_width};
                affected_cells.push_back(intermediate_cell);
            }
        }
    }
    
    // Apply swap
    state_.cells[cell_id1].x = new_x1;
    state_.cells[cell_id2].x = new_x2;
    
    // Update grid occupancy
    updateGrid1D(affected_cells);
    
    // Calculate HPWL change
    std::vector<int> affected_nets = getAffectedNets(cell_id1);
    std::vector<int> affected_nets2 = getAffectedNets(cell_id2);
    affected_nets.insert(affected_nets.end(), affected_nets2.begin(), affected_nets2.end());
    
    // Remove duplicates
    std::sort(affected_nets.begin(), affected_nets.end());
    affected_nets.erase(std::unique(affected_nets.begin(), affected_nets.end()), affected_nets.end());
    
    int64_t delta = calcDeltaHPWL(affected_nets);
    
    if (acceptMove(delta, temp_)) {
        updateHPWLCache(affected_nets);
        state_.total_hpwl += delta;
        accepted_swaps_1d_++;
        return true;
    } else {
        // Restore original positions
        for (size_t i = 0; i < affected_cells.size(); ++i) {
            state_.cells[affected_cells[i]].x = original_positions[i];
        }
        updateGrid1D(affected_cells);
        return false;
    }
}

bool SAWorker::tryMoveCell1D(int cell_id, int target_pos)
{
    int current_pos = getCellPosition1D(cell_id);
    if (current_pos == -1 || current_pos == target_pos) return false;
    
    if (target_pos < 0 || target_pos >= static_cast<int>(cell_ordering_1d_.size())) {
        return false;
    }
    
    const dpl::Node* node = network_->getNode(cell_id);
    GridX width = grid_info_->gridPaddedWidth(node);
    
    std::vector<int> affected_cells = {cell_id};
    std::vector<DbuX> original_positions = {state_.cells[cell_id].x};
    
    DbuX new_x;
    
    if (current_pos > target_pos) {
        // Moving left - insert before target position
        int target_cell = getCellAt1DPosition(target_pos);
        if (target_cell == -1) return false;
        
        new_x = state_.cells[target_cell].x;
        
        // Shift cells right
        for (int pos = target_pos; pos < current_pos; pos++) {
            int shift_cell = getCellAt1DPosition(pos);
            if (shift_cell != -1) {
                original_positions.push_back(state_.cells[shift_cell].x);
                state_.cells[shift_cell].x = DbuX{state_.cells[shift_cell].x.v + width.v};
                affected_cells.push_back(shift_cell);
            }
        }
    } else {
        // Moving right - insert after target position
        int target_cell = getCellAt1DPosition(target_pos);
        if (target_cell == -1) return false;
        
        const dpl::Node* target_node = network_->getNode(target_cell);
        GridX target_width = grid_info_->gridPaddedWidth(target_node);
        
        new_x = DbuX{state_.cells[target_cell].x.v + target_width.v - width.v};
        
        // Shift cells left
        for (int pos = current_pos + 1; pos <= target_pos; pos++) {
            int shift_cell = getCellAt1DPosition(pos);
            if (shift_cell != -1) {
                original_positions.push_back(state_.cells[shift_cell].x);
                state_.cells[shift_cell].x = DbuX{state_.cells[shift_cell].x.v - width.v};
                affected_cells.push_back(shift_cell);
            }
        }
    }
    
    // Apply move
    state_.cells[cell_id].x = new_x;
    
    // Update grid and calculate delta
    updateGrid1D(affected_cells);
    
    std::vector<int> affected_nets = getAffectedNets(cell_id);
    int64_t delta = calcDeltaHPWL(affected_nets);
    
    if (acceptMove(delta, temp_)) {
        updateHPWLCache(affected_nets);
        state_.total_hpwl += delta;
        updateOrdering1D();  // Maintain 1D ordering
        accepted_moves_1d_++;
        return true;
    } else {
        // Restore original positions
        for (size_t i = 0; i < affected_cells.size(); ++i) {
            state_.cells[affected_cells[i]].x = original_positions[i];
        }
        updateGrid1D(affected_cells);
        return false;
    }
}

bool SAWorker::tryRegionShuffle(int region_size)
{
    // For low row count designs, use speculative kick moves
    if (isLowRowDesign()) {
        return trySpeculativeKick();
    }
    
    // Original region-based shuffle for normal designs
    // Get movable cells (cache this if called frequently)
    std::vector<int> movable_cells;
    for (int i = 0; i < network_->getNumNodes(); ++i) {
        dpl::Node* node = const_cast<dpl::Node*>(network_->getNode(i));
        if (node->getType() == dpl::Node::CELL && !node->isFixed()) {
            movable_cells.push_back(i);
        }
    }
    
    if (movable_cells.empty()) {
        return false;  // No movable cells
    }
    
    // 1. Select random center point for region
    int max_x_coord = grid_info_->getRowSiteCount();
    int max_y_coord = grid_info_->getRowCount();
    
    // Debug: Log grid dimensions on first attempt
    /*if (kick_attempts_ == 0) {
        utl::Logger* logger = sa2d_->getLogger();
        logger->info(utl::SA2D, 474, "Grid dimensions: {} sites x {} rows", max_x_coord, max_y_coord);
    }*/
    
    // Fix region center selection to handle small grids
    int x_min_center = std::min(region_size/2, max_x_coord - 1);
    int x_max_center = std::max(x_min_center, max_x_coord - region_size/2);
    int y_min_center = std::min(region_size/2, max_y_coord - 1);
    int y_max_center = std::max(y_min_center, max_y_coord - region_size/2);
    
    std::uniform_int_distribution<int> x_dist(x_min_center, x_max_center);
    std::uniform_int_distribution<int> y_dist(y_min_center, y_max_center);
    GridX center_x{x_dist(rng_)};
    GridY center_y{y_dist(rng_)};
    
    // 2. Define region bounds
    GridX x_min{std::max(0, center_x.v - region_size/2)};
    GridX x_max{std::min(max_x_coord, center_x.v + region_size/2)};
    GridY y_min{std::max(0, center_y.v - region_size/2)};
    GridY y_max{std::min(max_y_coord, center_y.v + region_size/2)};
    
    // 3. Collect all movable cells in region
    std::vector<int> cells_in_region;
    
    for (int cell_id : movable_cells) {
        GridX gx = grid_info_->gridX(state_.cells[cell_id].x);
        GridY gy = grid_info_->gridSnapDownY(state_.cells[cell_id].y);
        
        if (gx >= x_min && gx < x_max && gy >= y_min && gy < y_max) {
            cells_in_region.push_back(cell_id);
        }
    }
    
    if (cells_in_region.size() < 2) {
        return false;  // Need at least 2 cells in region
    }
    
    // 4. Find free locations in the region
    std::vector<GridPt> free_locations;
    for (GridY y = y_min; y < y_max; ++y) {
        for (GridX x = x_min; x < x_max; ++x) {
            if (!grid_->isOccupied(x, y)) {
                free_locations.push_back(GridPt{x, y});
            }
        }
    }
    
    // 5. Create moves: both swaps (any dimensions) AND moves to free locations
    std::vector<std::pair<int, int>> swap_pairs;
    std::vector<std::pair<int, GridPt>> single_moves;
    
    // Strategy 1: Create random swap pairs (different dimensions allowed)
    int swaps_to_try = std::min(5, (int)cells_in_region.size() / 2);
    std::shuffle(cells_in_region.begin(), cells_in_region.end(), rng_);
    
    for (int i = 0; i < swaps_to_try * 2 && i + 1 < cells_in_region.size(); i += 2) {
        swap_pairs.push_back({cells_in_region[i], cells_in_region[i+1]});
    }
    
    // Strategy 2: Try to move some cells to free locations
    if (!free_locations.empty() && cells_in_region.size() > swaps_to_try * 2) {
        // Move cells that weren't involved in swaps
        int single_moves_to_try = std::min(5, (int)(cells_in_region.size() - swaps_to_try * 2));
        std::shuffle(free_locations.begin(), free_locations.end(), rng_);
        
        for (int i = swaps_to_try * 2; i < swaps_to_try * 2 + single_moves_to_try && i < cells_in_region.size(); ++i) {
            int cell_id = cells_in_region[i];
            const dpl::Node* node = network_->getNode(cell_id);
            GridX width = grid_info_->gridPaddedWidth(node);
            GridY height = grid_info_->gridHeight(node);
            
            // Find a free location that can fit this cell
            for (const auto& loc : free_locations) {
                // Check if cell can fit at this location
                bool fits = true;
                if (loc.x.v + width.v <= x_max.v && loc.y.v + height.v <= y_max.v) {
                    for (GridY y = loc.y; y.v < loc.y.v + height.v && fits; ++y) {
                        for (GridX x = loc.x; x.v < loc.x.v + width.v && fits; ++x) {
                            if (grid_->isOccupied(x, y)) {
                                fits = false;
                            }
                        }
                    }
                    
                    if (fits) {
                        single_moves.push_back({cell_id, loc});
                        break;  // Found a location for this cell
                    }
                }
            }
        }
    }
    
    if (swap_pairs.empty() && single_moves.empty()) {
        return false;  // No moves possible
    }
    
    kick_attempts_++;
    
    // Debug: Log kick attempt details
    /*if (kick_attempts_ % 1000 == 1) {  // Log every 1000th attempt
        utl::Logger* logger = sa2d_->getLogger();
        logger->info(utl::SA2D, 475, "Kick attempt {}: region ({},{}) size {}, {} cells in region, {} swaps, {} single moves",
                    kick_attempts_, center_x.v, center_y.v, region_size, 
                    cells_in_region.size(), swap_pairs.size(), single_moves.size());
    }*/
    
    // 6. Validate all moves can be legally applied together
    WorkerGrid temp_grid(grid_info_);
    temp_grid.copyFrom(*grid_);
    
    // First, remove all cells that will be moved
    for (const auto& [cell1, cell2] : swap_pairs) {
        temp_grid.removeCell(cell1);
        temp_grid.removeCell(cell2);
    }
    for (const auto& [cell_id, target] : single_moves) {
        temp_grid.removeCell(cell_id);
    }
    
    // Then check if all moves are legal
    bool all_moves_legal = true;
    
    // Check swap pairs (different dimensions allowed)
    for (const auto& [cell1, cell2] : swap_pairs) {
        const dpl::Node* node1 = network_->getNode(cell1);
        const dpl::Node* node2 = network_->getNode(cell2);
        GridX gw1 = grid_info_->gridPaddedWidth(node1);
        GridY gh1 = grid_info_->gridHeight(node1);
        GridX gw2 = grid_info_->gridPaddedWidth(node2);
        GridY gh2 = grid_info_->gridHeight(node2);
        
        // Get current positions
        GridX gx1 = grid_info_->gridX(state_.cells[cell1].x);
        GridY gy1 = grid_info_->gridSnapDownY(state_.cells[cell1].y);
        GridX gx2 = grid_info_->gridX(state_.cells[cell2].x);
        GridY gy2 = grid_info_->gridSnapDownY(state_.cells[cell2].y);
        
        // Check if cell1 can fit at cell2's position (with cell1's dimensions)
        bool can_place1 = true;
        if (gx2.v + gw1.v > grid_info_->getRowSiteCount() || gy2.v + gh1.v > grid_info_->getRowCount()) {
            can_place1 = false;
        } else {
            for (GridY y = gy2; y.v < gy2.v + gh1.v && can_place1; ++y) {
                for (GridX x = gx2; x.v < gx2.v + gw1.v && can_place1; ++x) {
                    if (temp_grid.isOccupied(x, y)) {
                        can_place1 = false;
                    }
                }
            }
        }
        
        // Check if cell2 can fit at cell1's position (with cell2's dimensions)
        bool can_place2 = true;
        if (gx1.v + gw2.v > grid_info_->getRowSiteCount() || gy1.v + gh2.v > grid_info_->getRowCount()) {
            can_place2 = false;
        } else {
            for (GridY y = gy1; y.v < gy1.v + gh2.v && can_place2; ++y) {
                for (GridX x = gx1; x.v < gx1.v + gw2.v && can_place2; ++x) {
                    if (temp_grid.isOccupied(x, y)) {
                        can_place2 = false;
                    }
                }
            }
        }
        
        // Also check basic legality (site compatibility, etc.)
        if (can_place1) {
            can_place1 = canPlaceCell(cell1, gx2, gy2);
        }
        if (can_place2) {
            can_place2 = canPlaceCell(cell2, gx1, gy1);
        }
        
        if (!can_place1 || !can_place2) {
            all_moves_legal = false;
            break;
        }
        
        // Place in temp grid with their own dimensions
        temp_grid.placeCell(cell1, gx2, gy2, gw1, gh1);
        temp_grid.placeCell(cell2, gx1, gy1, gw2, gh2);
    }
    
    // Check single moves
    if (all_moves_legal) {
        for (const auto& [cell_id, target] : single_moves) {
            const dpl::Node* node = network_->getNode(cell_id);
            GridX gw = grid_info_->gridPaddedWidth(node);
            GridY gh = grid_info_->gridHeight(node);
            
            if (!canPlaceCell(cell_id, target.x, target.y)) {
                all_moves_legal = false;
                break;
            }
            
            // Place in temp grid
            temp_grid.placeCell(cell_id, target.x, target.y, gw, gh);
        }
    }
    
    if (!all_moves_legal) {
        kick_rejected_++;
        /*if (kick_rejected_ % 1000 == 0) {  // Log every 1000th rejection
            utl::Logger* logger = sa2d_->getLogger();
            logger->info(utl::SA2D, 476, "Kick rejected due to legality: {} total rejections", kick_rejected_);
        }*/
        return false;
    }
    
    // 7. Save original states and calculate cost
    std::vector<CellState> saved_states;
    std::set<int> all_affected_nets;
    
    // Save states and perform moves temporarily
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
    
    for (const auto& [cell_id, target] : single_moves) {
        saved_states.push_back(state_.cells[cell_id]);
        
        // Move cell to target location
        state_.cells[cell_id].x = grid_info_->gridToDbuX(target.x);
        state_.cells[cell_id].y = grid_info_->gridYToDbu(target.y);
        state_.cells[cell_id].orient = getCellOrientation(cell_id, target.x, target.y);
        
        // Collect affected nets
        auto nets = getAffectedNets(cell_id);
        all_affected_nets.insert(nets.begin(), nets.end());
    }
    
    // 8. Calculate total delta
    std::vector<int> affected_nets_vec(all_affected_nets.begin(), all_affected_nets.end());
    int64_t total_delta = calcDeltaHPWL(affected_nets_vec);
    
    // 9. Accept/reject based on kick temperature
    float kick_temp = temp_ * kick_temp_multiplier_;
    
    if (acceptMove(total_delta, kick_temp)) {
        // Apply all moves to the actual grid
        grid_->clear();  // Clear and rebuild for consistency
        
        // Rebuild grid with new positions
        for (int i = 0; i < network_->getNumNodes(); i++) {
            const dpl::Node* node = network_->getNode(i);
            if (node->getType() == dpl::Node::CELL) {
                GridX gx = grid_info_->gridX(state_.cells[i].x);
                GridY gy = grid_info_->gridSnapDownY(state_.cells[i].y);
                GridX gw = grid_info_->gridPaddedWidth(node);
                GridY gh = grid_info_->gridHeight(node);
                
                if (grid_info_->isMultiHeightCell(node)) {
                    grid_->placeMultiHeightCell(i, gx, gy, gw, gh);
                } else {
                    grid_->placeCell(i, gx, gy, gw, gh);
                }
            }
        }
        
        // Update HPWL cache
        updateHPWLCache(affected_nets_vec);
        state_.total_hpwl += total_delta;
        
        kick_accepted_++;
        total_swaps_applied_ += swap_pairs.size() + single_moves.size();
        
        // Update best solution if this is better
        updateBestSolution();
        
        return true;
    } else {
        // Restore all original states
        int idx = 0;
        for (const auto& [cell1, cell2] : swap_pairs) {
            state_.cells[cell1] = saved_states[idx++];
            state_.cells[cell2] = saved_states[idx++];
        }
        for (const auto& [cell_id, target] : single_moves) {
            state_.cells[cell_id] = saved_states[idx++];
        }
        
        kick_rejected_++;
        if (kick_rejected_ % 1000 == 0) {  // Log every 1000th rejection
            utl::Logger* logger = sa2d_->getLogger();
            logger->info(utl::SA2D, 477, "Kick rejected by SA: delta={}, temp={}, kick_temp={}, total rejections={}",
                        total_delta, temp_, kick_temp, kick_rejected_);
        }
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
    
    // Get sizes for each cell (may be different!)
    const dpl::Node* node1 = network_->getNode(cell1_id);
    const dpl::Node* node2 = network_->getNode(cell2_id);
    GridX gw1 = grid_info_->gridPaddedWidth(node1);
    GridY gh1 = grid_info_->gridHeight(node1);
    GridX gw2 = grid_info_->gridPaddedWidth(node2);
    GridY gh2 = grid_info_->gridHeight(node2);
    
    // Place in swapped positions using each cell's own dimensions
    GridX gx1 = grid_info_->gridX(state_.cells[cell1_id].x);
    GridY gy1 = grid_info_->gridSnapDownY(state_.cells[cell1_id].y);
    GridX gx2 = grid_info_->gridX(state_.cells[cell2_id].x);
    GridY gy2 = grid_info_->gridSnapDownY(state_.cells[cell2_id].y);
    
    // Place cells using appropriate method based on height
    if (grid_info_->isMultiHeightCell(node1)) {
        grid_->placeMultiHeightCell(cell1_id, gx1, gy1, gw1, gh1);
    } else {
        grid_->placeCell(cell1_id, gx1, gy1, gw1, gh1);
    }
    
    if (grid_info_->isMultiHeightCell(node2)) {
        grid_->placeMultiHeightCell(cell2_id, gx2, gy2, gw2, gh2);
    } else {
        grid_->placeCell(cell2_id, gx2, gy2, gw2, gh2);
    }
}

// Multi-height cell operations

bool SAWorker::tryMoveMultiHeight(int cell_id)
{
    dpl::Node* node = const_cast<dpl::Node*>(network_->getNode(cell_id));
    
    // Get cell height in rows
    GridY cell_height = grid_info_->getCellHeightInRows(node);
    GridX cell_width = grid_info_->gridPaddedWidth(node);
    
    // Save current state
    state_.cells[cell_id].prev_x = state_.cells[cell_id].x;
    state_.cells[cell_id].prev_y = state_.cells[cell_id].y;
    state_.cells[cell_id].prev_orient = state_.cells[cell_id].orient;
    
    // Generate new position within displacement limits
    GridPt new_pos = generateRandomPosition(cell_id);
    
    // Ensure we don't go off the top
    if (new_pos.y.v + cell_height.v > grid_info_->getRowCount()) {
        new_pos.y = GridY{grid_info_->getRowCount() - cell_height.v};
    }
    
    // Check legality for multi-height cell
    if (!canPlaceMultiHeightCell(cell_id, new_pos.x, new_pos.y)) {
        illegal_moves_++;
        return false;
    }
    
    // Update position temporarily (multi-height cells typically keep orientation)
    state_.cells[cell_id].x = grid_info_->gridToDbuX(new_pos.x);
    state_.cells[cell_id].y = grid_info_->gridYToDbu(new_pos.y);
    // Keep current orientation for multi-height cells
    
    // Calculate cost change
    std::vector<int> affected_nets = getAffectedNets(cell_id);
    int64_t delta = calcDeltaHPWL(affected_nets);
    
    if (acceptMove(delta, temp_)) {
        // Update grid occupancy
        grid_->placeMultiHeightCell(cell_id, new_pos.x, new_pos.y, 
                                   cell_width, cell_height);
        
        // Update HPWL cache and total
        updateHPWLCache(affected_nets);
        state_.total_hpwl += delta;
        
        accepted_moves_++;
        accepted_single_moves_++;
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

bool SAWorker::trySwapMultiHeight(int cell1_id, int cell2_id)
{
    dpl::Node* node1 = const_cast<dpl::Node*>(network_->getNode(cell1_id));
    
    // Verify they have same dimensions (should be checked by caller)
    GridX width = grid_info_->gridPaddedWidth(node1);
    GridY height = grid_info_->getCellHeightInRows(node1);
    
    // Save current states
    state_.cells[cell1_id].prev_x = state_.cells[cell1_id].x;
    state_.cells[cell1_id].prev_y = state_.cells[cell1_id].y;
    state_.cells[cell1_id].prev_orient = state_.cells[cell1_id].orient;
    
    state_.cells[cell2_id].prev_x = state_.cells[cell2_id].x;
    state_.cells[cell2_id].prev_y = state_.cells[cell2_id].y;
    state_.cells[cell2_id].prev_orient = state_.cells[cell2_id].orient;
    
    // Get current positions
    GridX x1 = grid_info_->gridX(state_.cells[cell1_id].x);
    GridY y1 = grid_info_->gridSnapDownY(state_.cells[cell1_id].y);
    GridX x2 = grid_info_->gridX(state_.cells[cell2_id].x);
    GridY y2 = grid_info_->gridSnapDownY(state_.cells[cell2_id].y);
    
    // Skip if already in same position
    if (x1 == x2 && y1 == y2) {
        return false;
    }
    
    // Remove both cells from grid
    grid_->removeCell(cell1_id);
    grid_->removeCell(cell2_id);
    
    // Check if swap is legal (multi-height specific)
    bool legal1 = grid_->canPlaceMultiHeight(cell1_id, x2, y2, width, height);
    bool legal2 = grid_->canPlaceMultiHeight(cell2_id, x1, y1, width, height);
    
    if (!legal1 || !legal2) {
        // Restore grid state
        grid_->placeMultiHeightCell(cell1_id, x1, y1, width, height);
        grid_->placeMultiHeightCell(cell2_id, x2, y2, width, height);
        
        illegal_moves_++;
        return false;
    }
    
    // Perform swap in state
    state_.cells[cell1_id].x = grid_info_->gridToDbuX(x2);
    state_.cells[cell1_id].y = grid_info_->gridYToDbu(y2);
    // Keep orientations unchanged for multi-height cells
    
    state_.cells[cell2_id].x = grid_info_->gridToDbuX(x1);
    state_.cells[cell2_id].y = grid_info_->gridYToDbu(y1);
    // Keep orientations unchanged for multi-height cells
    
    // Calculate cost change
    std::set<int> affected_nets_set;
    auto nets1 = getAffectedNets(cell1_id);
    auto nets2 = getAffectedNets(cell2_id);
    affected_nets_set.insert(nets1.begin(), nets1.end());
    affected_nets_set.insert(nets2.begin(), nets2.end());
    
    std::vector<int> affected_nets(affected_nets_set.begin(), affected_nets_set.end());
    int64_t delta = calcDeltaHPWL(affected_nets);
    
    if (acceptMove(delta, temp_)) {
        // Update grid with new positions
        grid_->placeMultiHeightCell(cell1_id, x2, y2, width, height);
        grid_->placeMultiHeightCell(cell2_id, x1, y1, width, height);
        
        // Update HPWL cache and total
        updateHPWLCache(affected_nets);
        state_.total_hpwl += delta;
        
        accepted_moves_++;
        accepted_swaps_++;
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
        grid_->placeMultiHeightCell(cell1_id, x1, y1, width, height);
        grid_->placeMultiHeightCell(cell2_id, x2, y2, width, height);
        
        rejected_moves_++;
        return false;
    }
}

bool SAWorker::canPlaceMultiHeightCell(int cell_id, GridX x, GridY y)
{
    const dpl::Node* node = network_->getNode(cell_id);
    GridY height = grid_info_->getCellHeightInRows(node);
    GridX width = grid_info_->gridPaddedWidth(node);
    
    // Check row boundaries
    if (!grid_info_->rowsAvailable(y, height)) {
        return false;
    }
    
    // Check site alignment (x must be valid for ALL rows)
    if (x.v < 0 || x.v + width.v > grid_info_->getRowSiteCount()) {
        return false;
    }
    
    // For multi-height, check site compatibility in bottom row only (v0 simplification)
    if (height.v > 1) {
        odb::dbSite* cell_site = node->getSite();
        const auto& available_sites = grid_info_->getSitesAt(x, y);
        if (available_sites.find(cell_site) == available_sites.end()) {
            return false;
        }
    }
    
    // Check no overlaps across all spanned rows
    return grid_->canPlaceMultiHeight(cell_id, x, y, width, height);
}

bool SAWorker::checkGridStateConsistency() {
    bool consistent = true;
    
    // Create a fresh grid from state
    WorkerGrid check_grid(grid_info_);
    
    for (int i = 0; i < network_->getNumNodes(); i++) {
        const dpl::Node* node = network_->getNode(i);
        if (node->getType() == dpl::Node::CELL) {
            GridX gx = grid_info_->gridX(state_.cells[i].x);
            GridY gy = grid_info_->gridSnapDownY(state_.cells[i].y);
            GridX gw = grid_info_->gridPaddedWidth(node);
            GridY gh = grid_info_->gridHeight(node);
            
            // Check if cell is where grid thinks it is
            for (GridY y = gy; y.v < gy.v + gh.v; ++y) {
                for (GridX x = gx; x.v < gx.v + gw.v; ++x) {
                    int grid_cell = grid_->getCellAt(x, y);
                    if (grid_cell != i) {
                        /*logger->warn(utl::SA2D, 467, "Grid inconsistency! Cell {} should be at ({},{}), but grid has cell {}",
                                    i, x.v, y.v, grid_cell);*/
                        consistent = false;
                    }
                }
            }
            
            // Place in check grid
            if (grid_info_->isMultiHeightCell(node)) {
                check_grid.placeMultiHeightCell(i, gx, gy, gw, gh);
            } else {
                check_grid.placeCell(i, gx, gy, gw, gh);
            }
        }
    }
    
    return consistent;
}

bool SAWorker::trySlide(int cell_id)
{
    attempted_slides_++;
    
    dpl::Node* node = const_cast<dpl::Node*>(network_->getNode(cell_id));
    
    // Skip non-movable cells
    if (node->getType() != dpl::Node::CELL || node->isFixed()) {
        return false;
    }
    
    // Get current position and dimensions
    GridX curr_x = grid_info_->gridX(state_.cells[cell_id].x);
    GridY curr_y = grid_info_->gridSnapDownY(state_.cells[cell_id].y);
    GridX width = grid_info_->gridPaddedWidth(node);
    GridY height = grid_info_->gridHeight(node);
    
    // Save current state
    state_.cells[cell_id].prev_x = state_.cells[cell_id].x;
    state_.cells[cell_id].prev_y = state_.cells[cell_id].y;
    state_.cells[cell_id].prev_orient = state_.cells[cell_id].orient;
    
    // Determine slide range based on displacement limits
    GridX min_x = curr_x.v > max_displacement_x_ ? GridX{curr_x.v - max_displacement_x_} : GridX{0};
    GridX max_x{curr_x.v + max_displacement_x_};
    
    // Clip to grid bounds
    max_x = std::min(max_x, GridX{grid_info_->getRowSiteCount() - width.v});
    
    // If cell is in a group, further restrict range
    if (node->inGroup()) {
        const auto group_rect = grid_info_->gridWithin(node->getGroup()->getBBox());
        min_x = std::max(min_x, group_rect.xlo);
        max_x = std::min(max_x, GridX{group_rect.xhi.v - width.v});
    }
    
    // Evaluate positions along the slide path
    struct SlideCandidate {
        GridX x;
        int64_t hpwl;
        bool is_legal;
    };
    
    std::vector<SlideCandidate> candidates;
    std::vector<int> affected_nets = getAffectedNets(cell_id);
    
    // Calculate current HPWL for comparison
    int64_t current_hpwl = 0;
    for (int net_id : affected_nets) {
        current_hpwl += state_.net_hpwl_cache[net_id];
    }
    
    // Sample positions along the slide path (every few sites to reduce computation)
    int step = std::max(1, (max_x.v - min_x.v) / 20);  // Evaluate up to 20 positions
    
    // Temporarily remove cell from grid for legality checking
    grid_->removeCell(cell_id);
    
    for (GridX test_x = min_x; test_x.v <= max_x.v; test_x.v += step) {
        // Always include the exact end position
        if (test_x.v > max_x.v && test_x.v - step < max_x.v) {
            test_x = max_x;
        }
        
        // Skip current position
        if (test_x.v == curr_x.v) {
            continue;
        }
        
        // Check legality at this position
        bool is_legal = canPlaceCell(cell_id, test_x, curr_y);
        
        if (is_legal) {
            // Calculate HPWL at this position
            odb::dbOrientType new_orient = getCellOrientation(cell_id, test_x, curr_y);
            
            // Temporarily update position
            state_.cells[cell_id].x = grid_info_->gridToDbuX(test_x);
            state_.cells[cell_id].y = grid_info_->gridYToDbu(curr_y);
            state_.cells[cell_id].orient = new_orient;
            
            // Calculate new HPWL
            int64_t new_hpwl = 0;
            for (int net_id : affected_nets) {
                new_hpwl += calcNetHPWL(net_id);
            }
            
            candidates.push_back({test_x, new_hpwl, true});
        }
    }
    
    // Restore cell to grid at original position
    if (grid_info_->isMultiHeightCell(node)) {
        grid_->placeMultiHeightCell(cell_id, curr_x, curr_y, width, height);
    } else {
        grid_->placeCell(cell_id, curr_x, curr_y, width, height);
    }
    
    // Restore original state
    state_.cells[cell_id].x = state_.cells[cell_id].prev_x;
    state_.cells[cell_id].y = state_.cells[cell_id].prev_y;
    state_.cells[cell_id].orient = state_.cells[cell_id].prev_orient;
    
    // Find best candidate
    if (candidates.empty()) {
        return false;  // No legal positions found
    }
    
    // Sort by HPWL (ascending)
    std::sort(candidates.begin(), candidates.end(), 
              [](const SlideCandidate& a, const SlideCandidate& b) {
                  return a.hpwl < b.hpwl;
              });
    
    // Use the best position
    const SlideCandidate& best = candidates[0];
    int64_t delta = best.hpwl - current_hpwl;
    
    // Apply SA acceptance criterion even though we found the best position
    if (acceptMove(delta, temp_)) {
        // Remove from old position and place at new position
        grid_->removeCell(cell_id);
        
        // Update state to best position
        odb::dbOrientType new_orient = getCellOrientation(cell_id, best.x, curr_y);
        state_.cells[cell_id].x = grid_info_->gridToDbuX(best.x);
        state_.cells[cell_id].y = grid_info_->gridYToDbu(curr_y);
        state_.cells[cell_id].orient = new_orient;
        
        // Place at new position
        if (grid_info_->isMultiHeightCell(node)) {
            grid_->placeMultiHeightCell(cell_id, best.x, curr_y, width, height);
        } else {
            grid_->placeCell(cell_id, best.x, curr_y, width, height);
        }
        
        // Update HPWL cache and total
        updateHPWLCache(affected_nets);
        state_.total_hpwl += delta;
        
        accepted_moves_++;
        accepted_slides_++;
        return true;
    } else {
        rejected_moves_++;
        return false;
    }
}

void SAWorker::reportRuntimeStatistics() const
{
    utl::Logger* logger = sa2d_->getLogger();
    
    // Calculate total runtime
    int64_t total_time = single_move_time_ + swap_time_ + flip_time_ + 
                        slide_time_ + chain_move_time_ + kick_time_;
    
    if (total_time == 0) {
        logger->info(utl::SA2D, 470, "Worker {} runtime statistics not available (no timing data)", worker_id_);
        return;
    }
    
    logger->info(utl::SA2D, 471, "");  // Empty line for separation
    logger->info(utl::SA2D, 472, "Worker {} runtime statistics (accepted moves only):", worker_id_);
    logger->info(utl::SA2D, 473, "  Total accepted move time: {:.3f} seconds", total_time / 1e6);
    logger->info(utl::SA2D, 474, "");
    
    // Format: Operator | Count | Total Time | Avg Time | % of Total
    logger->info(utl::SA2D, 475, "  {:15} | {:>10} | {:>12} | {:>12} | {:>10}",
                "Operator", "Accepted", "Total (s)", "Avg (us)", "% Total");
    logger->info(utl::SA2D, 476, "  {:->15}-+-{:->10}-+-{:->12}-+-{:->12}-+-{:->10}",
                "", "", "", "", "");
    
    // Single moves
    if (accepted_single_moves_ > 0) {
        double avg_time = (double)single_move_time_ / accepted_single_moves_;
        double percent = 100.0 * single_move_time_ / total_time;
        logger->info(utl::SA2D, 477, "  {:15} | {:>10} | {:>12.3f} | {:>12.1f} | {:>9.1f}%",
                    "Single moves", accepted_single_moves_, 
                    single_move_time_ / 1e6, avg_time, percent);
    }
    
    // Swaps
    if (accepted_swaps_ > 0) {
        double avg_time = (double)swap_time_ / accepted_swaps_;
        double percent = 100.0 * swap_time_ / total_time;
        logger->info(utl::SA2D, 478, "  {:15} | {:>10} | {:>12.3f} | {:>12.1f} | {:>9.1f}%",
                    "Swaps", accepted_swaps_, 
                    swap_time_ / 1e6, avg_time, percent);
    }
    
    // Flips
    if (accepted_flips_ > 0) {
        double avg_time = (double)flip_time_ / accepted_flips_;
        double percent = 100.0 * flip_time_ / total_time;
        logger->info(utl::SA2D, 479, "  {:15} | {:>10} | {:>12.3f} | {:>12.1f} | {:>9.1f}%",
                    "Flips", accepted_flips_, 
                    flip_time_ / 1e6, avg_time, percent);
    }
    
    // Slides
    if (accepted_slides_ > 0) {
        double avg_time = (double)slide_time_ / accepted_slides_;
        double percent = 100.0 * slide_time_ / total_time;
        logger->info(utl::SA2D, 480, "  {:15} | {:>10} | {:>12.3f} | {:>12.1f} | {:>9.1f}%",
                    "Slides", accepted_slides_, 
                    slide_time_ / 1e6, avg_time, percent);
    }
    
    // Chain moves
    if (accepted_chain_moves_ > 0) {
        double avg_time = (double)chain_move_time_ / accepted_chain_moves_;
        double percent = 100.0 * chain_move_time_ / total_time;
        logger->info(utl::SA2D, 481, "  {:15} | {:>10} | {:>12.3f} | {:>12.1f} | {:>9.1f}%",
                    "Chain moves", accepted_chain_moves_, 
                    chain_move_time_ / 1e6, avg_time, percent);
    }
    
    // Kicks
    if (kick_accepted_ > 0) {
        double avg_time = (double)kick_time_ / kick_accepted_;
        double percent = 100.0 * kick_time_ / total_time;
        logger->info(utl::SA2D, 482, "  {:15} | {:>10} | {:>12.3f} | {:>12.1f} | {:>9.1f}%",
                    "Kicks", kick_accepted_, 
                    kick_time_ / 1e6, avg_time, percent);
    }
    
    logger->info(utl::SA2D, 483, "");  // Empty line for separation
}

// Implementation of specialized kick moves for low row count designs

bool SAWorker::tryLowRowKickMove()
{
    // For low row designs, use specialized strategies
    std::uniform_int_distribution<int> strategy_dist(0, 3);
    int strategy = strategy_dist(rng_);
    
    // Debug logging every 1000 attempts
    static int debug_counter = 0;
    debug_counter++;
    if (debug_counter % 1000 == 0) {
        utl::Logger* logger = sa2d_->getLogger();
        logger->info(utl::SA2D, 487, "Low-row kick attempt {}: temp={}, kick_temp_mult={}, strategy={}",
                    debug_counter, temp_, kick_temp_multiplier_, 
                    strategy == 0 ? "chain" : strategy == 1 ? "compress" : 
                    strategy == 2 ? "transfer" : "slide");
    }
    
    switch (strategy) {
        case 0: {
            // Horizontal chain swap within a row
            std::uniform_int_distribution<int> row_dist(0, grid_info_->getRowCount() - 1);
            std::uniform_int_distribution<int> chain_dist(3, 8);  // Chain length
            return tryHorizontalChainSwap(row_dist(rng_), chain_dist(rng_));
        }
        case 1: {
            // Row compression to create space
            std::uniform_int_distribution<int> row_dist(0, grid_info_->getRowCount() - 1);
            std::uniform_int_distribution<int> start_dist(0, grid_info_->getRowSiteCount() - 50);
            std::uniform_int_distribution<int> length_dist(20, 50);
            return tryRowCompression(row_dist(rng_), start_dist(rng_), length_dist(rng_));
        }
        case 2: {
            // Inter-row transfer
            return tryInterRowTransfer();
        }
        case 3: {
            // Sliding window move
            std::uniform_int_distribution<int> row_dist(0, grid_info_->getRowCount() - 1);
            std::uniform_int_distribution<int> window_dist(5, 15);
            return trySlidingWindowMove(row_dist(rng_), window_dist(rng_));
        }
    }
    
    return false;
}

bool SAWorker::tryHorizontalChainSwap(int row, int chain_length)
{
    if (row >= grid_info_->getRowCount()) return false;
    
    // Find cells in this row
    std::vector<int> cells_in_row;
    for (int i = 0; i < network_->getNumNodes(); ++i) {
        dpl::Node* node = const_cast<dpl::Node*>(network_->getNode(i));
        if (node->getType() == dpl::Node::CELL && !node->isFixed()) {
            GridY cell_y = grid_info_->gridSnapDownY(state_.cells[i].y);
            if (cell_y == row) {
                cells_in_row.push_back(i);
            }
        }
    }
    
    if (cells_in_row.size() < 2) return false;
    
    // Sort by x-coordinate
    std::sort(cells_in_row.begin(), cells_in_row.end(), 
              [this](int a, int b) { return state_.cells[a].x < state_.cells[b].x; });
    
    // Select random starting position
    if (cells_in_row.size() < chain_length) {
        chain_length = cells_in_row.size();
    }
    
    std::uniform_int_distribution<int> start_dist(0, cells_in_row.size() - chain_length);
    int start_idx = start_dist(rng_);
    
    // Build swap chain: cell[i] -> position of cell[i+1], last -> position of first
    std::vector<std::pair<int, GridPt>> moves;
    
    for (int i = 0; i < chain_length; ++i) {
        int cell_id = cells_in_row[start_idx + i];
        int next_idx = (i + 1) % chain_length;
        int target_cell = cells_in_row[start_idx + next_idx];
        
        GridX target_x = grid_info_->gridX(state_.cells[target_cell].x);
        GridY target_y = grid_info_->gridSnapDownY(state_.cells[target_cell].y);
        
        moves.push_back({cell_id, GridPt{target_x, target_y}});
    }
    
    // Validate all moves are legal
    WorkerGrid temp_grid(grid_info_);
    temp_grid.copyFrom(*grid_);
    
    // Remove all cells that will move
    for (const auto& [cell_id, _] : moves) {
        temp_grid.removeCell(cell_id);
    }
    
    // Check if all can be placed at new positions
    for (const auto& [cell_id, new_pos] : moves) {
        const dpl::Node* node = network_->getNode(cell_id);
        GridX width = grid_info_->gridPaddedWidth(node);
        GridY height = grid_info_->gridHeight(node);
        
        // Check bounds and overlaps
        if (new_pos.x < 0 || new_pos.x + width > grid_info_->getRowSiteCount() ||
            new_pos.y < 0 || new_pos.y + height > grid_info_->getRowCount()) {
            return false;
        }
        
        for (GridY yi = new_pos.y; yi < new_pos.y + height; ++yi) {
            for (GridX xi = new_pos.x; xi < new_pos.x + width; ++xi) {
                if (temp_grid.isOccupied(xi, yi)) {
                    return false;
                }
            }
        }
        
        // Add to temp grid
        temp_grid.placeCell(cell_id, new_pos.x, new_pos.y, width, height);
    }
    
    // Calculate total HPWL change
    double total_delta = 0;
    for (const auto& [cell_id, new_pos] : moves) {
        double old_contrib = calcCellHPWLContribution(cell_id);
        
        // Temporarily update position
        DbuX old_x = state_.cells[cell_id].x;
        state_.cells[cell_id].x = gridToDbu(new_pos.x, DbuX{grid_info_->getSiteWidth()});
        
        double new_contrib = calcCellHPWLContribution(cell_id);
        total_delta += new_contrib - old_contrib;
        
        // Restore
        state_.cells[cell_id].x = old_x;
    }
    
    // Use kick temperature for acceptance
    double kick_temp = temp_ * kick_temp_multiplier_;
    
    // For low-row designs, use more aggressive temperature
    if (isLowRowDesign()) {
        kick_temp *= 2.0;  // Double the kick temperature for dense designs
    }
    
    // Debug: Log large deltas
    if (kick_attempts_ % 500 == 0 && total_delta > 0) {
        utl::Logger* logger = sa2d_->getLogger();
        logger->info(utl::SA2D, 488, "Chain swap delta: {} at temp={}, kick_temp={}, chain_len={}, acceptance_prob={}",
                    total_delta, temp_, kick_temp, chain_length,
                    total_delta > 0 ? std::exp(-total_delta / kick_temp) : 1.0);
    }
    
    kick_attempts_++;
    double kick_temp_adjusted = temp_ * kick_temp_multiplier_;
    if (isLowRowDesign()) {
        kick_temp_adjusted *= 2.0;  // More aggressive for dense designs
    }
    if (acceptMove(total_delta, kick_temp_adjusted)) {
        // Apply all moves
        for (const auto& [cell_id, new_pos] : moves) {
            const dpl::Node* node = network_->getNode(cell_id);
            grid_->removeCell(cell_id);
            grid_->placeCell(cell_id, new_pos.x, new_pos.y,
                           grid_info_->gridPaddedWidth(node),
                           grid_info_->gridHeight(node));
            
            state_.cells[cell_id].x = gridToDbu(new_pos.x, DbuX{grid_info_->getSiteWidth()});
            state_.cells[cell_id].y = grid_info_->gridYToDbu(new_pos.y);
            state_.cells[cell_id].orient = getCellOrientation(cell_id, new_pos.x, new_pos.y);
        }
        
        // Update HPWL
        state_.total_hpwl += total_delta;
        kick_accepted_++;
        total_swaps_applied_ += chain_length;  // Count the swaps
        return true;
    }
    
    kick_rejected_++;
    return false;
}

bool SAWorker::tryRowCompression(int row, int start_x, int length)
{
    if (row >= grid_info_->getRowCount()) return false;
    
    // Find cells in the specified x-range of this row
    std::vector<int> cells_to_compress;
    for (int i = 0; i < network_->getNumNodes(); ++i) {
        dpl::Node* node = const_cast<dpl::Node*>(network_->getNode(i));
        if (node->getType() == dpl::Node::CELL && !node->isFixed()) {
            GridY cell_y = grid_info_->gridSnapDownY(state_.cells[i].y);
            GridX cell_x = grid_info_->gridX(state_.cells[i].x);
            
            if (cell_y == row && cell_x >= start_x && cell_x < start_x + length) {
                cells_to_compress.push_back(i);
            }
        }
    }
    
    if (cells_to_compress.size() < 2) return false;
    
    // Sort by x-coordinate
    std::sort(cells_to_compress.begin(), cells_to_compress.end(),
              [this](int a, int b) { return state_.cells[a].x < state_.cells[b].x; });
    
    // Calculate total width and find gaps
    int total_width = 0;
    std::vector<int> gaps;
    
    for (size_t i = 0; i < cells_to_compress.size(); ++i) {
        const dpl::Node* node = network_->getNode(cells_to_compress[i]);
        total_width += grid_info_->gridPaddedWidth(node).v;
        
        if (i > 0) {
            GridX curr_x = grid_info_->gridX(state_.cells[cells_to_compress[i]].x);
            const dpl::Node* prev_node = network_->getNode(cells_to_compress[i-1]);
            GridX prev_x = grid_info_->gridX(state_.cells[cells_to_compress[i-1]].x);
            GridX prev_width = grid_info_->gridPaddedWidth(prev_node);
            
            int gap = curr_x.v - (prev_x.v + prev_width.v);
            if (gap > 0) gaps.push_back(gap);
        }
    }
    
    // If no significant gaps, can't compress
    int total_gaps = std::accumulate(gaps.begin(), gaps.end(), 0);
    if (total_gaps < 5) return false;  // Need at least 5 sites of gaps
    
    // Try to compress by removing gaps
    std::vector<std::pair<int, GridPt>> new_positions;
    GridX current_x{start_x};
    
    for (int cell_id : cells_to_compress) {
        const dpl::Node* node = network_->getNode(cell_id);
        GridX width = grid_info_->gridPaddedWidth(node);
        
        new_positions.push_back({cell_id, GridPt{current_x, GridY{row}}});
        current_x = GridX{current_x.v + width.v};
    }
    
    // Validate compression
    WorkerGrid temp_grid(grid_info_);
    temp_grid.copyFrom(*grid_);
    
    // Remove cells to compress
    for (int cell_id : cells_to_compress) {
        temp_grid.removeCell(cell_id);
    }
    
    // Check new positions
    for (const auto& [cell_id, new_pos] : new_positions) {
        const dpl::Node* node = network_->getNode(cell_id);
        GridX width = grid_info_->gridPaddedWidth(node);
        GridY height = grid_info_->gridHeight(node);
        
        // Check bounds and overlaps
        if (new_pos.x + width > grid_info_->getRowSiteCount()) {
            return false;
        }
        
        for (GridY yi = new_pos.y; yi < new_pos.y + height; ++yi) {
            for (GridX xi = new_pos.x; xi < new_pos.x + width; ++xi) {
                if (temp_grid.isOccupied(xi, yi)) {
                    return false;
                }
            }
        }
        
        temp_grid.placeCell(cell_id, new_pos.x, new_pos.y, width, height);
    }
    
    // Calculate HPWL change and apply if beneficial
    double total_delta = 0;
    for (const auto& [cell_id, new_pos] : new_positions) {
        double old_contrib = calcCellHPWLContribution(cell_id);
        
        DbuX old_x = state_.cells[cell_id].x;
        state_.cells[cell_id].x = gridToDbu(new_pos.x, DbuX{grid_info_->getSiteWidth()});
        
        double new_contrib = calcCellHPWLContribution(cell_id);
        total_delta += new_contrib - old_contrib;
        
        state_.cells[cell_id].x = old_x;
    }
    
    kick_attempts_++;
    double kick_temp_adjusted = temp_ * kick_temp_multiplier_;
    if (isLowRowDesign()) {
        kick_temp_adjusted *= 2.0;  // More aggressive for dense designs
    }
    if (acceptMove(total_delta, kick_temp_adjusted)) {
        // Apply compression
        for (const auto& [cell_id, new_pos] : new_positions) {
            const dpl::Node* node = network_->getNode(cell_id);
            grid_->removeCell(cell_id);
            grid_->placeCell(cell_id, new_pos.x, new_pos.y,
                           grid_info_->gridPaddedWidth(node),
                           grid_info_->gridHeight(node));
            
            state_.cells[cell_id].x = gridToDbu(new_pos.x, DbuX{grid_info_->getSiteWidth()});
        }
        
        state_.total_hpwl += total_delta;
        kick_accepted_++;
        total_swaps_applied_ += cells_to_compress.size();  // Count moves as swaps
        return true;
    }
    
    kick_rejected_++;
    return false;
}

bool SAWorker::tryInterRowTransfer()
{
    // Find a cell to transfer between rows
    std::vector<int> movable_cells;
    for (int i = 0; i < network_->getNumNodes(); ++i) {
        dpl::Node* node = const_cast<dpl::Node*>(network_->getNode(i));
        if (node->getType() == dpl::Node::CELL && !node->isFixed() &&
            grid_info_->gridHeight(node).v == 1) {  // Single height cells only
            movable_cells.push_back(i);
        }
    }
    
    if (movable_cells.empty()) return false;
    
    // Select random cell
    std::uniform_int_distribution<int> cell_dist(0, movable_cells.size() - 1);
    int cell_id = movable_cells[cell_dist(rng_)];
    
    GridY current_row = grid_info_->gridSnapDownY(state_.cells[cell_id].y);
    GridX current_x = grid_info_->gridX(state_.cells[cell_id].x);
    
    // Try other rows
    std::vector<int> other_rows;
    for (int r = 0; r < grid_info_->getRowCount(); ++r) {
        if (r != current_row.v) {
            other_rows.push_back(r);
        }
    }
    
    if (other_rows.empty()) return false;
    
    std::shuffle(other_rows.begin(), other_rows.end(), rng_);
    
    // Try to find space in other rows
    const dpl::Node* node = network_->getNode(cell_id);
    GridX width = grid_info_->gridPaddedWidth(node);
    
    for (int target_row : other_rows) {
        // Search for free space near current x position
        std::vector<GridX> x_positions;
        
        // Try positions near current x
        int search_range = 50;
        for (int dx = -search_range; dx <= search_range; dx += 5) {
            GridX try_x{current_x.v + dx};
            if (try_x.v >= 0 && try_x.v + width.v <= grid_info_->getRowSiteCount()) {
                x_positions.push_back(try_x);
            }
        }
        
        std::shuffle(x_positions.begin(), x_positions.end(), rng_);
        
        for (GridX try_x : x_positions) {
            // Check if position is free
            bool can_place = true;
            for (GridX xi = try_x; xi < GridX{try_x.v + width.v}; ++xi) {
                if (grid_->isOccupied(xi, GridY{target_row})) {
                    can_place = false;
                    break;
                }
            }
            
            if (can_place) {
                // Check site compatibility
                const auto& available_sites = grid_info_->getSitesAt(try_x, GridY{target_row});
                if (available_sites.find(node->getSite()) == available_sites.end()) {
                    continue;
                }
                
                // Calculate HPWL change
                double old_contrib = calcCellHPWLContribution(cell_id);
                
                DbuX old_x = state_.cells[cell_id].x;
                DbuY old_y = state_.cells[cell_id].y;
                state_.cells[cell_id].x = gridToDbu(try_x, DbuX{grid_info_->getSiteWidth()});
                state_.cells[cell_id].y = grid_info_->gridYToDbu(GridY{target_row});
                
                double new_contrib = calcCellHPWLContribution(cell_id);
                double delta = new_contrib - old_contrib;
                
                state_.cells[cell_id].x = old_x;
                state_.cells[cell_id].y = old_y;
                
                kick_attempts_++;
                double kick_temp_adjusted = temp_ * kick_temp_multiplier_;
                if (isLowRowDesign()) {
                    kick_temp_adjusted *= 2.0;  // More aggressive for dense designs
                }
                if (acceptMove(delta, kick_temp_adjusted)) {
                    // Apply move
                    grid_->removeCell(cell_id);
                    grid_->placeCell(cell_id, try_x, GridY{target_row}, width, GridY{1});
                    
                    state_.cells[cell_id].x = gridToDbu(try_x, DbuX{grid_info_->getSiteWidth()});
                    state_.cells[cell_id].y = grid_info_->gridYToDbu(GridY{target_row});
                    state_.cells[cell_id].orient = getCellOrientation(cell_id, try_x, GridY{target_row});
                    
                    state_.total_hpwl += delta;
                    kick_accepted_++;
                    total_swaps_applied_++;  // Count single move as swap
                    return true;
                }
                
                kick_rejected_++;
                return false;
            }
        }
    }
    
    return false;
}

bool SAWorker::trySlidingWindowMove(int row, int window_size)
{
    if (row >= grid_info_->getRowCount()) return false;
    
    // Find cells in this row
    std::vector<std::pair<int, GridX>> cells_in_row;
    for (int i = 0; i < network_->getNumNodes(); ++i) {
        dpl::Node* node = const_cast<dpl::Node*>(network_->getNode(i));
        if (node->getType() == dpl::Node::CELL && !node->isFixed()) {
            GridY cell_y = grid_info_->gridSnapDownY(state_.cells[i].y);
            if (cell_y == row) {
                GridX cell_x = grid_info_->gridX(state_.cells[i].x);
                cells_in_row.push_back({i, cell_x});
            }
        }
    }
    
    if (cells_in_row.size() < 3) return false;
    
    // Sort by x-coordinate
    std::sort(cells_in_row.begin(), cells_in_row.end(),
              [](const auto& a, const auto& b) { return a.second < b.second; });
    
    // Select random window start
    std::uniform_int_distribution<int> start_dist(0, 
        std::max(0, static_cast<int>(cells_in_row.size()) - window_size));
    int window_start = start_dist(rng_);
    
    // Get cells in window
    std::vector<int> window_cells;
    for (int i = window_start; i < window_start + window_size && i < cells_in_row.size(); ++i) {
        window_cells.push_back(cells_in_row[i].first);
    }
    
    if (window_cells.size() < 2) return false;
    
    // Calculate window bounds
    GridX min_x = cells_in_row[window_start].second;
    GridX max_x = min_x;
    int total_width = 0;
    
    for (int cell_id : window_cells) {
        const dpl::Node* node = network_->getNode(cell_id);
        GridX cell_x = grid_info_->gridX(state_.cells[cell_id].x);
        GridX width = grid_info_->gridPaddedWidth(node);
        
        max_x = std::max(max_x, GridX{cell_x.v + width.v});
        total_width += width.v;
    }
    
    int window_span = max_x.v - min_x.v;
    int free_space = window_span - total_width;
    
    if (free_space < 5) return false;  // Need some free space to rearrange
    
    // Generate random permutation of cells
    std::vector<int> permuted = window_cells;
    std::shuffle(permuted.begin(), permuted.end(), rng_);
    
    // Try to place permuted cells
    std::vector<std::pair<int, GridPt>> new_positions;
    GridX current_x = min_x;
    
    for (int cell_id : permuted) {
        const dpl::Node* node = network_->getNode(cell_id);
        GridX width = grid_info_->gridPaddedWidth(node);
        
        // Add some random spacing
        if (free_space > 0 && current_x.v > min_x.v) {
            std::uniform_int_distribution<int> space_dist(0, std::min(3, free_space));
            int spacing = space_dist(rng_);
            current_x = GridX{current_x.v + spacing};
            free_space -= spacing;
        }
        
        new_positions.push_back({cell_id, GridPt{current_x, GridY{row}}});
        current_x = GridX{current_x.v + width.v};
    }
    
    // Validate new positions
    WorkerGrid temp_grid(grid_info_);
    temp_grid.copyFrom(*grid_);
    
    // Remove window cells
    for (int cell_id : window_cells) {
        temp_grid.removeCell(cell_id);
    }
    
    // Check new positions
    for (const auto& [cell_id, new_pos] : new_positions) {
        const dpl::Node* node = network_->getNode(cell_id);
        GridX width = grid_info_->gridPaddedWidth(node);
        GridY height = grid_info_->gridHeight(node);
        
        if (new_pos.x + width > grid_info_->getRowSiteCount()) {
            return false;
        }
        
        for (GridY yi = new_pos.y; yi < new_pos.y + height; ++yi) {
            for (GridX xi = new_pos.x; xi < new_pos.x + width; ++xi) {
                if (temp_grid.isOccupied(xi, yi)) {
                    return false;
                }
            }
        }
        
        temp_grid.placeCell(cell_id, new_pos.x, new_pos.y, width, height);
    }
    
    // Calculate HPWL change
    double total_delta = 0;
    for (const auto& [cell_id, new_pos] : new_positions) {
        double old_contrib = calcCellHPWLContribution(cell_id);
        
        DbuX old_x = state_.cells[cell_id].x;
        state_.cells[cell_id].x = gridToDbu(new_pos.x, DbuX{grid_info_->getSiteWidth()});
        
        double new_contrib = calcCellHPWLContribution(cell_id);
        total_delta += new_contrib - old_contrib;
        
        state_.cells[cell_id].x = old_x;
    }
    
    kick_attempts_++;
    double kick_temp_adjusted = temp_ * kick_temp_multiplier_;
    if (isLowRowDesign()) {
        kick_temp_adjusted *= 2.0;  // More aggressive for dense designs
    }
    if (acceptMove(total_delta, kick_temp_adjusted)) {
        // Apply moves
        for (const auto& [cell_id, new_pos] : new_positions) {
            const dpl::Node* node = network_->getNode(cell_id);
            grid_->removeCell(cell_id);
            grid_->placeCell(cell_id, new_pos.x, new_pos.y,
                           grid_info_->gridPaddedWidth(node),
                           grid_info_->gridHeight(node));
            
            state_.cells[cell_id].x = gridToDbu(new_pos.x, DbuX{grid_info_->getSiteWidth()});
            state_.cells[cell_id].y = grid_info_->gridYToDbu(new_pos.y);
            state_.cells[cell_id].orient = getCellOrientation(cell_id, new_pos.x, new_pos.y);
        }
        
        state_.total_hpwl += total_delta;
        kick_accepted_++;
        total_swaps_applied_ += window_cells.size();  // Count rearrangements as swaps
        return true;
    }
    
    kick_rejected_++;
    return false;
}

// Speculative kick move framework implementation

bool SAWorker::trySpeculativeKick()
{
    // Only use speculative kicks for low-row designs
    if (!isLowRowDesign()) {
        return tryLowRowKickMove();  // Fallback to standard approach
    }
    
    utl::Logger* logger = sa2d_->getLogger();
    
    // Save current state as baseline
    saveCurrentState();
    
    // Try different kick strategies
    for (int strategy = 0; strategy < max_kick_strategies_; ++strategy) {
        
        // Apply kick move (forced, regardless of immediate cost)
        if (applyKickMoveForced(strategy)) {
            
            // Log speculative attempt
            /*if (kick_attempts_ % 100 == 0) {
                logger->info(utl::SA2D, 489, "Speculative kick #{}: strategy={}, baseline_hpwl={}, post_kick_hpwl={}",
                            kick_attempts_, strategy, saved_baseline_hpwl_, state_.total_hpwl);
            }*/
            
            // Run exploration phase
            int64_t best_found = runExplorationPhase(speculation_exploration_iterations_);
            
            // Check if we found improvement
            if (best_found < saved_baseline_hpwl_) {
                kick_accepted_++;
                /*logger->info(utl::SA2D, 490, "Speculative kick SUCCESS: strategy={}, baseline={}, improved_to={}, delta={}",
                            strategy, saved_baseline_hpwl_, best_found, best_found - saved_baseline_hpwl_);*/
                return true;  // Keep the improved state
            }
            
            // No improvement - revert to saved state for next attempt
            restoreCurrentState();
        }
    }
    
    // No successful kick found - ensure we're back at baseline
    restoreCurrentState();
    kick_rejected_++;
    return false;
}

bool SAWorker::applyKickMoveForced(int strategy_index)
{
    // Apply kick moves without SA acceptance check
    switch (strategy_index) {
        case 0: {
            // Horizontal chain swap
            std::uniform_int_distribution<int> row_dist(0, grid_info_->getRowCount() - 1);
            std::uniform_int_distribution<int> chain_dist(2, 4);  // Smaller chains for dense designs
            return tryHorizontalChainSwapForced(row_dist(rng_), chain_dist(rng_));
        }
        case 1: {
            // Row compression
            std::uniform_int_distribution<int> row_dist(0, grid_info_->getRowCount() - 1);
            std::uniform_int_distribution<int> start_dist(0, grid_info_->getRowSiteCount() - 30);
            std::uniform_int_distribution<int> length_dist(15, 30);  // Smaller segments
            return tryRowCompressionForced(row_dist(rng_), start_dist(rng_), length_dist(rng_));
        }
        case 2: {
            // Inter-row transfer
            return tryInterRowTransferForced();
        }
        case 3: {
            // Sliding window move
            std::uniform_int_distribution<int> row_dist(0, grid_info_->getRowCount() - 1);
            std::uniform_int_distribution<int> window_dist(3, 8);  // Smaller windows
            return trySlidingWindowMoveForced(row_dist(rng_), window_dist(rng_));
        }
    }
    return false;
}

int64_t SAWorker::runExplorationPhase(int iterations)
{
    int64_t best_hpwl = state_.total_hpwl;
    
    // Run mini-SA exploration
    for (int i = 0; i < iterations; ++i) {
        
        // Standard SA moves during exploration
        std::uniform_real_distribution<float> move_dist(0.0f, 1.0f);
        float move_type = move_dist(rng_);
        
        if (move_type < 0.7f) {
            // Single move
            std::uniform_int_distribution<int> cell_dist(0, network_->getNumNodes() - 1);
            tryMove(cell_dist(rng_));
        } else if (move_type < 0.85f) {
            // Swap
            std::uniform_int_distribution<int> cell1_dist(0, network_->getNumNodes() - 1);
            std::uniform_int_distribution<int> cell2_dist(0, network_->getNumNodes() - 1);
            trySwap(cell1_dist(rng_), cell2_dist(rng_));
        } else {
            // Flip
            std::uniform_int_distribution<int> cell_dist(0, network_->getNumNodes() - 1);
            tryFlip(cell_dist(rng_));
        }
        
        // Track best found during exploration
        if (state_.total_hpwl < best_hpwl) {
            best_hpwl = state_.total_hpwl;
        }
    }
    
    return best_hpwl;
}

void SAWorker::saveCurrentState()
{
    saved_state_ = state_;
    if (!saved_grid_) {
        saved_grid_ = std::make_unique<WorkerGrid>(grid_info_);
    }
    saved_grid_->copyFrom(*grid_);
    saved_baseline_hpwl_ = state_.total_hpwl;
}

void SAWorker::restoreCurrentState()
{
    state_ = saved_state_;
    grid_->copyFrom(*saved_grid_);
}

// Forced kick move implementations (apply without SA acceptance)

bool SAWorker::tryHorizontalChainSwapForced(int row, int chain_length)
{
    if (row >= grid_info_->getRowCount()) return false;
    
    // Find cells in this row
    std::vector<int> cells_in_row;
    for (int i = 0; i < network_->getNumNodes(); ++i) {
        dpl::Node* node = const_cast<dpl::Node*>(network_->getNode(i));
        if (node->getType() == dpl::Node::CELL && !node->isFixed()) {
            GridY cell_y = grid_info_->gridSnapDownY(state_.cells[i].y);
            if (cell_y == row) {
                cells_in_row.push_back(i);
            }
        }
    }
    
    if (cells_in_row.size() < 2) return false;
    
    // Sort by x-coordinate
    std::sort(cells_in_row.begin(), cells_in_row.end(), 
              [this](int a, int b) { return state_.cells[a].x < state_.cells[b].x; });
    
    // Select random starting position
    if (cells_in_row.size() < chain_length) {
        chain_length = cells_in_row.size();
    }
    
    std::uniform_int_distribution<int> start_dist(0, cells_in_row.size() - chain_length);
    int start_idx = start_dist(rng_);
    
    // Build swap chain: cell[i] -> position of cell[i+1], last -> position of first
    std::vector<std::pair<int, GridPt>> moves;
    
    for (int i = 0; i < chain_length; ++i) {
        int cell_id = cells_in_row[start_idx + i];
        int next_idx = (i + 1) % chain_length;
        int target_cell = cells_in_row[start_idx + next_idx];
        
        GridX target_x = grid_info_->gridX(state_.cells[target_cell].x);
        GridY target_y = grid_info_->gridSnapDownY(state_.cells[target_cell].y);
        
        moves.push_back({cell_id, GridPt{target_x, target_y}});
    }
    
    // Validate all moves are legal
    WorkerGrid temp_grid(grid_info_);
    temp_grid.copyFrom(*grid_);
    
    // Remove all cells that will move
    for (const auto& [cell_id, _] : moves) {
        temp_grid.removeCell(cell_id);
    }
    
    // Check if all can be placed at new positions
    for (const auto& [cell_id, new_pos] : moves) {
        const dpl::Node* node = network_->getNode(cell_id);
        GridX width = grid_info_->gridPaddedWidth(node);
        GridY height = grid_info_->gridHeight(node);
        
        // Check bounds and overlaps
        if (new_pos.x < 0 || new_pos.x + width > grid_info_->getRowSiteCount() ||
            new_pos.y < 0 || new_pos.y + height > grid_info_->getRowCount()) {
            return false;
        }
        
        for (GridY yi = new_pos.y; yi < new_pos.y + height; ++yi) {
            for (GridX xi = new_pos.x; xi < new_pos.x + width; ++xi) {
                if (temp_grid.isOccupied(xi, yi)) {
                    return false;
                }
            }
        }
        
        // Add to temp grid
        temp_grid.placeCell(cell_id, new_pos.x, new_pos.y, width, height);
    }
    
    // Apply moves forcibly (no SA acceptance)
    double total_delta = 0;
    for (const auto& [cell_id, new_pos] : moves) {
        double old_contrib = calcCellHPWLContribution(cell_id);
        
        // Update position
        state_.cells[cell_id].x = gridToDbu(new_pos.x, DbuX{grid_info_->getSiteWidth()});
        state_.cells[cell_id].y = grid_info_->gridYToDbu(new_pos.y);
        state_.cells[cell_id].orient = getCellOrientation(cell_id, new_pos.x, new_pos.y);
        
        double new_contrib = calcCellHPWLContribution(cell_id);
        total_delta += new_contrib - old_contrib;
    }
    
    // Update grid
    for (const auto& [cell_id, new_pos] : moves) {
        const dpl::Node* node = network_->getNode(cell_id);
        grid_->removeCell(cell_id);
        grid_->placeCell(cell_id, new_pos.x, new_pos.y,
                       grid_info_->gridPaddedWidth(node),
                       grid_info_->gridHeight(node));
    }
    
    // Update total HPWL
    state_.total_hpwl += total_delta;
    total_swaps_applied_ += chain_length;
    
    return true;
}

// Additional forced kick move implementations

bool SAWorker::tryRowCompressionForced(int row, int start_x, int length)
{
    if (row >= grid_info_->getRowCount()) return false;
    
    // Find cells in the specified x-range of this row
    std::vector<int> cells_to_compress;
    for (int i = 0; i < network_->getNumNodes(); ++i) {
        dpl::Node* node = const_cast<dpl::Node*>(network_->getNode(i));
        if (node->getType() == dpl::Node::CELL && !node->isFixed()) {
            GridY cell_y = grid_info_->gridSnapDownY(state_.cells[i].y);
            GridX cell_x = grid_info_->gridX(state_.cells[i].x);
            
            if (cell_y == row && cell_x >= start_x && cell_x < start_x + length) {
                cells_to_compress.push_back(i);
            }
        }
    }
    
    if (cells_to_compress.size() < 2) return false;
    
    // Sort by x-coordinate
    std::sort(cells_to_compress.begin(), cells_to_compress.end(),
              [this](int a, int b) { return state_.cells[a].x < state_.cells[b].x; });
    
    // Calculate total width and find gaps
    int total_width = 0;
    std::vector<int> gaps;
    
    for (size_t i = 0; i < cells_to_compress.size(); ++i) {
        const dpl::Node* node = network_->getNode(cells_to_compress[i]);
        total_width += grid_info_->gridPaddedWidth(node).v;
        
        if (i > 0) {
            GridX curr_x = grid_info_->gridX(state_.cells[cells_to_compress[i]].x);
            const dpl::Node* prev_node = network_->getNode(cells_to_compress[i-1]);
            GridX prev_x = grid_info_->gridX(state_.cells[cells_to_compress[i-1]].x);
            GridX prev_width = grid_info_->gridPaddedWidth(prev_node);
            
            int gap = curr_x.v - (prev_x.v + prev_width.v);
            if (gap > 0) gaps.push_back(gap);
        }
    }
    
    // If no gaps, can't compress (but still allow for speculative kicks)
    int total_gaps = std::accumulate(gaps.begin(), gaps.end(), 0);
    if (total_gaps < 2) return false;  // Reduced threshold for speculative mode
    
    // Try to compress by removing gaps
    std::vector<std::pair<int, GridPt>> new_positions;
    GridX current_x{start_x};
    
    for (int cell_id : cells_to_compress) {
        const dpl::Node* node = network_->getNode(cell_id);
        GridX width = grid_info_->gridPaddedWidth(node);
        
        new_positions.push_back({cell_id, GridPt{current_x, GridY{row}}});
        current_x = GridX{current_x.v + width.v};
    }
    
    // Validate compression
    WorkerGrid temp_grid(grid_info_);
    temp_grid.copyFrom(*grid_);
    
    // Remove cells to compress
    for (int cell_id : cells_to_compress) {
        temp_grid.removeCell(cell_id);
    }
    
    // Check new positions
    for (const auto& [cell_id, new_pos] : new_positions) {
        const dpl::Node* node = network_->getNode(cell_id);
        GridX width = grid_info_->gridPaddedWidth(node);
        GridY height = grid_info_->gridHeight(node);
        
        // Check bounds and overlaps
        if (new_pos.x + width > grid_info_->getRowSiteCount()) {
            return false;
        }
        
        for (GridY yi = new_pos.y; yi < new_pos.y + height; ++yi) {
            for (GridX xi = new_pos.x; xi < new_pos.x + width; ++xi) {
                if (temp_grid.isOccupied(xi, yi)) {
                    return false;
                }
            }
        }
        
        temp_grid.placeCell(cell_id, new_pos.x, new_pos.y, width, height);
    }
    
    // Apply compression forcibly
    double total_delta = 0;
    for (const auto& [cell_id, new_pos] : new_positions) {
        double old_contrib = calcCellHPWLContribution(cell_id);
        
        // Update position
        state_.cells[cell_id].x = gridToDbu(new_pos.x, DbuX{grid_info_->getSiteWidth()});
        
        double new_contrib = calcCellHPWLContribution(cell_id);
        total_delta += new_contrib - old_contrib;
    }
    
    // Update grid
    for (const auto& [cell_id, new_pos] : new_positions) {
        const dpl::Node* node = network_->getNode(cell_id);
        grid_->removeCell(cell_id);
        grid_->placeCell(cell_id, new_pos.x, new_pos.y,
                       grid_info_->gridPaddedWidth(node),
                       grid_info_->gridHeight(node));
    }
    
    // Update total HPWL
    state_.total_hpwl += total_delta;
    total_swaps_applied_ += cells_to_compress.size();
    
    return true;
}

bool SAWorker::tryInterRowTransferForced()
{
    // Find a cell to transfer between rows
    std::vector<int> movable_cells;
    for (int i = 0; i < network_->getNumNodes(); ++i) {
        dpl::Node* node = const_cast<dpl::Node*>(network_->getNode(i));
        if (node->getType() == dpl::Node::CELL && !node->isFixed() &&
            grid_info_->gridHeight(node).v == 1) {  // Single height cells only
            movable_cells.push_back(i);
        }
    }
    
    if (movable_cells.empty()) return false;
    
    // Select random cell
    std::uniform_int_distribution<int> cell_dist(0, movable_cells.size() - 1);
    int cell_id = movable_cells[cell_dist(rng_)];
    
    GridY current_row = grid_info_->gridSnapDownY(state_.cells[cell_id].y);
    GridX current_x = grid_info_->gridX(state_.cells[cell_id].x);
    
    // Try other rows
    std::vector<int> other_rows;
    for (int r = 0; r < grid_info_->getRowCount(); ++r) {
        if (r != current_row.v) {
            other_rows.push_back(r);
        }
    }
    
    if (other_rows.empty()) return false;
    
    std::shuffle(other_rows.begin(), other_rows.end(), rng_);
    
    // Try to find space in other rows
    const dpl::Node* node = network_->getNode(cell_id);
    GridX width = grid_info_->gridPaddedWidth(node);
    
    for (int target_row : other_rows) {
        // Search for free space near current x position
        std::vector<GridX> x_positions;
        
        // Try positions near current x
        int search_range = 30;  // Reduced range for faster search
        for (int dx = -search_range; dx <= search_range; dx += 5) {
            GridX try_x{current_x.v + dx};
            if (try_x.v >= 0 && try_x.v + width.v <= grid_info_->getRowSiteCount()) {
                x_positions.push_back(try_x);
            }
        }
        
        std::shuffle(x_positions.begin(), x_positions.end(), rng_);
        
        for (GridX try_x : x_positions) {
            // Check if position is free
            bool can_place = true;
            for (GridX xi = try_x; xi < GridX{try_x.v + width.v}; ++xi) {
                if (grid_->isOccupied(xi, GridY{target_row})) {
                    can_place = false;
                    break;
                }
            }
            
            if (can_place) {
                // Check site compatibility
                const auto& available_sites = grid_info_->getSitesAt(try_x, GridY{target_row});
                if (available_sites.find(node->getSite()) == available_sites.end()) {
                    continue;
                }
                
                // Apply move forcibly
                double old_contrib = calcCellHPWLContribution(cell_id);
                
                // Update position
                grid_->removeCell(cell_id);
                grid_->placeCell(cell_id, try_x, GridY{target_row}, width, GridY{1});
                
                state_.cells[cell_id].x = gridToDbu(try_x, DbuX{grid_info_->getSiteWidth()});
                state_.cells[cell_id].y = grid_info_->gridYToDbu(GridY{target_row});
                state_.cells[cell_id].orient = getCellOrientation(cell_id, try_x, GridY{target_row});
                
                double new_contrib = calcCellHPWLContribution(cell_id);
                double delta = new_contrib - old_contrib;
                
                state_.total_hpwl += delta;
                total_swaps_applied_++;
                
                return true;
            }
        }
    }
    
    return false;
}

bool SAWorker::trySlidingWindowMoveForced(int row, int window_size)
{
    if (row >= grid_info_->getRowCount()) return false;
    
    // Find cells in this row
    std::vector<std::pair<int, GridX>> cells_in_row;
    for (int i = 0; i < network_->getNumNodes(); ++i) {
        dpl::Node* node = const_cast<dpl::Node*>(network_->getNode(i));
        if (node->getType() == dpl::Node::CELL && !node->isFixed()) {
            GridY cell_y = grid_info_->gridSnapDownY(state_.cells[i].y);
            if (cell_y == row) {
                GridX cell_x = grid_info_->gridX(state_.cells[i].x);
                cells_in_row.push_back({i, cell_x});
            }
        }
    }
    
    if (cells_in_row.size() < 3) return false;
    
    // Sort by x-coordinate
    std::sort(cells_in_row.begin(), cells_in_row.end(),
              [](const auto& a, const auto& b) { return a.second < b.second; });
    
    // Select random window start
    std::uniform_int_distribution<int> start_dist(0, 
        std::max(0, static_cast<int>(cells_in_row.size()) - window_size));
    int window_start = start_dist(rng_);
    
    // Get cells in window
    std::vector<int> window_cells;
    for (int i = window_start; i < window_start + window_size && i < cells_in_row.size(); ++i) {
        window_cells.push_back(cells_in_row[i].first);
    }
    
    if (window_cells.size() < 2) return false;
    
    // Calculate window bounds
    GridX min_x = cells_in_row[window_start].second;
    GridX max_x = min_x;
    int total_width = 0;
    
    for (int cell_id : window_cells) {
        const dpl::Node* node = network_->getNode(cell_id);
        GridX cell_x = grid_info_->gridX(state_.cells[cell_id].x);
        GridX width = grid_info_->gridPaddedWidth(node);
        
        max_x = std::max(max_x, GridX{cell_x.v + width.v});
        total_width += width.v;
    }
    
    int window_span = max_x.v - min_x.v;
    int free_space = window_span - total_width;
    
    if (free_space < 2) return false;  // Reduced threshold for speculative mode
    
    // Generate random permutation of cells
    std::vector<int> permuted = window_cells;
    std::shuffle(permuted.begin(), permuted.end(), rng_);
    
    // Try to place permuted cells
    std::vector<std::pair<int, GridPt>> new_positions;
    GridX current_x = min_x;
    
    for (int cell_id : permuted) {
        const dpl::Node* node = network_->getNode(cell_id);
        GridX width = grid_info_->gridPaddedWidth(node);
        
        // Add some random spacing
        if (free_space > 0 && current_x.v > min_x.v) {
            std::uniform_int_distribution<int> space_dist(0, std::min(2, free_space));
            int spacing = space_dist(rng_);
            current_x = GridX{current_x.v + spacing};
            free_space -= spacing;
        }
        
        new_positions.push_back({cell_id, GridPt{current_x, GridY{row}}});
        current_x = GridX{current_x.v + width.v};
    }
    
    // Validate new positions
    WorkerGrid temp_grid(grid_info_);
    temp_grid.copyFrom(*grid_);
    
    // Remove window cells
    for (int cell_id : window_cells) {
        temp_grid.removeCell(cell_id);
    }
    
    // Check new positions
    for (const auto& [cell_id, new_pos] : new_positions) {
        const dpl::Node* node = network_->getNode(cell_id);
        GridX width = grid_info_->gridPaddedWidth(node);
        GridY height = grid_info_->gridHeight(node);
        
        if (new_pos.x + width > grid_info_->getRowSiteCount()) {
            return false;
        }
        
        for (GridY yi = new_pos.y; yi < new_pos.y + height; ++yi) {
            for (GridX xi = new_pos.x; xi < new_pos.x + width; ++xi) {
                if (temp_grid.isOccupied(xi, yi)) {
                    return false;
                }
            }
        }
        
        temp_grid.placeCell(cell_id, new_pos.x, new_pos.y, width, height);
    }
    
    // Apply moves forcibly
    double total_delta = 0;
    for (const auto& [cell_id, new_pos] : new_positions) {
        double old_contrib = calcCellHPWLContribution(cell_id);
        
        // Update position
        state_.cells[cell_id].x = gridToDbu(new_pos.x, DbuX{grid_info_->getSiteWidth()});
        state_.cells[cell_id].y = grid_info_->gridYToDbu(new_pos.y);
        state_.cells[cell_id].orient = getCellOrientation(cell_id, new_pos.x, new_pos.y);
        
        double new_contrib = calcCellHPWLContribution(cell_id);
        total_delta += new_contrib - old_contrib;
    }
    
    // Update grid
    for (const auto& [cell_id, new_pos] : new_positions) {
        const dpl::Node* node = network_->getNode(cell_id);
        grid_->removeCell(cell_id);
        grid_->placeCell(cell_id, new_pos.x, new_pos.y,
                       grid_info_->gridPaddedWidth(node),
                       grid_info_->gridHeight(node));
    }
    
    // Update total HPWL
    state_.total_hpwl += total_delta;
    total_swaps_applied_ += window_cells.size();
    
    return true;
}

}  // namespace sa2d 