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

#include "ThreadSafeGrid.h"
#include <algorithm>
#include <cassert>
#include "utl/Logger.h"

// Include DPL headers for implementation
#include "infrastructure/Grid.h"
#include "infrastructure/architecture.h"
#include "infrastructure/Objects.h"  // For Node definition

namespace sa2d {

// Helper functions
static int divCeil(int a, int b) 
{
  return (a + b - 1) / b;
}

static int divFloor(int a, int b)
{
  return a / b;
}

// ImmutableGridInfo implementation

void ImmutableGridInfo::initFromDPL(const dpl::Grid* dpl_grid,
                                   const dpl::Architecture* arch,
                                   dbBlock* block,
                                   utl::Logger* logger)
{
  // Get core area from block
  core_ = block->getCoreArea();
  
  // Get site information from first row
  odb::dbRow* first_row = nullptr;
  for (auto row : block->getRows()) {
    if (row->getSite()->getClass() != odb::dbSiteClass::PAD) {
      first_row = row;
      break;
    }
  }
  
  if (!first_row) {
    // No rows found - create minimal grid
    site_width_ = 1;
    row_count_ = 1;
    row_site_count_ = core_.dx();
    row_index_to_y_dbu_.push_back(core_.yMin());
    return;
  }
  
  site_width_ = first_row->getSite()->getWidth();
  int site_height = first_row->getSite()->getHeight();
  uniform_row_height_ = site_height;
  
  // Count rows and build row index
  row_count_ = 0;
  row_index_to_y_dbu_.clear();
  
  for (auto row : block->getRows()) {
    // Skip PAD rows - DPL doesn't consider them valid for placement
    if (row->getSite()->getClass() != odb::dbSiteClass::PAD) {
      row_index_to_y_dbu_.push_back(row->getOrigin().y() - core_.yMin());
      row_count_++;
    }
  }
  
  // Sort row indices
  std::sort(row_index_to_y_dbu_.begin(), row_index_to_y_dbu_.end());
  
  // Debug: Check for gaps in row Y coordinates
  if (row_count_ > 1 && logger) {
    bool has_gaps = false;
    int gap_count = 0;
    for (int i = 1; i < row_count_; i++) {
      int gap = row_index_to_y_dbu_[i] - row_index_to_y_dbu_[i-1];
      if (gap != site_height) {
        has_gaps = true;
        gap_count++;
        // Log first few gaps
        if (gap_count <= 5) {
          logger->warn(utl::SA2D, 211, 
            "Row gap detected: row[{}] Y={}, row[{}] Y={}, gap={} (expected {})",
            i-1, row_index_to_y_dbu_[i-1], i, row_index_to_y_dbu_[i], gap, site_height);
        }
      }
    }
    if (has_gaps) {
      logger->warn(utl::SA2D, 212, 
        "Row Y coordinates have {} gaps! This may cause invalid Y positions in SA2D.", gap_count);
    }
    
    // Also log info about the row structure
    logger->info(utl::SA2D, 213, "Row structure: {} rows, Y range [{}, {}], site_height={}",
                 row_count_, row_index_to_y_dbu_.front(), row_index_to_y_dbu_.back(), site_height);
                 
    // Debug: Look for the problematic Y coordinate 1464400
    for (int i = 0; i < row_count_; i++) {
      if (row_index_to_y_dbu_[i] == 1464400) {
        logger->warn(utl::SA2D, 215, "Found Y=1464400 at row index {}", i);
        if (i > 0) {
          logger->warn(utl::SA2D, 216, "  Previous row[{}] Y={}", i-1, row_index_to_y_dbu_[i-1]);
        }
        if (i < row_count_ - 1) {
          logger->warn(utl::SA2D, 217, "  Next row[{}] Y={}", i+1, row_index_to_y_dbu_[i+1]);
        }
        break;
      }
    }
  }
  
  // Calculate sites per row
  row_site_count_ = core_.dx() / site_width_;
  
  // Initialize pixel sites (simplified - all sites have same orientation)
  pixel_sites_.resize(row_count_);
  for (int y = 0; y < row_count_; y++) {
    pixel_sites_[y].resize(row_site_count_);
    for (int x = 0; x < row_site_count_; x++) {
      // Add default site with R0 orientation
      pixel_sites_[y][x].sites[first_row->getSite()] = odb::dbOrientType::R0;
    }
  }
}

GridX ImmutableGridInfo::gridX(DbuX x) const
{
  // Simple conversion - DPL already works in core-relative coordinates
  return GridX{x.v / site_width_};
}

GridY ImmutableGridInfo::gridSnapDownY(DbuY y) const
{
  // Binary search to find the row containing this y coordinate
  auto it = std::upper_bound(
      row_index_to_y_dbu_.begin(), row_index_to_y_dbu_.end(), y.v);
  if (it == row_index_to_y_dbu_.begin()) {
    return GridY{0};
  }
  --it;
  return GridY{static_cast<int>(it - row_index_to_y_dbu_.begin())};
}

GridX ImmutableGridInfo::gridPaddedWidth(const dpl::Node* cell) const
{
  // For v0, we don't handle padding - just return regular width
  // Get cell width and convert to grid units
  if (site_width_ == 0) return GridX{1};
  
  // Use DPL's divCeil function for consistency
  return GridX{divCeil(cell->getWidth().v, site_width_)};
}

GridY ImmutableGridInfo::gridHeight(const dpl::Node* cell) const
{
  // Get cell height in number of rows
  if (uniform_row_height_.has_value()) {
    return GridY{std::max(1, divCeil(cell->getHeight().v, uniform_row_height_.value()))};
  }
  // For now, assume single-row cells if no uniform height
  return GridY{1};
}

DbuY ImmutableGridInfo::gridYToDbu(GridY y) const
{
  if (y.v < 0 || y.v >= row_count_) {
    return DbuY{0};  // Or throw error
  }
  return DbuY{row_index_to_y_dbu_[y.v]};
}

DbuX ImmutableGridInfo::gridToDbuX(GridX x) const
{
  // Simple conversion - DPL already works in core-relative coordinates
  DbuX result{x.v * site_width_};
  return result;
}

const std::map<dbSite*, dbOrientType>& 
ImmutableGridInfo::getSitesAt(GridX x, GridY y) const
{
  static std::map<dbSite*, dbOrientType> empty_map;
  
  if (y.v >= 0 && y.v < row_count_ && x.v >= 0 && x.v < row_site_count_) {
    return pixel_sites_[y.v][x.v].sites;
  }
  return empty_map;
}

dbOrientType ImmutableGridInfo::getValidOrientation(dbSite* site, 
                                                    GridX x, 
                                                    GridY y) const
{
  const auto& sites = getSitesAt(x, y);
  auto it = sites.find(site);
  if (it != sites.end()) {
    return it->second;
  }
  // Default orientation if not found
  return dbOrientType::R0;
}

GridRect ImmutableGridInfo::gridWithin(const Rect& rect) const
{
  // Convert rectangle to grid coordinates (snapping inward)
  GridRect result;
  result.xlo = GridX{divCeil(rect.xMin(), site_width_)};
  result.ylo = gridSnapDownY(DbuY{rect.yMin()});
  result.xhi = GridX{divFloor(rect.xMax(), site_width_)};
  
  // For yhi, find the largest row that fits within rect.yMax()
  auto it = std::upper_bound(
      row_index_to_y_dbu_.begin(), row_index_to_y_dbu_.end(), rect.yMax());
  if (it != row_index_to_y_dbu_.begin()) {
    --it;
    result.yhi = GridY{static_cast<int>(it - row_index_to_y_dbu_.begin())};
  } else {
    result.yhi = GridY{0};
  }
  
  return result;
}

// WorkerGrid implementation

WorkerGrid::WorkerGrid(const ImmutableGridInfo* info)
  : info_(info)
{
  // Initialize pixel grid
  int row_count = info_->getRowCount();
  int row_site_count = info_->getRowSiteCount();
  
  pixels_.resize(row_count);
  for (int y = 0; y < row_count; y++) {
    pixels_[y].resize(row_site_count);
    // Initialize all pixels as empty
    for (int x = 0; x < row_site_count; x++) {
      pixels_[y][x].cell_id = -1;
    }
  }
}

bool WorkerGrid::isOccupied(GridX x, GridY y) const
{
  if (y.v >= 0 && y.v < static_cast<int>(pixels_.size()) &&
      x.v >= 0 && x.v < static_cast<int>(pixels_[y.v].size())) {
    return pixels_[y.v][x.v].cell_id != -1;
  }
  return false;
}

int WorkerGrid::getCellAt(GridX x, GridY y) const
{
  if (y.v >= 0 && y.v < static_cast<int>(pixels_.size()) &&
      x.v >= 0 && x.v < static_cast<int>(pixels_[y.v].size())) {
    return pixels_[y.v][x.v].cell_id;
  }
  return -1;
}

void WorkerGrid::placeCell(int cell_id, GridX x, GridY y,
                          GridX width, GridY height)
{
  // Place cell in all pixels it occupies
  for (int yi = y.v; yi < y.v + height.v && yi < static_cast<int>(pixels_.size()); yi++) {
    for (int xi = x.v; xi < x.v + width.v && xi < static_cast<int>(pixels_[yi].size()); xi++) {
      pixels_[yi][xi].cell_id = cell_id;
    }
  }
  
  // Update cell location tracking
  GridRect rect;
  rect.xlo = x;
  rect.ylo = y;
  rect.xhi = GridX{x.v + width.v};
  rect.yhi = GridY{y.v + height.v};
  cell_locations_[cell_id] = rect;
}

void WorkerGrid::removeCell(int cell_id)
{
  // Find cell location
  auto it = cell_locations_.find(cell_id);
  if (it != cell_locations_.end()) {
    const auto& rect = it->second;
    
    // Clear pixels
    for (int y = rect.ylo.v; y < rect.yhi.v && y < static_cast<int>(pixels_.size()); y++) {
      for (int x = rect.xlo.v; x < rect.xhi.v && x < static_cast<int>(pixels_[y].size()); x++) {
        if (pixels_[y][x].cell_id == cell_id) {
          pixels_[y][x].cell_id = -1;
        }
      }
    }
    
    // Remove from tracking
    cell_locations_.erase(it);
  }
}

void WorkerGrid::clear()
{
  // Clear all pixels
  for (auto& row : pixels_) {
    for (auto& pixel : row) {
      pixel.cell_id = -1;
    }
  }
  
  // Clear location tracking
  cell_locations_.clear();
}

void WorkerGrid::copyFrom(const WorkerGrid& other)
{
  // Copy pixel data
  pixels_ = other.pixels_;
  
  // Copy location tracking
  cell_locations_ = other.cell_locations_;
}

}  // namespace sa2d 