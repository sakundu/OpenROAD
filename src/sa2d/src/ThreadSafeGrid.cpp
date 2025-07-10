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

// ImmutableGridInfo implementation

void ImmutableGridInfo::initFromDPL(const dpl::Grid* dpl_grid,
                                    const dpl::Architecture* arch,
                                    dbBlock* block,
                                    utl::Logger* logger)
{
    // Store architecture pointer
    arch_ = arch;
    
    // Initialize basic grid properties
    core_ = dpl_grid->getCore();
    site_width_ = dpl_grid->getSiteWidth().v;  // Extract int value from DbuX
    row_count_ = dpl_grid->getRowCount().v;    // Extract int value from GridY
    row_site_count_ = dpl_grid->getRowSiteCount().v;  // Extract int value from GridX
    
    // Copy row index to Y mapping
    row_index_to_y_dbu_.resize(row_count_);
    for (int y = 0; y < row_count_; ++y) {
        // Get Y coordinate for this row from DPL grid
        row_index_to_y_dbu_[y] = dpl_grid->gridYToDbu(GridY{y}).v;
    }
    
    // Check for uniform row height - estimate from first two rows
    if (row_count_ >= 2) {
        int height0 = row_index_to_y_dbu_[1] - row_index_to_y_dbu_[0];
        bool uniform = true;
        for (int y = 2; y < row_count_; ++y) {
            int height = row_index_to_y_dbu_[y] - row_index_to_y_dbu_[y-1];
            if (height != height0) {
                uniform = false;
                break;
            }
        }
        if (uniform) {
            uniform_row_height_ = height0;
        }
    }
    
    // Initialize site information for each pixel
    pixel_sites_.resize(row_count_);
    for (int y = 0; y < row_count_; ++y) {
        pixel_sites_[y].resize(row_site_count_);
    }
    
    // Initialize row symmetry information
    row_y_symmetric_.resize(row_count_, false);
    
    // Populate site information and symmetry from rows
    for (int y = 0; y < row_count_; ++y) {
        // Get row symmetry from DPL architecture
        if (y < arch->getNumRows()) {
            dpl::Architecture::Row* dpl_row = arch->getRow(y);
            row_y_symmetric_[y] = (dpl_row->getSymmetry() & 0x2) != 0;  // Check Symmetry_Y bit
        }
        
        // Use DPL's grid to get site information for each pixel
        for (int x = 0; x < row_site_count_; ++x) {
            GridX gx{x};
            GridY gy{y};
            
            // Get site and orientation from DPL grid
            auto pixel = dpl_grid->gridPixel(gx, gy);
            if (pixel) {
                // Copy all available sites and their orientations
                pixel_sites_[y][x].sites = pixel->sites;
                // Copy validity flag - critical for discontinuous rows!
                pixel_sites_[y][x].is_valid = pixel->is_valid;
            } else {
                // No pixel at this location means invalid
                pixel_sites_[y][x].is_valid = false;
            }
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
    GridX xlo = gridX(DbuX{rect.xMin()});
    GridX xhi = gridX(DbuX{rect.xMax()});
    GridY ylo = gridSnapDownY(DbuY{rect.yMin()});
    GridY yhi = gridSnapDownY(DbuY{rect.yMax()});
    
    return GridRect{xlo, ylo, xhi, yhi};
}

bool ImmutableGridInfo::isRowYSymmetric(GridY y) const
{
    if (y.v >= 0 && y.v < row_count_) {
        return row_y_symmetric_[y.v];
    }
    return false;
}

bool ImmutableGridInfo::isSingleHeightCell(const dpl::Node* cell) const
{
    if (arch_) {
        return arch_->isSingleHeightCell(cell);
    }
    // Fallback: assume single height if cell height equals one row
    return gridHeight(cell).v == 1;
}

GridY ImmutableGridInfo::getCellHeightInRows(const dpl::Node* cell) const
{
    if (arch_) {
        return GridY{arch_->getCellHeightInRows(cell)};
    }
    // Fallback: calculate based on uniform row height if available
    if (uniform_row_height_.has_value() && uniform_row_height_.value() > 0) {
        return GridY{std::max(1, divCeil(cell->getHeight().v, uniform_row_height_.value()))};
    }
    // Default to single height
    return GridY{1};
}

bool ImmutableGridInfo::isMultiHeightCell(const dpl::Node* cell) const
{
    return getCellHeightInRows(cell).v > 1;
}

////////////////////////////////////////////////////////////////////
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

bool WorkerGrid::canPlaceMultiHeight(int cell_id, GridX x, GridY y,
                                    GridX width, GridY height) const
{
  // Check bounds
  if (x.v < 0 || y.v < 0 || 
      x.v + width.v > static_cast<int>(pixels_[0].size()) ||
      y.v + height.v > static_cast<int>(pixels_.size())) {
    return false;
  }
  
  // Check all rows the cell would span
  for (int row = y.v; row < y.v + height.v; ++row) {
    for (int col = x.v; col < x.v + width.v; ++col) {
      if (pixels_[row][col].cell_id != -1 && 
          pixels_[row][col].cell_id != cell_id) {
        // Occupied by another cell
        return false;
      }
    }
  }
  return true;
}

void WorkerGrid::placeMultiHeightCell(int cell_id, GridX x, GridY y,
                                     GridX width, GridY height)
{
  // First remove if already placed
  removeCell(cell_id);
  
  // Place in all spanned rows (same as placeCell but explicit for clarity)
  placeCell(cell_id, x, y, width, height);
}

}  // namespace sa2d 