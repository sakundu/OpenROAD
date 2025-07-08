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

#pragma once

#include <map>
#include <vector>
#include <unordered_map>
#include <optional>
#include <cstring>  // for memcpy

#include "odb/geom.h"
#include "odb/db.h"

// Include DPL's coordinate definitions
#include "infrastructure/Coordinates.h"

// Forward declarations from DPL
namespace dpl {
class Grid;
class Architecture;
class Network;
class Node;
class Opendp;
}  // namespace dpl

namespace sa2d {

using odb::Rect;
using odb::dbSite;
using odb::dbOrientType;
using odb::dbBlock;

// Use DPL's coordinate types
using dpl::GridX;
using dpl::GridY;
using dpl::DbuX;
using dpl::DbuY;
using dpl::GridPt;
using dpl::GridRect;

// Immutable grid information (shared across all workers)
class ImmutableGridInfo {
public:
  ImmutableGridInfo() = default;
  ~ImmutableGridInfo() = default;
  
  void initFromDPL(const dpl::Grid* dpl_grid,
                   const dpl::Architecture* arch,
                   dbBlock* block,
                   utl::Logger* logger);
  
  // All conversion functions are const and thread-safe
  GridX gridX(DbuX x) const;
  GridY gridSnapDownY(DbuY y) const;
  GridX gridPaddedWidth(const dpl::Node* cell) const;
  GridY gridHeight(const dpl::Node* cell) const;
  
  // Basic grid properties
  int getRowCount() const { return row_count_; }
  int getRowSiteCount() const { return row_site_count_; }
  int getSiteWidth() const { return site_width_; }
  DbuY gridYToDbu(GridY y) const;
  DbuX gridToDbuX(GridX x) const;  // Remove site_width parameter
  
  // Get row height (uniform height if available, otherwise must compute per-row)
  std::optional<int> getUniformRowHeight() const { return uniform_row_height_; }
  
  // Check if a GridY is valid (corresponds to an actual row)
  bool isValidGridY(GridY y) const { 
    return y.v >= 0 && y.v < row_count_; 
  }
  
  // Get the nearest valid GridY to a target GridY
  GridY nearestValidGridY(GridY target) const {
    if (target.v < 0) return GridY{0};
    if (target.v >= row_count_) return GridY{row_count_ - 1};
    return target;
  }
  
  // Multi-height cell support
  GridY getCellHeightInRows(const dpl::Node* cell) const;
  bool isMultiHeightCell(const dpl::Node* cell) const;
  bool rowsAvailable(GridY start_row, GridY height) const {
    return start_row.v >= 0 && (start_row.v + height.v) <= row_count_;
  }
  
  // Site information access
  const std::map<dbSite*, dbOrientType>& getSitesAt(GridX x, GridY y) const;
  dbOrientType getValidOrientation(dbSite* site, GridX x, GridY y) const;
  
  // Grid boundary operations
  GridRect gridWithin(const Rect& rect) const;
  
  // Row information access
  bool isRowYSymmetric(GridY y) const;
  bool isSingleHeightCell(const dpl::Node* cell) const;
  
private:
  // Immutable after initialization
  Rect core_;
  int site_width_{0};
  int row_count_{0};
  int row_site_count_{0};
  std::vector<int> row_index_to_y_dbu_;
  std::optional<int> uniform_row_height_;
  
  // Site information for each pixel
  struct PixelSiteInfo {
    std::map<dbSite*, dbOrientType> sites;  // Available sites and their orientations
  };
  std::vector<std::vector<PixelSiteInfo>> pixel_sites_;  // [y][x]
  
  // Row symmetry information
  std::vector<bool> row_y_symmetric_;  // [y] = true if row supports Y symmetry
  const dpl::Architecture* arch_{nullptr};  // Cached pointer for cell height queries
};

// Mutable pixel state per worker
struct WorkerPixel {
  int cell_id = -1;  // Index into cell array (-1 = empty)
};

// Complete worker grid state
class WorkerGrid {
public:
  explicit WorkerGrid(const ImmutableGridInfo* info);
  ~WorkerGrid() = default;
  
  // Pixel access
  bool isOccupied(GridX x, GridY y) const;
  int getCellAt(GridX x, GridY y) const;
  
  // Multi-height cell support
  bool canPlaceMultiHeight(int cell_id, GridX x, GridY y, 
                          GridX width, GridY height) const;
  void placeMultiHeightCell(int cell_id, GridX x, GridY y,
                           GridX width, GridY height);
  
  // Modifications (for single worker)
  void placeCell(int cell_id, GridX x, GridY y, 
                 GridX width, GridY height);
  void removeCell(int cell_id);
  void clear();
  
  // Efficient state copying for GWTW
  void copyFrom(const WorkerGrid& other);
  
private:
  const ImmutableGridInfo* info_;  // Shared, immutable
  std::vector<std::vector<WorkerPixel>> pixels_;  // Per-worker state
  
  // Optional: sparse representation for memory efficiency
  std::unordered_map<int, GridRect> cell_locations_;  // cell_id -> location
};

}  // namespace sa2d 