///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (c) 2021, Andrew Kennings
// All rights reserved.
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

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Includes.
////////////////////////////////////////////////////////////////////////////////
#include "detailed_manager.h"
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <boost/format.hpp>
#include <boost/tokenizer.hpp>
#include <cmath>
#include <iostream>
#include <set>
#include <stack>
#include <utility>
#include "detailed_orient.h"
#include "detailed_segment.h"
#include "plotgnu.h"
#include "utility.h"
#include "utl/Logger.h"

using utl::DPO;

namespace dpo {

////////////////////////////////////////////////////////////////////////////////
// Classes.
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
DetailedMgr::DetailedMgr(Architecture* arch, Network* network,
                         RoutingParams* rt)
    : m_arch(arch), m_network(network), m_rt(rt), m_logger(0), m_rng(0) {
  m_singleRowHeight = m_arch->getRow(0)->getHeight();
  m_numSingleHeightRows = m_arch->getNumRows();

  // For random numbers...
  m_rng = new Placer_RNG;
  m_rng->seed(static_cast<unsigned>(1));

  // For limiting displacement...
  int limit = std::max(m_arch->getWidth(), m_arch->getHeight())<<1;
  m_maxDispX = limit;
  m_maxDispY = limit;

  // Utilization...
  m_targetUt = 1.0;

  // For generating a move list...
  m_moveLimit = 10;
  m_nMoved = 0;
  m_curLeft.resize(m_moveLimit);
  m_curBottom.resize(m_moveLimit);
  m_newLeft.resize(m_moveLimit);
  m_newBottom.resize(m_moveLimit);
  m_curOri.resize(m_moveLimit);
  m_newOri.resize(m_moveLimit);
  m_curSeg.resize(m_moveLimit);
  m_newSeg.resize(m_moveLimit);
  m_movedNodes.resize(m_moveLimit);
  for (size_t i = 0; i < m_moveLimit; i++) {
    m_curSeg[i] = std::vector<int>();
    m_newSeg[i] = std::vector<int>();
  }

  // The purpose of this reverse map is to be able to remove the cell from
  // all segments that it has been placed into.  It only works (i.e., is
  // only up-to-date) if you use the proper routines to add and remove cells
  // to and from segments.
  m_reverseCellToSegs.resize(m_network->getNumNodes() );
  for (size_t i = 0; i < m_reverseCellToSegs.size(); i++) {
    m_reverseCellToSegs[i] = std::vector<DetailedSeg*>();
  }

  recordOriginalPositions();
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
DetailedMgr::~DetailedMgr() {
  m_blockages.clear();

  m_segsInRow.clear();

  for (int i = 0; i < m_segments.size(); i++) {
    delete m_segments[i];
  }
  m_segments.clear();

  if (m_rng != 0) {
    delete m_rng;
  }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void DetailedMgr::setSeed(int seed) {
  m_logger->info(DPO, 401, "Setting random seed to {:d}.", seed);
  m_rng->seed(static_cast<unsigned>(seed));
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void DetailedMgr::setMaxDisplacement(int x, int y) {
  int limit = std::max(m_arch->getWidth(), m_arch->getHeight())<<1;
  if (x != 0) {
    m_maxDispX = x*m_arch->getRow(0)->getHeight();
  }
  m_maxDispX = std::min(m_maxDispX, limit);
  if (y != 0) {
    m_maxDispY = y*m_arch->getRow(0)->getHeight();
  }
  m_maxDispY = std::min(m_maxDispY, limit);

  m_logger->info(DPO, 402, "Setting maximum displacement {:d} {:d} to "
    "{:d} {:d} units.", x, y, m_maxDispX, m_maxDispY);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void DetailedMgr::internalError( std::string msg )
{
    m_logger->error(DPO, 400, "Detailed improvement internal error: {:s}.", msg.c_str());
    exit(-1);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void DetailedMgr::findBlockages(bool includeRouteBlockages) {
  // Blockages come from filler, from fixed nodes (possibly with shapes) and
  // from larger macros which are now considered fixed...

  m_blockages.clear();

  // Determine the single height segments and blockages.
  m_blockages.resize(m_numSingleHeightRows);
  for (int i = 0; i < m_blockages.size(); i++) {
    m_blockages[i] = std::vector<std::pair<double, double> >();
  }

  for (int i = 0; i < m_fixedCells.size(); i++) {
    Node* nd = m_fixedCells[i];

    int xmin = std::max(m_arch->getMinX(), nd->getLeft());
    int xmax = std::min(m_arch->getMaxX(), nd->getRight());
    int ymin = std::max(m_arch->getMinY(), nd->getBottom());
    int ymax = std::min(m_arch->getMaxY(), nd->getTop());

    // HACK!  So a fixed cell might split a row into multiple
    // segments.  However, I don't take into account the
    // spacing or padding requirements of this cell!  This
    // means I could get an error later on.
    //
    // I don't think this is guaranteed to fix the problem,
    // but I suppose I can grab spacing/padding between this
    // cell and "no other cell" on either the left or the
    // right.  This might solve the problem since it will
    // make the blockage wider.
    xmin -= m_arch->getCellSpacing(0, nd);
    xmax += m_arch->getCellSpacing(nd, 0);

    for (int r = 0; r < m_numSingleHeightRows; r++) {
      int yb = m_arch->getRow(r)->getBottom();
      int yt = m_arch->getRow(r)->getTop();

      if (!(ymin >= yt || ymax <= yb)) {
        m_blockages[r].push_back(std::pair<double, double>(xmin, xmax));
      }
    }
  }

  if (includeRouteBlockages && m_rt != 0) {
    // Turn M1 and M2 routing blockages into placement blockages.  The idea
    // here is to be quite conservative and prevent the possibility of pin
    // access problems.  We *ONLY* consider routing obstacles to be placement
    // obstacles if they overlap with an *ENTIRE* site.

    for (int layer = 0; layer <= 1 && layer < m_rt->m_num_layers; layer++) {
      std::vector<Rectangle>& rects = m_rt->m_layerBlockages[layer];
      for (int b = 0; b < rects.size(); b++) {
        double xmin = rects[b].xmin();
        double xmax = rects[b].xmax();
        double ymin = rects[b].ymin();
        double ymax = rects[b].ymax();

        for (int r = 0; r < m_numSingleHeightRows; r++) {
          double lb = m_arch->getMinY() + r * m_singleRowHeight;
          double ub = lb + m_singleRowHeight;

          if (ymax >= ub && ymin <= lb) {
            // Blockage overlaps with the entire row span in the Y-dir...
            // Sites are possibly completely covered!

            double originX = m_arch->getRow(r)->getLeft();
            double siteSpacing = m_arch->getRow(r)->getSiteSpacing();

            int i0 = (int)std::floor((xmin - originX) / siteSpacing);
            int i1 = (int)std::floor((xmax - originX) / siteSpacing);
            if (originX + i1 * siteSpacing != xmax) ++i1;

            if (i1 > i0) {
              m_blockages[r].push_back(std::pair<double, double>(
                  originX + i0 * siteSpacing, originX + i1 * siteSpacing));
            }
          }
        }
      }
    }
  }

  // Sort blockages and merge.
  for (int r = 0; r < m_numSingleHeightRows; r++) {
    if (m_blockages[r].size() == 0) {
      continue;
    }

    std::sort(m_blockages[r].begin(), m_blockages[r].end(), compareBlockages());

    std::stack<std::pair<double, double> > s;
    s.push(m_blockages[r][0]);
    for (int i = 1; i < m_blockages[r].size(); i++) {
      std::pair<double, double> top = s.top();  // copy.
      if (top.second < m_blockages[r][i].first) {
        s.push(m_blockages[r][i]);  // new interval.
      } else {
        if (top.second < m_blockages[r][i].second) {
          top.second = m_blockages[r][i].second;  // extend interval.
        }
        s.pop();      // remove old.
        s.push(top);  // expanded interval.
      }
    }

    m_blockages[r].erase(m_blockages[r].begin(), m_blockages[r].end());
    while (!s.empty()) {
      std::pair<double, double> temp = s.top();  // copy.
      m_blockages[r].push_back(temp);
      s.pop();
    }

    // Intervals need to be sorted, but they are currently in reverse order. Can
    // either resort or reverse.
    std::sort(m_blockages[r].begin(), m_blockages[r].end(),
              compareBlockages());  // Sort to get them left to right.
  }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void DetailedMgr::findSegments() {
  // Create the segments into which movable cells are placed.  I do make
  // segment ends line up with sites and that segments don't extend off
  // the chip.

  m_logger->info(DPO, 322, "Image ({:d}, {:d}) - ({:d}, {:d})",
                m_arch->getMinX(), m_arch->getMinY(), 
                m_arch->getMaxX(), m_arch->getMaxY());

  for (int i = 0; i < m_segments.size(); i++) {
    delete m_segments[i];
  }
  m_segments.erase(m_segments.begin(), m_segments.end());

  int numSegments = 0;
  m_segsInRow.resize(m_numSingleHeightRows);
  for (int r = 0; r < m_numSingleHeightRows; r++) {
    int lx = m_arch->getRow(r)->getLeft();
    int rx = m_arch->getRow(r)->getRight();

    m_segsInRow[r] = std::vector<DetailedSeg*>();

    int n = (int)m_blockages[r].size();
    if (n == 0) {
      // Entire row free.

      int x1 = std::max(m_arch->getMinX(), lx);
      int x2 = std::min(m_arch->getMaxX(), rx);

      if (x2 > x1) {
        DetailedSeg* segment = new DetailedSeg();
        segment->setSegId(numSegments);
        segment->setRowId(r);
        segment->setMinX(x1);
        segment->setMaxX(x2);

        m_segsInRow[r].push_back(segment);
        m_segments.push_back(segment);

        ++numSegments;
      }
    } else {
      // Divide row.
      if (m_blockages[r][0].first > std::max(m_arch->getMinX(), lx)) {
        int x1 = std::max(m_arch->getMinX(), lx);
        int x2 = std::min(std::min(m_arch->getMaxX(), rx), 
                          (int)std::floor(m_blockages[r][0].first));

        if (x2 > x1) {
          DetailedSeg* segment = new DetailedSeg();
          segment->setSegId(numSegments);
          segment->setRowId(r);
          segment->setMinX(x1);
          segment->setMaxX(x2);

          m_segsInRow[r].push_back(segment);
          m_segments.push_back(segment);

          ++numSegments;
        }
      }
      for (int i = 1; i < n; i++) {
        if (m_blockages[r][i].first > m_blockages[r][i-1].second) {
          int x1 = std::max(std::max(m_arch->getMinX(), lx), 
                            (int)std::ceil(m_blockages[r][i-1].second));
          int x2 = std::min(std::min(m_arch->getMaxX(), rx), 
                            (int)std::floor(m_blockages[r][i].first));

          if (x2 > x1) {
            DetailedSeg* segment = new DetailedSeg();
            segment->setSegId(numSegments);
            segment->setRowId(r);
            segment->setMinX(x1);
            segment->setMaxX(x2);

            m_segsInRow[r].push_back(segment);
            m_segments.push_back(segment);

            ++numSegments;
          }
        }
      }
      if (m_blockages[r][n-1].second < std::min(m_arch->getMaxX(), rx)) {
        int x1 = std::min(std::min(m_arch->getMaxX(), rx),
                          std::max(std::max(m_arch->getMinX(), lx),
                                   (int)std::ceil(m_blockages[r][n-1].second)));
        int x2 = std::min(m_arch->getMaxX(), rx);

        if (x2 > x1) {
          DetailedSeg* segment = new DetailedSeg();
          segment->setSegId(numSegments);
          segment->setRowId(r);
          segment->setMinX(x1);
          segment->setMaxX(x2);

          m_segsInRow[r].push_back(segment);
          m_segments.push_back(segment);

          ++numSegments;
        }
      }
    }
  }

  // Here, we need to slice up the segments to account for regions.
  std::vector<std::vector<std::pair<double, double> > > intervals;
  for (int reg = 1; reg < m_arch->getNumRegions(); reg++) {
    Architecture::Region* regPtr = m_arch->getRegion(reg);

    findRegionIntervals(regPtr->getId(), intervals);

    int split = 0;

    for (int r = 0; r < m_numSingleHeightRows; r++) {
      int n = (int)intervals[r].size();
      if (n == 0) {
        continue;
      }

      // Since the intervals do not overlap, I think the following is fine:
      // Pick an interval and pick a segment.  If the interval and segment
      // do not overlap, do nothing.  If the segment and the interval do
      // overlap, then there are cases.  Let <sl,sr> be the span of the
      // segment.  Let <il,ir> be the span of the interval.  Then:
      //
      // Case 1: il <= sl && ir >= sr: The interval entirely overlaps the
      //         segment.  So, we can simply change the segment's region
      //         type.
      // Case 2: il  > sl && ir >= sr: The segment needs to be split into
      //         two segments.  The left segment remains retains it's
      //         original type while the right segment is new and assigned
      //         to the region type.
      // Case 3: il <= sl && ir  < sr: Switch the meaning of left and right
      //         per case 2.
      // Case 4: il  > sl && ir  < sr: The original segment needs to be
      //         split into 2 with the original region type.  A new segment
      //         needs to be created with the new region type.

      for (size_t i = 0; i < intervals[r].size(); i++) {
        double il = intervals[r][i].first;
        double ir = intervals[r][i].second;
        for (size_t s = 0; s < m_segsInRow[r].size(); s++) {
          DetailedSeg* segPtr = m_segsInRow[r][s];

          int sl = segPtr->getMinX();
          int sr = segPtr->getMaxX();

          // Check for no overlap.
          if (ir <= sl) continue;
          if (il >= sr) continue;

          // Case 1:
          if (il <= sl && ir >= sr) {
            segPtr->setRegId(reg);
          }
          // Case 2:
          else if (il > sl && ir >= sr) {
            ++split;

            segPtr->setMaxX((int)std::floor(il));

            DetailedSeg* newPtr = new DetailedSeg();
            newPtr->setSegId(numSegments);
            newPtr->setRowId(r);
            newPtr->setRegId(reg);
            newPtr->setMinX((int)std::ceil(il));
            newPtr->setMaxX(sr);

            m_segsInRow[r].push_back(newPtr);
            m_segments.push_back(newPtr);

            ++numSegments;
          }
          // Case 3:
          else if (ir < sr && il <= sl) {
            ++split;

            segPtr->setMinX((int)std::ceil(ir));

            DetailedSeg* newPtr = new DetailedSeg();
            newPtr->setSegId(numSegments);
            newPtr->setRowId(r);
            newPtr->setRegId(reg);
            newPtr->setMinX(sl);
            newPtr->setMaxX((int)std::floor(ir));

            m_segsInRow[r].push_back(newPtr);
            m_segments.push_back(newPtr);

            ++numSegments;
          }
          // Case 4:
          else if (il > sl && ir < sr) {
            ++split;
            ++split;

            segPtr->setMaxX((int)std::floor(il));

            DetailedSeg* newPtr = new DetailedSeg();
            newPtr->setSegId(numSegments);
            newPtr->setRowId(r);
            newPtr->setRegId(reg);
            newPtr->setMinX((int)std::ceil(il));
            newPtr->setMaxX((int)std::floor(ir));

            m_segsInRow[r].push_back(newPtr);
            m_segments.push_back(newPtr);

            ++numSegments;

            newPtr = new DetailedSeg();
            newPtr->setSegId(numSegments);
            newPtr->setRowId(r);
            newPtr->setRegId(segPtr->getRegId());
            newPtr->setMinX((int)std::ceil(ir));
            newPtr->setMaxX(sr);

            m_segsInRow[r].push_back(newPtr);
            m_segments.push_back(newPtr);

            ++numSegments;
          } else {
            internalError("Unexpected problem while constructing segments");
          }
        }
      }
    }
  }

  // Make sure segment boundaries line up with sites.
  for (int s = 0; s < m_segments.size(); s++) {
    int rowId = m_segments[s]->getRowId();

    int originX = m_arch->getRow(rowId)->getLeft();
    int siteSpacing = m_arch->getRow(rowId)->getSiteSpacing();

    int ix;

    ix = (int)((m_segments[s]->getMinX() - originX) / siteSpacing);
    if (originX + ix * siteSpacing < m_segments[s]->getMinX()) ++ix;

    if (originX + ix * siteSpacing != m_segments[s]->getMinX())
      m_segments[s]->setMinX(originX + ix * siteSpacing);

    ix = (int)((m_segments[s]->getMaxX() - originX) / siteSpacing);
    if (originX + ix * siteSpacing != m_segments[s]->getMaxX())
      m_segments[s]->setMaxX(originX + ix * siteSpacing);
  }

  // Create the structure for cells in segments.
  m_cellsInSeg.clear();
  m_cellsInSeg.resize(m_segments.size());
  for (size_t i = 0; i < m_cellsInSeg.size(); i++) {
    m_cellsInSeg[i] = std::vector<Node*>();
  }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
DetailedSeg* DetailedMgr::findClosestSegment(Node* nd) {
  // Find the closest segment for the cell which is wide enough to
  // accommodate the cell.

  // Guess at the closest row.  Assumes rows are stacked.
  int row = m_arch->find_closest_row(nd->getBottom());

  double hori;
  double vert;
  double dist1 = std::numeric_limits<double>::max();
  double dist2 = std::numeric_limits<double>::max();
  DetailedSeg* best1 = 0;  // closest segment...
  // closest segment which is wide enough to accomodate the cell...
  DetailedSeg* best2 = 0; 

  // Segments in the current row...
  for (int k = 0; k < m_segsInRow[row].size(); k++) {
    DetailedSeg* curr = m_segsInRow[row][k];

    // Updated for regions.
    if (nd->getRegionId() != curr->getRegId()) {
      continue;
    }

    // Work with left edge.
    int x1 = curr->getMinX();
    int x2 = curr->getMaxX() - nd->getWidth();
    int xx = std::max(x1, std::min(x2, nd->getLeft()));

    hori = std::max(0, std::abs(xx - nd->getLeft()));
    vert = 0.0;

    bool closer1 = (hori + vert < dist1) ? true : false;
    bool closer2 = (hori + vert < dist2) ? true : false;
    bool fits =
        (nd->getWidth() <= (curr->getMaxX() - curr->getMinX())) ? true : false;

    // Keep track of the closest segment.
    if (best1 == 0 || (best1 != 0 && closer1)) {
      best1 = curr;
      dist1 = hori + vert;
    }
    // Keep track of the closest segment which is wide enough to accomodate the
    // cell.
    if (fits && (best2 == 0 || (best2 != 0 && closer2))) {
      best2 = curr;
      dist2 = hori + vert;
    }
  }

  // Consider rows above and below the current row.
  for (int offset = 1; offset <= m_numSingleHeightRows; offset++) {
    int below = row - offset;
    vert = offset * m_singleRowHeight;

    if (below >= 0) {
      // Consider the row if we could improve on either of the best segments we
      // are recording.
      if ((vert <= dist1 || vert <= dist2)) {
        for (int k = 0; k < m_segsInRow[below].size(); k++) {
          DetailedSeg* curr = m_segsInRow[below][k];

          // Updated for regions.
          if (nd->getRegionId() != curr->getRegId()) {
            continue;
          }

          // Work with left edge.
          int x1 = curr->getMinX();
          int x2 = curr->getMaxX() - nd->getWidth();
          int xx = std::max(x1, std::min(x2, nd->getLeft()));

          hori = std::max(0, std::abs(xx - nd->getLeft()));

          bool closer1 = (hori + vert < dist1) ? true : false;
          bool closer2 = (hori + vert < dist2) ? true : false;
          bool fits =
              (nd->getWidth() <= (curr->getMaxX() - curr->getMinX())) ? true : false;

          // Keep track of the closest segment.
          if (best1 == 0 || (best1 != 0 && closer1)) {
            best1 = curr;
            dist1 = hori + vert;
          }
          // Keep track of the closest segment which is wide enough to
          // accomodate the cell.
          if (fits && (best2 == 0 || (best2 != 0 && closer2))) {
            best2 = curr;
            dist2 = hori + vert;
          }
        }
      }
    }

    int above = row + offset;
    vert = offset * m_singleRowHeight;

    if (above <= m_numSingleHeightRows - 1) {
      // Consider the row if we could improve on either of the best segments we
      // are recording.
      if ((vert <= dist1 || vert <= dist2)) {
        for (int k = 0; k < m_segsInRow[above].size(); k++) {
          DetailedSeg* curr = m_segsInRow[above][k];

          // Updated for regions.
          if (nd->getRegionId() != curr->getRegId()) {
            continue;
          }

          // Work with left edge.
          int x1 = curr->getMinX();
          int x2 = curr->getMaxX() - nd->getWidth();
          int xx = std::max(x1, std::min(x2, nd->getLeft()));

          hori = std::max(0, std::abs(xx - nd->getLeft()));

          bool closer1 = (hori + vert < dist1) ? true : false;
          bool closer2 = (hori + vert < dist2) ? true : false;
          bool fits =
              (nd->getWidth() <= (curr->getMaxX() - curr->getMinX())) ? true : false;

          // Keep track of the closest segment.
          if (best1 == 0 || (best1 != 0 && closer1)) {
            best1 = curr;
            dist1 = hori + vert;
          }
          // Keep track of the closest segment which is wide enough to
          // accomodate the cell.
          if (fits && (best2 == 0 || (best2 != 0 && closer2))) {
            best2 = curr;
            dist2 = hori + vert;
          }
        }
      }
    }
  }

  return (best2) ? best2 : best1;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void DetailedMgr::findClosestSpanOfSegmentsDfs(
    Node* ndi, DetailedSeg* segPtr, double xmin, double xmax, int bot, int top,
    std::vector<DetailedSeg*>& stack,
    std::vector<std::vector<DetailedSeg*> >& candidates

) {
  stack.push_back(segPtr);
  int rowId = segPtr->getRowId();

  if (rowId < top) {
    ++rowId;
    for (size_t s = 0; s < m_segsInRow[rowId].size(); s++) {
      segPtr = m_segsInRow[rowId][s];
      double overlap =
          std::min(xmax, (double)segPtr->getMaxX()) - std::max(xmin, (double)segPtr->getMinX());

      if (overlap >= 1.0e-3) {
        // Must find the reduced X-interval.
        double xl = std::max(xmin, (double)segPtr->getMinX());
        double xr = std::min(xmax, (double)segPtr->getMaxX());
        findClosestSpanOfSegmentsDfs(ndi, segPtr, xl, xr, bot, top, stack,
                                     candidates);
      }
    }
  } else {
    // Reaching this point should imply that we have a consecutive set of
    // segments which is potentially valid for placing the cell.
    int spanned = top - bot + 1;
    if (stack.size() != spanned) {
      internalError( "Multi-height cell spans an incorrect number of segments" );
    }
    candidates.push_back(stack);
  }
  stack.pop_back();
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
bool DetailedMgr::findClosestSpanOfSegments(
    Node* nd, std::vector<DetailedSeg*>& segments) {
  // Intended for multi-height cells...  Finds the number of rows the cell
  // spans and then attempts to find a vector of segments (in different
  // rows) into which the cell can be assigned.

  int spanned = m_arch->getCellHeightInRows(nd);
  if(spanned <= 1) {
    return false;
  }

  double disp1 = std::numeric_limits<double>::max();
  double disp2 = std::numeric_limits<double>::max();

  std::vector<std::vector<DetailedSeg*> > candidates;
  std::vector<DetailedSeg*> stack;

  std::vector<DetailedSeg*> best1;  // closest.
  std::vector<DetailedSeg*> best2;  // closest that fits.

  // The efficiency of this is not good.  The information about overlapping
  // segments for multi-height cells could easily be precomputed for efficiency.
  bool flip = false;
  for (int r = 0; r < m_arch->getNumRows(); r++) {
    // XXX: NEW! Check power compatibility of this cell with the row.  A
    // call to this routine will check both the bottom and the top rows
    // for power compatibility.
    if (!m_arch->power_compatible(nd, m_arch->getRow(r), flip)) {
      continue;
    }

    // Scan the segments in this row and look for segments in the required
    // number of rows above and below that result in non-zero interval.
    int b = r;
    int t = r + spanned - 1;
    if (t >= m_arch->getRows().size()) {
      continue;
    }

    for (size_t sb = 0; sb < m_segsInRow[b].size(); sb++) {
      DetailedSeg* segPtr = m_segsInRow[b][sb];

      candidates.clear();
      stack.clear();

      findClosestSpanOfSegmentsDfs(nd, segPtr, segPtr->getMinX(), segPtr->getMaxX(),
                                   b, t, stack, candidates);
      if (candidates.size() == 0) {
        continue;
      }

      // Evaluate the candidate segments.  Determine the distance of the bottom
      // of the node to the bottom of the first segment.  Determine the overlap
      // in the interval in the X-direction and determine the required distance.

      for (size_t i = 0; i < candidates.size(); i++) {
        // NEW: All of the segments must have the same region ID and that region
        // ID must be the same as the region ID of the cell.  If not, then we
        // are going to violate a fence region constraint.
        bool regionsOkay = true;
        for (size_t j = 0; j < candidates[i].size(); j++) {
          DetailedSeg* segPtr = candidates[i][j];
          if (segPtr->getRegId() != nd->getRegionId()) {
            regionsOkay = false;
          }
        }

        // XXX: Should region constraints be hard or soft?  If hard, there is
        // more change for failure!
        if (!regionsOkay) {
          continue;
        }

        DetailedSeg* segPtr = candidates[i][0];

        int xmin = segPtr->getMinX();
        int xmax = segPtr->getMaxX();
        for (size_t j = 0; j < candidates[i].size(); j++) {
          segPtr = candidates[i][j];
          xmin = std::max(xmin, segPtr->getMinX());
          xmax = std::min(xmax, segPtr->getMaxX());
        }
        int width = xmax-xmin;

        // Work with bottom edge.
        double ymin = m_arch->getRow(segPtr->getRowId())->getBottom();
        double dy = std::fabs(nd->getBottom() - ymin);

        // Still work with cell center.
        double ww = std::min(nd->getWidth(), width);
        double lx = xmin+0.5*ww;
        double rx = xmax-0.5*ww;
        double xc = nd->getLeft()+0.5*nd->getWidth();
        double xx = std::max(lx, std::min(rx, xc));
        double dx = std::fabs(xc - xx);

        if (best1.size() == 0 || (dx + dy < disp1)) {
          if (1) {
            best1 = candidates[i];
            disp1 = dx + dy;
          }
        }
        if (best2.size() == 0 || (dx + dy < disp2)) {
          if (nd->getWidth() <= width + 1.0e-3) {
            best2 = candidates[i];
            disp2 = dx + dy;
          }
        }
      }
    }
  }

  segments.erase(segments.begin(), segments.end());
  if (best2.size() != 0) {
    segments = best2;
    return true;
  }
  if (best1.size() != 0) {
    segments = best1;
    return true;
  }

  return false;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void DetailedMgr::assignCellsToSegments(std::vector<Node*>& nodesToConsider) {
  // For the provided list of cells which are assumed movable, assign those
  // cells to segments.
  //
  // XXX: Multi height cells are assigned to multiple rows!  In other words,
  // a cell can exist in multiple rows.

  // Assign cells to segments.
  int nAssigned = 0;
  double movementX = 0.;
  double movementY = 0.;
  for (int i = 0; i < nodesToConsider.size(); i++) {
    Node* nd = nodesToConsider[i];

    int nRowsSpanned = m_arch->getCellHeightInRows(nd);

    if (nRowsSpanned == 1) {
      // Single height.
      DetailedSeg* segPtr = findClosestSegment(nd);
      if (segPtr == 0) {
        internalError("Unable to assign single height cell to segment" );
      }

      int rowId = segPtr->getRowId();
      int segId = segPtr->getSegId();

      // Add to segment.
      addCellToSegment(nd, segId);
      ++nAssigned;

      // Move the cell's position into the segment.  Use left edge.
      int x1 = segPtr->getMinX();
      int x2 = segPtr->getMaxX() - nd->getWidth();
      int xx = std::max(x1, std::min(x2, nd->getLeft()));
      int yy = m_arch->getRow(rowId)->getBottom();

      movementX += std::abs(nd->getLeft() - xx);
      movementY += std::abs(nd->getBottom() - yy);

      nd->setLeft(xx);
      nd->setBottom(yy);
    } else {
      // Multi height.
      std::vector<DetailedSeg*> segments;
      if (!findClosestSpanOfSegments(nd, segments)) {
        internalError("Unable to assign multi-height cell to segment" );
      } else {
        if (segments.size() != nRowsSpanned) {
          internalError("Unable to assign multi-height cell to segment" );
        }
        // NB: adding a cell to a segment does _not_ change its position.
        DetailedSeg* segPtr = segments[0];
        int xmin = segPtr->getMinX();
        int xmax = segPtr->getMaxX();
        for (size_t s = 0; s < segments.size(); s++) {
          segPtr = segments[s];
          xmin = std::max(xmin, segPtr->getMinX());
          xmax = std::min(xmax, segPtr->getMaxX());
          addCellToSegment(nd, segments[s]->getSegId());
        }
        ++nAssigned;

        segPtr = segments[0];
        int rowId = segPtr->getRowId();

        // Work with left edge and bottom edge.
        int x1 = xmin;
        int x2 = xmax-nd->getWidth();
        int xx = std::max(x1, std::min(x2, nd->getLeft()));
        int yy = m_arch->getRow(rowId)->getBottom();

        movementX += std::abs(nd->getLeft() - xx);
        movementY += std::abs(nd->getBottom() - yy);

        nd->setLeft(xx);
        nd->setBottom(yy);
      }
    }
  }
  m_logger->info(DPO, 310,
                 "Assigned {:d} cells into segments.  Movement in X-direction "
                 "is {:f}, movement in Y-direction is {:f}.",
                 nAssigned, movementX, movementY);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void DetailedMgr::removeCellFromSegment(Node* nd, int seg) {
  // Removing a node from a segment means a few things...  It means: 1) removing
  // it from the cell list for the segment; 2) removing its width from the
  // segment utilization; 3) updating the required gaps between cells in the
  // segment.

  int width = (int)std::ceil(nd->getWidth());

  std::vector<Node*>::iterator it =
      std::find(m_cellsInSeg[seg].begin(), m_cellsInSeg[seg].end(), nd);
  if (m_cellsInSeg[seg].end() == it) {
    // Should not happen.
    internalError("Cell not found in expected segment" );
  }

  // Remove this segment from the reverse map.
  std::vector<DetailedSeg*>::iterator its =
      std::find(m_reverseCellToSegs[nd->getId()].begin(),
                m_reverseCellToSegs[nd->getId()].end(), m_segments[seg]);
  if (m_reverseCellToSegs[nd->getId()].end() == its) {
    // Should not happen.
    internalError("Cannot find segment for cell" );
  }
  m_reverseCellToSegs[nd->getId()].erase(its);

  m_cellsInSeg[seg].erase(it);                // Removes the cell...
  m_segments[seg]->remUtil(width);   // Removes the utilization...
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void DetailedMgr::addCellToSegment(Node* nd, int seg) {
  // Adding a node to a segment means a few things...  It means:
  // 1) adding it to the SORTED cell list for the segment;
  // 2) adding its width to the segment utilization;
  // 3) adding the required gaps between cells in the segment.

  // Need to figure out where the cell goes in the sorted list...

  double x = nd->getLeft()+0.5*nd->getWidth();
  int width = (int)std::ceil(nd->getWidth());
  std::vector<Node*>::iterator it =
      std::lower_bound(m_cellsInSeg[seg].begin(), m_cellsInSeg[seg].end(),
                       x, compareNodesX());
  if (it == m_cellsInSeg[seg].end()) {
    // Cell is at the end of the segment.
    m_cellsInSeg[seg].push_back(nd);            // Add the cell...
    m_segments[seg]->addUtil(width);  // Adds the utilization...
  } else {
    m_cellsInSeg[seg].insert(it, nd);           // Adds the cell...
    m_segments[seg]->addUtil(width);  // Adds the utilization...
  }

  std::vector<DetailedSeg*>::iterator its =
      std::find(m_reverseCellToSegs[nd->getId()].begin(),
                m_reverseCellToSegs[nd->getId()].end(), m_segments[seg]);
  if (m_reverseCellToSegs[nd->getId()].end() != its) {
    internalError("Segment already present in cell to segment map");
  }
  int spanned = m_arch->getCellHeightInRows(nd);
  if (m_reverseCellToSegs[nd->getId()].size() >= spanned) {
    internalError("Cell to segment map incorrectly sized");
  }
  m_reverseCellToSegs[nd->getId()].push_back(m_segments[seg]);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void DetailedMgr::recordOriginalPositions() {
  m_origBottom.resize(m_network->getNumNodes());
  m_origLeft.resize(m_network->getNumNodes());
  for (int i = 0; i < m_network->getNumNodes() ; i++) {
    Node* nd = m_network->getNode(i);
    m_origBottom[nd->getId()] = nd->getBottom();
    m_origLeft[nd->getId()] = nd->getLeft();
  }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void DetailedMgr::restoreOriginalPositions() {
  for (int i = 0; i < m_network->getNumNodes() ; i++) {
    Node* nd = m_network->getNode(i);
    nd->setBottom(m_origBottom[nd->getId()]);
    nd->setLeft(m_origLeft[nd->getId()]);
  }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
double DetailedMgr::measureMaximumDisplacement(
    double& maxX, double& maxY, 
    int& violatedX, int& violatedY) {
  // Measure some things about displacement from original
  // positions.
  maxX = 0.;
  maxY = 0.;
  violatedX = 0;
  violatedY = 0;

  double maxL1 = 0.;
  for (int i = 0; i < m_network->getNumNodes() ; i++) {
    Node* nd = m_network->getNode(i);
    if (nd->isTerminal() || nd->isTerminalNI() || nd->isFixed()) {
      continue;
    }

    double dy = std::fabs(nd->getBottom()-m_origBottom[nd->getId()]);
    double dx = std::fabs(nd->getLeft()-m_origLeft[nd->getId()]);
    maxL1 = std::max(maxL1,dx+dy);
    maxX = std::max(maxX,std::ceil(dx));
    maxY = std::max(maxY,std::ceil(dy));
    if (dx > (double)m_maxDispX) {
      ++violatedX;
    }
    if (dy > (double)m_maxDispY) {
      ++violatedY;
    }
  }
  return maxL1;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void DetailedMgr::setupObstaclesForDrc() {
  // Setup rectangular obstacles for short and pin access checks.  Do only as
  // rectangles per row and per layer.  I had used rtrees, but it wasn't working
  // any better.
  double xmin, xmax, ymin, ymax;

  m_obstacles.resize(m_arch->getRows().size());

  for (int row_id = 0; row_id < m_arch->getRows().size(); row_id++) {
    m_obstacles[row_id].resize(m_rt->m_num_layers);

    double originX = m_arch->getRow(row_id)->getLeft();
    double siteSpacing = m_arch->getRow(row_id)->getSiteSpacing();
    int numSites = m_arch->getRow(row_id)->getNumSites();

    // Blockages relevant to this row...
    for (int layer_id = 0; layer_id < m_rt->m_num_layers; layer_id++) {
      m_obstacles[row_id][layer_id].clear();

      std::vector<Rectangle>& rects = m_rt->m_layerBlockages[layer_id];
      for (int b = 0; b < rects.size(); b++) {
        // Extract obstacles which interfere with this row only.
        xmin = originX;
        xmax = originX + numSites * siteSpacing;
        ymin = m_arch->getRow(row_id)->getBottom();
        ymax = m_arch->getRow(row_id)->getTop();

        if (rects[b].xmax() <= xmin) continue;
        if (rects[b].xmin() >= xmax) continue;
        if (rects[b].ymax() <= ymin) continue;
        if (rects[b].ymin() >= ymax) continue;

        m_obstacles[row_id][layer_id].push_back(rects[b]);
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void DetailedMgr::collectSingleHeightCells() {
  // Routine to collect only the movable single height cells.
  //
  // XXX: This code also shifts cells to ensure that they are within the
  // placement area.  It also lines the cell up with its bottom row by
  // assuming rows are stacked continuously one on top of the other which
  // may or may not be a correct assumption.
  // Do I need to do any of this really?????????????????????????????????

  m_singleHeightCells.erase(m_singleHeightCells.begin(),
                            m_singleHeightCells.end());
  m_singleRowHeight = m_arch->getRow(0)->getHeight();
  m_numSingleHeightRows = m_arch->getNumRows();

  for (int i = 0; i < m_network->getNumNodes() ; i++) {
    Node* nd = m_network->getNode(i);
  
    if (nd->isTerminal() || nd->isTerminalNI() || nd->isFixed()) {
      continue;
    }
    if (m_arch->isMultiHeightCell(nd)) {
      continue;
    }

    m_singleHeightCells.push_back(nd);
  }
  m_logger->info(DPO, 318, "Collected {:d} single height cells.", 
                 m_singleHeightCells.size());
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void DetailedMgr::collectMultiHeightCells() {
  // Routine to collect only the movable multi height cells.
  //
  // XXX: This code also shifts cells to ensure that they are within the
  // placement area.  It also lines the cell up with its bottom row by
  // assuming rows are stacked continuously one on top of the other which
  // may or may not be a correct assumption.
  // Do I need to do any of this really?????????????????????????????????

  m_multiHeightCells.erase(m_multiHeightCells.begin(),
                           m_multiHeightCells.end());
  // Just in case...  Make the matrix for holding multi-height cells at
  // least large enough to hold single height cells (although we don't
  // even bothering storing such cells in this matrix).
  m_multiHeightCells.resize(2);
  for (size_t i = 0; i < m_multiHeightCells.size(); i++) {
    m_multiHeightCells[i] = std::vector<Node*>();
  }
  m_singleRowHeight = m_arch->getRow(0)->getHeight();
  m_numSingleHeightRows = m_arch->getNumRows();

  int m_numMultiHeightCells = 0;
  for (int i = 0; i < m_network->getNumNodes() ; i++) {
    Node* nd = m_network->getNode(i);

    if (nd->isTerminal() || nd->isTerminalNI() || nd->isFixed() || m_arch->isSingleHeightCell(nd)) {
      continue;
    }

    int nRowsSpanned = m_arch->getCellHeightInRows(nd);

    if (nRowsSpanned >= m_multiHeightCells.size()) {
      m_multiHeightCells.resize(nRowsSpanned + 1, std::vector<Node*>());
    }
    m_multiHeightCells[nRowsSpanned].push_back(nd);
    ++m_numMultiHeightCells;
  }
  for (size_t i = 0; i < m_multiHeightCells.size(); i++) {
    if (m_multiHeightCells[i].size() == 0) {
      continue;
    }
    m_logger->info(DPO, 319, "Collected {:d} multi-height cells spanning {:d} rows.", 
        m_multiHeightCells[i].size(), i);
  }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void DetailedMgr::collectFixedCells() {
  // Fixed cells are used only to create blockages which, in turn, are used to
  // create obstacles.  Obstacles are then used to create the segments into
  // which cells can be placed.
  //
  // AAK: 01-dec-2021.  I noticed an error with respect to bookshelf format
  // and the handling of TERMINAL_NI cells.  One can place movable cells on
  // top of these sorts of terminals.  Therefore, they should NOT be considered
  // as fixed, at least with respect to creating blockages.

  m_fixedCells.erase(m_fixedCells.begin(), m_fixedCells.end());

  // Insert fixed items, shapes AND macrocells.
  for (int i = 0; i < m_network->getNumNodes() ; i++) {
    Node* nd = m_network->getNode(i);

    if (!nd->isFixed()) {
      // Not fixed, so skip.
      continue;
    }

    if (nd->isTerminalNI()) {
      // Skip these since we can place over them.
      continue;
    }

    // If a cell is fixed, but defined by shapes,
    // then skip it.  We _will_ encounter the 
    // shapes at some point.
    if (nd->isDefinedByShapes()) {
      continue;
    }

    m_fixedCells.push_back(nd);
  }

  m_logger->info(DPO, 320, "Collected {:d} fixed cells (excluded terminal_NI).", m_fixedCells.size());
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void DetailedMgr::collectWideCells() {
  // This is sort of a hack.  Some standard cells might be extremely wide and
  // based on how we set up segments (e.g., to take into account blockages of
  // different sorts), we might not be able to find a segment wide enough to
  // accomodate the cell.  In this case, we will not be able to resolve a bunch
  // of problems.
  //
  // My current solution is to (1) detect such cells; (2) recreate the segments
  // without blockages; (3) insert the wide cells into segments; (4) fix the
  // wide cells; (5) recreate the entire problem with the wide cells considered
  // as fixed.

  m_wideCells.erase(m_wideCells.begin(), m_wideCells.end());
  for (int s = 0; s < m_segments.size(); s++) {
    DetailedSeg* curr = m_segments[s];

    std::vector<Node*>& nodes = m_cellsInSeg[s];
    for (int k = 0; k < nodes.size(); k++) {
      Node* ndi = nodes[k];
      if (ndi->getWidth() > curr->getMaxX() - curr->getMinX()) {
        m_wideCells.push_back(ndi);
      }
    }
  }
  m_logger->info(DPO, 321, "Collected {:d} wide cells.", m_wideCells.size());
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void DetailedMgr::cleanup() {
  // Various cleanups.
  for (int i = 0; i < m_wideCells.size(); i++) {
    Node* ndi = m_wideCells[i];
    ndi->setFixed(NodeFixed_NOT_FIXED);
  }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
int DetailedMgr::checkOverlapInSegments() {
  // Scan segments and check if adjacent cells overlap.  Do not 
  // consider spacing or padding in this check.

  std::vector<Node*> temp;
  temp.reserve(m_network->getNumNodes() );

  int err_n = 0;
  // The following is for some printing if we need help finding a bug.
  // I don't want to print all the potential errors since that could
  // be too overwhelming.
  for (int s = 0; s < m_segments.size(); s++) {
    int xmin = m_segments[s]->getMinX();
    int xmax = m_segments[s]->getMaxX();

    // To be safe, gather cells in each segment and re-sort them.
    temp.erase(temp.begin(), temp.end());
    for (int j = 0; j < m_cellsInSeg[s].size(); j++) {
      Node* ndj = m_cellsInSeg[s][j];
      temp.push_back(ndj);
    }
    std::sort(temp.begin(), temp.end(), compareNodesX());

    for (int j = 1; j < temp.size(); j++) {
      Node* ndi = temp[j-1];
      Node* ndj = temp[j];

      int ri = ndi->getRight();
      int lj = ndj->getLeft();

      if (ri > lj) {
        // Overlap.
        err_n++;
      }
    }
    for (int j = 0; j < temp.size(); j++) {
      Node* ndi = temp[j];
      if (ndi->getLeft() < xmin || ndi->getRight() > xmax) {
        // Out of range.
        err_n++;
      }
    }
  }

  m_logger->info(DPO, 311, "Found {:d} overlaps between adjacent cells.",
                 err_n);
  return err_n;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
int DetailedMgr::checkEdgeSpacingInSegments() {
  // Check for spacing violations according to the spacing table.  Note
  // that there might not be a spacing table in which case we will
  // return no errors.  I should also check for padding errors although
  // we might not have any paddings either! :).

  std::vector<Node*> temp;
  temp.reserve(m_network->getNumNodes() );

  int dummyPadding = 0;
  int rightPadding = 0;
  int leftPadding = 0;

  int err_n = 0;
  int err_p = 0;
  for (int s = 0; s < m_segments.size(); s++) {

    // To be safe, gather cells in each segment and re-sort them.
    temp.erase(temp.begin(), temp.end());
    for (int j = 0; j < m_cellsInSeg[s].size(); j++) {
      Node* ndj = m_cellsInSeg[s][j];
      temp.push_back(ndj);
    }
    std::sort(temp.begin(), temp.end(), compareNodesL());

    for (int j = 1; j < temp.size(); j++) {
      Node* ndl = temp[j - 1];
      Node* ndr = temp[j];

      double rlx_l = ndl->getRight();
      double llx_r = ndr->getLeft();

      double gap = llx_r - rlx_l;

      double spacing = m_arch->getCellSpacingUsingTable(ndl->getRightEdgeType(),
                                                        ndr->getLeftEdgeType());

      m_arch->getCellPadding(ndl, dummyPadding, rightPadding);
      m_arch->getCellPadding(ndr, leftPadding, dummyPadding);
      int padding = leftPadding + rightPadding;

      if (!(gap >= spacing - 1.0e-3)) {
        ++err_n;
      }
      if (!(gap >= padding - 1.0e-3)) {
        ++err_p;
      }
    }
  }

  m_logger->info(
      DPO, 312,
      "Found {:d} edge spacing violations and {:d} padding violations.", err_n,
      err_p);

  return err_n + err_p;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
int DetailedMgr::checkRegionAssignment() {
  // Check cells are assigned (within) their proper regions.  This is sort
  // of a hack/cheat.  We assume that we have set up the segments correctly
  // and that all cells are in segments.  Multi-height cells can be in
  // multiple segments.
  //
  // Therefore, if we scan the segments and the cells have a region ID that
  // matches the region ID for the segment, the cell must be within its
  // region.  Note: This is not true if the cell is somehow outside of its
  // assigned segments.  However, that issue would be caught when checking
  // the segments themselves.

  std::vector<Node*> temp;
  temp.reserve(m_network->getNumNodes() );

  int err_n = 0;
  for (int s = 0; s < m_segments.size(); s++) {

    // To be safe, gather cells in each segment and re-sort them.
    temp.erase(temp.begin(), temp.end());
    for (int j = 0; j < m_cellsInSeg[s].size(); j++) {
      Node* ndj = m_cellsInSeg[s][j];
      temp.push_back(ndj);
    }
    std::sort(temp.begin(), temp.end(), compareNodesL());

    for (int j = 0; j < temp.size(); j++) {
      Node* ndi = temp[j];
      if (ndi->getRegionId() != m_segments[s]->getRegId()) {
        ++err_n;
      }
    }
  }

  m_logger->info(DPO, 313, "Found {:d} cells in wrong regions.", err_n);

  return err_n;
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
int DetailedMgr::checkSiteAlignment() {
  // Ensure that the left edge of each cell is aligned with a site.  We only
  // consider cells that are within segments.
  int err_n = 0;

  double singleRowHeight = getSingleRowHeight();
  int nCellsInSegments = 0;
  int nCellsNotInSegments = 0;
  for (int i = 0; i < m_network->getNumNodes() ; i++) {
    Node* nd = m_network->getNode(i); 

    if (nd->isTerminal() || nd->isTerminalNI() || nd->isFixed()) {
      continue;
    }

    double xl = nd->getLeft();
    double yb = nd->getBottom();

    // Determine the spanned rows. XXX: Is this strictly correct?  It
    // assumes rows are continuous and that the bottom row lines up
    // with the bottom of the architecture.
    int rb = (int)((yb - m_arch->getMinY()) / singleRowHeight);
    int spanned = (int)((nd->getHeight() / singleRowHeight) + 0.5);
    int rt = rb + spanned - 1;

    if (m_reverseCellToSegs[nd->getId()].size() == 0) {
      ++nCellsNotInSegments;
      continue;
    } else if (m_reverseCellToSegs[nd->getId()].size() != spanned) {
      internalError( "Reverse cell map incorrectly sized." );
    }
    ++nCellsInSegments;

    if (rb < 0 || rt >= m_arch->getRows().size()) {
      // Either off the top of the bottom of the chip, so this is not
      // exactly an alignment problem, but still a problem so count it.
      ++err_n;
    }
    rb = std::max(rb, 0);
    rt = std::min(rt, (int)m_arch->getRows().size() - 1);

    for (int r = rb; r <= rt; r++) {
      double originX = m_arch->getRow(r)->getLeft();
      double siteSpacing = m_arch->getRow(r)->getSiteSpacing();

      // XXX: Should I check the site to the left and right to avoid rounding
      // errors???
      int sid = (int)(((xl - originX) / siteSpacing) + 0.5);
      double xt = originX + sid * siteSpacing;
      if (std::fabs(xl - xt) > 1.0e-3) {
        ++err_n;
      }
    }
  }
  m_logger->info(DPO, 314, "Found {:d} site alignment problems.", err_n);
  return err_n;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
int DetailedMgr::checkRowAlignment() {
  // Ensure that the bottom of each cell is aligned with a row.
  int err_n = 0;

  for (int i = 0; i < m_network->getNumNodes() ; i++) {
    Node* nd = m_network->getNode(i);

    if (nd->isTerminal() || nd->isTerminalNI() || nd->isFixed()) {
      continue;
    }

    int rb = m_arch->find_closest_row(nd->getBottom());
    int rt = rb + m_arch->getCellHeightInRows(nd)-1;
    if (rb < 0 || rt >= m_arch->getRows().size()) {
      // Apparently, off the bottom or top of hte chip.
      ++err_n;
      continue;
    }
    else {
      int ymin = m_arch->getRow(rb)->getBottom();
      int ymax = m_arch->getRow(rt)->getTop();
      if (std::abs(nd->getBottom() - ymin) != 0 || std::abs(nd->getTop() - ymax) != 0) {
        ++err_n;
      }
    }
  }
  m_logger->info(DPO, 315, "Found {:d} row alignment problems.", err_n);
  return err_n;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
double DetailedMgr::getCellSpacing(Node* ndl, Node* ndr,
                                   bool checkPinsOnCells) {
  // Compute any required spacing between cells.  This could be from an edge
  // type rule, or due to adjacent pins on the cells.  Checking pins on cells is
  // more time consuming.

  if (ndl == 0 || ndr == 0) {
    return 0.0;
  }
  double spacing1 = m_arch->getCellSpacing(ndl, ndl);
  if (!checkPinsOnCells) {
    return spacing1;
  }
  double spacing2 = 0.0;
  {
    Pin* pinl = 0;
    Pin* pinr = 0;

    // Right-most pin on the left cell.
    for (int i = 0; i < ndl->getPins().size(); i++) {
      Pin* pin = ndl->getPins()[i];
      if (pinl == 0 || pin->getOffsetX() > pinl->getOffsetX()) {
        pinl = pin;
      }
    }

    // Left-most pin on the right cell.
    for (int i = 0; i < ndr->getPins().size(); i++) {
      Pin* pin = ndr->getPins()[i];
      if (pinr == 0 || pin->getOffsetX() < pinr->getOffsetX()) {
        pinr = pin;
      }
    }
    // If pins on the same layer, do something.
    if (pinl != 0 && pinr != 0 && pinl->getPinLayer() == pinr->getPinLayer()) {
      // Determine the spacing requirements between these two pins.   Then,
      // translate this into a spacing requirement between the two cells.  XXX:
      // Since it is implicit that the cells are in the same row, we can
      // determine the widest pin and the parallel run length without knowing
      // the actual location of the cells...  At least I think so...

      double xmin1 = pinl->getOffsetX() - 0.5 * pinl->getPinWidth();
      double xmax1 = pinl->getOffsetX() + 0.5 * pinl->getPinWidth();
      double ymin1 = pinl->getOffsetY() - 0.5 * pinl->getPinHeight();
      double ymax1 = pinl->getOffsetY() + 0.5 * pinl->getPinHeight();

      double xmin2 = pinr->getOffsetX() - 0.5 * pinr->getPinWidth();
      double xmax2 = pinr->getOffsetX() + 0.5 * pinr->getPinWidth();
      double ymin2 = pinr->getOffsetY() - 0.5 * pinr->getPinHeight();
      double ymax2 = pinr->getOffsetY() + 0.5 * pinr->getPinHeight();

      double ww = std::max(std::min(ymax1 - ymin1, xmax1 - xmin1),
                           std::min(ymax2 - ymin2, xmax2 - xmin2));
      double py =
          std::max(0.0, std::min(ymax1, ymax2) - std::max(ymin1, ymin2));

      spacing2 = m_rt->get_spacing(pinl->getPinLayer(), ww, py);
      double gapl = (+0.5 * ndl->getWidth()) - xmax1;
      double gapr = xmin2 - (-0.5 * ndr->getWidth());
      spacing2 = std::max(0.0, spacing2 - gapl - gapr);

      if (spacing2 > spacing1) {
        // The spacing requirement due to the routing layer is larger than the
        // spacing requirement due to the edge constraint.  Interesting.
        ;
      }
    }
  }
  return std::max(spacing1, spacing2);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void DetailedMgr::getSpaceAroundCell(int seg, int ix, double& space,
                                     double& larger, int limit) {
  // XXX: UPDATE TO ACCOMMODATE MULTI-HEIGHT CELLS.  Likely requires using the
  // bottom of the cell instead of the center of the cell.  Need to assign a
  // cell to multiple segments.

  Node* ndi = m_cellsInSeg[seg][ix];
  Node* ndj = 0;
  Node* ndk = 0;

  int n = (int)m_cellsInSeg[seg].size();
  double xmin = m_segments[seg]->getMinX();
  double xmax = m_segments[seg]->getMaxX();

  // Space to the immediate left and right of the cell.
  double space_left = 0;
  if (ix == 0) {
    space_left += (ndi->getLeft()) - xmin;
  } else {
    --ix;
    ndj = m_cellsInSeg[seg][ix];

    space_left += (ndi->getLeft()) -
                  (ndj->getRight());
    ++ix;
  }

  double space_right = 0;
  if (ix == n - 1) {
    space_right += xmax - (ndi->getRight());
  } else {
    ++ix;
    ndj = m_cellsInSeg[seg][ix];
    space_right += (ndj->getLeft()) -
                   (ndi->getRight());
  }
  space = space_left + space_right;

  // Space three cells 'limit' cells to the left and 'limit' cells to the right.
  if (ix < limit) {
    ndj = m_cellsInSeg[seg][0];
    larger = ndj->getLeft() - xmin;
  } else {
    larger = 0;
  }
  for (int j = std::max(0, ix - limit); j <= std::min(n - 1, ix + limit); j++) {
    ndj = m_cellsInSeg[seg][j];
    if (j < n - 1) {
      ndk = m_cellsInSeg[seg][j + 1];
      larger += (ndk->getLeft()) -
                (ndj->getRight());
    } else {
      larger += xmax - (ndj->getRight());
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void DetailedMgr::getSpaceAroundCell(int seg, int ix, double& space_left,
                                     double& space_right, double& large_left,
                                     double& large_right, int limit) {
  // XXX: UPDATE TO ACCOMMODATE MULTI-HEIGHT CELLS.  Likely requires using the
  // bottom of the cell instead of the center of the cell.  Need to assign a
  // cell to multiple segments.

  Node* ndi = m_cellsInSeg[seg][ix];
  Node* ndj = 0;
  Node* ndk = 0;

  int n = (int)m_cellsInSeg[seg].size();
  double xmin = m_segments[seg]->getMinX();
  double xmax = m_segments[seg]->getMaxX();

  // Space to the immediate left and right of the cell.
  space_left = 0;
  if (ix == 0) {
    space_left += (ndi->getLeft()) - xmin;
  } else {
    --ix;
    ndj = m_cellsInSeg[seg][ix];
    space_left += (ndi->getLeft()) -
                  (ndj->getRight());
    ++ix;
  }

  space_right = 0;
  if (ix == n - 1) {
    space_right += xmax - (ndi->getRight());
  } else {
    ++ix;
    ndj = m_cellsInSeg[seg][ix];
    space_right += (ndj->getLeft()) -
                   (ndi->getRight());
  }
  // Space three cells 'limit' cells to the left and 'limit' cells to the right.
  large_left = 0;
  if (ix < limit) {
    ndj = m_cellsInSeg[seg][0];
    large_left = ndj->getLeft() - xmin;
  }
  for (int j = std::max(0, ix - limit); j < ix; j++) {
    ndj = m_cellsInSeg[seg][j];
    ndk = m_cellsInSeg[seg][j + 1];
    large_left += (ndk->getLeft()) -
                  (ndj->getRight());
  }
  large_right = 0;
  for (int j = ix; j <= std::min(n - 1, ix + limit); j++) {
    ndj = m_cellsInSeg[seg][j];
    if (j < n - 1) {
      ndk = m_cellsInSeg[seg][j + 1];
      large_right += (ndk->getLeft()) -
                     (ndj->getRight());
    } else {
      large_right += xmax - (ndj->getRight());
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void DetailedMgr::findRegionIntervals(
    int regId,
    std::vector<std::vector<std::pair<double, double> > >& intervals) {
  // Find intervals within each row that are spanned by the specified region.
  // We ignore the default region 0, since it is "everywhere".

  if (regId < 1 || regId >= m_arch->getRegions().size() ||
      m_arch->getRegion(regId)->getId() != regId) {
    internalError("Improper region id");
  }
  Architecture::Region* regPtr = m_arch->getRegion(regId);

  // Initialize.
  intervals.clear();
  intervals.resize(m_numSingleHeightRows);
  for (int i = 0; i < intervals.size(); i++) {
    intervals[i] = std::vector<std::pair<double, double> >();
  }

  // Look at the rectangles within the region.
  for (const Rectangle_i& rect : regPtr->getRects()) {
    double xmin = rect.xmin();
    double xmax = rect.xmax();
    double ymin = rect.ymin();
    double ymax = rect.ymax();

    for (int r = 0; r < m_numSingleHeightRows; r++) {
      double lb = m_arch->getMinY() + r * m_singleRowHeight;
      double ub = lb + m_singleRowHeight;

      if (ymax >= ub && ymin <= lb) {
        // Blockage overlaps with the entire row span in the Y-dir... Sites
        // are possibly completely covered!

        double originX = m_arch->getRow(r)->getLeft();
        double siteSpacing = m_arch->getRow(r)->getSiteSpacing();

        int i0 = (int)std::floor((xmin - originX) / siteSpacing);
        int i1 = (int)std::floor((xmax - originX) / siteSpacing);
        if (originX + i1 * siteSpacing != xmax) ++i1;

        if (i1 > i0) {
          intervals[r].push_back(std::pair<double, double>(
              originX + i0 * siteSpacing, originX + i1 * siteSpacing));
        }
      }
    }
  }

  // Sort intervals and merge.  We merge, since the region might have been
  // defined with rectangles that touch (so it is "wrong" to create an
  // artificial boundary).
  for (int r = 0; r < m_numSingleHeightRows; r++) {
    if (intervals[r].size() == 0) {
      continue;
    }

    // Sort to get intervals left to right.
    std::sort(intervals[r].begin(), intervals[r].end(), compareBlockages());

    std::stack<std::pair<double, double> > s;
    s.push(intervals[r][0]);
    for (int i = 1; i < intervals[r].size(); i++) {
      std::pair<double, double> top = s.top();  // copy.
      if (top.second < intervals[r][i].first) {
        s.push(intervals[r][i]);  // new interval.
      } else {
        if (top.second < intervals[r][i].second) {
          top.second = intervals[r][i].second;  // extend interval.
        }
        s.pop();      // remove old.
        s.push(top);  // expanded interval.
      }
    }

    intervals[r].erase(intervals[r].begin(), intervals[r].end());
    while (!s.empty()) {
      std::pair<double, double> temp = s.top();  // copy.
      intervals[r].push_back(temp);
      s.pop();
    }

    // Sort to get them left to right.
    std::sort(intervals[r].begin(), intervals[r].end(), compareBlockages());
  }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/*
void DetailedMgr::removeSegmentOverlapSingle(int regId) {
  // Loops over the segments.  Finds intervals of single height cells and
  // attempts to do a min shift to remove overlap.

  for (size_t s = 0; s < this->m_segments.size(); s++) {
    DetailedSeg* segPtr = this->m_segments[s];

    if (!(segPtr->getRegId() == regId || regId == -1)) {
      continue;
    }

    int segId = segPtr->getSegId();
    int rowId = segPtr->getRowId();

    int right = segPtr->getMaxX();
    int left = segPtr->getMinX();

    std::vector<Node*> nodes;
    for (size_t n = 0; n < this->m_cellsInSeg[segId].size(); n++) {
      Node* ndi = this->m_cellsInSeg[segId][n];

      int spanned = m_arch->getCellHeightInRows(ndi);
      if (spanned == 1) {
        ndi->setBottom(m_arch->getRow(rowId)->getBottom());
      }

      if (spanned == 1) {
        nodes.push_back(ndi);
      } else {
        // Multi-height.
        if (nodes.size() == 0) {
          left = ndi->getRight();
        } else {
          right = ndi->getLeft();
          // solve.
          removeSegmentOverlapSingleInner(nodes, left, right, rowId);
          // prepare for next.
          nodes.erase(nodes.begin(), nodes.end());
          left = ndi->getRight();
          right = segPtr->getMaxX();
        }
      }
    }
    if (nodes.size() != 0) {
      // solve.
      removeSegmentOverlapSingleInner(nodes, left, right, rowId);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void DetailedMgr::removeSegmentOverlapSingleInner(std::vector<Node*>& nodes_in,
                                                  int xmin, int xmax,
                                                  int rowId) {
  // Quickly remove overlap between a range of cells in a segment.  Try to
  // satisfy gaps and try to align with sites.

  std::vector<Node*> nodes = nodes_in;
  std::stable_sort(nodes.begin(), nodes.end(), compareNodesX());

  std::vector<double> llx;
  std::vector<double> tmp;
  std::vector<double> wid;

  llx.resize(nodes.size());
  tmp.resize(nodes.size());
  wid.resize(nodes.size());

  double x;
  double originX = m_arch->getRow(rowId)->getLeft();
  double siteSpacing = m_arch->getRow(rowId)->getSiteSpacing();
  int ix;

  double space = xmax - xmin;
  double util = 0.;

  for (int k = 0; k < nodes.size(); k++) {
    util += nodes[k]->getWidth();
  }

  // Get width for each cell.
  for (int i = 0; i < nodes.size(); i++) {
    Node* nd = nodes[i];
    wid[i] = nd->getWidth();
  }

  // Try to get site alignment.  Adjust the left and right into which
  // we are placing cells.  If we don't need to shrink cells, then I
  // think this should work.
  {
    double tot = 0;
    for (int i = 0; i < nodes.size(); i++) {
      tot += wid[i];
    }
    if (space > tot) {
      // Try to fix the left boundary.
      ix = (int)(((xmin)-originX) / siteSpacing);
      if (originX + ix * siteSpacing < xmin) ++ix;
      x = originX + ix * siteSpacing;
      if (xmax - x >= tot + 1.0e-3 && std::fabs(xmin - x) > 1.0e-3) {
        xmin = x;
      }
      space = xmax - xmin;
    }
    if (space > tot) {
      // Try to fix the right boundary.
      ix = (int)(((xmax)-originX) / siteSpacing);
      if (originX + ix * siteSpacing > xmax) --ix;
      x = originX + ix * siteSpacing;
      if (x - xmin >= tot + 1.0e-3 && std::fabs(xmax - x) > 1.0e-3) {
        xmax = x;
      }
      space = xmax - xmin;
    }
  }

  // Try to include the necessary gap as long as we don't exceed the
  // available space.
  for (int i = 1; i < nodes.size(); i++) {
    Node* ndl = nodes[i - 1];
    Node* ndr = nodes[i - 0];

    double gap = m_arch->getCellSpacing(ndl, ndr);
    if (gap != 0.0) {
      if (util + gap <= space) {
        wid[i - 1] += gap;
        util += gap;
      }
    }
  }

  double cell_width = 0.;
  for (int i = 0; i < nodes.size(); i++) {
    cell_width += wid[i];
  }
  if (cell_width > space) {
    // Scale... now things should fit, but will overlap...
    double scale = space / cell_width;
    for (int i = 0; i < nodes.size(); i++) {
      wid[i] *= scale;
    }
  }

  // The position for the left edge of each cell; this position will be
  // site aligned.
  for (int i = 0; i < nodes.size(); i++) {
    Node* nd = nodes[i];
    ix = (int)(((nd->getLeft()) - originX) / siteSpacing);
    llx[i] = originX + ix * siteSpacing;
  }
  // The leftmost position for the left edge of each cell.  Should also
  // be site aligned unless we had to shrink cell widths to make things
  // fit (which means overlap anyway).
  // ix = (int)(((xmin) - originX)/siteSpacing);
  // if( originX + ix*siteSpacing < xmin )
  //    ++ix;
  // x = originX + ix * siteSpacing;
  x = xmin;
  for (int i = 0; i < nodes.size(); i++) {
    tmp[i] = x;
    x += wid[i];
  }

  // The rightmost position for the left edge of each cell.  Should also
  // be site aligned unless we had to shrink cell widths to make things
  // fit (which means overlap anyway).
  // ix = (int)(((xmax) - originX)/siteSpacing);
  // if( originX + ix*siteSpacing > xmax )
  //    --ix;
  // x = originX + ix * siteSpacing;
  x = xmax;
  for (int i = (int)nodes.size() - 1; i >= 0; i--) {
    llx[i] = std::max(tmp[i], std::min(x - wid[i], llx[i]));
    x = llx[i];  // Update rightmost position.
  }
  for (int i = 0; i < nodes.size(); i++) {
    nodes[i]->setLeft(llx[i]);
  }
}
*/

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void DetailedMgr::resortSegments() {
  // Resort the nodes in the segments.  This might be required if we did
  // something to move cells around and broke the ordering.
  for (size_t i = 0; i < m_segments.size(); i++) {
    DetailedSeg* segPtr = m_segments[i];
    resortSegment(segPtr);
  }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void DetailedMgr::resortSegment(DetailedSeg* segPtr) {
  int segId = segPtr->getSegId();
  std::stable_sort(m_cellsInSeg[segId].begin(), m_cellsInSeg[segId].end(),
                   compareNodesX());
  segPtr->setUtil(0.0);
  for (size_t n = 0; n < m_cellsInSeg[segId].size(); n++) {
    Node* ndi = m_cellsInSeg[segId][n];
    int width = (int)std::ceil(ndi->getWidth());
    segPtr->addUtil(width);
  }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void DetailedMgr::removeAllCellsFromSegments() {
  // This routine removes _ALL_ cells from all segments.  It clears all
  // reverse maps and so forth.  Basically, it leaves things as if the
  // segments have all been created, but nothing has been inserted.
  for (size_t i = 0; i < m_segments.size(); i++) {
    DetailedSeg* segPtr = m_segments[i];
    int segId = segPtr->getSegId();
    m_cellsInSeg[segId].erase(m_cellsInSeg[segId].begin(),
                              m_cellsInSeg[segId].end());
    segPtr->setUtil(0.0);
  }
  for (size_t i = 0; i < m_reverseCellToSegs.size(); i++) {
    m_reverseCellToSegs[i].erase(m_reverseCellToSegs[i].begin(),
                                 m_reverseCellToSegs[i].end());
  }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
bool DetailedMgr::alignPos(Node* ndi, int& xi, int xl, int xr) {
  // Given a cell with a target location, xi, determine a 
  // site-aligned position such that the cell falls 
  // within the interval [xl,xr].
  //
  // This routine works with the left edge of the cell.

  int originX = m_arch->getRow(0)->getLeft();
  int siteSpacing = m_arch->getRow(0)->getSiteSpacing();
  int w = ndi->getWidth();

  xr -= w;  // [xl,xr] is now range for left edge of cell.

  // Left edge of cell within [xl,xr] closest to target.
  int xp = std::max(xl, std::min(xr, xi));

  int ix = (xp - originX) / siteSpacing;
  xp = originX + ix * siteSpacing;  // Left edge aligned.

  if (xp < xl) {
    xp += siteSpacing;
  } else if (xp > xr) {
    xp -= siteSpacing;
  }

  if (xp < xl || xp > xr) {
    // Left edge out of range so cell will also be out of range.
    return false;
  }

  // Set new target.
  xi = xp;
  return true;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
bool DetailedMgr::shift(std::vector<Node*>& cells, 
                        std::vector<int>& targetLeft,
                        std::vector<int>& posLeft,
                        int leftLimit, int rightLimit,
                        int segId, int rowId) {
  // Shift a set of ordered cells given target positions.
  // The final site-aligned positions are returned.  Only 
  // works for a vector of single height cells.

  // Note: The segment id is not really required.  The 
  // segment id is only required to be able to get the 
  // origin and site spacing to align cells.

  int originX = m_arch->getRow(rowId)->getLeft();
  int siteSpacing = m_arch->getRow(rowId)->getSiteSpacing();
  int siteWidth = m_arch->getRow(rowId)->getSiteWidth();

  // Number of cells.
  int ncells = (int)cells.size();

  // Sites within the provided range.
  int i0 = (int)((leftLimit - originX) / siteSpacing);
  if (originX + i0 * siteSpacing < leftLimit) {
    ++i0;
  }
  int i1 = (int)((rightLimit - originX) / siteSpacing);
  if (originX + i1 * siteSpacing + siteWidth >= rightLimit) {
    --i1;
  }
  int nsites = i1 - i0 + 1;
  int ii, jj;

  // Get cell widths while accounting for spacing/padding.  We
  // ignore spacing/padding at the ends (should adjust left
  // and right edges prior to calling).  Convert spacing into
  // number of sites.
  // Change cell widths to be in terms of number of sites.
  std::vector<int> swid;
  swid.resize(ncells);
  std::fill(swid.begin(), swid.end(), 0);
  int rsites = 0;
  for (int i = 0; i < ncells; i++) {
    Node* ndi = cells[i];
    double width = ndi->getWidth();
    if (i != ncells-1) {
      width += m_arch->getCellSpacing(ndi, cells[i+1]);
    }
    swid[i] = (int)std::ceil(width / siteSpacing); 
    rsites += swid[i];
  }
  if (rsites > nsites) {
    return false;
  }

  // Determine leftmost and rightmost site for each cell.
  std::vector<int> site_l, site_r;
  site_l.resize(ncells);
  site_r.resize(ncells);
  int k = i0;
  for (int i = 0; i < ncells; i++) {
    site_l[i] = k;
    k += swid[i];
  }
  k = i1 + 1;
  for (int i = ncells - 1; i >= 0; i--) {
    site_r[i] = k - swid[i];
    k = site_r[i];
    if (site_r[i] < site_l[i]) {
      return false;
    }
  }

  // Create tables.
  std::vector<std::vector<std::pair<int, int> > > prev;
  std::vector<std::vector<double> > tcost;
  std::vector<std::vector<double> > cost;
  tcost.resize(nsites + 1);
  prev.resize(nsites + 1);
  cost.resize(nsites + 1);
  for (size_t i = 0; i <= nsites; i++) {
    tcost[i].resize(ncells + 1);
    prev[i].resize(ncells + 1);
    cost[i].resize(ncells + 1);

    std::fill(tcost[i].begin(), tcost[i].end(),
              std::numeric_limits<double>::max());
    std::fill(prev[i].begin(), prev[i].end(), std::make_pair(-1, -1));
    std::fill(cost[i].begin(), cost[i].end(), 0.0);
  }

  // Fill in costs of cells to sites.
  for (int j = 1; j <= ncells; j++) {
    // Skip invalid sites.
    for (int i = 1; i <= nsites; i++) {
      // Cell will cover real sites from [site_id,site_id+width-1].

      int site_id = i0 + i - 1;
      if (site_id < site_l[j - 1] || site_id > site_r[j - 1]) {
        continue;
      }

      // Figure out cell position if cell aligned to current site.
      double x = originX + site_id * siteSpacing;
      cost[i][j] = std::fabs(x - targetLeft[j - 1]);
    }
  }

  // Fill in total costs.
  tcost[0][0] = 0.;
  for (int j = 1; j <= ncells; j++) {
    // Width info; for indexing.
    int prev_wid = (j - 1 == 0) ? 1 : swid[j - 2];
    int curr_wid = swid[j - 1];

    for (int i = 1; i <= nsites; i++) {
      // Current site is site_id and covers [site_id,site_id+width-1].
      int site_id = i0 + i - 1;

      // Cost if site skipped.
      ii = i - 1;
      jj = j;
      {
        double c = tcost[ii][jj];
        if (c < tcost[i][j]) {
          tcost[i][j] = c;
          prev[i][j] = std::make_pair(ii, jj);
        }
      }

      // Cost if site used; avoid if invalid (too far left or right).
      ii = i - prev_wid;
      jj = j - 1;
      if (!(ii < 0 || site_id + curr_wid - 1 > i1)) {
        double c = tcost[ii][jj] + cost[i][j];
        if (c < tcost[i][j]) {
          tcost[i][j] = c;
          prev[i][j] = std::make_pair(ii, jj);
        }
      }
    }
  }

  // Test.
  {
    bool okay = false;
    std::pair<int, int> curr = std::make_pair(nsites, ncells);
    while (curr.first != -1 && curr.second != -1) {
      if (curr.first == 0 && curr.second == 0) {
        okay = true;
      }
      curr = prev[curr.first][curr.second];
    }
    if (!okay) {
      // Odd.  Should not fail.
      return false;
    }
  }

  // Determine placement.
  {
    std::pair<int, int> curr = std::make_pair(nsites, ncells);
    while (curr.first != -1 && curr.second != -1) {
      if (curr.first == 0 && curr.second == 0) {
        break;
      }
      int curr_i = curr.first;   // Site.
      int curr_j = curr.second;  // Cell.

      if (curr_j != prev[curr_i][curr_j].second) {
        // We've placed the cell at the site.
        int ix = i0 + curr_i - 1;
        posLeft[curr_j - 1] = originX + ix * siteSpacing;
      }

      curr = prev[curr_i][curr_j];
    }
  }
  return true;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
bool DetailedMgr::shiftRightHelper(Node *ndi, int xj, int sj, Node* ndr)
{
  // Helper routine for shifting single height cells in a specified
  // segment to the right.
  // 
  // We assume cell "ndi" is going to be positioned (left edge) at
  // "xj" within segment "sj".  The cell "ndr" is the cell which is
  // to the immediate right of "ndi" after the insertion.
  //
  // We will attempt to push cells starting at "ndr" to the right to
  // maintain no overlap, satisfy spacing, etc.

  std::vector<Node*>::iterator it = 
            std::find(m_cellsInSeg[sj].begin(), m_cellsInSeg[sj].end(), ndr);
  if (m_cellsInSeg[sj].end() == it) {
    // Error.
    return false;
  }
  int ix = (int)(it - m_cellsInSeg[sj].begin());
  int n = (int)m_cellsInSeg[sj].size() - 1;

  int rj = m_segments[sj]->getRowId();
  int originX = m_arch->getRow(rj)->getLeft();
  int siteSpacing = m_arch->getRow(rj)->getSiteSpacing();

  // Shift single height cells to the right until we encounter some 
  // sort of problem.
  while ((ix <= n) && 
         (ndr->getLeft() < xj+ndi->getWidth()+m_arch->getCellSpacing(ndi, ndr))) {

    if (m_arch->getCellHeightInRows(ndr) != 1) {
      return false;
    }

    // Determine a proper site-aligned position for cell ndr.
    xj += ndi->getWidth();
    xj += m_arch->getCellSpacing(ndi, ndr);

    int site = (xj - originX) / siteSpacing;

    int sx = originX + site * siteSpacing;
    if (xj != sx) {
      // Might need to go another site to the right.
      if (xj > sx) {
        sx += siteSpacing;
      }
      if (xj != sx) {
        if (xj < sx) {
          xj = sx;
        }
      }
    }

    // Build the move list.
    if (!addToMoveList(ndr, ndr->getLeft(), ndr->getBottom(), sj, xj, ndr->getBottom(), sj)) {
      return false;
    }

    // Fail if we shift off end of segment.
    if (xj + ndr->getWidth() + m_arch->getCellSpacing(ndr, 0) > m_segments[sj]->getMaxX()) {
      return false;
    }

    if (ix == n) {
      // We shifted down to the last cell... Everything must be okay!
      break;
    }

    ndi = ndr;
    ndr = m_cellsInSeg[sj][++ix];
  }
  return true;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
bool DetailedMgr::shiftLeftHelper(Node *ndi, int xj, int sj, Node* ndl) {
  // Helper routine for shifting single height cells in a specified
  // segment to the left.
  // 
  // We assume cell "ndi" is going to be positioned (left edge) at
  // "xj" within segment "sj".  The cell "ndl" is the cell which is
  // to the immediate left of "ndi" after the insertion.
  //
  // We will attempt to push cells starting at "ndl" to the left to
  // maintain no overlap, satisfy spacing, etc.

  // Need the index of "ndl".
  std::vector<Node*>::iterator it =
            std::find(m_cellsInSeg[sj].begin(), m_cellsInSeg[sj].end(), ndl);
  if (m_cellsInSeg[sj].end() == it) {
    return false;
  }
  int ix = (int)(it - m_cellsInSeg[sj].begin());
  int n = 0;

  int rj = m_segments[sj]->getRowId();
  int originX = m_arch->getRow(rj)->getLeft();
  int siteSpacing = m_arch->getRow(rj)->getSiteSpacing();

  // Shift single height cells to the left until we encounter some 
  // sort of problem.
  while ((ix >= n) && 
         (ndl->getRight() + m_arch->getCellSpacing(ndl, ndi) > xj)) {

    if (m_arch->getCellHeightInRows(ndl) != 1) {
      return false;
    }

    // Determine a proper site-aligned position for cell ndl.
    xj -= m_arch->getCellSpacing(ndl, ndi);
    xj -= ndl->getWidth();

    int site = (xj - originX) / siteSpacing;

    int sx = originX + site * siteSpacing;
    if (xj != sx) {
      if (xj > sx) {
        xj = sx;
      }
    }

    // Build the move list.
    if (!addToMoveList(ndl, ndl->getLeft(), ndl->getBottom(), sj, xj, ndl->getBottom(), sj)) {
      return false;
    }

    // Fail if we shift off the end of a segment.
    if (xj - m_arch->getCellSpacing(0, ndl) < m_segments[sj]->getMinX()) {
      return false;
    }
    if (ix == n) {
      // We shifted down to the last cell... Everything must be okay!
      break;
    }

    ndi = ndl;
    ndl = m_cellsInSeg[sj][--ix];
  }
  return true;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
bool DetailedMgr::tryMove(Node* ndi, 
                           int xi, int yi, int si, 
                           int xj, int yj, int sj) {
  // Based on the input, call an appropriate routine to try
  // and generate a move.
  if (m_arch->getCellHeightInRows(ndi) == 1) {
    // Single height cell.
    if (si != sj) {
      // Different segment.
      if (tryMove1(ndi, xi, yi, si, xj, yj, sj)) {
        return true;
      }
    }
    else {
      // Same segment.
      if (tryMove2(ndi, xi, yi, si, xj, yj, sj)) {
        return true;
      }
    }
  }
  else {
    // Currently only a single, simple routine for trying to move
    // a multi-height cell.
    if (tryMove3(ndi, xi, yi, si, xj, yj, sj)) {
      return true;
    }
  }
  return false;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
bool DetailedMgr::trySwap(Node* ndi, 
                          int xi, int yi, int si, 
                          int xj, int yj, int sj) {
  if (trySwap1(ndi, xi, yi, si, xj, yj, sj)) {
    return true;
  }
  return false;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
bool DetailedMgr::tryMove1(Node* ndi, 
                           int xi, int yi, int si, 
                           int xj, int yj, int sj) {
  // Try to move a single height cell to a new position in another segment.
  // Positions are understood to be positions for the left, bottom corner
  // of the cell.

  // Clear the move list.
  clearMoveList();

  // Reasons to fail.  Same or bogus segment, wrong region, or
  // not single height cell.
  int spanned = m_arch->getCellHeightInRows(ndi);
  if (sj == si || sj == -1 || 
      ndi->getRegionId() != m_segments[sj]->getRegId() ||
      spanned != 1) {
    return false;
  }

  int rj = m_segments[sj]->getRowId();
  if (std::abs(yj-m_arch->getRow(rj)->getBottom()) != 0) {
    // Weird.
    yj = m_arch->getRow(rj)->getBottom();
  }

  // Find the cells to the left and to the right of the target location.
  Node* ndr = 0;
  Node* ndl = 0;
  if (m_cellsInSeg[sj].size() != 0) {
    std::vector<Node*>::iterator it = 
        std::lower_bound(m_cellsInSeg[sj].begin(), m_cellsInSeg[sj].end(), xj,
                         DetailedMgr::compareNodesX());

    if (it == m_cellsInSeg[sj].end()) {
      // Nothing to the right of the target position.  But, there must be
      // something to the left since we know the segment is not empty.
      ndl = m_cellsInSeg[sj].back();
    } else {
      ndr = *it;
      if (it != m_cellsInSeg[sj].begin()) {
        --it;
        ndl = *it;
      }
    }
  }

  // What we do depends on if there are cells to the left or right.
  if (ndl == 0 && ndr == 0) {
    // No left or right cell implies an empty segment.
    DetailedSeg* segPtr = m_segments[sj];

    // Reject if not enough space.
    int required = ndi->getWidth() + 
                   m_arch->getCellSpacing(0, ndi) + 
                   m_arch->getCellSpacing(ndi, 0);
    if (required + segPtr->getUtil() > segPtr->getWidth()) {
      return false;
    }
      
    int lx = segPtr->getMinX() + m_arch->getCellSpacing(0, ndi);
    int rx = segPtr->getMaxX() - m_arch->getCellSpacing(ndi, 0);
    if (!alignPos(ndi, xj, lx, rx)) {
      return false;
    }
    // Build the move list.
    if (!addToMoveList(ndi, ndi->getLeft(), ndi->getBottom(), si, xj, yj, sj)) {
      return false;
    }
    return true;
  } 
  else if (ndl != 0 && ndr == 0) {
    // End of segment, cells to the left.
    DetailedSeg* segPtr = m_segments[sj];

    // Reject if not enough space.
    int required = ndi->getWidth() + 
                   m_arch->getCellSpacing(ndl, ndi) + 
                   m_arch->getCellSpacing(ndi, 0);
    if (required + segPtr->getUtil() > segPtr->getWidth()) {
      return false;
    }

    int lx = ndl->getRight() + m_arch->getCellSpacing(ndl,ndi);
    int rx = m_segments[sj]->getMaxX() - m_arch->getCellSpacing(ndi,0);
    if (!alignPos(ndi, xj, lx, rx)) {
      return false;
    }

    // Build the move list.
    if (!addToMoveList(ndi, ndi->getLeft(), ndi->getBottom(), si, xj, yj, sj)) {
      return false;
    }
    // Shift cells left if required.
    if (!shiftLeftHelper(ndi, xj, sj, ndl)) {
      return false;
    }
    return true;
  } 
  else if (ndl == 0 && ndr != 0) {
    // End of segment, cells to the left.
    DetailedSeg* segPtr = m_segments[sj];

    // Reject if not enough space.
    int required = ndi->getWidth() + 
                   m_arch->getCellSpacing(0, ndi) + 
                   m_arch->getCellSpacing(ndi, ndr);
    if (required + segPtr->getUtil() > segPtr->getWidth()) {
      return false;
    }

    int lx = segPtr->getMinX() + m_arch->getCellSpacing(0, ndi);;
    int rx = ndr->getLeft() - m_arch->getCellSpacing(ndi, ndr);
    if (!alignPos(ndi, xj, lx, rx)) {
      return false;
    }

    // Build the move list.
    if (!addToMoveList(ndi, ndi->getLeft(), ndi->getBottom(), si, xj, yj, sj)) {
      return false;
    }
    // Shift cells right if required.
    if (!shiftRightHelper(ndi, xj, sj, ndr)) {
      return false;
    }
    return true;
  } else if (ndl != 0 && ndr != 0) {
    // In between two cells.
    DetailedSeg* segPtr = m_segments[sj];

    // Reject if not enough space.
    int required = ndi->getWidth() +
                   m_arch->getCellSpacing(ndl, ndi) + 
                   m_arch->getCellSpacing(ndi, ndr) -
                   m_arch->getCellSpacing(ndl, ndr);
    if (required + segPtr->getUtil() > segPtr->getWidth()) {
      return false;
    }

    int lx = ndl->getRight() + m_arch->getCellSpacing(ndl, ndi);
    int rx = ndr->getLeft() - m_arch->getCellSpacing(ndi, ndr);
    if (!alignPos(ndi, xj, lx, rx)) {
      return false;
    }

    // Build the move list.
    if (!addToMoveList(ndi, ndi->getLeft(), ndi->getBottom(), si, xj, yj, sj)) {
      return false;
    }
    // Shift cells right if required.
    if (!shiftRightHelper(ndi, xj, sj, ndr)) {
      return false;
    }
    // Shift cells left if necessary.
    if (!shiftLeftHelper(ndi, xj, sj, ndl)) {
      return false;
    }
    return true;
  }

  return false;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
bool DetailedMgr::tryMove2(Node* ndi, 
                           int xi, int yi, int si, 
                           int xj, int yj, int sj) {
  // Very simple move within the same segment.

  // Nothing to move.
  clearMoveList();

  // Reasons to fail.  Different or bogus segment, wrong region, or
  // not single height cell.
  int spanned = m_arch->getCellHeightInRows(ndi);
  if (sj != si || sj == -1 || 
      ndi->getRegionId() != m_segments[sj]->getRegId() ||
      spanned != 1) {
    return false;
  }

  int rj = m_segments[sj]->getRowId();
  if (std::abs(yj-m_arch->getRow(rj)->getBottom()) != 0) {
    // Weird.
    yj = m_arch->getRow(rj)->getBottom();
  }

  int n = (int)m_cellsInSeg[si].size() - 1;

  // Find closest cell to the right of the target location.  It's fine
  // to get "ndi" since we are just attempting a move to a new 
  // location.
  Node* ndj = nullptr;
  int ix_j = -1;
  if (m_cellsInSeg[sj].size() != 0) {
    std::vector<Node*>::iterator it_j =
            std::lower_bound(m_cellsInSeg[sj].begin(), m_cellsInSeg[sj].end(), 
                             xj, DetailedMgr::compareNodesX());

    if (it_j == m_cellsInSeg[sj].end()) {
      ndj = m_cellsInSeg[sj].back();
      ix_j = (int)m_cellsInSeg[sj].size() - 1;
    } else {
      ndj = *it_j;
      ix_j = (int)(it_j - m_cellsInSeg[sj].begin());
    }
  }
  // We should find something...  At least "ndi"!
  if (ix_j == -1 || ndj == nullptr) {
    return false;
  }

  // Note that it is fine if ndj is the same as ndi; we are just trying
  // to move to a new position adjacent to some block.
  Node* prev = (ix_j == 0) ? nullptr : m_cellsInSeg[sj][ix_j-1];
  Node* next = (ix_j == n) ? nullptr : m_cellsInSeg[sj][ix_j+1];

  // Try to the left of ndj, then to the right.
  int lx, rx;
  DetailedSeg* segPtr = m_segments[sj];

  // Try left.
  if (prev) {
    lx = prev->getRight() + m_arch->getCellSpacing(prev, ndi);
  }
  else {
    lx = segPtr->getMinX() + m_arch->getCellSpacing(0, ndi);
  }
  rx = ndj->getLeft() - m_arch->getCellSpacing(ndi, ndj);
  if (ndi->getWidth() <= rx - lx) {
    if (!alignPos(ndi, xj, lx, rx)) {
      return false;
    }
    if (!addToMoveList(ndi, ndi->getLeft(), ndi->getBottom(), si, xj, yj, sj)) {
      return false;
    }
    return true;
  }

  // Try right.
  lx = ndj->getRight() + m_arch->getCellSpacing(ndj, ndi);
  if (next) {
    rx = next->getLeft() - m_arch->getCellSpacing(ndi, next);
  }
  else {
    rx = segPtr->getMaxX() - m_arch->getCellSpacing(ndi, 0);
  }
  if (ndi->getWidth() <= rx - lx) {
    if (!alignPos(ndi, xj, lx, rx)) {
      return false;
    }
    if (!addToMoveList(ndi, ndi->getLeft(), ndi->getBottom(), si, xj, yj, sj)) {
      return false;
    }
    return true;
  }
  return false;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
bool DetailedMgr::tryMove3(Node* ndi, 
                           int xi, int yi, int si, 
                           int xj, int yj, int sj) {
  clearMoveList();

  // Code to try and move a multi-height cell to another location.  Simple
  // in that it only looks for gaps.


  // Ensure multi-height, although I think this code should work for single
  // height cells too.
  int spanned = m_arch->getCellHeightInRows(ndi);
  if (spanned <= 1 || spanned != m_reverseCellToSegs[ndi->getId()].size()) {
    return false;
  }

  // Turn the target location into a set of rows.  The target position
  // in the y-direction should be the target position for the bottom
  // of the cell which should also correspond to the row in which the
  // segment is found.
  int rb = m_segments[sj]->getRowId();
  if (std::abs(yj-m_arch->getRow(rb)->getBottom()) != 0) {
    // Weird.
    yj = m_arch->getRow(rb)->getBottom();
  }
  while (rb + spanned >= m_arch->getRows().size()) {
    --rb;
  }
  // We might need to adjust the target position if we needed to move
  // the rows "down"...
  yj = m_arch->getRow(rb)->getBottom();
  int rt = rb + spanned - 1;  // Cell would occupy rows [rb,rt].

  bool flip = false;
  if (!m_arch->power_compatible(ndi, m_arch->getRow(rb), flip)) {
    return false;
  }

  // Next find the segments based on the targeted x location.  We might be
  // outside of our region or there could be a blockage.  So, we need a flag.
  std::vector<int> segs;
  for (int r = rb; r <= rt; r++) {
    bool gotSeg = false;
    for (int s = 0; s < m_segsInRow[r].size() && !gotSeg; s++) {
      DetailedSeg* segPtr = m_segsInRow[r][s];
      if (segPtr->getRegId() == ndi->getRegionId()) {
        if (xj >= segPtr->getMinX() && xj <= segPtr->getMaxX()) {
          gotSeg = true;
          segs.push_back(segPtr->getSegId());
        }
      }
    }
    if (!gotSeg) {
      break;
    }
  }
  // Extra check.
  if (segs.size() != spanned) {
    return false;
  }


  // So, the goal is to try and move the cell into the segments contained within
  // the "segs" vector.  Determine if there is space.  To do this, we loop over
  // the segments and look for the cell to the right of the target location.  We
  // then grab the cell to the left.  We can determine if the the gap is large
  // enough.
  int xmin = std::numeric_limits<int>::lowest();
  int xmax = std::numeric_limits<int>::max();
  for (size_t s = 0; s < segs.size(); s++) {
    DetailedSeg* segPtr = m_segments[segs[s]];

    int segId = m_segments[segs[s]]->getSegId();
    Node* left = nullptr;
    Node* rite = nullptr;

    if (m_cellsInSeg[segId].size() != 0) {
      std::vector<Node*>::iterator it_j = 
             std::lower_bound(m_cellsInSeg[segId].begin(),
                              m_cellsInSeg[segId].end(), xj,
                              DetailedMgr::compareNodesX());
      if (it_j == m_cellsInSeg[segId].end()) {
        // Nothing to the right; the last cell in the row will be on the left.
        left = m_cellsInSeg[segId].back();

        // If the cell on the left turns out to be the current cell, then we
        // can assume this cell is not there and look to the left "one cell
        // more".
        if (left == ndi) {
          if (it_j != m_cellsInSeg[segId].begin()) {
            --it_j;
            left = *it_j;
            ++it_j;
          } else {
            left = nullptr;
          }
        }
      } else {
        rite = *it_j;
        if (it_j != m_cellsInSeg[segId].begin()) {
          --it_j;
          left = *it_j;
          if (left == ndi) {
            if (it_j != m_cellsInSeg[segId].begin()) {
              --it_j;
              left = *it_j;
              ++it_j;
            } else {
              left = nullptr;
            }
          }
          ++it_j;
        }
      }
    }

    // If the left or the right cells are the same as the current cell, then
    // we aren't moving.
    if (ndi == left || ndi == rite) {
      return false;
    }

    int lx = (left == nullptr) ? segPtr->getMinX() : left->getRight();
    int rx = (rite == nullptr) ? segPtr->getMaxX() : rite->getLeft();
    if (left != nullptr) lx += m_arch->getCellSpacing(left, ndi);
    if (rite != nullptr) rx -= m_arch->getCellSpacing(ndi, rite);

    if (ndi->getWidth() <= rx - lx) {
      // The cell will fit without moving the left and right cell.
      xmin = std::max(xmin, lx);
      xmax = std::min(xmax, rx);
    } else {
      // The cell will not fit in between the left and right cell
      // in this segment.  So, we cannot faciliate the single move.
      return false;
    }
  }

  // Here, we can fit.
  if (ndi->getWidth() <= xmax - xmin) {
    if (!alignPos(ndi, xj, xmin, xmax)) {
      return false;
    }

    std::vector<int> old_segs;
    old_segs.reserve(m_reverseCellToSegs[ndi->getId()].size());
    for (size_t i = 0; i < m_reverseCellToSegs[ndi->getId()].size(); i++) {
      old_segs.push_back(
          m_reverseCellToSegs[ndi->getId()][i]->getSegId());
    }
    
    if (!addToMoveList(ndi, 
                       ndi->getLeft(), ndi->getBottom(), old_segs, 
                       xj, m_arch->getRow(rb)->getBottom(), segs)) {
      return false;
    }
    return true;
  } 
  return false;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
bool DetailedMgr::trySwap1(Node* ndi,
                           int xi, int yi, int si,
                           int xj, int yj, int sj) {
  // Tries to swap cell "ndi" with another cell, "ndj", which it finds
  // near the target.  No cell shifting is involved; only the two cells
  // are considered.  So, it is a very simple swap.  It also only works
  // for single height cells.

  clearMoveList();

  Node* ndj = 0;
  if (m_cellsInSeg[sj].size() != 0) {
    std::vector<Node*>::iterator it_j = 
            std::lower_bound(m_cellsInSeg[sj].begin(), m_cellsInSeg[sj].end(),
                             xj, DetailedMgr::compareNodesX());
    if (it_j == m_cellsInSeg[sj].end()) {
      ndj = m_cellsInSeg[sj].back();
    } else {
      ndj = *it_j;
    }
  }
  if (ndj == ndi || ndj == 0) {
    return false;
  }
  if (m_arch->getCellHeightInRows(ndi) != 1 ||
      m_arch->getCellHeightInRows(ndj) != 1) {
    return false;
  }

  // Determine the indices of the cells in their respective 
  // segments.  Determine if cells are adjacent.
  std::vector<Node*>::iterator it_i = 
        std::find(m_cellsInSeg[si].begin(), m_cellsInSeg[si].end(), ndi);
  int ix_i = (int)(it_i - m_cellsInSeg[si].begin());

  std::vector<Node*>::iterator it_j = 
        std::find(m_cellsInSeg[sj].begin(), m_cellsInSeg[sj].end(), ndj);
  int ix_j = (int)(it_j - m_cellsInSeg[sj].begin());

  bool adjacent = ((si == sj) && (ix_i + 1 == ix_j || ix_j + 1 == ix_i)) 
                ? true 
                : false;

  Node* prev;
  Node* next;
  int n;
  int lx, rx;
  if (!adjacent) {
    // Determine if "ndi" can fit into the gap created
    // by removing "ndj" and visa-versa.
    n = (int)m_cellsInSeg[si].size() - 1;
    next = (ix_i == n) ? nullptr : m_cellsInSeg[si][ix_i+1];
    prev = (ix_i == 0) ? nullptr : m_cellsInSeg[si][ix_i-1];
    rx = m_segments[si]->getMaxX();
    if (next) {
      rx = next->getLeft();
    }
    rx -= m_arch->getCellSpacing(ndj, next);

    lx = m_segments[si]->getMinX();
    if (prev) {
      lx = prev->getRight();
    }
    lx += m_arch->getCellSpacing(prev, ndj);

    if (ndj->getWidth() > (rx - lx)) {
      // Cell "ndj" will not fit into gap created by removing "ndi".
      return false;
    }

    // Determine aligned position for "ndj" in spot created by
    // removing "ndi".
    if (!alignPos(ndj, xi, lx, rx)) {
      return false;
    }

    n = (int)m_cellsInSeg[sj].size() - 1;
    next = (ix_j == n) ? nullptr : m_cellsInSeg[sj][ix_j+1];
    prev = (ix_j == 0) ? nullptr : m_cellsInSeg[sj][ix_j-1];
    rx = m_segments[sj]->getMaxX();
    if (next) {
      rx = next->getLeft();
    }
    rx -= m_arch->getCellSpacing(ndi, next);

    lx = m_segments[sj]->getMinX();
    if (prev) {
      lx = prev->getRight();
    }
    lx += m_arch->getCellSpacing(prev, ndi);

    if (ndi->getWidth() > (rx - lx)) {
      // Cell "ndi" will not fit into gap created by removing "ndj".
      return false;
    }

    // Determine aligned position for "ndi" in spot created by
    // removing "ndj".
    if (!alignPos(ndi, xj, lx, rx)) {
      return false;
    }

    // Build move list.
    if (!addToMoveList(ndi, ndi->getLeft(), ndi->getBottom(), si, xj, ndj->getBottom(), sj)) {
      return false;
    }
    if (!addToMoveList(ndj, ndj->getLeft(), ndj->getBottom(), sj, xi, ndi->getBottom(), si)) {
      return false;
    }
    return true;
  }
  else {
    // Same row and adjacent.
    if (ix_i + 1 == ix_j) {
      // cell "ndi" is left of cell "ndj".
      n = (int)m_cellsInSeg[sj].size() - 1;
      next = (ix_j == n) ? nullptr : m_cellsInSeg[sj][ix_j+1];
      prev = (ix_i == 0) ? nullptr : m_cellsInSeg[si][ix_i-1];

      rx = m_segments[sj]->getMaxX();
      if (next) {
        rx = next->getLeft();
      }
      rx -= m_arch->getCellSpacing(ndi, next);

      lx = m_segments[si]->getMinX();
      if (prev) {
        lx = prev->getRight();
      }
      lx += m_arch->getCellSpacing(prev, ndj);

      if (ndj->getWidth() + ndi->getWidth() + m_arch->getCellSpacing(ndj, ndi) > (rx - lx)) {
        return false;
      }

      // Shift...
      std::vector<Node*> cells;
      std::vector<int> targetLeft;
      std::vector<int> posLeft;
      cells.push_back(ndj);
      targetLeft.push_back(xi);
      posLeft.push_back(0);
      cells.push_back(ndi);
      targetLeft.push_back(xj);
      posLeft.push_back(0);
      int ri = m_segments[si]->getRowId();
      if (!shift(cells, targetLeft, posLeft, lx, rx, si, ri)) {
        return false;
      }
      xi = posLeft[0];
      xj = posLeft[1];
    } 
    else if (ix_j + 1 == ix_i) {
      // cell "ndj" is left of cell "ndi".
      n = (int)m_cellsInSeg[si].size() - 1;
      next = (ix_i == n) ? nullptr : m_cellsInSeg[si][ix_i+1];
      prev = (ix_j == 0) ? nullptr : m_cellsInSeg[sj][ix_j-1];

      rx = m_segments[si]->getMaxX();
      if (next) {
        rx = next->getLeft();
      }
      rx -= m_arch->getCellSpacing(ndj, next);

      lx = m_segments[sj]->getMinX();
      if (prev) {
        lx = prev->getRight();
      }
      lx += m_arch->getCellSpacing(prev, ndi);

      if (ndi->getWidth() + ndj->getWidth() + m_arch->getCellSpacing(ndi, ndj) > (rx - lx)) {
        return false;
      }

      // Shift...
      std::vector<Node*> cells;
      std::vector<int> targetLeft;
      std::vector<int> posLeft;
      cells.push_back(ndi);
      targetLeft.push_back(xj);
      posLeft.push_back(0);
      cells.push_back(ndj);
      targetLeft.push_back(xi);
      posLeft.push_back(0);
      int ri = m_segments[si]->getRowId();
      if (!shift(cells, targetLeft, posLeft, lx, rx, si, ri)) {
        return false;
      }
      xj = posLeft[0];
      xi = posLeft[1];
    } else {
      // Shouldn't get here.
      return false;
    }

    // Build move list.
    if (!addToMoveList(ndi, ndi->getLeft(), ndi->getBottom(), si, xj, ndj->getBottom(), sj)) {
      return false;
    }
    if (!addToMoveList(ndj, ndj->getLeft(), ndj->getBottom(), sj, xi, ndi->getBottom(), si)) {
      return false; 
    }
    return true;
  }
  return false;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void DetailedMgr::clearMoveList() {
  m_nMoved = 0;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
bool DetailedMgr::addToMoveList(Node *ndi, 
        int curLeft,
        int curBottom,
        int curSeg,
        int newLeft,
        int newBottom,
        int newSeg) {
  // Limit maximum number of cells that can move at once.
  if (m_nMoved >= m_moveLimit) {
    return false;
  }

  // Easy to observe displacement limit if using the 
  // manager to compose a move list.  We can check
  // only here whether or not a cell will violate its
  // displacement limit.
  double dy = std::fabs(newBottom - ndi->getOrigBottom());
  double dx = std::fabs(newLeft - ndi->getOrigLeft());
  if ((int)std::ceil(dx) > m_maxDispX ||
      (int)std::ceil(dy) > m_maxDispY) {
    return false;
  }

  m_movedNodes[m_nMoved] = ndi;
  m_curLeft[m_nMoved] = curLeft;
  m_curBottom[m_nMoved] = curBottom;
  m_curSeg[m_nMoved].clear();
  m_curSeg[m_nMoved].push_back(curSeg);
  m_newLeft[m_nMoved] = newLeft;
  m_newBottom[m_nMoved] = newBottom;
  m_newSeg[m_nMoved].clear();
  m_newSeg[m_nMoved].push_back(newSeg);
  ++m_nMoved;
  return true;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
bool DetailedMgr::addToMoveList(Node *ndi, 
        int curLeft,
        int curBottom,
        std::vector<int>& curSegs,
        int newLeft,
        int newBottom,
        std::vector<int>& newSegs) {
  // Most number of cells that can move.
  if (m_nMoved >= m_moveLimit) {
    return false;
  }

  m_movedNodes[m_nMoved] = ndi;
  m_curLeft[m_nMoved] = curLeft;
  m_curBottom[m_nMoved] = curBottom;
  m_curSeg[m_nMoved] = curSegs;
  m_newLeft[m_nMoved] = newLeft;
  m_newBottom[m_nMoved] = newBottom;
  m_newSeg[m_nMoved] = newSegs;
  ++m_nMoved;
  return true;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void DetailedMgr::acceptMove() {
  // Moves stored list of cells.  XXX: Only single height cells.

  for (int i = 0; i < m_nMoved; i++) {
    Node* ndi = m_movedNodes[i];

    // Remove node from current segment.
    for (size_t s = 0; s < m_curSeg[i].size(); s++) {
      this->removeCellFromSegment(ndi, m_curSeg[i][s]);
    }

    // Update position and orientation.
    ndi->setLeft(m_newLeft[i]);
    ndi->setBottom(m_newBottom[i]);
    // XXX: Need to do the orientiation.
    ;

    // Insert into new segment.
    for (size_t s = 0; s < m_newSeg[i].size(); s++) {
      this->addCellToSegment(ndi, m_newSeg[i][s]);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void DetailedMgr::rejectMove() {
  clearMoveList();
}

}  // namespace dpo
