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
// File: router.cxx
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Includes.
////////////////////////////////////////////////////////////////////////////////
#include "router.h"
#include <stdio.h>
#include <iostream>
#include <map>
#include <vector>
#include "architecture.h"
#include "network.h"
#include "utility.h"

////////////////////////////////////////////////////////////////////////////////
// Forward declarations.
////////////////////////////////////////////////////////////////////////////////

namespace dpo {

////////////////////////////////////////////////////////////////////////////////
// Classes.
////////////////////////////////////////////////////////////////////////////////
void RoutingParams::postProcess() {}

double RoutingParams::get_spacing(int layer, double xmin1, double xmax1,
                                  double ymin1, double ymax1, double xmin2,
                                  double xmax2, double ymin2, double ymax2) {
  double ww = std::max(std::min(ymax1 - ymin1, xmax1 - xmin1),
                       std::min(ymax2 - ymin2, xmax2 - xmin2));

  // Parallel run-length in the Y-dir.  Will be zero if the objects are above or
  // below each other.
  double py = std::max(0.0, std::min(ymax1, ymax2) - std::max(ymin1, ymin2));

  // Parallel run-length in the X-dir.  Will be zero if the objects are left or
  // right of each other.
  double px = std::max(0.0, std::min(xmax1, xmax2) - std::max(xmin1, xmin2));

  return get_spacing(layer, ww, std::max(px, py));
}

double RoutingParams::get_spacing(int layer, double width, double parallel) {
  std::vector<double>& w = m_spacingTableWidth[layer];
  std::vector<double>& p = m_spacingTableLength[layer];

  if (w.size() == 0 || p.size() == 0) {
    // This means no spacing table is present.  So, return the minimum wire
    // spacing for the layer...
    return m_wire_spacing[layer];
  }

  int i = (int)w.size() - 1;
  while (i > 0 && width <= w[i]) i--;
  int j = (int)p.size() - 1;
  while (j > 0 && parallel <= p[j]) j--;

  return m_spacingTable[layer][i][j];
}

double RoutingParams::get_maximum_spacing(int layer) {
  std::vector<double>& w = m_spacingTableWidth[layer];
  std::vector<double>& p = m_spacingTableLength[layer];

  if (w.size() == 0 || p.size() == 0) {
    // This means no spacing table is present.  So, return the minimum wire
    // spacing for the layer...
    return m_wire_spacing[layer];
  }

  int i = (int)w.size() - 1;
  int j = (int)p.size() - 1;

  return m_spacingTable[layer][i][j];
}

}  // namespace dpo
