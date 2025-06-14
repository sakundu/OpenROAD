/////////////////////////////////////////////////////////////////////////////
// Authors: Sayak Kundu (sakundu@ucsd.edu), Zhiang Wang (zhw033@ucsd.edu)
//          Dooseok Yoon (d3yoon@ucsd.edu)
// Copyright (c) 2024, The Regents of the University of California
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

#include "sa1d/Objects.h"
#include "sa1d/OptSA.h"

namespace sa1d {

using odb::dbBTerm;
using odb::dbBPin;

std::pair<int, int> Cell::getPinLocation(
  int llx, int lly, 
  int mterm_id,
  const odb::dbOrientType& orient)
{
  int x = mterm_locs[mterm_id].x();
  int y = mterm_locs[mterm_id].y();
  switch (orient) {
    case dbOrientType::R0:
      return {llx + x, lly + y};
    case dbOrientType::R90:
      return {llx + y, lly + width - x};
    case dbOrientType::R180:
      return {llx + width - x, lly + height - y};
    case dbOrientType::R270:
      return {llx + height - y, lly + x};
    case dbOrientType::MY:
      return {llx + width - x, lly + y};
    case dbOrientType::MYR90:
      return {llx + y, lly + x};
    case dbOrientType::MX:
      return {llx + x, lly + height - y};
    case dbOrientType::MXR90:
      return {llx + height - y, lly + width - x};
    default:
      return {llx + x, lly + y};
  }
}


void Net::updateBTerm()
{
  bterm_flag = false;
  bterm_box.mergeInit();
  for (dbBTerm* bterm : db_net->getBTerms()) {
    bterm_flag = true;
    for (dbBPin* bpin : bterm->getBPins()) {
      Rect pin_bbox = bpin->getBBox();
      int center_x = (pin_bbox.xMin() + pin_bbox.xMax()) / 2;
      int center_y = (pin_bbox.yMin() + pin_bbox.yMax()) / 2;
      Rect pin_center(center_x, center_y, center_x, center_y);
      bterm_box.merge(pin_center);
    }
  }
}

// Utility Functions
dbOrientType orientMirrorY(const dbOrientType& orient)
{
  switch (orient) {
    case dbOrientType::R0:
      return dbOrientType::MY;
    case dbOrientType::R90:
      return dbOrientType::MYR90;
    case dbOrientType::R180:
      return dbOrientType::R180;
    case dbOrientType::R270:
      return dbOrientType::MYR90;
    case dbOrientType::MY:
      return dbOrientType::R0;
    case dbOrientType::MYR90:
      return dbOrientType::R90;
    case dbOrientType::MX:
      return dbOrientType::MX;
    case dbOrientType::MXR90:
      return dbOrientType::MXR90;
    default:
      return dbOrientType::R0;
  }
}

}  // namespace sa1d
