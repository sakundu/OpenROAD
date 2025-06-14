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

#pragma once

#include <vector>
#include <set>
#include <algorithm>
#include <limits>

namespace sa1d {

// Forward declaration for netHPWL struct (defined in Worker.h)
struct netHPWL;

struct IntervalEvent {
  int x_coord;
  int net_id; 
  bool is_start; // true for start, false for end
  
  // For sorting: x_coord ascending, START before END for ties
  bool operator<(const IntervalEvent& other) const {
    if (x_coord != other.x_coord) return x_coord < other.x_coord;
    return is_start && !other.is_start; // START (true) before END (false)
  }
};

class NetOverlapCalculator {
private:
  std::vector<IntervalEvent> events_;
  std::set<int> active_nets_;
  int peak_overlap_;
  
  // For incremental updates
  int min_affected_x_;
  int max_affected_x_;
  
public:
  NetOverlapCalculator();
  
  // Full computation
  int computePeakOverlap(const std::vector<netHPWL>& net_spans);
  
  // Incremental computation
  int computeIncrementalOverlap(const std::set<int>& affected_nets, 
                               const std::vector<netHPWL>& net_spans);
  
  // Helper functions
  void buildEvents(const std::vector<netHPWL>& net_spans);
  void buildEventsForNets(const std::set<int>& net_ids, 
                         const std::vector<netHPWL>& net_spans);
  int sweepLine();
  int sweepRange(int min_x, int max_x);
  
  int getCurrentPeakOverlap() const { return peak_overlap_; }
};

} // namespace sa1d