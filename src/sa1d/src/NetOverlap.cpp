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

#include "sa1d/NetOverlap.h"
#include "Worker.h" // For netHPWL struct

namespace sa1d {

NetOverlapCalculator::NetOverlapCalculator()
  : peak_overlap_(0), min_affected_x_(0), max_affected_x_(0)
{
}

int NetOverlapCalculator::computePeakOverlap(const std::vector<netHPWL>& net_spans)
{
  buildEvents(net_spans);
  peak_overlap_ = sweepLine();
  return peak_overlap_;
}

int NetOverlapCalculator::computeIncrementalOverlap(
    const std::set<int>& affected_nets, 
    const std::vector<netHPWL>& net_spans)
{
  if (affected_nets.empty()) {
    return peak_overlap_;
  }
  
  // Find the x-range that could be affected
  min_affected_x_ = std::numeric_limits<int>::max();
  max_affected_x_ = std::numeric_limits<int>::min();
  
  for (int net_id : affected_nets) {
    const auto& net = net_spans[net_id];
    // Consider both old and new positions
    min_affected_x_ = std::min({min_affected_x_, net.lx, net.pre_lx});
    max_affected_x_ = std::max({max_affected_x_, net.ux, net.pre_ux});
  }
  
  // For now, do full computation - can optimize later with range-limited sweep
  buildEvents(net_spans);
  peak_overlap_ = sweepLine();
  return peak_overlap_;
}

void NetOverlapCalculator::buildEvents(const std::vector<netHPWL>& net_spans)
{
  events_.clear();
  events_.reserve(net_spans.size() * 2);
  
  for (int net_id = 0; net_id < static_cast<int>(net_spans.size()); net_id++) {
    const auto& net = net_spans[net_id];
    // Only consider nets with non-zero span
    if (net.lx < net.ux) {
      events_.emplace_back();
      events_.back().x_coord = net.lx;
      events_.back().net_id = net_id;
      events_.back().is_start = true;
      
      events_.emplace_back();
      events_.back().x_coord = net.ux;
      events_.back().net_id = net_id;
      events_.back().is_start = false;
    }
  }
  
  std::sort(events_.begin(), events_.end());
}

void NetOverlapCalculator::buildEventsForNets(
    const std::set<int>& net_ids, 
    const std::vector<netHPWL>& net_spans)
{
  events_.clear();
  events_.reserve(net_ids.size() * 2);
  
  for (int net_id : net_ids) {
    const auto& net = net_spans[net_id];
    // Only consider nets with non-zero span
    if (net.lx < net.ux) {
      events_.emplace_back();
      events_.back().x_coord = net.lx;
      events_.back().net_id = net_id;
      events_.back().is_start = true;
      
      events_.emplace_back();
      events_.back().x_coord = net.ux;
      events_.back().net_id = net_id;
      events_.back().is_start = false;
    }
  }
  
  std::sort(events_.begin(), events_.end());
}

int NetOverlapCalculator::sweepLine()
{
  active_nets_.clear();
  int max_overlap = 0;
  
  for (const auto& event : events_) {
    if (event.is_start) {
      active_nets_.insert(event.net_id);
    } else {
      active_nets_.erase(event.net_id);
    }
    
    int current_overlap = static_cast<int>(active_nets_.size());
    max_overlap = std::max(max_overlap, current_overlap);
  }
  
  return max_overlap;
}

int NetOverlapCalculator::sweepRange(int min_x, int max_x)
{
  // For now, use full sweep - can optimize later
  return sweepLine();
}

} // namespace sa1d