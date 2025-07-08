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

#include <vector>
#include "odb/db.h"
#include "ThreadSafeGrid.h"

namespace sa2d {

// Simple pin structure
struct SimplePin {
    int node_id;
    int net_id;
    DbuX offset_x;
    DbuY offset_y;
};

// Simple net structure
struct SimpleNet {
    std::vector<SimplePin*> pins;
};

// Simple group structure
struct SimpleGroup {
    odb::Rect bbox;
};

// Simplified node structure that doesn't depend on DPL internals
struct SimpleNode {
    enum Type { CELL, TERMINAL, MACRO, FILLER };
    
    Type type;
    int id;
    odb::dbInst* db_inst;
    odb::dbSite* site;
    
    // Position and orientation
    DbuX left;
    DbuY bottom;
    DbuX width;
    DbuY height;
    odb::dbOrientType orient;
    
    // Pins
    std::vector<SimplePin*> pins;
    
    // Placement status
    bool is_placed;
    bool is_fixed;
    
    // Group info
    SimpleGroup* group;
    
    // Helper methods
    bool isCell() const { return type == CELL; }
    bool isPlaced() const { return is_placed; }
    bool isFixed() const { return is_fixed; }
    bool inGroup() const { return group != nullptr; }
    DbuX getLeft() const { return left; }
    DbuY getBottom() const { return bottom; }
    DbuX getWidth() const { return width; }
    DbuY getHeight() const { return height; }
    odb::dbOrientType getOrient() const { return orient; }
    odb::dbSite* getSite() const { return site; }
    const std::vector<SimplePin*>& getPins() const { return pins; }
    SimpleGroup* getGroup() const { return group; }
    
    void setLeft(DbuX x) { left = x; }
    void setBottom(DbuY y) { bottom = y; }
    void setOrient(odb::dbOrientType o) { orient = o; }
};

// Simple network to hold nodes and nets
struct SimpleNetwork {
    std::vector<SimpleNode> nodes;
    std::vector<SimpleNet> nets;
    
    int getNumNodes() const { return nodes.size(); }
    int getNumNets() const { return nets.size(); }
    const SimpleNode* getNode(int id) const { return &nodes[id]; }
    SimpleNode* getNode(int id) { return &nodes[id]; }
    const SimpleNet* getNet(int id) const { return &nets[id]; }
};

}  // namespace sa2d 