/////////////////////////////////////////////////////////////////////////////
//
// BSD 3-Clause License
//
// Copyright (c) 2019, The Regents of the University of California
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
//
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "Util.h"
#include "db.h"
#include "stt/SteinerTreeBuilder.h"
#include "utl/Logger.h"

namespace cts {

class CtsOptions
{
 public:
  CtsOptions(utl::Logger* logger, stt::SteinerTreeBuilder* sttBuildder)
      : logger_(logger), sttBuilder_(sttBuildder)
  {
  }

  void setBlockName(const std::string& blockName) { blockName_ = blockName; }
  std::string getBlockName() const { return blockName_; }

  void setClockNets(const std::string& clockNets) { clockNets_ = clockNets; }
  std::string getClockNets() const { return clockNets_; }
  void setRootBuffer(const std::string& buffer) { rootBuffer_ = buffer; }
  std::string getRootBuffer() const { return rootBuffer_; }
  void setBufferList(std::vector<std::string> buffers)
  {
    bufferList_ = buffers;
  }
  std::vector<std::string> getBufferList() const { return bufferList_; }
  void setDbUnits(int units) { dbUnits_ = units; }
  int getDbUnits() const { return dbUnits_; }
  void setWireSegmentUnit(unsigned wireSegmentUnit)
  {
    wireSegmentUnit_ = wireSegmentUnit;
  }
  unsigned getWireSegmentUnit() const { return wireSegmentUnit_; }
  void setPlotSolution(bool plot) { plotSolution_ = plot; }
  bool getPlotSolution() const { return plotSolution_; }
  void setSimpleCts(bool enable) { simpleCts_ = enable; }
  bool getSimpleCts() const { return simpleCts_; }
  void setSinkClustering(bool enable) { sinkClusteringEnable_ = enable; }
  bool getSinkClustering() const { return sinkClusteringEnable_; }
  void setSinkClusteringUseMaxCap(bool useMaxCap)
  {
    sinkClusteringUseMaxCap_ = useMaxCap;
  }
  bool getSinkClusteringUseMaxCap() const { return sinkClusteringUseMaxCap_; }
  void setNumMaxLeafSinks(unsigned numSinks) { numMaxLeafSinks_ = numSinks; }
  unsigned getNumMaxLeafSinks() const { return numMaxLeafSinks_; }
  void setMaxSlew(unsigned slew) { maxSlew_ = slew; }
  unsigned getMaxSlew() const { return maxSlew_; }
  void setMaxCharSlew(double slew) { maxCharSlew_ = slew; }
  double getMaxCharSlew() const { return maxCharSlew_; }
  void setMaxCharCap(double cap) { maxCharCap_ = cap; }
  double getMaxCharCap() const { return maxCharCap_; }
  void setCharLoadIterations(unsigned loadIterations)
  {
    charLoadIterations_ = loadIterations;
  }
  unsigned getCharLoadIterations() const { return charLoadIterations_; }
  void setCharSlewIterations(unsigned slewIterations)
  {
    charSlewIterations_ = slewIterations;
  }
  unsigned getCharSlewIterations() const { return charSlewIterations_; }
  void setCharWirelengthIterations(unsigned wirelengthIterations)
  {
    charWirelengthIterations_ = wirelengthIterations;
  }
  unsigned getCharWirelengthIterations() const
  {
    return charWirelengthIterations_;
  }
  void setOutputPath(const std::string& path) { outputPath_ = path; }
  std::string getOutputPath() const { return outputPath_; }
  void setCapInter(double cap) { capInter_ = cap; }
  double getCapInter() const { return capInter_; }
  void setSlewInter(double slew) { slewInter_ = slew; }
  double getSlewInter() const { return slewInter_; }
  void setClockTreeMaxDepth(unsigned depth) { clockTreeMaxDepth_ = depth; }
  unsigned getClockTreeMaxDepth() const { return clockTreeMaxDepth_; }
  void setEnableFakeLutEntries(bool enable) { enableFakeLutEntries_ = enable; }
  unsigned isFakeLutEntriesEnabled() const { return enableFakeLutEntries_; }
  void setForceBuffersOnLeafLevel(bool force)
  {
    forceBuffersOnLeafLevel_ = force;
  }
  bool forceBuffersOnLeafLevel() const { return forceBuffersOnLeafLevel_; }
  void setWriteOnlyClockNets(bool writeOnlyClk)
  {
    writeOnlyClockNets_ = writeOnlyClk;
  }
  bool writeOnlyClockNets() const { return writeOnlyClockNets_; }
  void setRunPostCtsOpt(bool run) { runPostCtsOpt_ = run; }
  bool runPostCtsOpt() { return runPostCtsOpt_; }
  void setBufDistRatio(double ratio) { bufDistRatio_ = ratio; }
  double getBufDistRatio() { return bufDistRatio_; }
  void setClockNetsObjs(std::vector<odb::dbNet*> nets)
  {
    clockNetsObjs_ = nets;
  }
  std::vector<odb::dbNet*> getClockNetsObjs() const { return clockNetsObjs_; }
  void setMetricsFile(const std::string& metricFile)
  {
    metricFile_ = metricFile;
  }
  std::string getMetricsFile() const { return metricFile_; }
  void setNumClockRoots(unsigned roots) { clockRoots_ = roots; }
  long int getNumClockRoots() const { return clockRoots_; }
  void setNumClockSubnets(long int nets) { clockSubnets_ = nets; }
  long int getNumClockSubnets() const { return clockSubnets_; }
  void setNumBuffersInserted(long int buffers) { buffersInserted_ = buffers; }
  long int getNumBuffersInserted() const { return buffersInserted_; }
  void setNumSinks(long int sinks) { sinks_ = sinks; }
  long int getNumSinks() const { return sinks_; }
  void setTreeBuffer(const std::string& buffer) { treeBuffer_ = buffer; }
  std::string getTreeBuffer() const { return treeBuffer_; }
  unsigned getClusteringPower() const { return clusteringPower_; }
  void setClusteringPower(unsigned power) { clusteringPower_ = power; }
  double getClusteringCapacity() const { return clusteringCapacity_; }
  void setClusteringCapacity(double capacity)
  {
    clusteringCapacity_ = capacity;
  }
  double getBufferDistance() const { return bufDistance_; }
  void setBufferDistance(double distance) { bufDistance_ = distance; }
  double getVertexBufferDistance() const { return vertexBufDistance_; }
  void setVertexBufferDistance(double distance)
  {
    vertexBufDistance_ = distance;
  }
  bool isVertexBuffersEnabled() const { return vertexBuffersEnable_; }
  void setVertexBuffersEnabled(bool enable) { vertexBuffersEnable_ = enable; }
  bool isSimpleSegmentEnabled() const { return simpleSegmentsEnable_; }
  void setSimpleSegmentsEnabled(bool enable) { simpleSegmentsEnable_ = enable; }
  double getMaxDiameter() const { return maxDiameter_; }
  void setMaxDiameter(double distance)
  {
    maxDiameter_ = distance;
    sinkClusteringUseMaxCap_ = false;
  }
  unsigned getSizeSinkClustering() const { return sinkClustersSize_; }
  void setSizeSinkClustering(unsigned size)
  {
    sinkClustersSize_ = size;
    sinkClusteringUseMaxCap_ = false;
  }
  unsigned getSinkClusteringLevels() const { return sinkClusteringLevels_; }
  void setSinkClusteringLevels(unsigned levels)
  {
    sinkClusteringLevels_ = levels;
  }
  unsigned getNumStaticLayers() const { return numStaticLayers_; }
  void setBalanceLevels(bool balance) { balanceLevels_ = balance; }
  bool getBalanceLevels() const { return balanceLevels_; }
  void setNumStaticLayers(unsigned num) { numStaticLayers_ = num; }
  void setSinkBuffer(const std::string& buffer) { sinkBuffer_ = buffer; }
  void setSinkBufferMaxCap(double cap) { sinkBufferMaxCap_ = cap; }
  double getSinkBufferMaxCap() const { return sinkBufferMaxCap_; }
  void setSinkBufferInputCap(double cap) { sinkBufferInputCap_ = cap; }
  double getSinkBufferInputCap() const { return sinkBufferInputCap_; }
  std::string getSinkBuffer() const { return sinkBuffer_; }
  utl::Logger* getLogger() { return logger_; }
  stt::SteinerTreeBuilder* getSttBuilder() { return sttBuilder_; }

 private:
  std::string blockName_ = "";
  std::string outputPath_ = "";
  std::string clockNets_ = "";
  std::string rootBuffer_ = "";
  std::string sinkBuffer_ = "";
  std::string treeBuffer_ = "";
  std::string metricFile_ = "";
  int dbUnits_ = -1;
  unsigned wireSegmentUnit_ = 0;
  bool plotSolution_ = false;
  bool simpleCts_ = false;
  bool sinkClusteringEnable_ = true;
  bool sinkClusteringUseMaxCap_ = true;
  bool simpleSegmentsEnable_ = false;
  bool vertexBuffersEnable_ = false;
  double vertexBufDistance_ = 240;
  double bufDistance_ = 100;
  double clusteringCapacity_ = 0.6;
  unsigned clusteringPower_ = 4;
  unsigned numMaxLeafSinks_ = 15;
  unsigned maxSlew_ = 4;
  double maxCharSlew_ = 0;
  double maxCharCap_ = 0;
  double sinkBufferMaxCap_ = 0;
  double sinkBufferInputCap_ = 0;
  double capInter_ = 0;
  double slewInter_ = 0;
  unsigned charWirelengthIterations_ = 4;
  unsigned charLoadIterations_ = 34;
  unsigned charSlewIterations_ = 12;
  unsigned clockTreeMaxDepth_ = 100;
  bool enableFakeLutEntries_ = true;
  bool forceBuffersOnLeafLevel_ = true;
  bool writeOnlyClockNets_ = false;
  bool runPostCtsOpt_ = true;
  double bufDistRatio_ = 0.1;
  long int clockRoots_ = 0;
  long int clockSubnets_ = 0;
  long int buffersInserted_ = 0;
  long int sinks_ = 0;
  double maxDiameter_ = 50;
  unsigned sinkClustersSize_ = 20;
  bool balanceLevels_ = false;
  unsigned sinkClusteringLevels_ = 0;
  unsigned numStaticLayers_ = 0;
  std::vector<std::string> bufferList_;
  std::vector<odb::dbNet*> clockNetsObjs_;
  utl::Logger* logger_;
  stt::SteinerTreeBuilder* sttBuilder_;
};

}  // namespace cts
