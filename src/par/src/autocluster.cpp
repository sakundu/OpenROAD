#include "autocluster.h"

#include <sys/stat.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <queue>
#include <string>
#include <tuple>
#include <vector>

#include "MLPart.h"
#include "odb/db.h"
#include "sta/ArcDelayCalc.hh"
#include "sta/Bfs.hh"
#include "sta/Corner.hh"
#include "sta/DcalcAnalysisPt.hh"
#include "sta/ExceptionPath.hh"
#include "sta/FuncExpr.hh"
#include "sta/Graph.hh"
#include "sta/GraphDelayCalc.hh"
#include "sta/Liberty.hh"
#include "sta/Network.hh"
#include "sta/PathAnalysisPt.hh"
#include "sta/PathEnd.hh"
#include "sta/PathExpanded.hh"
#include "sta/PathRef.hh"
#include "sta/PatternMatch.hh"
#include "sta/PortDirection.hh"
#include "sta/Sdc.hh"
#include "sta/Search.hh"
#include "sta/SearchPred.hh"
#include "sta/Sequential.hh"
#include "sta/Sta.hh"
#include "sta/Units.hh"
#include "utl/Logger.h"

using utl::PAR;

namespace par {
using std::ceil;
using std::cout;
using std::endl;
using std::find;
using std::floor;
using std::map;
using std::max;
using std::min;
using std::ofstream;
using std::pair;
using std::pow;
using std::queue;
using std::sort;
using std::string;
using std::to_string;
using std::tuple;
using std::vector;

using odb::dbBlock;
using odb::dbBox;
using odb::dbBTerm;
using odb::dbDatabase;
using odb::dbInst;
using odb::dbITerm;
using odb::dbMaster;
using odb::dbMPin;
using odb::dbMTerm;
using odb::dbSet;
using odb::dbSigType;
using odb::dbStringProperty;
using odb::Rect;

using sta::Cell;
using sta::Instance;
using sta::InstanceChildIterator;
using sta::InstancePinIterator;
using sta::LeafInstanceIterator;
using sta::LibertyCell;
using sta::LibertyCellPortIterator;
using sta::LibertyPort;
using sta::Net;
using sta::NetConnectedPinIterator;
using sta::NetIterator;
using sta::NetPinIterator;
using sta::NetTermIterator;
using sta::Pin;
using sta::PinSeq;
using sta::PortDirection;
using sta::Term;

// ******************************************************************************
// Class Cluster
// ******************************************************************************
float Cluster::calculateArea(ord::dbVerilogNetwork* network) const
{
  float area = 0.0;
  for (auto* inst : inst_vec_) {
    const LibertyCell* liberty_cell = network->libertyCell(inst);
    area += liberty_cell->area();
  }

  for (auto* macro : macro_vec_) {
    const LibertyCell* liberty_cell = network->libertyCell(macro);
    area += liberty_cell->area();
  }

  return area;
}

void Cluster::calculateNumSeq(ord::dbVerilogNetwork* network)
{
  for (auto* inst : inst_vec_) {
    const LibertyCell* lib_cell = network->libertyCell(inst);
    if (lib_cell->hasSequentials())
      num_seq_ += 1;
  }
}

// ******************************************************************************
// Class AutoClusterMgr
// ******************************************************************************
//
//  traverseLogicalHierarchy
//  Recursive function to collect the design metrics (number of instances, hard
//  macros, area) in the logical hierarchy
//
Metric AutoClusterMgr::computeMetrics(sta::Instance* inst)
{
  float area = 0.0;
  unsigned int num_inst = 0;
  unsigned int num_macro = 0;

  InstanceChildIterator* child_iter = network_->childIterator(inst);
  while (child_iter->hasNext()) {
    Instance* child = child_iter->next();
    if (network_->isHierarchical(child)) {
      const Metric metric = computeMetrics(child);
      area += metric.area;
      num_inst += metric.num_inst;
      num_macro += metric.num_macro;
    } else {
      const LibertyCell* liberty_cell = network_->libertyCell(child);
      area += liberty_cell->area();
      if (liberty_cell->isBuffer()) {
        num_buffer_ += 1;
        area_buffer_ += liberty_cell->area();
        buffer_map_[child] = ++buffer_id_;
      }

      const Cell* cell = network_->cell(child);
      const char* cell_name = network_->name(cell);
      const dbMaster* master = db_->findMaster(cell_name);
      if (master->isBlock()) {
        num_macro += 1;
        macros_.push_back(child);
      } else {
        num_inst += 1;
      }
    }
  }
  const Metric metric = Metric(area, num_macro, num_inst);
  logical_cluster_map_[inst] = metric;
  return metric;
}

static bool isConnectedNet(const pair<const Net*, const Net*>& p1,
                           const pair<const Net*, const Net*>& p2)
{
  if (p1.first != nullptr) {
    if (p1.first == p2.first || p1.first == p2.second)
      return true;
  }

  if (p1.second != nullptr) {
    if (p1.second == p2.first || p1.second == p2.second)
      return true;
  }

  return false;
}

static void appendNet(vector<const Net*>& vec,
                      const pair<const Net*, const Net*>& p)
{
  if (p.first != nullptr) {
    if (find(vec.begin(), vec.end(), p.first) == vec.end())
      vec.push_back(p.first);
  }

  if (p.second != nullptr) {
    if (find(vec.begin(), vec.end(), p.second) == vec.end())
      vec.push_back(p.second);
  }
}

//
// Handle Buffer transparency for handling net connection across buffers
//
void AutoClusterMgr::getBufferNet()
{
  vector<pair<const Net*, const Net*>> buffer_net;
  for (int i = 0; i <= buffer_id_; i++) {
    buffer_net.push_back({nullptr, nullptr});
  }
  getBufferNetUtil(network_->topInstance(), buffer_net);

  vector<int> class_array(buffer_id_ + 1);
  for (int i = 0; i <= buffer_id_; i++)
    class_array[i] = i;

  int unique_id = 0;

  for (int i = 0; i <= buffer_id_; i++) {
    if (class_array[i] == i)
      class_array[i] = unique_id++;

    for (int j = i + 1; j <= buffer_id_; j++) {
      if (isConnectedNet(buffer_net[i], buffer_net[j]))
        class_array[j] = class_array[i];
    }
  }

  buffer_net_vec_.resize(unique_id);

  for (int i = 0; i <= buffer_id_; i++) {
    appendNet(buffer_net_vec_[class_array[i]], buffer_net[i]);
  }
}

void AutoClusterMgr::getBufferNetUtil(
    const Instance* inst,
    vector<pair<const Net*, const Net*>>& buffer_net)
{
  const bool is_top = (inst == network_->topInstance());
  NetIterator* net_iter = network_->netIterator(inst);
  while (net_iter->hasNext()) {
    const Net* net = net_iter->next();
    bool with_buffer = false;
    if (is_top || !hasTerminals(net)) {
      NetConnectedPinIterator* pin_iter = network_->connectedPinIterator(net);
      while (pin_iter->hasNext()) {
        const Pin* pin = pin_iter->next();
        if (network_->isLeaf(pin)) {
          const Instance* child_inst = network_->instance(pin);
          const LibertyCell* liberty_cell = network_->libertyCell(child_inst);
          if (liberty_cell->isBuffer()) {
            with_buffer = true;
            const int buffer_id = buffer_map_[child_inst];

            if (buffer_net[buffer_id].first == nullptr)
              buffer_net[buffer_id].first = net;
            else if (buffer_net[buffer_id].second == nullptr)
              buffer_net[buffer_id].second = net;
            else
              logger_->error(
                  PAR, 401, "Buffer Net has more than two net connection...");
          }
        }
      }

      if (with_buffer == true) {
        buffer_net_list_.push_back(net);
      }
    }
  }

  delete net_iter;

  InstanceChildIterator* child_iter = network_->childIterator(inst);
  while (child_iter->hasNext()) {
    const Instance* child = child_iter->next();
    getBufferNetUtil(child, buffer_net);
  }

  delete child_iter;
}

//
//  Create a bundled model for external pins.  Group boundary pins into bundles
//  Currently creates 3 groups per side
//  TODO
//     1) Make it customizable  number of bins per side
//     2) Use area to determine bundles
//     3) handle rectilinear boundaries and area pins (3D connections)
//
void AutoClusterMgr::createBundledIO()
{
  // Get the floorplan information

  Rect die_box;
  block_->getDieArea(die_box);

  floorplan_lx_ = die_box.xMin();
  floorplan_ly_ = die_box.yMin();
  floorplan_ux_ = die_box.xMax();
  floorplan_uy_ = die_box.yMax();

  // Map all the BTerms to IORegions
  for (auto term : block_->getBTerms()) {
    const std::string bterm_name = term->getName();
    int lx = INT_MAX;
    int ly = INT_MAX;
    int ux = 0;
    int uy = 0;
    for (const auto pin : term->getBPins()) {
      for (const auto box : pin->getBoxes()) {
        lx = min(lx, box->xMin());
        ly = min(ly, box->yMin());
        ux = max(ux, box->xMax());
        uy = max(uy, box->yMax());
      }
    }

    const int x_third = floorplan_ux_ / 3;
    const int y_third = floorplan_uy_ / 3;

    if (lx == floorplan_lx_) {  // Left
      if (uy <= y_third)
        bterm_map_[bterm_name] = LeftLower;
      else if (ly >= 2 * y_third)
        bterm_map_[bterm_name] = LeftUpper;
      else
        bterm_map_[bterm_name] = LeftMiddle;
      L_pin_.push_back((ly + uy) / 2.0);
    } else if (ux == floorplan_ux_) {  // Right
      if (uy <= y_third)
        bterm_map_[bterm_name] = RightLower;
      else if (ly >= 2 * y_third)
        bterm_map_[bterm_name] = RightUpper;
      else
        bterm_map_[bterm_name] = RightMiddle;
      R_pin_.push_back((ly + uy) / 2.0);
    } else if (ly == floorplan_ly_) {  // Bottom
      if (ux <= x_third)
        bterm_map_[bterm_name] = BottomLower;
      else if (lx >= 2 * x_third)
        bterm_map_[bterm_name] = BottomUpper;
      else
        bterm_map_[bterm_name] = BottomMiddle;
      B_pin_.push_back((lx + ux) / 2.0);
    } else if (uy == floorplan_uy_) {  // Top
      if (ux <= x_third)
        bterm_map_[bterm_name] = TopLower;
      else if (lx >= 2 * x_third)
        bterm_map_[bterm_name] = TopUpper;
      else
        bterm_map_[bterm_name] = TopMiddle;
      T_pin_.push_back((lx + ux) / 2.0);
    } else {
      logger_->error(
          PAR, 400, "Floorplan has not been initialized? Pin location error.");
    }
  }
}

void AutoClusterMgr::createCluster(int& cluster_id)
{
  // This function will only be called by top instance
  const Instance* inst = network_->topInstance();
  const Metric metric = logical_cluster_map_[inst];
  bool is_hier = false;
  if (metric.num_macro > max_num_macro_ || metric.num_inst > max_num_inst_) {
    InstanceChildIterator* child_iter = network_->childIterator(inst);
    vector<const Instance*> glue_inst_vec;
    while (child_iter->hasNext()) {
      const Instance* child = child_iter->next();
      if (network_->isHierarchical(child)) {
        createClusterUtil(child, cluster_id);
        is_hier = true;
      } else
        glue_inst_vec.push_back(child);
    }

    // Create cluster for glue logic
    if (glue_inst_vec.size() >= 1) {
      string name = "top";
      if (!is_hier)
        name += "_glue_logic";
      Cluster* cluster = new Cluster(++cluster_id, name);
      for (const Instance* inst : glue_inst_vec) {
        const LibertyCell* liberty_cell = network_->libertyCell(inst);
        if (liberty_cell->isBuffer() == true)
          continue;

        const Cell* cell = network_->cell(inst);
        const char* cell_name = network_->name(cell);
        const dbMaster* master = db_->findMaster(cell_name);
        if (master->isBlock())
          cluster->addMacro(inst);
        else
          cluster->addInst(inst);
        inst_map_[inst] = cluster_id;
      }
      cluster_map_[cluster_id] = cluster;

      if (cluster->getNumInst() >= min_num_inst_
          || cluster->getNumMacro() >= min_num_macro_) {
        cluster_list_.push_back(cluster);
      } else {
        merge_cluster_list_.push_back(cluster);
      }
    }
  } else {
    // This no need to do any clustering
    Cluster* cluster = new Cluster(++cluster_id, string("top_instance"));
    cluster_map_[cluster_id] = cluster;
    cluster->setTopInst(inst);
    cluster->addLogicalModule(string("top_instance"));
    LeafInstanceIterator* leaf_iter = network_->leafInstanceIterator(inst);
    while (leaf_iter->hasNext()) {
      const Instance* leaf_inst = leaf_iter->next();
      const LibertyCell* liberty_cell = network_->libertyCell(leaf_inst);
      if (liberty_cell->isBuffer() == false) {
        const Cell* cell = network_->cell(leaf_inst);
        const char* cell_name = network_->name(cell);
        const dbMaster* master = db_->findMaster(cell_name);
        if (master->isBlock())
          cluster->addMacro(leaf_inst);
        else
          cluster->addInst(leaf_inst);

        inst_map_[leaf_inst] = cluster_id;
      }
    }
    cluster_list_.push_back(cluster);
  }
}

void AutoClusterMgr::createClusterUtil(const Instance* inst, int& cluster_id)
{
  Cluster* cluster
      = new Cluster(++cluster_id, string(network_->pathName(inst)));
  cluster->setTopInst(inst);
  cluster->addLogicalModule(string(network_->pathName(inst)));
  cluster_map_[cluster_id] = cluster;
  LeafInstanceIterator* leaf_iter = network_->leafInstanceIterator(inst);
  while (leaf_iter->hasNext()) {
    const Instance* leaf_inst = leaf_iter->next();
    const LibertyCell* liberty_cell = network_->libertyCell(leaf_inst);
    if (liberty_cell->isBuffer() == false) {
      const Cell* cell = network_->cell(leaf_inst);
      const char* cell_name = network_->name(cell);
      const dbMaster* master = db_->findMaster(cell_name);
      if (master->isBlock())
        cluster->addMacro(leaf_inst);
      else
        cluster->addInst(leaf_inst);

      inst_map_[leaf_inst] = cluster_id;
    }
  }

  if (cluster->getNumMacro() >= max_num_macro_
      || cluster->getNumInst() >= max_num_inst_) {
    cluster_list_.push_back(cluster);
    break_cluster_list_.push(cluster);
  } else if (cluster->getNumMacro() >= min_num_macro_
             || cluster->getNumInst() >= min_num_inst_) {
    cluster_list_.push_back(cluster);
  } else {
    merge_cluster_list_.push_back(cluster);
  }
}

void AutoClusterMgr::updateConnection()
{
  for (auto [id, cluster] : cluster_map_)
    cluster->initConnection();

  calculateConnection(network_->topInstance());
  calculateBufferNetConnection();
}

bool AutoClusterMgr::hasTerminals(const Net* net)
{
  NetTermIterator* term_iter = network_->termIterator(net);
  const bool has_terms = term_iter->hasNext();
  delete term_iter;
  return has_terms;
}

void AutoClusterMgr::calculateBufferNetConnection()
{
  for (int i = 0; i < buffer_net_vec_.size(); i++) {
    int driver_id = 0;
    vector<int> loads_id;
    for (int j = 0; j < buffer_net_vec_[i].size(); j++) {
      const Net* net = buffer_net_vec_[i][j];
      const bool is_top = network_->instance(net) == network_->topInstance();
      if (is_top || !hasTerminals(net)) {
        NetConnectedPinIterator* pin_iter = network_->connectedPinIterator(net);
        while (pin_iter->hasNext()) {
          const Pin* pin = pin_iter->next();
          if (network_->isTopLevelPort(pin)) {
            const char* port_name = network_->portName(pin);
            const int id = bundled_io_map_[bterm_map_[string(port_name)]];
            const PortDirection* port_dir = network_->direction(pin);
            if (port_dir == PortDirection::input()) {
              driver_id = id;
            } else {
              loads_id.push_back(id);
            }
          } else if (network_->isLeaf(pin)) {
            const Instance* inst = network_->instance(pin);
            const LibertyCell* liberty_cell = network_->libertyCell(inst);
            if (liberty_cell->isBuffer() == false) {
              const PortDirection* port_dir = network_->direction(pin);
              const int id = inst_map_[inst];
              if (port_dir == PortDirection::output()) {
                driver_id = id;
              } else {
                loads_id.push_back(id);
              }
            }
          }
        }
      }
    }

    if (driver_id != 0 && loads_id.size() > 0) {
      for (int i = 0; i < loads_id.size(); i++) {
        if (driver_id != loads_id[i]) {
          cluster_map_[driver_id]->addOutputConnection(loads_id[i]);
          cluster_map_[loads_id[i]]->addInputConnection(driver_id);
        }
      }
    }
  }
}

void AutoClusterMgr::calculateConnection(const Instance* inst)
{
  const bool is_top = (inst == network_->topInstance());
  NetIterator* net_iter = network_->netIterator(inst);
  while (net_iter->hasNext()) {
    const Net* net = net_iter->next();
    int driver_id = 0;
    vector<int> loads_id;
    bool buffer_flag = false;
    if (find(buffer_net_list_.begin(), buffer_net_list_.end(), net)
        != buffer_net_list_.end())
      buffer_flag = true;

    if ((buffer_flag == false) && (is_top || !hasTerminals(net))) {
      NetConnectedPinIterator* pin_iter = network_->connectedPinIterator(net);
      while (pin_iter->hasNext()) {
        const Pin* pin = pin_iter->next();
        if (network_->isTopLevelPort(pin)) {
          const char* port_name = network_->portName(pin);
          const int id = bundled_io_map_[bterm_map_[string(port_name)]];
          const PortDirection* port_dir = network_->direction(pin);
          if (port_dir == PortDirection::input()) {
            driver_id = id;
          } else {
            loads_id.push_back(id);
          }
        } else if (network_->isLeaf(pin)) {
          const Instance* inst = network_->instance(pin);
          const PortDirection* port_dir = network_->direction(pin);
          const int id = inst_map_[inst];
          if (port_dir == PortDirection::output()) {
            driver_id = id;
          } else {
            loads_id.push_back(id);
          }
        }
      }

      if (driver_id != 0 && loads_id.size() > 0) {
        for (int i = 0; i < loads_id.size(); i++) {
          if (loads_id[i] != driver_id) {
            cluster_map_[driver_id]->addOutputConnection(loads_id[i]);
            cluster_map_[loads_id[i]]->addInputConnection(driver_id);
          }
        }
      }
    }
  }

  delete net_iter;

  InstanceChildIterator* child_iter = network_->childIterator(inst);
  while (child_iter->hasNext()) {
    const Instance* child = child_iter->next();
    calculateConnection(child);
  }

  delete child_iter;
}

void AutoClusterMgr::merge(const string& parent_name)
{
  if (merge_cluster_list_.size() == 0)
    return;

  if (merge_cluster_list_.size() == 1) {
    cluster_list_.push_back(merge_cluster_list_[0]);
    merge_cluster_list_.clear();
    updateConnection();
    return;
  }

  unsigned int num_inst = calculateClusterNumInst(merge_cluster_list_);
  unsigned int num_macro = calculateClusterNumMacro(merge_cluster_list_);
  int merge_index = 0;
  while (num_inst > max_num_inst_ || num_macro > max_num_macro_) {
    const int num_merge_cluster = merge_cluster_list_.size();
    mergeUtil(parent_name, merge_index);
    if (num_merge_cluster == merge_cluster_list_.size())
      break;

    num_inst = calculateClusterNumInst(merge_cluster_list_);
    num_macro = calculateClusterNumMacro(merge_cluster_list_);
  }

  if (merge_cluster_list_.size() > 1)
    for (int i = 1; i < merge_cluster_list_.size(); i++) {
      mergeCluster(merge_cluster_list_[0], merge_cluster_list_[i]);
      delete merge_cluster_list_[i];
    }

  if (merge_cluster_list_.size() > 0) {
    merge_cluster_list_[0]->setName(parent_name + string("_cluster_")
                                    + to_string(merge_index++));
    cluster_list_.push_back(merge_cluster_list_[0]);
    merge_cluster_list_.clear();
  }
  updateConnection();
}

unsigned int AutoClusterMgr::calculateClusterNumMacro(
    const vector<Cluster*>& cluster_vec)
{
  unsigned int num_macro = 0;
  for (const auto cluster : cluster_vec)
    num_macro += cluster->getNumMacro();

  return num_macro;
}

unsigned int AutoClusterMgr::calculateClusterNumInst(
    const vector<Cluster*>& cluster_vec)
{
  unsigned int num_inst = 0;
  for (const auto cluster : cluster_vec)
    num_inst += cluster->getNumInst();

  return num_inst;
}

//
// Merge target cluster into src
// Target cluster will be deleted outside the function
//
void AutoClusterMgr::mergeCluster(Cluster* src, const Cluster* target)
{
  const int src_id = src->getId();
  const int target_id = target->getId();
  cluster_map_.erase(target_id);
  src->addLogicalModuleVec(target->getLogicalModuleVec());

  for (const auto inst : target->getInsts()) {
    src->addInst(inst);
    inst_map_[inst] = src_id;
  }

  for (auto macro : target->getMacros()) {
    src->addMacro(macro);
    inst_map_[macro] = src_id;
  }
}

void AutoClusterMgr::mergeUtil(const string& parent_name, int& merge_index)
{
  vector<int> outside_vec;
  vector<int> merge_vec;

  for (const auto cluster : cluster_list_)
    outside_vec.push_back(cluster->getId());

  for (const auto cluster : merge_cluster_list_)
    merge_vec.push_back(cluster->getId());

  const int M = merge_vec.size();
  const int N = outside_vec.size();
  vector<bool> internal_flag(M);
  vector<int> class_id(M);
  vector<vector<bool>> graph(M);

  for (int i = 0; i < M; i++) {
    graph[i].resize(N);
    internal_flag[i] = true;
    class_id[i] = i;
  }

  for (int i = 0; i < M; i++) {
    for (int j = 0; j < N; j++) {
      const unsigned int input
          = merge_cluster_list_[i]->getInputConnection(outside_vec[j]);
      const unsigned int output
          = merge_cluster_list_[i]->getOutputConnection(outside_vec[j]);
      if (input + output > net_threshold_) {
        graph[i][j] = true;
        internal_flag[i] = false;
      }
    }
  }

  for (int i = 0; i < M; i++) {
    if (internal_flag[i] == false && class_id[i] == i) {
      for (int j = i + 1; j < M; j++) {
        bool flag = true;
        for (int k = 0; k < N; k++) {
          if (flag == false)
            break;
          flag = flag && (graph[i][k] == graph[j][k]);
        }
        if (flag == true)
          class_id[j] = i;
      }
    }
  }

  // Merge clusters with same connection topology
  for (int i = 0; i < M; i++) {
    if (internal_flag[i] == false && class_id[i] == i) {
      for (int j = i + 1; j < M; j++) {
        if (class_id[j] == i) {
          mergeCluster(merge_cluster_list_[i], merge_cluster_list_[j]);
        }
      }
    }
  }

  vector<Cluster*> temp_cluster_vec;
  for (int i = 0; i < M; i++) {
    if (class_id[i] == i) {
      const int num_inst = merge_cluster_list_[i]->getNumInst();
      const int num_macro = merge_cluster_list_[i]->getNumMacro();
      if (num_inst >= min_num_inst_ || num_macro >= min_num_macro_) {
        cluster_list_.push_back(merge_cluster_list_[i]);
        merge_cluster_list_[i]->setName(parent_name + string("_cluster_")
                                        + to_string(merge_index++));
      } else {
        temp_cluster_vec.push_back(merge_cluster_list_[i]);
      }
    } else {
      delete merge_cluster_list_[i];
    }
  }

  merge_cluster_list_.clear();
  merge_cluster_list_ = temp_cluster_vec;

  updateConnection();
}

//
// Break a cluser (logical module) into its child modules and create a cluster
// each of the child modules
//
void AutoClusterMgr::breakCluster(Cluster* cluster_old, int& cluster_id)
{
  const Instance* inst = cluster_old->getTopInstance();
  InstanceChildIterator* child_iter = network_->childIterator(inst);
  vector<const Instance*> glue_inst_vec;
  bool is_hier = false;
  while (child_iter->hasNext()) {
    const Instance* child = child_iter->next();
    if (network_->isHierarchical(child)) {
      is_hier = true;
      createClusterUtil(child, cluster_id);
    } else
      glue_inst_vec.push_back(child);
  }

  if (!is_hier) {
    return;
  }

  // Create cluster for glue logic
  if (glue_inst_vec.size() >= 1) {
    const string name = network_->pathName(inst) + string("_glue_logic");
    Cluster* cluster = new Cluster(++cluster_id, name);
    for (auto inst : glue_inst_vec) {
      const LibertyCell* liberty_cell = network_->libertyCell(inst);
      if (liberty_cell->isBuffer() == false) {
        const Cell* cell = network_->cell(inst);
        const char* cell_name = network_->name(cell);
        const dbMaster* master = db_->findMaster(cell_name);
        if (master->isBlock())
          cluster->addMacro(inst);
        else
          cluster->addInst(inst);

        inst_map_[inst] = cluster_id;
      }
    }
    cluster_map_[cluster_id] = cluster;

    //
    // Check cluster size. If it is smaller than min_inst threshold, add it to
    // merge_cluster list
    //
    if (cluster->getNumInst() >= min_num_inst_
        || cluster->getNumMacro() >= min_num_macro_) {
      cluster_list_.push_back(cluster);
    } else {
      merge_cluster_list_.push_back(cluster);
    }
  }

  cluster_map_.erase(cluster_old->getId());
  auto vec_it = find(cluster_list_.begin(), cluster_list_.end(), cluster_old);
  cluster_list_.erase(vec_it);
  delete cluster_old;
  updateConnection();
  merge(string(network_->pathName(inst)));
}

//
// For clusters that are greater than max_inst threshold, use MLPart to break
// the cluster into smaller clusters
//
void AutoClusterMgr::MLPart(Cluster* cluster, int& cluster_id)
{
  const int num_inst = cluster->getNumInst();
  if (num_inst < 2 * min_num_inst_)
    return;

  cluster_map_.erase(cluster->getId());
  auto vec_it = find(cluster_list_.begin(), cluster_list_.end(), cluster);
  cluster_list_.erase(vec_it);

  const int src_id = cluster->getId();
  map<int, const Instance*> idx_to_inst;
  map<const Instance*, int> inst_to_idx;
  vector<double> vertex_weight;
  vector<double> edge_weight;
  vector<int> col_idx;  // edges represented by vertex indices
  vector<int> row_ptr;  // pointers for edges
  int inst_id = 0;
  map<Cluster*, int> node_map;
  // we also consider outside world
  for (int i = 0; i < cluster_list_.size(); i++) {
    vertex_weight.push_back(1.0);
    node_map[cluster_list_[i]] = inst_id++;
  }

  vector<const Instance*> inst_vec = cluster->getInsts();
  for (int i = 0; i < inst_vec.size(); i++) {
    idx_to_inst[inst_id] = inst_vec[i];
    inst_to_idx[inst_vec[i]] = inst_id++;
    vertex_weight.push_back(1.0);
  }

  int count = 0;
  MLPartNetUtil(network_->topInstance(),
                src_id,
                count,
                col_idx,
                row_ptr,
                edge_weight,
                node_map,
                idx_to_inst,
                inst_to_idx);

  MLPartBufferNetUtil(src_id,
                      count,
                      col_idx,
                      row_ptr,
                      edge_weight,
                      node_map,
                      idx_to_inst,
                      inst_to_idx);

  row_ptr.push_back(count);

  // Convert it to MLPart Format
  const int num_vertices = vertex_weight.size();
  const int num_edge = row_ptr.size() - 1;
  const int num_col_idx = col_idx.size();

  vector<double> vertexWeight(num_vertices);
  vector<int> rowPtr(num_edge + 1);
  vector<int> colIdx(num_col_idx);
  vector<double> edgeWeight(num_edge);
  vector<int> part(num_vertices);

  for (int i = 0; i < num_vertices; i++) {
    part[i] = -1;
    vertexWeight[i] = 1.0;
  }

  for (int i = 0; i < num_edge; i++) {
    edgeWeight[i] = 1.0;
    rowPtr[i] = row_ptr[i];
  }

  rowPtr[num_edge] = row_ptr[num_edge];

  for (int i = 0; i < num_col_idx; i++)
    colIdx[i] = col_idx[i];

  // MLPart only support 2-way partition
  const int npart = 2;
  double balanceArray[2] = {0.5, 0.5};
  double tolerance = 0.05;
  unsigned int seed = 0;

  UMpack_mlpart(num_vertices,
                num_edge,
                vertexWeight.data(),
                rowPtr.data(),
                colIdx.data(),
                edgeWeight.data(),
                npart,  // Number of Partitions
                balanceArray,
                tolerance,
                part.data(),
                1,  // Starts Per Run #TODO: add a tcl command
                1,  // Number of Runs
                0,  // Debug Level
                seed);

  const string name_part0 = cluster->getName() + string("_cluster_0");
  const string name_part1 = cluster->getName() + string("_cluster_1");
  Cluster* cluster_part0 = new Cluster(++cluster_id, name_part0);
  const int id_part0 = cluster_id;
  cluster_map_[id_part0] = cluster_part0;
  cluster_list_.push_back(cluster_part0);
  Cluster* cluster_part1 = new Cluster(++cluster_id, name_part1);
  const int id_part1 = cluster_id;
  cluster_map_[id_part1] = cluster_part1;
  cluster_list_.push_back(cluster_part1);
  cluster_part0->addLogicalModuleVec(cluster->getLogicalModuleVec());
  cluster_part1->addLogicalModuleVec(cluster->getLogicalModuleVec());

  for (int i = cluster_list_.size() - 2; i < num_vertices; i++) {
    if (part[i] == 0) {
      cluster_part0->addInst(idx_to_inst[i]);
      inst_map_[idx_to_inst[i]] = id_part0;
    } else {
      cluster_part1->addInst(idx_to_inst[i]);
      inst_map_[idx_to_inst[i]] = id_part1;
    }
  }

  if (cluster_part0->getNumInst() > max_num_inst_)
    mlpart_cluster_list_.push(cluster_part0);

  if (cluster_part1->getNumInst() > max_num_inst_)
    mlpart_cluster_list_.push(cluster_part1);

  delete cluster;
}

void AutoClusterMgr::MLPartNetUtil(const Instance* inst,
                                   const int src_id,
                                   int& count,
                                   vector<int>& col_idx,
                                   vector<int>& row_ptr,
                                   vector<double>& edge_weight,
                                   map<Cluster*, int>& node_map,
                                   map<int, const Instance*>& idx_to_inst,
                                   map<const Instance*, int>& inst_to_idx)
{
  const bool is_top = (inst == network_->topInstance());
  NetIterator* net_iter = network_->netIterator(inst);
  while (net_iter->hasNext()) {
    const Net* net = net_iter->next();
    int driver_id = -1;
    vector<int> loads_id;
    bool buffer_flag = false;
    if (find(buffer_net_list_.begin(), buffer_net_list_.end(), net)
        != buffer_net_list_.end())
      buffer_flag = true;

    if ((buffer_flag == false) && (is_top || !hasTerminals(net))) {
      NetConnectedPinIterator* pin_iter = network_->connectedPinIterator(net);
      while (pin_iter->hasNext()) {
        const Pin* pin = pin_iter->next();
        if (network_->isTopLevelPort(pin)) {
          const char* port_name = network_->portName(pin);
          int id = bundled_io_map_[bterm_map_[string(port_name)]];
          id = node_map[cluster_map_[id]];
          const PortDirection* port_dir = network_->direction(pin);
          if (port_dir == PortDirection::input()) {
            driver_id = id;
          } else {
            auto vec_iter = find(loads_id.begin(), loads_id.end(), id);
            if (vec_iter == loads_id.end())
              loads_id.push_back(id);
          }
        } else if (network_->isLeaf(pin)) {
          const Instance* inst = network_->instance(pin);
          const PortDirection* port_dir = network_->direction(pin);
          int id = inst_map_[inst];
          if (id == src_id)
            id = inst_to_idx[inst];
          else
            id = node_map[cluster_map_[id]];
          if (port_dir == PortDirection::output()) {
            driver_id = id;
          } else {
            auto vec_iter = find(loads_id.begin(), loads_id.end(), id);
            if (vec_iter == loads_id.end())
              loads_id.push_back(id);
          }
        }
      }

      if (driver_id != -1 && loads_id.size() > 0) {
        row_ptr.push_back(count);
        edge_weight.push_back(1.0);
        col_idx.push_back(driver_id);
        count++;
        for (int i = 0; i < loads_id.size(); i++) {
          col_idx.push_back(loads_id[i]);
          count++;
        }
      }
    }
  }

  delete net_iter;

  InstanceChildIterator* child_iter = network_->childIterator(inst);
  while (child_iter->hasNext()) {
    const Instance* child = child_iter->next();
    MLPartNetUtil(child,
                  src_id,
                  count,
                  col_idx,
                  row_ptr,
                  edge_weight,
                  node_map,
                  idx_to_inst,
                  inst_to_idx);
  }

  delete child_iter;
}

void AutoClusterMgr::MLPartBufferNetUtil(const int src_id,
                                         int& count,
                                         vector<int>& col_idx,
                                         vector<int>& row_ptr,
                                         vector<double>& edge_weight,
                                         map<Cluster*, int>& node_map,
                                         map<int, const Instance*>& idx_to_inst,
                                         map<const Instance*, int>& inst_to_idx)
{
  for (int i = 0; i < buffer_net_vec_.size(); i++) {
    int driver_id = -1;
    vector<int> loads_id;
    for (int j = 0; j < buffer_net_vec_[i].size(); j++) {
      const Net* net = buffer_net_vec_[i][j];
      const bool is_top = network_->instance(net) == network_->topInstance();
      if (is_top || !hasTerminals(net)) {
        NetConnectedPinIterator* pin_iter = network_->connectedPinIterator(net);
        while (pin_iter->hasNext()) {
          const Pin* pin = pin_iter->next();
          if (network_->isTopLevelPort(pin)) {
            const char* port_name = network_->portName(pin);
            int id = bundled_io_map_[bterm_map_[string(port_name)]];
            id = node_map[cluster_map_[id]];
            const PortDirection* port_dir = network_->direction(pin);
            if (port_dir == PortDirection::input()) {
              driver_id = id;
            } else {
              auto vec_iter = find(loads_id.begin(), loads_id.end(), id);
              if (vec_iter == loads_id.end())
                loads_id.push_back(id);
            }
          } else if (network_->isLeaf(pin)) {
            const Instance* inst = network_->instance(pin);
            const LibertyCell* liberty_cell = network_->libertyCell(inst);
            if (liberty_cell->isBuffer() == false) {
              const PortDirection* port_dir = network_->direction(pin);
              int id = inst_map_[inst];
              if (id == src_id)
                id = inst_to_idx[inst];
              else
                id = node_map[cluster_map_[id]];

              if (port_dir == PortDirection::output()) {
                driver_id = id;
              } else {
                auto vec_iter = find(loads_id.begin(), loads_id.end(), id);
                if (vec_iter == loads_id.end())
                  loads_id.push_back(id);
              }
            }
          }
        }
      }
    }

    if (driver_id != -1 && loads_id.size() > 0) {
      row_ptr.push_back(count);
      edge_weight.push_back(1.0);
      col_idx.push_back(driver_id);
      count++;
      for (int i = 0; i < loads_id.size(); i++) {
        col_idx.push_back(loads_id[i]);
        count++;
      }
    }
  }
}

//
//  For a cluster that contains macros, further split groups based on macro
//  size. Identical size macros are grouped together
//
void AutoClusterMgr::MacroPart(Cluster* cluster_old, int& cluster_id)
{
  vector<const Instance*> macro_vec = cluster_old->getMacros();
  map<int, vector<const Instance*>> macro_map;
  for (auto macro : macro_vec) {
    const Cell* cell = network_->cell(macro);
    const char* cell_name = network_->name(cell);
    const dbMaster* master = db_->findMaster(cell_name);
    const int area = master->getWidth() * master->getHeight();
    if (macro_map.find(area) != macro_map.end()) {
      macro_map[area].push_back(macro);
    } else {
      vector<const Instance*> temp_vec;
      temp_vec.push_back(macro);
      macro_map[area] = temp_vec;
    }
  }

  const string parent_name = cluster_old->getName();
  int part_id = 0;

  vector<int> cluster_id_list;
  for (auto& [area, macros] : macro_map) {
    vector<const Instance*> temp_vec = macros;
    const string name = parent_name + "_part_" + to_string(part_id++);
    Cluster* cluster = new Cluster(++cluster_id, name);
    cluster_id_list.push_back(cluster->getId());
    cluster->addLogicalModule(parent_name);
    cluster_list_.push_back(cluster);
    cluster_map_[cluster_id] = cluster;
    virtual_map_[cluster->getId()] = virtual_map_[cluster_old->getId()];
    for (int i = 0; i < temp_vec.size(); i++) {
      inst_map_[temp_vec[i]] = cluster_id;
      cluster->addMacro(temp_vec[i]);
    }
  }

  for (int i = 0; i < cluster_id_list.size(); i++)
    for (int j = i + 1; j < cluster_id_list.size(); j++) {
      virtual_map_[cluster_id_list[i]] = cluster_id_list[j];
    }

  cluster_map_.erase(cluster_old->getId());
  virtual_map_.erase(cluster_old->getId());
  auto vec_it = find(cluster_list_.begin(), cluster_list_.end(), cluster_old);
  cluster_list_.erase(vec_it);
  delete cluster_old;
}

void AutoClusterMgr::printMacroCluster(Cluster* cluster_old, int& cluster_id)
{
  queue<Cluster*> temp_cluster_queue;
  vector<const Instance*> macro_vec = cluster_old->getMacros();
  string module_name = cluster_old->getName();
  for (int i = 0; i < module_name.size(); i++) {
    if (module_name[i] == '/')
      module_name[i] = '*';
  }

  const string block_file_name
      = string("./rtl_mp/") + module_name + string(".txt.block");
  const string net_file_name
      = string("./rtl_mp/") + module_name + string(".txt.net");

  ofstream output_file;
  output_file.open(block_file_name.c_str());
  for (int i = 0; i < macro_vec.size(); i++) {
    const pair<float, float> pin_pos = printPinPos(macro_vec[i]);
    const Cell* cell = network_->cell(macro_vec[i]);
    const char* cell_name = network_->name(cell);
    const dbMaster* master = db_->findMaster(cell_name);
    const float width = master->getWidth() / dbu_;
    const float height = master->getHeight() / dbu_;
    output_file << network_->pathName(macro_vec[i]) << "  ";
    output_file << width << "   " << height << "    ";
    output_file << pin_pos.first << "   " << pin_pos.second << "  ";
    output_file << endl;
    Cluster* cluster
        = new Cluster(++cluster_id, network_->pathName(macro_vec[i]));
    cluster_map_[cluster_id] = cluster;
    inst_map_[macro_vec[i]] = cluster_id;
    cluster->addMacro(macro_vec[i]);
    temp_cluster_queue.push(cluster);
    cluster_list_.push_back(cluster);
  }

  output_file.close();
  updateConnection();

  output_file.open(net_file_name.c_str());
  int net_id = 0;
  for (auto [src_id, cluster] : cluster_map_) {
    map<int, unsigned int> connection_map = cluster->getOutputConnections();
    map<int, unsigned int>::iterator iter = connection_map.begin();
    bool flag = true;
    while (iter != connection_map.end()) {
      if (iter->first != src_id) {
        if (flag == true) {
          output_file << endl;
          output_file << "Net_" << ++net_id << ":  " << endl;
          output_file << "source: " << cluster->getName() << "   ";
          flag = false;
        }
        output_file << cluster_map_[iter->first]->getName() << "   "
                    << iter->second << "   ";
      }
      iter++;
    }
  }
  output_file << endl;
  output_file.close();

  while (!temp_cluster_queue.empty()) {
    Cluster* cluster = temp_cluster_queue.front();
    temp_cluster_queue.pop();
    cluster_map_.erase(cluster->getId());
    vector<Cluster*>::iterator vec_it
        = find(cluster_list_.begin(), cluster_list_.end(), cluster);
    cluster_list_.erase(vec_it);
    delete cluster;
  }

  for (auto macro : macro_vec) {
    inst_map_[macro] = cluster_old->getId();
  }
}

pair<float, float> AutoClusterMgr::printPinPos(const Instance* macro_inst)
{
  const float dbu = db_->getTech()->getDbUnitsPerMicron();
  Rect bbox;
  bbox.mergeInit();
  const Cell* cell = network_->cell(macro_inst);
  const char* cell_name = network_->name(cell);
  dbMaster* master = db_->findMaster(cell_name);
  for (dbMTerm* mterm : master->getMTerms()) {
    if (mterm->getSigType() == odb::dbSigType::SIGNAL) {
      for (dbMPin* mpin : mterm->getMPins()) {
        for (dbBox* box : mpin->getGeometry()) {
          Rect rect;
          box->getBox(rect);
          bbox.merge(rect);
        }
      }
    }
  }
  const float x_center = (bbox.xMin() + bbox.xMax()) / (2 * dbu);
  const float y_center = (bbox.yMin() + bbox.yMax()) / (2 * dbu);
  return std::pair<float, float>(x_center, y_center);
}

void AutoClusterMgr::mergeMacro(const string& parent_name, int std_cell_id)
{
  if (merge_cluster_list_.size() == 0)
    return;

  if (merge_cluster_list_.size() == 1) {
    virtual_map_[merge_cluster_list_[0]->getId()] = std_cell_id;
    cluster_list_.push_back(merge_cluster_list_[0]);
    merge_cluster_list_.clear();
    return;
  }

  int merge_index = 0;
  mergeMacroUtil(parent_name, merge_index, std_cell_id);
}

void AutoClusterMgr::mergeMacroUtil(const string& parent_name,
                                    int& merge_index,
                                    int std_cell_id)
{
  vector<int> outside_vec;
  vector<int> merge_vec;

  for (auto cluster : cluster_list_)
    outside_vec.push_back(cluster->getId());

  for (auto cluster : merge_cluster_list_)
    merge_vec.push_back(cluster->getId());

  const int M = merge_vec.size();
  const int N = outside_vec.size();
  vector<int> class_id(M);
  vector<vector<bool>> graph(M);

  for (int i = 0; i < M; i++) {
    graph[i].resize(N);
    class_id[i] = i;
  }

  for (int i = 0; i < M; i++) {
    for (int j = 0; j < N; j++) {
      const unsigned int input
          = merge_cluster_list_[i]->getInputConnection(outside_vec[j]);
      const unsigned int output
          = merge_cluster_list_[i]->getOutputConnection(outside_vec[j]);
      if (input + output > net_threshold_) {
        graph[i][j] = true;
      } else {
        graph[i][j] = false;
      }
    }
  }

  for (int i = 0; i < M; i++) {
    if (class_id[i] == i) {
      for (int j = i + 1; j < M; j++) {
        bool flag = true;
        for (int k = 0; k < N; k++) {
          if (flag == false)
            break;
          flag = flag && (graph[i][k] == graph[j][k]);
        }
        if (flag == true)
          class_id[j] = i;
      }
    }
  }

  // Merge clusters with same connection topology
  for (int i = 0; i < M; i++) {
    if (class_id[i] == i) {
      for (int j = i + 1; j < M; j++) {
        if (class_id[j] == i) {
          mergeCluster(merge_cluster_list_[i], merge_cluster_list_[j]);
        }
      }
    }
  }

  vector<int> cluster_id_list;
  for (int i = 0; i < M; i++) {
    if (class_id[i] == i) {
      cluster_list_.push_back(merge_cluster_list_[i]);
      virtual_map_[merge_cluster_list_[i]->getId()] = std_cell_id;
      merge_cluster_list_[i]->setName(parent_name + string("_cluster_")
                                      + to_string(merge_index++));
      cluster_id_list.push_back(merge_cluster_list_[i]->getId());
    } else {
      delete merge_cluster_list_[i];
    }
  }
  merge_cluster_list_.clear();
}

// Timing-driven related functions
// Sequential Graph based timing driven
void AutoClusterMgr::findAdjacencies()
{
  sta_->ensureLevelized();
  sta_->ensureClkNetwork();
  sta::SearchPred2 srch_pred(sta_);
  sta::BfsFwdIterator bfs(sta::BfsIndex::other, &srch_pred, sta_);

  // calculate the seed
  calculateSeed();

  // seed the BFS
  seedFaninBfs(bfs);

  for (int i = 0; i < num_hops_; i++) {
    // Propagate fanins through combinational logics
    findFanins(bfs);
    logger_->info(PAR, 490, "Number of hops:  {}", i);
    // add timing weights
    addTimingWeight(1.0 / pow(2.0, i * 1.0));
    // Propagate fanins through register D->Q
    copyFaninsAcrossRegisters(bfs);
  }
}

void AutoClusterMgr::calculateSeed()
{
  seeds_ = macros_;
}

void AutoClusterMgr::addFanin(sta::Vertex* vertex, Pin* pin, int num_bit)
{
  vertex_fanins_[vertex][pin] = 1;
}

void AutoClusterMgr::seedFaninBfs(sta::BfsFwdIterator& bfs)
{
  sta::dbNetwork* network = sta_->getDbNetwork();
  sta::Graph* graph = sta_->ensureGraph();

  // Seed the BFS with macro output pins (or boundary pins)
  for (auto inst : seeds_) {
    std::string inst_name = network_->pathName(inst);
    dbInst* db_inst = block_->findInst(inst_name.c_str());
    for (dbITerm* iterm : db_inst->getITerms()) {
      sta::Pin* pin = network->dbToSta(iterm);
      if (network->direction(pin)->isAnyOutput() && !sta_->isClock(pin)) {
        pin_inst_map_[pin] = inst;
        sta::Vertex* vertex = graph->pinDrvrVertex(pin);
        addFanin(vertex, pin, 1);
        bfs.enqueueAdjacentVertices(vertex);
      }
    }
  }

  // Seed top level ports input ports
  for (dbBTerm* bterm : block_->getBTerms()) {
    sta::Pin* pin = network->dbToSta(bterm);
    string bterm_name = bterm->getName();
    if (network->direction(pin)->isAnyInput() && !sta_->isClock(pin)) {
      sta::Vertex* vertex = graph->pinDrvrVertex(pin);
      addFanin(vertex, pin, 1);
      bfs.enqueueAdjacentVertices(vertex);
    }
  }
}

void AutoClusterMgr::findFanins(sta::BfsFwdIterator& bfs)
{
  sta::dbNetwork* network = sta_->getDbNetwork();
  sta::Graph* graph = sta_->ensureGraph();
  while (bfs.hasNext()) {
    sta::Vertex* vertex = bfs.next();
    sta::VertexInEdgeIterator fanin_iter(vertex, graph);
    string fanin_name = "";
    while (fanin_iter.hasNext()) {
      sta::Edge* edge = fanin_iter.next();
      sta::Vertex* fanin = edge->from(graph);
      if (fanin->name(network) == fanin_name) {
        continue;
      }
      fanin_name = fanin->name(network);
      // Union fanins sets of fanin vertices
      if (vertex_fanins_.find(fanin) != vertex_fanins_.end()) {
        std::map<Pin*, int> macro_fanin = vertex_fanins_[fanin];
        std::map<Pin*, int>::iterator map_iter = macro_fanin.begin();
        for (; map_iter != macro_fanin.end(); map_iter++) {
          addFanin(vertex, map_iter->first, map_iter->second);
        }
      }
    }
    bfs.enqueueAdjacentVertices(vertex);
  }
}

sta::Pin* AutoClusterMgr::findSeqOutPin(sta::Instance* inst,
                                        sta::LibertyPort* out_port)
{
  sta::dbNetwork* network = sta_->getDbNetwork();
  if (out_port->direction()->isInternal()) {
    sta::InstancePinIterator* pin_iter = network->pinIterator(inst);
    while (pin_iter->hasNext()) {
      sta::Pin* pin = pin_iter->next();
      sta::LibertyPort* lib_port = network->libertyPort(pin);
      if (lib_port->direction()->isAnyOutput()) {
        sta::FuncExpr* func = lib_port->function();
        if (func->hasPort(out_port)) {
          sta::Pin* out_pin = network->findPin(inst, lib_port);
          if (out_pin) {
            delete pin_iter;
            return out_pin;
          }
        }
      }
    }
    delete pin_iter;
    return nullptr;
  } else {
    return network->findPin(inst, out_port);
  }
}

void AutoClusterMgr::copyFaninsAcrossRegisters(sta::BfsFwdIterator& bfs)
{
  std::map<sta::Vertex*, std::map<Pin*, int>> vertex_fanins;
  sta::dbNetwork* network = sta_->getDbNetwork();
  sta::Graph* graph = sta_->ensureGraph();
  sta::Instance* top_inst = network->topInstance();
  sta::LeafInstanceIterator* leaf_iter
      = network->leafInstanceIterator(top_inst);
  while (leaf_iter->hasNext()) {
    sta::Instance* inst = leaf_iter->next();
    sta::LibertyCell* lib_cell = network->libertyCell(inst);
    if (lib_cell->hasSequentials() && !lib_cell->isMacro()) {
      sta::LibertyCellSequentialIterator seq_iter(lib_cell);
      while (seq_iter.hasNext()) {
        sta::Sequential* seq = seq_iter.next();
        sta::FuncExpr* data_expr = seq->data();
        sta::FuncExprPortIterator data_port_iter(data_expr);
        while (data_port_iter.hasNext()) {
          sta::LibertyPort* data_port = data_port_iter.next();
          sta::Pin* data_pin = network->findPin(inst, data_port);
          sta::LibertyPort* out_port = seq->output();
          sta::Pin* out_pin = findSeqOutPin(inst, out_port);
          if (data_pin && out_pin) {
            sta::Vertex* data_vertex = graph->pinLoadVertex(data_pin);
            sta::Vertex* out_vertex = graph->pinDrvrVertex(out_pin);
            // Copy fanins from D to Q on register.
            if (vertex_fanins_.find(data_vertex) != vertex_fanins_.end()) {
              vertex_fanins[out_vertex] = vertex_fanins_[data_vertex];
              bfs.enqueueAdjacentVertices(out_vertex);
            }
          }
        }
      }
    }
  }

  vertex_fanins_.clear();
  vertex_fanins_ = vertex_fanins;
  delete leaf_iter;
}

void AutoClusterMgr::addWeight(int src_id, int target_id, int weight)
{
  if (virtual_timing_map_.find(src_id) != virtual_timing_map_.end())
    if (virtual_timing_map_[src_id].find(target_id)
        != virtual_timing_map_[src_id].end())
      virtual_timing_map_[src_id][target_id] += weight;
    else
      virtual_timing_map_[src_id][target_id] = weight;
  else
    virtual_timing_map_[src_id][target_id] = weight;
}

void AutoClusterMgr::addTimingWeight(float weight)
{
  sta::dbNetwork* network = sta_->getDbNetwork();
  sta::Graph* graph = sta_->ensureGraph();
  weight = weight * timing_weight_;

  virtual_timing_map_.clear();
  virtual_vertex_map_.clear();

  // Find adjacencies from macro input pin fanins (boundary pin fanins)
  for (auto inst : seeds_) {
    std::string inst_name = network_->pathName(inst);
    dbInst* db_inst = block_->findInst(inst_name.c_str());
    virtual_vertex_map_.clear();
    int sink_id = inst_map_[inst];
    for (dbITerm* iterm : db_inst->getITerms()) {
      sta::Pin* pin = network->dbToSta(iterm);
      if (network->direction(pin)->isAnyInput()) {
        sta::Vertex* vertex = graph->pinLoadVertex(pin);
        if (vertex_fanins_.find(vertex) == vertex_fanins_.end())
          continue;

        std::map<Pin*, int> pin_fanins = vertex_fanins_[vertex];
        std::map<Pin*, int>::iterator map_it = pin_fanins.begin();
        for (; map_it != pin_fanins.end(); map_it++) {
          virtual_vertex_map_[sink_id][map_it->first] = 1;
        }
      }
    }

    for (const auto& virtual_vertex : virtual_vertex_map_) {
      int src_id = 0;
      for (const auto& pin_fanin : virtual_vertex.second) {
        std::string src_pin_name = network->pathName(pin_fanin.first);
        if (bterm_map_.find(src_pin_name) != bterm_map_.end())
          src_id = bundled_io_map_[bterm_map_[src_pin_name]];
        else
          src_id = inst_map_[pin_inst_map_[pin_fanin.first]];
        if (src_id != virtual_vertex.first)
          addWeight(src_id, virtual_vertex.first, 1);
      }
    }
  }

  virtual_vertex_map_.clear();
  // Find adjacencies from output pin fanins
  for (dbBTerm* bterm : block_->getBTerms()) {
    string bterm_name = bterm->getName();
    int sink_id = bundled_io_map_[bterm_map_[bterm_name]];
    sta::Pin* pin = network->dbToSta(bterm);
    if (network->direction(pin)->isAnyOutput() && !sta_->isClock(pin)) {
      sta::Vertex* vertex = graph->pinDrvrVertex(pin);
      if (vertex_fanins_.find(vertex) == vertex_fanins_.end())
        continue;
      std::map<Pin*, int> pin_fanins = vertex_fanins_[vertex];
      std::map<Pin*, int>::iterator map_it = pin_fanins.begin();
      for (; map_it != pin_fanins.end(); map_it++) {
        virtual_vertex_map_[sink_id][map_it->first] = 1;
      }
    }
  }

  for (const auto& virtual_vertex : virtual_vertex_map_) {
    int src_id = 0;
    for (const auto& pin_fanin : virtual_vertex.second) {
      std::string src_pin_name = network->pathName(pin_fanin.first);
      if (bterm_map_.find(src_pin_name) != bterm_map_.end())
        src_id = bundled_io_map_[bterm_map_[src_pin_name]];
      else
        src_id = inst_map_[pin_inst_map_[pin_fanin.first]];
      if (src_id != virtual_vertex.first)
        addWeight(src_id, virtual_vertex.first, 1);
    }
  }

  map<int, map<int, int>>::iterator map_iter = virtual_timing_map_.begin();
  for (; map_iter != virtual_timing_map_.end(); map_iter++) {
    int src_id = map_iter->first;
    map<int, int> sinks = map_iter->second;
    for (const auto& sink : sinks) {
      float level_weight = weight;
      bool src_io = src_id <= bundled_io_map_.size();
      bool sink_io = sink.first <= bundled_io_map_.size();
      bool src_macro = cluster_map_[src_id]->getNumMacro() > 0;
      bool sink_macro = cluster_map_[sink.first]->getNumMacro() > 0;
      if ((src_io && sink_io) || (src_io && sink_macro)
          || (src_macro && sink_io))
        level_weight = weight * 100.0;
      else if (src_macro && sink_macro)
        level_weight = weight * 1;
      else
        level_weight = 0.0;
      level_weight = sink.second * level_weight;
      cluster_map_[src_id]->addOutputConnection(sink.first, level_weight);
      cluster_map_[sink.first]->addInputConnection(src_id, level_weight);
    }
  }
}

//
//  Auto clustering by traversing the design hierarchy
//
//  Parameters:
//     max_num_macro, min_num_macro:   max and min number of marcos in a macro
//     cluster. max_num_inst min_num_inst:  max and min number of std cell
//     instances in a soft cluster. If a logical module has greater than the max
//     threshold of instances, we descend down the hierarchy to examine the
//     children. If multiple clusters are created for child modules that are
//     smaller than the min threshold value, we merge them based on connectivity
//     signatures
//
void AutoClusterMgr::partitionDesign(unsigned int max_num_macro,
                                     unsigned int min_num_macro,
                                     unsigned int max_num_inst,
                                     unsigned int min_num_inst,
                                     unsigned int net_threshold,
                                     unsigned int virtual_weight,
                                     unsigned int ignore_net_threshold,
                                     unsigned int num_hops,
                                     unsigned int timing_weight,
                                     bool std_cell_timing_flag,
                                     const char* report_directory,
                                     const char* file_name)
{
  logger_->report("Running Partition Design...");

  block_ = db_->getChip()->getBlock();
  dbu_ = db_->getTech()->getDbUnitsPerMicron();
  max_num_macro_ = max_num_macro;
  min_num_macro_ = min_num_macro;
  max_num_inst_ = max_num_inst;
  min_num_inst_ = min_num_inst;
  net_threshold_ = net_threshold;
  virtual_weight_ = virtual_weight;
  num_hops_ = num_hops;
  timing_weight_ = timing_weight;
  std_cell_timing_flag_ = std_cell_timing_flag;

  createBundledIO();
  int cluster_id = 0;

  //
  // Map each bundled IO to cluster with zero area
  // Create a cluster for each bundled io
  //
  for (auto [io, name] : {pair(LeftMiddle, "LM"),
                          pair(RightMiddle, "RM"),
                          pair(TopMiddle, "TM"),
                          pair(BottomMiddle, "BM"),

                          pair(LeftLower, "LL"),
                          pair(RightLower, "RL"),
                          pair(TopLower, "TL"),
                          pair(BottomLower, "BL"),

                          pair(LeftUpper, "LU"),
                          pair(RightUpper, "RU"),
                          pair(TopUpper, "TU"),
                          pair(BottomUpper, "BU")}) {
    Cluster* cluster = new Cluster(++cluster_id, name);
    bundled_io_map_[io] = cluster_id;
    cluster_map_[cluster_id] = cluster;
    cluster_list_.push_back(cluster);
  }

  Metric metric = computeMetrics(network_->topInstance());
  logger_->info(PAR,
                402,
                "Traversed logical hierarchy\n"
                "\tNumber of std cell instances: {}\n"
                "\tTotal area: {}\n"
                "\tNumber of hard macros: {}",
                metric.num_inst,
                metric.area,
                metric.num_macro);

  // get all the nets with buffers
  getBufferNet();

  // Break down the top-level instance
  createCluster(cluster_id);
  updateConnection();
  merge("top");

  //
  // Break down clusters
  // Walk down the tree and create clusters for logical modules
  // Stop when the clusters are smaller than the max size threshold
  //
  while (!break_cluster_list_.empty()) {
    Cluster* cluster = break_cluster_list_.front();
    break_cluster_list_.pop();
    breakCluster(cluster, cluster_id);
  }

  //
  // Use MLPart to partition large clusters
  // For clusters that are larger than max threshold size (flat insts) break
  // down the cluster by netlist partitioning using MLPart
  //
  for (int i = 0; i < cluster_list_.size(); i++) {
    if (cluster_list_[i]->getNumInst() > max_num_inst_) {
      mlpart_cluster_list_.push(cluster_list_[i]);
    }
  }

  while (!mlpart_cluster_list_.empty()) {
    Cluster* cluster = mlpart_cluster_list_.front();
    mlpart_cluster_list_.pop();
    MLPart(cluster, cluster_id);
  }

  //
  // split the macros and std cells
  // For clusters that contains HM and std cell -- split the cluster into two
  // a HM part and a std cell part
  //
  vector<Cluster*> par_cluster_vec;
  for (auto cluster : cluster_list_) {
    if (cluster->getNumMacro() > 0) {
      par_cluster_vec.push_back(cluster);
    }
  }

  for (int i = 0; i < par_cluster_vec.size(); i++) {
    Cluster* cluster_old = par_cluster_vec[i];
    int id = (-1) * cluster_old->getId();
    virtual_map_[id] = cluster_old->getId();
    string name = cluster_old->getName() + string("_macro");
    Cluster* cluster = new Cluster(id, name);
    cluster->addLogicalModule(name);
    cluster_map_[id] = cluster;
    vector<const Instance*> macro_vec = cluster_old->getMacros();
    for (int j = 0; j < macro_vec.size(); j++) {
      inst_map_[macro_vec[j]] = id;
      cluster->addMacro(macro_vec[j]);
    }
    cluster_list_.push_back(cluster);
    name = cluster_old->getName() + string("_std_cell");
    cluster_old->setName(name);
    cluster_old->removeMacro();
  }
  par_cluster_vec.clear();
  updateConnection();

  //
  // group macros based on connection signature
  // Use connection signatures to group and split macros
  //
  queue<Cluster*> par_cluster_queue;
  for (int i = 0; i < cluster_list_.size(); i++)
    if (cluster_list_[i]->getNumMacro() > 0)
      par_cluster_queue.push(cluster_list_[i]);

  while (!par_cluster_queue.empty()) {
    Cluster* cluster_old = par_cluster_queue.front();
    par_cluster_queue.pop();
    vector<const Instance*> macro_vec = cluster_old->getMacros();
    string name = cluster_old->getName();
    for (int i = 0; i < macro_vec.size(); i++) {
      Cluster* cluster
          = new Cluster(++cluster_id, network_->pathName(macro_vec[i]));
      cluster->addLogicalModule(network_->pathName(macro_vec[i]));
      cluster_map_[cluster_id] = cluster;
      inst_map_[macro_vec[i]] = cluster_id;
      cluster->addMacro(macro_vec[i]);
      merge_cluster_list_.push_back(cluster);
    }
    int std_cell_id = virtual_map_[cluster_old->getId()];
    virtual_map_.erase(cluster_old->getId());
    cluster_map_.erase(cluster_old->getId());
    vector<Cluster*>::iterator vec_it
        = find(cluster_list_.begin(), cluster_list_.end(), cluster_old);
    cluster_list_.erase(vec_it);
    delete cluster_old;
    updateConnection();
    mergeMacro(name, std_cell_id);
  }

  //
  // group macros based on area footprint, This will allow for more efficient
  // tiling with limited wasted space between the macros
  //
  for (int i = 0; i < cluster_list_.size(); i++)
    if (cluster_list_[i]->getNumMacro() > min_num_macro_)
      par_cluster_queue.push(cluster_list_[i]);

  while (!par_cluster_queue.empty()) {
    Cluster* cluster = par_cluster_queue.front();
    par_cluster_queue.pop();
    MacroPart(cluster, cluster_id);
  }

  updateConnection();

  // add virtual weights between std cell and hard macro portion of the cluster
  // add virtual weights between hard macros
  map<int, int>::iterator weight_it = virtual_map_.begin();
  while (weight_it != virtual_map_.end()) {
    const int id = weight_it->second;
    const int target_id = weight_it->first;
    cluster_map_[id]->addOutputConnection(target_id, virtual_weight_);
    cluster_map_[target_id]->addInputConnection(id, virtual_weight_);
    weight_it++;
  }

  for (int i = 0; i < cluster_list_.size(); i++) {
    cluster_list_[i]->calculateNumSeq(network_);
  }

  // Timing-driven flow
  findAdjacencies();

  Rect die_box;
  block_->getCoreArea(die_box);
  floorplan_lx_ = die_box.xMin();
  floorplan_ly_ = die_box.yMin();
  floorplan_ux_ = die_box.xMax();
  floorplan_uy_ = die_box.yMax();

  //
  // generate block file
  // Generates the output files needed by the macro placer
  //

  const float outline_width = (floorplan_ux_ - floorplan_lx_) / dbu_;
  const float outline_height = (floorplan_uy_ - floorplan_ly_) / dbu_;
  const float blockage_width
      = outline_width / 5.0;  // the depth (0.2) of macro blockage
  const float blockage_height
      = outline_height / 5.0;  // the depth (0.2) of macro blockage

  ofstream output_file;

  string blockage_file
      = string(report_directory) + '/' + file_name + ".blockage";
  output_file.open(blockage_file);
  if (B_pin_.size() > 0) {
    sort(B_pin_.begin(), B_pin_.end());
    output_file << "pin_blockage "
                << "  ";
    output_file << B_pin_[0] / dbu_ << "  0.0  ";
    output_file << B_pin_[B_pin_.size() - 1] / dbu_ << "  " << blockage_height;
    output_file << endl;
  }

  if (T_pin_.size() > 0) {
    sort(T_pin_.begin(), T_pin_.end());
    output_file << "pin_blockage  "
                << "   ";
    output_file << T_pin_[0] / dbu_ << " " << outline_height - blockage_height
                << "   ";
    output_file << T_pin_[T_pin_.size() - 1] / dbu_ << "  " << outline_height;
    output_file << endl;
  }

  if (L_pin_.size() > 0) {
    sort(L_pin_.begin(), L_pin_.end());
    output_file << "pin_blockage  "
                << "   ";
    output_file << "0.0  " << L_pin_[0] / dbu_ << "   ";
    output_file << blockage_width << "  " << L_pin_[L_pin_.size() - 1] / dbu_;
    output_file << endl;
  }

  if (R_pin_.size() > 0) {
    sort(R_pin_.begin(), R_pin_.end());
    output_file << "pin_blockage  "
                << "  ";
    output_file << outline_width - blockage_width << "   " << R_pin_[0] / dbu_
                << "   ";
    output_file << outline_width << "   " << R_pin_[R_pin_.size() - 1] / dbu_;
    output_file << endl;
  }
  output_file.close();

  map<int, Cluster*>::iterator map_iter = cluster_map_.begin();

  string block_file = string(report_directory) + '/' + file_name + ".block";
  output_file.open(block_file);
  output_file << "[INFO] Num clusters: " << cluster_list_.size() << endl;
  output_file << "[INFO] Floorplan width: "
              << (floorplan_ux_ - floorplan_lx_) / dbu_ << endl;
  output_file << "[INFO] Floorplan height:  "
              << (floorplan_uy_ - floorplan_ly_) / dbu_ << endl;
  output_file << "[INFO] Floorplan_lx: " << floorplan_lx_ / dbu_ << endl;
  output_file << "[INFO] Floorplan_ly: " << floorplan_ly_ / dbu_ << endl;
  output_file << "[INFO] Num std cells: "
              << logical_cluster_map_[network_->topInstance()].num_inst << endl;
  output_file << "[INFO] Num macros: "
              << logical_cluster_map_[network_->topInstance()].num_macro
              << endl;
  output_file << "[INFO] Total area: "
              << logical_cluster_map_[network_->topInstance()].area << endl;
  output_file << "[INFO] Num buffers:  " << num_buffer_ << endl;
  output_file << "[INFO] Buffer area:  " << area_buffer_ << endl;
  output_file << endl;
  logger_->info(
      PAR, 403, "Number of Clusters created: {}", cluster_list_.size());
  map_iter = cluster_map_.begin();
  const float dbu = db_->getTech()->getDbUnitsPerMicron();
  while (map_iter != cluster_map_.end()) {
    const float area = map_iter->second->calculateArea(network_);
    if (area != 0.0) {
      output_file << "cluster: " << map_iter->second->getName() << endl;
      output_file << "area:  " << area << endl;
      if (map_iter->second->getNumMacro() > 0) {
        vector<const Instance*> macro_vec = map_iter->second->getMacros();
        for (int i = 0; i < macro_vec.size(); i++) {
          const char* inst_name = network_->pathName(macro_vec[i]);
          dbInst* inst = block_->findInst(inst_name);
          dbMaster* master = inst->getMaster();
          const float width = master->getWidth() / dbu;
          const float height = master->getHeight() / dbu;
          output_file << network_->pathName(macro_vec[i]) << "  ";
          output_file << width << "   " << height << endl;
        }
      }
      output_file << endl;
    }
    map_iter++;
  }

  output_file.close();

  // generate net file
  string net_file = string(report_directory) + '/' + file_name + ".net";
  output_file.open(net_file);
  int net_id = 0;
  map_iter = cluster_map_.begin();
  while (map_iter != cluster_map_.end()) {
    const int src_id = map_iter->first;
    map<int, unsigned int> connection_map
        = map_iter->second->getOutputConnections();
    map<int, unsigned int>::iterator iter = connection_map.begin();

    if (!(connection_map.size() == 0
          || (connection_map.size() == 1 && iter->first == src_id))) {
      output_file << "Net_" << ++net_id << ":  " << endl;
      output_file << "source: " << map_iter->second->getName() << "   ";
      while (iter != connection_map.end()) {
        if (iter->first != src_id) {
          int weight = iter->second;
          if (weight < ignore_net_threshold) {
            weight = 0;
          }
          output_file << cluster_map_[iter->first]->getName() << "   ";
          output_file << weight << "   ";
        }
        iter++;
      }
      output_file << endl;
    }
    map_iter++;
  }
  output_file << endl;
  output_file.close();

  // print connections for each hard macro cluster
  for (int i = 0; i < cluster_list_.size(); i++)
    if (cluster_list_[i]->getNumMacro() > 0)
      par_cluster_queue.push(cluster_list_[i]);

  while (!par_cluster_queue.empty()) {
    Cluster* cluster_old = par_cluster_queue.front();
    par_cluster_queue.pop();
    printMacroCluster(cluster_old, cluster_id);
  }

  // delete all the clusters
  for (int i = 0; i < cluster_list_.size(); i++) {
    delete cluster_list_[i];
  }
}

}  // namespace par
