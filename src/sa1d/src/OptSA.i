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

%{
#include <vector>
#include "ord/OpenRoad.hh"
#include "sa1d/OptSA.h"
#include "sa1d/VertexOrdering.h"
#include "sa1d/BestOrderings.h"
#include "utl/Logger.h"

namespace sa1d {

using std::vector;

// Swig vector type in does not seem to work at all.
// (see odb/src/swig/common/polgon.i)
// Copied from opensta/tcl/StaTcl.i
template <class TYPE>
vector<TYPE> *
tclListSeq(Tcl_Obj *const source,
	   swig_type_info *swig_type,
	   Tcl_Interp *interp)
{
  int argc;
  Tcl_Obj **argv;

  if (Tcl_ListObjGetElements(interp, source, &argc, &argv) == TCL_OK
      && argc > 0) {
    vector<TYPE> *seq = new vector<TYPE>;
    for (int i = 0; i < argc; i++) {
      void *obj;
      // Ignore returned TCL_ERROR because can't get swig_type_info.
      SWIG_ConvertPtr(argv[i], &obj, swig_type, false);
      seq->push_back(reinterpret_cast<TYPE>(obj));
    }
    return seq;
  }
  else
    return nullptr;
}
}
%}


%inline %{

namespace sa1d {

// Read SA params from Json file
void setSAParams(const char* filename) {
  sa1d::OptSA *optsa = ord::OpenRoad::openRoad()->getOptSA();
  optsa->setSAParams(filename);
}

// Report PACKED HPWL
void report_pack_hpwl_cmd() {
  sa1d::OptSA *optsa = ord::OpenRoad::openRoad()->getOptSA();
  optsa->reportPackHPWL();
}

void opt_sa_1d_cmd() {
  sa1d::OptSA *optsa = ord::OpenRoad::openRoad()->getOptSA();
  optsa->runSA();
}

// Vertex ordering integration commands
void set_vertex_ordering_method_cmd(const char* method, bool verbose) {
  sa1d::OptSA *optsa = ord::OpenRoad::openRoad()->getOptSA();
  
  sa1d::VertexOrderingParams params;
  params.verbose = verbose;
  
  std::string method_str(method);
  if (method_str == "random") {
    params.method = sa1d::OrderingMethod::RANDOM;
  } else if (method_str == "size_based") {
    params.method = sa1d::OrderingMethod::SIZE_BASED;
  } else if (method_str == "fiedler") {
    params.method = sa1d::OrderingMethod::FIEDLER;
  } else if (method_str == "rcm") {
    params.method = sa1d::OrderingMethod::RCM;
  } else if (method_str == "rcm_boost") {
    params.method = sa1d::OrderingMethod::RCM_BOOST;
  } else {
    utl::Logger* logger = ord::OpenRoad::openRoad()->getLogger();
    logger->error(utl::SA1D, 300, "Unknown vertex ordering method: {}", method_str);
    return;
  }
  
  optsa->setVertexOrderingMethod(params);
}

void enable_custom_ordering_cmd(bool enable) {
  sa1d::OptSA *optsa = ord::OpenRoad::openRoad()->getOptSA();
  optsa->enableCustomOrdering(enable);
}

bool compute_custom_ordering_cmd() {
  sa1d::OptSA *optsa = ord::OpenRoad::openRoad()->getOptSA();
  auto result = optsa->computeCustomOrdering();
  
  if (result.success) {
    utl::Logger* logger = ord::OpenRoad::openRoad()->getLogger();
    double hpwl_improvement = result.initial_hpwl > 0 ? 100.0 * (result.initial_hpwl - result.final_hpwl) / result.initial_hpwl : 0.0;
    
    logger->info(utl::SA1D, 301, "=== Custom Ordering Results ===");
    logger->info(utl::SA1D, 302, "Algorithm: {}", result.algorithm_used);
    logger->info(utl::SA1D, 303, "Cells processed: {}", result.cell_ordering.size());
    logger->info(utl::SA1D, 304, "Initial HPWL: {:.0f}", result.initial_hpwl);
    logger->info(utl::SA1D, 305, "Final HPWL: {:.0f}", result.final_hpwl);
    logger->info(utl::SA1D, 306, "HPWL improvement: {:.1f}%", hpwl_improvement);
    
    // Show overlap/cutwidth metrics if available
    if (result.initial_peak_cutwidth > 0 || result.final_peak_cutwidth > 0) {
      logger->info(utl::SA1D, 307, "Initial peak cutwidth: {}", result.initial_peak_cutwidth);
      logger->info(utl::SA1D, 308, "Final peak cutwidth: {}", result.final_peak_cutwidth);
      double cutwidth_improvement = result.initial_peak_cutwidth > 0 ? 
        100.0 * (result.initial_peak_cutwidth - result.final_peak_cutwidth) / result.initial_peak_cutwidth : 0.0;
      logger->info(utl::SA1D, 309, "Cutwidth improvement: {:.1f}%", cutwidth_improvement);
    }
    
    if (result.initial_overlap > 0 || result.final_overlap > 0) {
      logger->info(utl::SA1D, 310, "Initial peak overlap: {}", result.initial_overlap);
      logger->info(utl::SA1D, 311, "Final peak overlap: {}", result.final_overlap);
      double overlap_improvement = result.initial_overlap > 0 ? 
        100.0 * (result.initial_overlap - result.final_overlap) / result.initial_overlap : 0.0;
      logger->info(utl::SA1D, 312, "Overlap improvement: {:.1f}%", overlap_improvement);
    }
    
    logger->info(utl::SA1D, 313, "Computation time: {:.1f} ms", result.computation_time_ms);
    
    if (result.vertices_processed > 0) {
      logger->info(utl::SA1D, 314, "Hypergraph vertices: {}", result.vertices_processed);
      logger->info(utl::SA1D, 315, "Hypergraph edges: {}", result.hyperedges_processed);
    }
    
    logger->info(utl::SA1D, 316, "===========================");
  } else {
    utl::Logger* logger = ord::OpenRoad::openRoad()->getLogger();
    logger->error(utl::SA1D, 311, "Custom ordering failed: {}", result.error_message);
  }
  
  return result.success;
}

// Best orderings integration commands (SAIT multi-algorithm)
void set_best_orderings_params_cmd(bool verbose, bool use_parallel, int max_threads, int top_count, 
                                   bool include_advanced, bool apply_refinement, bool use_constrained_refinement) {
  sa1d::OptSA *optsa = ord::OpenRoad::openRoad()->getOptSA();
  
  sa1d::BestOrderingsParams params;
  params.verbose = verbose;
  params.use_parallel = use_parallel;
  params.max_threads = max_threads;
  params.top_count = top_count;
  params.include_advanced_methods = include_advanced;
  params.apply_refinement = apply_refinement;
  params.use_constrained_refinement = use_constrained_refinement;
  
  optsa->setBestOrderingsParams(params);
}

void enable_best_orderings_cmd(bool enable) {
  sa1d::OptSA *optsa = ord::OpenRoad::openRoad()->getOptSA();
  optsa->enableBestOrderings(enable);
}

bool compute_best_orderings_cmd(bool verbose) {
  sa1d::OptSA *optsa = ord::OpenRoad::openRoad()->getOptSA();
  
  try {
    // Set verbose mode if requested
    if (verbose) {
      sa1d::BestOrderingsParams params;
      params.verbose = true;
      optsa->setBestOrderingsParams(params);
    }
    
    auto result = optsa->computeBestOrderings();
  
  if (result.success) {
    utl::Logger* logger = ord::OpenRoad::openRoad()->getLogger();
    logger->info(utl::SA1D, 320, "=== Best Orderings Results ===");
    logger->info(utl::SA1D, 321, "Algorithms tested: {}", result.algorithms_tested);
    logger->info(utl::SA1D, 322, "Top solutions found: {}", result.top_orderings.size());
    logger->info(utl::SA1D, 323, "Total computation time: {:.1f} ms", result.total_computation_time_ms);
    logger->info(utl::SA1D, 324, "");
    
    for (size_t i = 0; i < result.top_orderings.size(); ++i) {
      const auto& info = result.top_orderings[i];
      logger->info(utl::SA1D, 325, "Rank {}: {}", i + 1, info.algorithm_name);
      logger->info(utl::SA1D, 326, "  Initial HPWL: {:.0f}", info.initial_hpwl);
      logger->info(utl::SA1D, 327, "  Final HPWL: {:.0f}", info.final_hpwl);
      logger->info(utl::SA1D, 328, "  HPWL improvement: {:.1f}%", 
                  info.initial_hpwl > 0 ? 100.0 * (info.initial_hpwl - info.final_hpwl) / info.initial_hpwl : 0.0);
      logger->info(utl::SA1D, 329, "  Initial peak cutwidth: {}", info.initial_peak_cutwidth);
      logger->info(utl::SA1D, 330, "  Final peak cutwidth: {}", info.final_peak_cutwidth);
      logger->info(utl::SA1D, 331, "  Cutwidth improvement: {:.1f}%", info.improvement_percentage);
      logger->info(utl::SA1D, 332, "  Computation time: {:.1f} ms", info.computation_time_ms);
      logger->info(utl::SA1D, 333, "");
    }
    
    if (!result.top_orderings.empty()) {
      const auto& best = result.top_orderings[0];
      logger->info(utl::SA1D, 334, "BEST SOLUTION: {} with HPWL={:.0f}, Peak Cutwidth={}", 
                  best.algorithm_name, best.final_hpwl, best.final_peak_cutwidth);
    }
    logger->info(utl::SA1D, 335, "=============================");
  } else {
    utl::Logger* logger = ord::OpenRoad::openRoad()->getLogger();
    logger->error(utl::SA1D, 336, "Best orderings computation failed: {}", result.error_message);
  }
  
  return result.success;
  
  } catch (const std::exception& e) {
    utl::Logger* logger = ord::OpenRoad::openRoad()->getLogger();
    logger->error(utl::SA1D, 337, "Best orderings computation crashed: {}", e.what());
    return false;
  } catch (...) {
    utl::Logger* logger = ord::OpenRoad::openRoad()->getLogger();
    logger->error(utl::SA1D, 338, "Best orderings computation crashed with unknown error");
    return false;
  }
}

} // namespace

%} // inline

// Note: We don't include the full header files to avoid exposing incomplete types
// All functionality is accessed through the command functions above
