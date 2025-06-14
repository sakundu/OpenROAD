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
    logger->info(utl::SA1D, 301, "Custom ordering computed successfully: {} cells, algorithm: {}, HPWL improvement: {:.1f}%",
                result.cell_ordering.size(), result.algorithm_used, 
                result.initial_hpwl > 0 ? 100.0 * (result.initial_hpwl - result.final_hpwl) / result.initial_hpwl : 0.0);
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
  
  // Set verbose mode if requested
  if (verbose) {
    sa1d::BestOrderingsParams params;
    params.verbose = true;
    optsa->setBestOrderingsParams(params);
  }
  
  auto result = optsa->computeBestOrderings();
  
  if (result.success) {
    utl::Logger* logger = ord::OpenRoad::openRoad()->getLogger();
    logger->info(utl::SA1D, 302, "Best orderings computed successfully: {} algorithms tested, {} top solutions found",
                result.algorithms_tested, result.top_orderings.size());
    
    for (size_t i = 0; i < result.top_orderings.size(); ++i) {
      const auto& info = result.top_orderings[i];
      logger->info(utl::SA1D, 303, "  {}. {} - Peak cutwidth: {} ({:.1f}% improvement), {:.0f}ms",
                  i + 1, info.algorithm_name, info.final_peak_cutwidth, 
                  info.improvement_percentage, info.computation_time_ms);
    }
  } else {
    utl::Logger* logger = ord::OpenRoad::openRoad()->getLogger();
    logger->error(utl::SA1D, 304, "Best orderings computation failed: {}", result.error_message);
  }
  
  return result.success;
}

} // namespace

%} // inline

// Note: We don't include the full header files to avoid exposing incomplete types
// All functionality is accessed through the command functions above
