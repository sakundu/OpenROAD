#############################################################################
## Authors: Sayak Kundu (sakundu@ucsd.edu), Zhiang Wang (zhw033@ucsd.edu)
##          Dooseok Yoon (d3yoon@ucsd.edu)
## Copyright (c) 2024, The Regents of the University of California
## All rights reserved.
##
## BSD 3-Clause License
##
## Redistribution and use in source and binary forms, with or without
## modification, are permitted provided that the following conditions are met:
##
## * Redistributions of source code must retain the above copyright notice, this
##   list of conditions and the following disclaimer.
##
## * Redistributions in binary form must reproduce the above copyright notice,
##   this list of conditions and the following disclaimer in the documentation
##   and/or other materials provided with the distribution.
##
## * Neither the name of the copyright holder nor the names of its
##   contributors may be used to endorse or promote products derived from
##   this software without specific prior written permission.
##
## THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
## AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
## IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
## ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
## LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
## CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
## SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
## INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
## CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
## ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
## POSSIBILITY OF SUCH DAMAGE.
#############################################################################

sta::define_cmd_args "setSAParams" {
  [-json_file json_file] \
}

proc setSAParams { args } {
  sta::parse_key_args "setSAParams" args \
    keys {-json_file}
  
  if { [info exists keys(-json_file)] } {
    set json_file $keys(-json_file)
    if { [file exists $json_file] } {
      sa1d::setSAParams $json_file
    }
  }
}

sta::define_cmd_args "report_pack_hpwl" {}
proc report_pack_hpwl { args } {
  sa1d::report_pack_hpwl_cmd
}

sta::define_cmd_args "opt_sa_1d" {}

proc opt_sa_1d { args } {
  sa1d::opt_sa_1d_cmd
}

# Vertex ordering integration commands

sta::define_cmd_args "set_vertex_ordering_method" {
  method \
  [-verbose] \
}

proc set_vertex_ordering_method { args } {
  sta::parse_key_args "set_vertex_ordering_method" args \
    keys {} \
    flags {-verbose}
  
  set method [lindex $args 0]
  set verbose [info exists flags(-verbose)]
  
  sa1d::set_vertex_ordering_method_cmd $method $verbose
}

sta::define_cmd_args "enable_custom_ordering" {
  enable \
}

proc enable_custom_ordering { args } {
  set enable [lindex $args 0]
  sa1d::enable_custom_ordering_cmd $enable
}

sta::define_cmd_args "compute_custom_ordering" {}

proc compute_custom_ordering { args } {
  return [sa1d::compute_custom_ordering_cmd]
}

# Best orderings integration commands (SAIT multi-algorithm)

sta::define_cmd_args "set_best_orderings_params" {
  [-verbose] \
  [-use_parallel] \
  [-max_threads max_threads] \
  [-top_count top_count] \
  [-include_advanced] \
  [-apply_refinement] \
  [-use_constrained_refinement] \
}

proc set_best_orderings_params { args } {
  sta::parse_key_args "set_best_orderings_params" args \
    keys {-max_threads -top_count} \
    flags {-verbose -use_parallel -include_advanced -apply_refinement -use_constrained_refinement}
  
  set verbose [info exists flags(-verbose)]
  set use_parallel [info exists flags(-use_parallel)]
  set include_advanced [info exists flags(-include_advanced)]
  set apply_refinement [info exists flags(-apply_refinement)]
  set use_constrained_refinement [info exists flags(-use_constrained_refinement)]
  
  set max_threads 0
  if { [info exists keys(-max_threads)] } {
    set max_threads $keys(-max_threads)
  }
  
  set top_count 6
  if { [info exists keys(-top_count)] } {
    set top_count $keys(-top_count)
  }
  
  sa1d::set_best_orderings_params_cmd $verbose $use_parallel $max_threads $top_count $include_advanced $apply_refinement $use_constrained_refinement
}

sta::define_cmd_args "enable_best_orderings" {
  enable \
}

proc enable_best_orderings { args } {
  set enable [lindex $args 0]
  sa1d::enable_best_orderings_cmd $enable
}

sta::define_cmd_args "compute_best_orderings" {
  [-verbose] \
}

proc compute_best_orderings { args } {
  sta::parse_key_args "compute_best_orderings" args \
    keys {} \
    flags {-verbose}
  
  set verbose [info exists flags(-verbose)]
  return [sa1d::compute_best_orderings_cmd $verbose]
}


