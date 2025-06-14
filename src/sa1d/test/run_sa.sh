#!/bin/bash
source ~/SCRIPT/open_road_setup
export SA1D_DEF_FILE="${1}"
export SA1D_ENABLE_UPDATED_COST="${2}"
export SA1D_ENABLE_BEST_ORDERINGS="${3}"

echo "SA1D_DEF_FILE: $SA1D_DEF_FILE"
echo "SA1D_ENABLE_UPDATED_COST: $SA1D_ENABLE_UPDATED_COST"
echo "SA1D_ENABLE_BEST_ORDERINGS: $SA1D_ENABLE_BEST_ORDERINGS"
log_file="run_sa_m_${SA1D_DEF_FILE}_${SA1D_ENABLE_UPDATED_COST}_${SA1D_ENABLE_BEST_ORDERINGS}.log"
# After installing OpenROAD, place the OpenROAD binary 'openroad' here
./openroad run_openroadSA.tcl -log $log_file
