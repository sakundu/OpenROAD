%module sa2d

%{
#include "sa2d/MakeSA2D.h"
#include "sa2d/SA2D.h"
#include "ord/OpenRoad.hh"
#include "odb/db.h"
#include "utl/Logger.h"
#include "dpl/Opendp.h"
%}

%include "../../Exception.i"

%inline %{
namespace sa2d {
  
sa2d::SA2D* getSA2D() {
  return ord::OpenRoad::openRoad()->getSA2D();
}

void set_num_workers(int num_workers) {
  getSA2D()->setNumWorkers(num_workers);
}

void set_max_temp(float max_temp) {
  getSA2D()->setMaxTemp(max_temp);
}

void set_min_temp(float min_temp) {
  getSA2D()->setMinTemp(min_temp);
}

void set_cooling_rate(float cooling_rate) {
  getSA2D()->setCoolingRate(cooling_rate);
}

void set_moves_per_iter(int moves_per_iter) {
  getSA2D()->setMovesPerIter(moves_per_iter);
}

void set_max_iter(int max_iter) {
  getSA2D()->setMaxIter(max_iter);
}

void set_move_budget(int move_budget) {
  getSA2D()->setMoveBudget(move_budget);
}

void set_gwtw_interval(int interval) {
  getSA2D()->setGWTWInterval(interval);
}

void set_elite_ratio(float ratio) {
  getSA2D()->setEliteRatio(ratio);
}

void set_seed(int seed) {
  getSA2D()->setSeed(seed);
}

void set_max_displacement(int max_displacement_x, int max_displacement_y) {
  getSA2D()->setMaxDisplacement(max_displacement_x, max_displacement_y);
}

// LSMC parameters
void set_kick_interval(int interval) {
  getSA2D()->setKickInterval(interval);
}

void set_kick_threshold(float threshold) {
  getSA2D()->setKickThreshold(threshold);
}

void set_kick_strength(int strength) {
  getSA2D()->setKickStrength(strength);
}

void set_kick_temp_multiplier(float multiplier) {
  getSA2D()->setKickTempMultiplier(multiplier);
}

void set_enable_kicks(bool enable) {
  getSA2D()->setEnableKicks(enable);
}

void set_enable_chain_moves(bool enable) {
  getSA2D()->setEnableChainMoves(enable);
}

void set_chain_move_interval(int interval) {
  getSA2D()->setChainMoveInterval(interval);
}

void set_chain_moves_per_round(int moves) {
  getSA2D()->setChainMovesPerRound(moves);
}

void set_enable_slides(bool enable) {
  getSA2D()->setEnableSlides(enable);
}

// SA1D operators for single-row scenarios
void set_use_sa1d_operators(bool enable) {
  getSA2D()->setUseSA1DOperators(enable);
}

void set_sa1d_move_probs(const std::vector<float>& probs) {
  getSA2D()->setSA1DMoveProbs(probs);
}

void set_use_best_orderings_1d(bool enable) {
  getSA2D()->setUseBestOrderings1D(enable);
}

void set_sa1d_overlap_weight(float weight) {
  getSA2D()->setSA1DOverlapWeight(weight);
}

void set_enable_reordering(bool enable) {
  getSA2D()->setEnableReordering(enable);
}

void set_reorder_window_size(int size) {
  getSA2D()->setReorderWindowSize(size);
}

void set_use_dpl_reordering(bool enable) {
  getSA2D()->setUseDPLReordering(enable);
}

void set_pre_sa_reordering(bool enable) {
  getSA2D()->setPreSAReordering(enable);
}

void set_dpl_reordering_passes(int passes) {
  getSA2D()->setReorderingPasses(passes);
}

void set_dpl_reordering_tolerance(double tolerance) {
  getSA2D()->setReorderingTolerance(tolerance);
}

void run_dpl_reordering_only() {
  getSA2D()->runDPLReorderingOnly();
}

void run() {
  getSA2D()->runSA();
}

}  // namespace sa2d
%} 