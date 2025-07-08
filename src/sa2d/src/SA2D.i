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

void run() {
  getSA2D()->runSA();
}

}  // namespace sa2d
%} 