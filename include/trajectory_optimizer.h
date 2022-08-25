#pragma once

#include "KDTree.hpp"
#include "trajectory_nlp.h"

#include "cartesian_planner_config.h"

namespace cartesian_planner {

class TrajectoryOptimizer {
public:
  TrajectoryOptimizer(const CartesianPlannerConfig &config);

  bool OptimizeIteratively(const Trajectory &coarse, KDTree &obsTree,
                           std::vector<std::vector<double>> &boxes, States &result, pair<PointBoundry, PointBoundry> &tpb);

private:
  CartesianPlannerConfig config_;
  VehicleParam vehicle_;
  TrajectoryNLP nlp_;

  ObsInfo getCloseObsAndUpdateDisGrad(States &state, KDTree &obsTree);

  void CalculateInitialGuess(States &states) const;

  bool ResizeState(States &states);


};

}
