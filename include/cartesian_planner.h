#pragma once

#include "KDTree.hpp"

#include "cartesian_planner_config.h"

#include "trajectory_optimizer.h"

namespace cartesian_planner {

class CartesianPlanner {
public:
  struct StartState {
    double x, y, theta, v, phi, a, omega;
  };

  explicit CartesianPlanner(const CartesianPlannerConfig &config)
    : config_(config),  opti_(config) {}

  bool Plan(const Trajectory coarse_trajectory,  KDTree &obsTree, std::vector<std::vector<double>> &boxes, 
            Trajectory &result, pair<PointBoundry, PointBoundry> &tpb);

private:

  CartesianPlannerConfig config_;
  TrajectoryOptimizer opti_;

};

}
