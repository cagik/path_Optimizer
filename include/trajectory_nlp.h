#pragma once

#include <array>
#include <casadi/casadi.hpp>

using namespace  std;

#include "cartesian_planner_config.h"
#include "trajectory.h"

#define E_ 2.71828182

namespace cartesian_planner {

using namespace casadi;

struct States {
  std::vector<double> x, y, theta, v, phi, a, omega, jerk, xf, yf, xr, yr;
};

struct ObsInfo
{
    std::vector<std::pair<double, double>> mid_nearest;
    std::vector<std::pair<double, double>> front_nearest;
    std::vector<std::pair<double, double>> rear_nearest;
};

class TrajectoryNLP {
public:
  explicit TrajectoryNLP(const CartesianPlannerConfig &);

  double SolveIteratively(double w_inf, const States &guess, const Trajectory &reference, ObsInfo &obs,
                          std::vector<std::vector<double>> &boxes, States &result, pair<PointBoundry, PointBoundry> &tpb);

private:
  CartesianPlannerConfig config_;
  Dict nlp_config_;
  Function iterative_solver_;
  Function infeasibility_evaluator_;

  void BuildIterativeNLP();
};

}
