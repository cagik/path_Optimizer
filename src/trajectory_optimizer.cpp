#include "trajectory_optimizer.h"

#include <bitset>

namespace cartesian_planner {

TrajectoryOptimizer::TrajectoryOptimizer(const CartesianPlannerConfig &config)
  : config_(config), nlp_(config) {
  vehicle_ = config_.vehicle;
}

bool TrajectoryOptimizer::OptimizeIteratively(const Trajectory &coarse, KDTree &obsTree,
                           std::vector<std::vector<double>> &boxes, States &result, pair<PointBoundry, PointBoundry> &tpb)
{
    States guess;

    for (size_t i = 0; i < coarse.size(); i++)
    {
        guess.x.push_back(coarse[i].x);
        guess.y.push_back(coarse[i].y);
        guess.theta.push_back(coarse[i].theta);
    }
    
    CalculateInitialGuess(guess);
    ResizeState(guess);

    ObsInfo obsInfo;
    obsInfo = getCloseObsAndUpdateDisGrad(guess, obsTree);


    int iter = 0;
    double w_penalty = config_.opti_w_penalty0;

    while (iter < config_.opti_iter_max) {

      if(iter > 0){
          obsInfo = getCloseObsAndUpdateDisGrad(guess, obsTree);
      }
      double cur_infeasibility = nlp_.SolveIteratively(w_penalty, guess, coarse, obsInfo, boxes, guess, tpb);

      printf("iter = %d, cur_infeasibility = %f, w_penalty = %f\n", iter, cur_infeasibility, w_penalty);

      if (cur_infeasibility < config_.opti_varepsilon_tol) {
        result = guess;
        return true;
      } else {
        w_penalty *= config_.opti_alpha;
        iter++;
      }
    }
    result = guess;
    return false;
}

void TrajectoryOptimizer::CalculateInitialGuess(States &states) const {
  states.v.resize(config_.nfe, 0.0);
  states.phi.resize(config_.nfe, 0.0);

  double hi = config_.tf / (config_.nfe - 1);
  for (size_t i = 1; i < states.x.size(); i++) {
    double velocity = hypot(states.y[i] - states.y[i - 1], states.x[i] - states.x[i - 1]) / hi;

    states.v[i] = std::min(vehicle_.max_velocity, velocity);
    states.phi[i] = std::min(vehicle_.phi_max, std::max(-vehicle_.phi_max, atan(
      (states.theta[i] - states.theta[i - 1]) * vehicle_.wheel_base / (states.v[i] * hi))));
  }

  states.a.resize(config_.nfe, 0.0);
  states.omega.resize(config_.nfe, 0.0);
  for (size_t i = 1; i < states.x.size(); i++) {
    states.a[i] = std::min(vehicle_.max_acceleration,
                           std::max(vehicle_.min_acceleration, (states.v[i] - states.v[i - 1]) / hi));
    states.omega[i] = std::min(vehicle_.omega_max,
                               std::max(-vehicle_.omega_max, (states.phi[i] - states.phi[i - 1]) / hi));
  }

  states.jerk.resize(config_.nfe, 0.0);
  for (size_t i = 1; i < states.x.size(); i++) {
    states.jerk[i] = std::min(vehicle_.jerk_max, std::max(-vehicle_.jerk_max, (states.a[i] - states.a[i - 1]) / hi));
  }
}

bool TrajectoryOptimizer::ResizeState(States &states) {
  states.xf.resize(config_.nfe);
  states.yf.resize(config_.nfe);
  states.xr.resize(config_.nfe);
  states.yr.resize(config_.nfe);
  double hi = config_.tf / (config_.nfe - 1);
  for (size_t i = 0; i < config_.nfe; i++) {
    double time = hi * i;
    std::tie(states.xf[i], states.yf[i], states.xr[i], states.yr[i]) = vehicle_.GetDiscPositions(states.x[i],
                                                                                                 states.y[i],
                                                                                                 states.theta[i]);
  }
  return true;
}

ObsInfo TrajectoryOptimizer::getCloseObsAndUpdateDisGrad(States &state, KDTree &obsTree)
{
    ObsInfo tmp;
    tmp.mid_nearest.resize(state.x.size());
    tmp.front_nearest.resize(state.xf.size());
    tmp.rear_nearest.resize(state.xr.size());

    point_t pt;
    for (size_t i = 0; i < tmp.mid_nearest.size(); i++) {
        //mid
        pt.clear();
        pt.push_back(state.x[i]);
        pt.push_back(state.y[i]);
        auto res_mid = obsTree.nearest_point(pt);
        tmp.mid_nearest[i] = std::make_pair(res_mid[0], res_mid[1]);

        //front

        pt.clear();
        pt.push_back(state.xf[i]);
        pt.push_back(state.yf[i]);
        auto res_front = obsTree.nearest_point(pt);
        tmp.front_nearest[i] = std::make_pair(res_front[0], res_front[1]);

        //rear
        pt.clear();
        pt.push_back(state.xr[i]);
        pt.push_back(state.yr[i]);
        auto res_rear = obsTree.nearest_point(pt);
        tmp.rear_nearest[i] = std::make_pair(res_rear[0], res_rear[1]);

    }

    return tmp;
}

}
