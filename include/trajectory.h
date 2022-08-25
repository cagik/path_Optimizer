#pragma once

#include <vector>
#include <cmath>
#include <tuple>

using namespace std;

namespace cartesian_planner {

struct TrajectoryPoint {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double s = 0.0;
    double kappa = 0.0;
    double velocity = 0.0;
    double left_bound = 0.0;
    double right_bound = 0.0;
};

struct mapBoundary{
    double x_min;
    double x_max;
    double y_min;
    double y_max;
};

struct PointBoundry{
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double v = 5.0;
    double phi = 0.0;
    double a = 0.0;
    double omega = 0.0;
    double jerk = 0.0;
};

typedef std::vector<TrajectoryPoint> Trajectory;

void calReferTraj(Trajectory &traj, std::vector<std::tuple<double, double, double, double>> &pts, 
                  pair<PointBoundry, PointBoundry> &tpb, mapBoundary &mB);

std::vector<std::pair<double, double>> getPathFromTraj(Trajectory &traj);

}
