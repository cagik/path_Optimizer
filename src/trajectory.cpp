#include "trajectory.h"

using namespace std;

namespace cartesian_planner {

void calReferTraj(Trajectory &traj, std::vector<std::tuple<double, double, double, double>> &pts, 
                  pair<PointBoundry, PointBoundry> &tpb, mapBoundary &mB){
    traj.clear();
    double sum_dis = 0.0;
    for(size_t i = 0; i < pts.size(); i++){
        TrajectoryPoint tmp_point;
        tmp_point.x = get<0>(pts[i]);
        tmp_point.y = get<1>(pts[i]);
        if(tmp_point.x > mB.x_max){
            mB.x_max = tmp_point.x;
        }
        if(tmp_point.x < mB.x_min){
            mB.x_min = tmp_point.x;
        }
        if(tmp_point.y > mB.y_max){
            mB.y_max = tmp_point.y;
        }
        if(tmp_point.y < mB.y_min){
            mB.y_min = tmp_point.y;
        }
        tmp_point.theta = get<2>(pts[i]) * M_PI / 180;
        if(i == 0){
            tmp_point.s = sum_dis;
        }
        else{
            double dis_tmp = hypot(get<0>(pts[i]) - get<0>(pts[i - 1]), get<1>(pts[i]) - get<1>(pts[i - 1]));
            sum_dis += dis_tmp;
            tmp_point.s = sum_dis;
        }
        tmp_point.kappa = get<3>(pts[i]);;
        tmp_point.velocity = 5.0;
        tmp_point.left_bound = 100.;
        tmp_point.right_bound = 100.;
        traj.push_back(tmp_point);
    }
    tpb.first.x = 0.0;
    tpb.first.y = 0.0;
    tpb.first.theta = traj[0].theta;
    tpb.first.v = 5.0;
    tpb.first.phi = 0.0;
    tpb.first.omega = 0.0;
    tpb.first.jerk = 0.0;
    tpb.first.a = 0.0;
    int end = traj.size() - 1;
    tpb.second.x = traj[end].x;
    tpb.second.y = traj[end].y;
    tpb.second.theta = traj[end].theta;
    tpb.second.v = 5.0;
    tpb.second.phi = 0.0;
    tpb.second.omega = 0.0;
    tpb.second.jerk = 0.0;
    tpb.second.a = 0.0;
}

std::vector<std::pair<double, double>> getPathFromTraj(Trajectory &traj){
    std::vector<std::pair<double, double>> path;
    path.resize(traj.size());
    for (size_t i = 0; i < traj.size(); i++)
    {
        path[i].first = traj[i].x;
        path[i].second = traj[i].y;
    }
    return path;
}

}