#include "dataStream.h"
#include "trajectory.h"
#include "corridor.h"
#include "cartesian_planner_config.h"
#include "cartesian_planner.h"


KDTree getKDTree(const std::vector<std::pair<double, double>> &obs){
    pointVec Points_vec;
    point_t pt;
    for(size_t i = 0; i < obs.size(); ++i)
    {
        pt.push_back(obs[i].first);
        pt.push_back(obs[i].second);
        Points_vec.push_back(pt);
        pt.clear();
    }
    KDTree myTree(Points_vec);
    return myTree;
}


void CartesianPlannerTest(){

    vector<tuple<double, double, double, double>> ref_pts_4d;
    std::vector<std::pair<double, double>> obs_pts;
    pair<cartesian_planner::PointBoundry, cartesian_planner::PointBoundry> twoPointBoundry;
    cartesian_planner::mapBoundary mapboundry = {10000, -10000, 10000, -10000};

    dataStream::ReadTxtEdgeData("./referData/134_58717_IN_param/edge_inertia_in.txt", obs_pts);
    dataStream::ReadTxtPathData("./referData/134_58717_IN_param/path_inertia_in.txt", ref_pts_4d);

    cartesian_planner::Trajectory coarseTraj;
    cartesian_planner::calReferTraj(coarseTraj, ref_pts_4d, twoPointBoundry, mapboundry);

    KDTree obsTree = getKDTree(obs_pts);

    corridor C;
    C.initCorridor(obsTree, mapboundry.x_min - 2, mapboundry.x_max + 2, mapboundry.y_min - 2, mapboundry.y_max + 2);
    vector<pair<double, double>> refPath = cartesian_planner::getPathFromTraj(coarseTraj);
    C.update(refPath);

    std::vector<std::vector<double>> test_vis_boxes = C.vis_boxes;
    std::vector<std::vector<double>> boxes_corridor = C.boxes;

    dataStream::PathDataToTxt("./resultData/referPath.txt", ref_pts_4d);
    dataStream::obsDataToTxt("./resultData/obsfile.txt", obs_pts);
    dataStream::BoxDataToTxt("./resultData/boxfile.txt", test_vis_boxes);

    cartesian_planner::CartesianPlannerConfig config_;
    config_.nfe = coarseTraj.size();
    //config_.tf = abs(config_.nfe * 0.4) + 1;
    std::shared_ptr<cartesian_planner::CartesianPlanner> planner_;
    planner_ = std::make_shared<cartesian_planner::CartesianPlanner>(config_);
    cartesian_planner::Trajectory result;
    planner_->Plan(coarseTraj, obsTree, boxes_corridor, result, twoPointBoundry);
    dataStream::TrajDataToTxt("./resultData/optTraj.txt", result);
}

int main(){
    cout << "test2222222" << endl;
    CartesianPlannerTest();
    return 0;
}
