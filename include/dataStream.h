#pragma once

#include <boost/algorithm/string.hpp>
#include <vector>
#include <fstream>
#include <iomanip>

#include "trajectory.h"

using namespace std;

namespace dataStream{

    void ReadTxtEdgeData(std::string filePath, std::vector<std::pair<double, double>> &pts);
    void ReadTxtPathData(std::string filePath, std::vector<std::tuple<double, double, double, double>> &pts);
    void PathDataToTxt(std::string txtPath, std::vector<std::tuple<double, double, double, double>> &pts);
    void obsDataToTxt(std::string txtPath, std::vector<std::pair<double,double>> &pts);
    void BoxDataToTxt(std::string txtPath, std::vector<std::vector<double>> &pts);
    void TrajDataToTxt(std::string txtPath, cartesian_planner::Trajectory &pts);
}
