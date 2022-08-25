#include "dataStream.h"

namespace dataStream{


void ReadTxtEdgeData(std::string filePath, std::vector<std::pair<double, double>> &pts)
{
    pts.clear();
    std::string s;
    double x,y;
    std::fstream file(filePath);
    while(file >> s)
    {
        vector<string> s_tmp;
        boost::split(s_tmp, s, boost::is_any_of(","));
        x = atof(s_tmp[0].c_str());
        y = atof(s_tmp[1].c_str());
        pts.push_back(make_pair(x,y));
    }
}

void ReadTxtPathData(std::string filePath, std::vector<std::tuple<double, double, double, double>> &pts)
{
    pts.clear();
    std::string s;
    double h,x,y,cur;
    std::fstream file(filePath);
    while(file >> s)
    {
        vector<string> s_tmp;
        boost::split(s_tmp, s, boost::is_any_of(","));
        x = atof(s_tmp[0].c_str());
        y = atof(s_tmp[1].c_str());
        h = atof(s_tmp[2].c_str());
        cur = atof(s_tmp[3].c_str());
        pts.push_back(make_tuple(x,y,h,cur));
    }
}

void PathDataToTxt(std::string txtPath, std::vector<std::tuple<double, double, double, double>> &pts)
{
    std::ofstream file(txtPath, std::ios::out);
    for(size_t i = 0; i < pts.size(); i++){
        file << setprecision(10) << get<0>(pts[i])  << ","
             << setprecision(10) << get<1>(pts[i])  << "," 
             << setprecision(10) << get<2>(pts[i])  << "," 
             << setprecision(10) << get<3>(pts[i]) << std::endl;
    }

}

void obsDataToTxt(std::string txtPath, std::vector<std::pair<double,double>> &pts)
{
    std::ofstream file(txtPath, std::ios::out);
    for(size_t i = 0; i < pts.size(); i++){
        file << setprecision(10) << pts[i].first  << ","
             << setprecision(10) << pts[i].second << std::endl;
    }

}

void BoxDataToTxt(std::string txtPath, std::vector<std::vector<double>> &pts)
{
    std::ofstream file(txtPath, std::ios::out);
    for(size_t i = 0; i < pts.size(); i++){
        file << setprecision(10) << pts[i][0]  << ","
             << setprecision(10) << pts[i][1] << "," 
             << setprecision(10) << pts[i][2]  << "," 
             << setprecision(10) << pts[i][3] << std::endl;
    }
}

void TrajDataToTxt(std::string txtPath, cartesian_planner::Trajectory &pts)
{
    std::ofstream file(txtPath, std::ios::out);
    for(size_t i = 0; i < pts.size(); i++){
        file << setprecision(10) << pts[i].x  << ","
             << setprecision(10) << pts[i].y << "," 
             << setprecision(10) << pts[i].theta  << ","
             << setprecision(10) << pts[i].velocity  << "," 
             << setprecision(10) << pts[i].kappa << std::endl;
    }
}

}
