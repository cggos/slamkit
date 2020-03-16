#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <sophus/se3.h>

#include "pose_estimation.h"
#include "trajectory.h"

using namespace std;
using namespace Eigen;

string strFileCompare = "../pose_estimation_3d3d_data.txt";

int main()
{
    ifstream fileCompare(strFileCompare);
    if(!fileCompare.is_open()){
        std::cout << "icp_compare.txt open FAILED!" << std::endl;
       return -1; 
    }
    Eigen::Vector3d v3T;
    Eigen::Quaterniond q;
    Sophus::SE3 se3;
    std::vector<cv::Point3f> ptsTe;
    std::vector<cv::Point3f> ptsTg;
    std::vector<Sophus::SE3> vse3_e;
    std::vector<Sophus::SE3> vse3_g;
    double time_e, tx_e, ty_e, tz_e, qx_e, qy_e, qz_e, qw_e;
    double time_g, tx_g, ty_g, tz_g, qx_g, qy_g, qz_g, qw_g;
    while(!fileCompare.eof()){
        if(fileCompare.fail())
            break;
        fileCompare >> time_e >> tx_e >> ty_e >> tz_e >> qx_e >> qy_e >> qz_e >> qw_e
                    >> time_g >> tx_g >> ty_g >> tz_g >> qx_g >> qy_g >> qz_g >> qw_g;
        
        v3T = Eigen::Vector3d(tx_e,ty_e,tz_e);  
        q   = Eigen::Quaterniond(qw_e, qx_e, qy_e, qz_e);

        ptsTe.push_back(cv::Point3f(v3T[0], v3T[1], v3T[2]));
        se3.translation() = v3T;
        se3.so3() = Sophus::SO3(q);
        vse3_e.push_back(se3);
        
        v3T = Eigen::Vector3d(tx_g,ty_g,tz_g);  
        q   = Eigen::Quaterniond(qw_g, qx_g, qy_g, qz_g);

        ptsTg.push_back(cv::Point3f(v3T[0], v3T[1], v3T[2]));
        se3.translation() = v3T;
        se3.so3() = Sophus::SO3(q);
        vse3_g.push_back(se3);
    }

    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    pose_estimation_3d3d(ptsTg, ptsTe, R, t);

    Sophus::SE3 se3_ge = Sophus::SE3(R, t);

    cg::draw_trajectory(vse3_g, vse3_e, se3_ge);

    return 0;
}