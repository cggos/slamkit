// 从Essential矩阵计算R,t

#include <iostream>

#include <sophus/so3.hpp>

#include "epipolar_geometry.h"

using namespace std;

int main(int argc, char **argv) {

    // 给定Essential矩阵
    Matrix3d E;
    E << -0.0203618550523477,   -0.4007110038118445,  -0.03324074249824097,
          0.3939270778216369,   -0.03506401846698079,  0.5857110303721015,
         -0.006788487241438284, -0.5815434272915686,  -0.01438258684486258;

    cout << "E = \n" << E << endl;

    Matrix3d R1;
    Matrix3d R2;
    Vector3d t1; 
    Vector3d t2;
    decompose_essential_mat(E, R1, R2, t1, t2);

    cout << "R1 = \n" << R1 << endl;
    cout << "R2 = \n" << R2 << endl;
    cout << "t1 = \n" << t1 << endl;
    cout << "t2 = \n" << t2 << endl;

    // check t^R=E up to scale
    Matrix3d tR = Sophus::SO3d::hat(t1) * R1;
    cout << "t^R = \n" << tR << endl;

    return 0;
}
