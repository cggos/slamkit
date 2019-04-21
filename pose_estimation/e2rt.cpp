// 从Essential矩阵计算R,t

#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <sophus/so3.h>

using namespace std;
using namespace Eigen;

int main(int argc, char **argv) {

    // 给定Essential矩阵
    Matrix3d E;
    E << -0.0203618550523477,   -0.4007110038118445,  -0.03324074249824097,
          0.3939270778216369,   -0.03506401846698079,  0.5857110303721015,
         -0.006788487241438284, -0.5815434272915686,  -0.01438258684486258;

    cout << "E = \n" << E << endl;

    // 待计算的R,t
    Matrix3d R;
    Vector3d t;

    // SVD and fix sigular values
    JacobiSVD<MatrixXd> svd(E, ComputeThinU | ComputeThinV);
    Matrix3d m3U = svd.matrixU();
    Matrix3d m3V = svd.matrixV();
    Vector3d v3S = svd.singularValues();

    double temp = (v3S[0]+v3S[1])/2;
    Matrix3d m3S(Vector3d(temp, temp, 0).asDiagonal());

    Eigen::Matrix3d m3R_z_p = Eigen::AngleAxisd( M_PI/2, Eigen::Vector3d(0,0,1)).toRotationMatrix();
    Eigen::Matrix3d m3R_z_n = Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d(0,0,1)).toRotationMatrix();
    cout << "m3R_z_p = \n" << m3R_z_p << endl;
    cout << "m3R_z_n = \n" << m3R_z_n << endl;

    // set t1, t2, R1, R2 
    Matrix3d t_wedge1;
    Matrix3d t_wedge2;
    t_wedge1 = m3U * m3R_z_p * m3S * m3U.transpose();
    t_wedge2 = m3U * m3R_z_n * m3S * m3U.transpose();

    Matrix3d R1;
    Matrix3d R2;
    R1 = m3U * m3R_z_p.transpose() * m3V.transpose();
    R2 = m3U * m3R_z_n.transpose() * m3V.transpose();

    cout << "R1 = \n" << R1 << endl;
    cout << "R2 = \n" << R2 << endl;
    cout << "t1 = \n" << Sophus::SO3::vee(t_wedge1) << endl;
    cout << "t2 = \n" << Sophus::SO3::vee(t_wedge2) << endl;

    // check t^R=E up to scale
    Matrix3d tR = t_wedge1 * R1;
    cout << "t^R = \n" << tR << endl;

    return 0;
}
