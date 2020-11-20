#include <cmath>
#include <iostream>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

int main(int argc, char** argv) {
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
    Eigen::Quaterniond q(R);

    Sophus::SO3d SO3_R(R);
    Sophus::SO3d SO3_q(q);

    cout << "SO(3) from matrix:     " << SO3_R.matrix() << endl;
    cout << "SO(3) from quaternion: " << SO3_q.matrix() << endl;

    Eigen::Vector3d so3 = SO3_R.log();
    cout << "so3 = " << so3.transpose() << endl;
    cout << "so3 hat=\n"
         << Sophus::SO3d::hat(so3) << endl;
    cout << "so3 hat vee = " << Sophus::SO3d::vee(Sophus::SO3d::hat(so3)).transpose() << endl;

    Eigen::Vector3d update_so3(1e-4, 0, 0);
    Sophus::SO3d SO3_updated = Sophus::SO3d::exp(update_so3) * SO3_R;
    cout << "SO3 updated = " << SO3_updated.matrix() << endl;

    cout << endl
         << endl;

    Eigen::Vector3d t(1, 0, 0);
    Sophus::SE3d SE3_Rt(R, t);
    Sophus::SE3d SE3_qt(q, t);
    cout << "SE3 from R,t= " << endl
         << SE3_Rt.matrix() << endl;
    cout << "SE3 from q,t= " << endl
         << SE3_qt.matrix() << endl;

    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    Vector6d se3 = SE3_Rt.log();
    cout << "se3 = " << se3.transpose() << endl;
    // 观察输出，会发现在Sophus中，se(3)的平移在前，旋转在后.
    cout << "se3 hat = " << endl
         << Sophus::SE3d::hat(se3) << endl;
    cout << "se3 hat vee = " << Sophus::SE3d::vee(Sophus::SE3d::hat(se3)).transpose() << endl;

    Vector6d update_se3;
    update_se3.setZero();
    update_se3(0, 0) = 1e-4d;
    Sophus::SE3d SE3_updated = Sophus::SE3d::exp(update_se3) * SE3_Rt;
    cout << "SE3 updated = " << endl
         << SE3_updated.matrix() << endl;

    return 0;
}
