#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <sophus/so3.hpp>

/**
 * @brief Eigen::UnitRandom(), ref: http://planning.cs.uiuc.edu/node198.html
 * @return
 */
Eigen::Quaterniond unit_random() {
    double u1 = rand() / double(RAND_MAX);  // [0, 1]
    double u2 = rand() / double(RAND_MAX) * M_2_PI;
    double u3 = rand() / double(RAND_MAX) * M_2_PI;
    double a = std::sqrt(1 - u1);
    double b = std::sqrt(u1);
    return Eigen::Quaterniond(a * sin(u2), a * cos(u2), b * sin(u3), b * cos(u3)).normalized();
}

int main() {
    // Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(0,0,1)).toRotationMatrix();
    Eigen::Matrix3d R(unit_random());
    std::cout << "the random initial rotation matrix R:\n"
              << R << std::endl
              << std::endl;

    Eigen::Vector3d omega(0.01, 0.02, 0.03);

    Sophus::SO3d R_SO3(R);
    Sophus::SO3d R_SO3_updated = R_SO3 * Sophus::SO3d::exp(omega);
    Eigen::Matrix3d R1 = R_SO3_updated.matrix();

    Eigen::Quaterniond R_q(R);
    Eigen::Quaterniond q_update;
    q_update.w() = 1;
    q_update.vec() = 0.5 * omega;
    Eigen::Quaterniond R_q_updated = (R_q * q_update).normalized();
    Eigen::Matrix3d R2 = R_q_updated.toRotationMatrix();

    std::cout << "R1:\n"
              << R1 << std::endl
              << std::endl;
    std::cout << "R2:\n"
              << R2 << std::endl
              << std::endl;

    Eigen::Matrix3d R_error = R1 - R2;
    std::cout << "the Frobenius Norm of the error matrix: " << R_error.norm() << std::endl;
    std::cout << "the sum of the error matrix: " << R_error.sum() << std::endl;

    std::cout << std::endl;

    //------------------------------------------------------------------------------------

    Eigen::Vector3d P(1.0, 2.0, 3.0);
    Eigen::Vector3d P1 = R * P;

    Eigen::Quaterniond Pq;
    Pq.w() = 0.0;
    Pq.vec() = P;
    Eigen::Quaterniond P2q = R_q * Pq * R_q.inverse();
    Eigen::Vector3d P2 = P2q.vec();

    std::cout << "P1: " << P1.transpose() << std::endl;
    std::cout << "P2: " << P2.transpose() << std::endl;

    return 0;
}
