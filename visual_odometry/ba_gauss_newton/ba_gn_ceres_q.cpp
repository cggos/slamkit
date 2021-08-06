/**
 * @file ba_gn_ceres_q.cpp
 * @author cggos (cggos@outlook.com)
 * @brief 带等式约束的优化问题求解，还有问题
 * @version 0.1
 * @date 2021-08-06
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <ceres/ceres.h>

#include <fstream>

typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVector3d;
typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;

inline Eigen::Matrix3d skew_matrix(const Eigen::Vector3d &v) {
    Eigen::Matrix3d w;
    w << 0., -v(2), v(1),
        v(2), 0., -v(0),
        -v(1), v(0), 0.;

    return w;
}

inline Eigen::Matrix4d right_quat_product_mat(const Eigen::Quaterniond &q) {
    Eigen::Matrix4d m4;
    m4.setZero();
    m4.block<3, 1>(1, 0) = q.vec();
    m4.block<1, 3>(0, 1) = -q.vec();
    m4.block<3, 3>(1, 1) = -skew_matrix(q.vec());
    return m4 + q.w() * Eigen::Matrix4d::Identity();  // w x y z
}

class FactorLagrange : public ceres::SizedCostFunction<1, 7, 1> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FactorLagrange() {}

    virtual ~FactorLagrange() {}

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> tq(parameters[0]);
        double lambda = *parameters[1];

        Eigen::Matrix<double, 4, 1> vq = tq.bottomRows(4);

        residuals[0] = (lambda * (vq.dot(vq) - 1.0));

        if (jacobians != NULL) {
            if (jacobians[0] != NULL) {
                Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> J(jacobians[0]);
                J.segment<3>(0).setZero();
                J.segment<4>(3) = 2.0 * lambda * vq;
            }
            if (jacobians[1] != NULL) {
                Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> J(jacobians[1]);
                J(0, 0) = (vq.dot(vq) - 1.0);
            }
        }

        return true;
    }
};

class FactorBA : public ceres::SizedCostFunction<2, 7> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FactorBA(Eigen::Vector2d observed_p, Eigen::Vector3d observed_P) : observed_p_(observed_p), observed_P_(observed_P) {}

    virtual ~FactorBA() {}

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> tq(parameters[0]);

        auto t = tq.topRows(3);
        Eigen::Vector4d vq = tq.bottomRows(4);  // x, y, z, w
        Eigen::Quaterniond q(vq);

        Eigen::Vector3d Pc = q.normalized() * observed_P_ + t;

        Eigen::Quaterniond Pc_q;
        Pc_q.w() = 0;
        Pc_q.vec() = Pc;

        Eigen::Matrix3d K;
        double fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7;
        K << fx, 0, cx, 0, fy, cy, 0, 0, 1;

        Eigen::Vector2d residual = (K * Pc).hnormalized() - observed_p_;

        residuals[0] = residual[0];
        residuals[1] = residual[1];

        if (jacobians != NULL) {
            if (jacobians[0] != NULL) {
                Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> J(jacobians[0]);

                double x = Pc[0];
                double y = Pc[1];
                double z = Pc[2];

                double x2 = x * x;
                double y2 = y * y;
                double z2 = z * z;

                Eigen::Matrix2d J1 = Eigen::Matrix2d::Zero();
                J1(0, 0) = fx;
                J1(1, 1) = fy;

                Eigen::Matrix<double, 2, 3> J2 = Eigen::Matrix<double, 2, 3>::Identity();
                J2(0, 2) = -x / z;
                J2(1, 2) = -y / z;

                Eigen::Matrix<double, 3, 7> J3;
                J3.block<3, 3>(0, 0).setIdentity();
                J3.block<3, 4>(0, 3) = (right_quat_product_mat(q.inverse()) * right_quat_product_mat(Pc_q)).block<3, 4>(1, 0);

                J = J1 * J2 * J3;
            }
        }

        return true;
    }

   private:
    const Eigen::Vector2d observed_p_;
    const Eigen::Vector3d observed_P_;
};

int main(int argc, char **argv) {
    int n_points = 0;

    VecVector2d p2d;
    VecVector3d p3d;
    {
        std::string p2d_file = "../ba_gauss_newton/data/p2d.txt";
        std::string p3d_file = "../ba_gauss_newton/data/p3d.txt";

        std::ifstream fileP2D(p2d_file);
        std::ifstream fileP3D(p3d_file);

        if (!fileP2D.is_open() || !fileP3D.is_open()) {
            std::cout << "fileP2D or fileP3D open FAILED!" << std::endl;
            return -1;
        }

        double u = 0, v = 0;
        double x = 0, y = 0, z = 0;
        while (!fileP2D.eof()) {
            if (fileP2D.fail())
                break;
            fileP2D >> u >> v;
            fileP3D >> x >> y >> z;
            p2d.push_back(Eigen::Vector2d(u, v));
            p3d.push_back(Eigen::Vector3d(x, y, z));
        }

        if (fileP2D.is_open())
            fileP2D.close();
        if (fileP3D.is_open())
            fileP3D.close();

        assert(p3d.size() == p2d.size());

        n_points = p3d.size();
        std::cout << "points: " << n_points << std::endl;
    }

    Eigen::Matrix<double, 7, 1> vT;  // t q(x y z w)
    vT.segment<3>(0).setZero();
    vT.segment<4>(3) = Eigen::Vector4d(0, 0, 0, 1);
    double lambda[1] = {0};

    ceres::Problem problem;
    for (int i = 0; i < n_points; ++i) {
        ceres::CostFunction *cf_BA;
        cf_BA = new FactorBA(p2d[i], p3d[i]);
        problem.AddResidualBlock(cf_BA, new ceres::CauchyLoss(0.5), vT.data());

        // ceres::CostFunction *cf_lagrange = new FactorLagrange();
        // problem.AddResidualBlock(cf_lagrange, NULL, vT.data(), lambda);
    }

    ceres::Solver::Options options;
    options.dynamic_sparsity = true;
    options.max_num_iterations = 100;
    options.minimizer_progress_to_stdout = true;
    options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_type = ceres::TRUST_REGION;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.dogleg_type = ceres::SUBSPACE_DOGLEG;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";

    Eigen::Matrix<double, 3, 4> T;
    Eigen::Vector4d vq = vT.bottomRows(4);  // x, y, z, w
    T << Eigen::Quaterniond(vq).normalized().toRotationMatrix(), vT.topRows(3);

    std::cout << "estimated pose: \n"
              << T << std::endl;

    return 0;
}
