//
// Created by cg on 11/2/19.
//

#include "optical_flow.h"

#include <Eigen/Core>
#include <Eigen/Dense>

void OpticalFlowSingleLevel(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const vector<cv::Point2f> &kpt1,
        vector<cv::Point2f> &kpt2,
        vector<unsigned char> &success,
        int path_size,
        int max_iters,
        bool inverse
) {
    // parameters
    int half_patch_size = path_size / 2 + 1;
    int iterations = max_iters;
    bool have_initial = !kpt2.empty();

    if(!success.empty())
        success.clear();

    for (size_t i = 0; i < kpt1.size(); i++) {
        auto kpt = kpt1[i];
        double dx = 0, dy = 0; // dx,dy need to be estimated
        if (have_initial) {
            dx = kpt2[i].x - kpt.x;
            dy = kpt2[i].y - kpt.y;
        }

        double cost = 0, lastCost = 0;
        bool succ = 1; // indicate if this point succeeded

        // Gauss-Newton iterations
        for (int iter = 0; iter < iterations; iter++) {
            Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
            Eigen::Vector2d b = Eigen::Vector2d::Zero();
            cost = 0;

            if (kpt.x + dx <= half_patch_size || kpt.x + dx >= img1.cols - half_patch_size ||
                kpt.y + dy <= half_patch_size || kpt.y + dy >= img1.rows - half_patch_size) {   // go outside
                succ = 0;
                break;
            }

            // compute cost and jacobian
            for (int x = -half_patch_size; x < half_patch_size; x++) {
                for (int y = -half_patch_size; y < half_patch_size; y++) {

                    double error = 0;
                    Eigen::Vector2d J;  // Jacobian

                    float xf = kpt.x + x;
                    float yf = kpt.y + y;

                    if (inverse == false) {
                        // Forward Jacobian
                        J[0] = (GetPixelValue(img2, xf + dx + 1, yf + dy) - GetPixelValue(img2, xf + dx - 1, yf + dy)) / 2;
                        J[1] = (GetPixelValue(img2, xf + dx, yf + dy + 1) - GetPixelValue(img2, xf + dx, yf + dy - 1)) / 2;
                    } else {
                        // Inverse Jacobian
                        // NOTE this J does not change when dx, dy is updated, so we can store it and only compute error
                        J[0] = (GetPixelValue(img1, xf + 1, yf) - GetPixelValue(img1, xf - 1, yf)) / 2;
                        J[1] = (GetPixelValue(img1, xf, yf + 1) - GetPixelValue(img1, xf, yf - 1)) / 2;
                    }

                    // compute H, b and set cost;
                    error = GetPixelValue(img2, xf + dx, yf + dy) - GetPixelValue(img1, xf, yf);
                    H +=  J * J.transpose();
                    b += -J.transpose() * error;
                    cost += error * error;
                }
            }

            // compute update
            Eigen::Vector2d update;
            update = H.ldlt().solve(b);

            if (isnan(update[0])) {
                // sometimes occurred when we have a black or white patch and H is irreversible
                cout << "update is nan" << endl;
                succ = 0;
                break;
            }
            if (iter > 0 && cost > lastCost) {
//                    cout << "cost increased: " << cost << ", " << lastCost << endl;
                break;
            }

            // update dx, dy
            dx += update[0];
            dy += update[1];
            lastCost = cost;
            succ = 1;
        }

        success.push_back(succ);

        // set kp2
        if (have_initial) {
            kpt2[i] = kpt + cv::Point2f(dx, dy);
        } else {
            cv::Point2f tracked = kpt;
            tracked += cv::Point2f(dx, dy);
            kpt2.push_back(tracked);
        }
    }
}

void OpticalFlowMultiLevel(
        const vector<cv::Mat> &pyr1,
        const vector<cv::Mat> &pyr2,
        const vector<cv::Point2f> &kpt1,
        vector<cv::Point2f> &kpt2,
        vector<unsigned char> &success,
        int path_size,
        int max_iters,
        bool inverse) {

    // parameters
    int pyramids = pyr1.size();

    double pyramid_scale = pyr1[1].cols / (double)pyr1[0].cols; // <=1

    bool have_initial = !kpt2.empty();

    // coarse-to-fine LK tracking in pyramids
    size_t size_kp1 = kpt1.size();
    vector<cv::Point2f> kpt1_top;
    kpt1_top.reserve(size_kp1);
    for (int i = 0; i<size_kp1; i++) {
        cv::Point2f kpt = kpt1[i];
        kpt *= std::pow(pyramid_scale, pyramids-1);
        kpt1_top.push_back(kpt);
    }
    if(have_initial) {
        for (int i = 0; i<kpt2.size(); i++) {
            cv::Point2f &kpt = kpt2[i];
            kpt *= std::pow(pyramid_scale, pyramids-1);
        }
    }
    for (int l=pyramids-1; l>=0; l--) {
        if(l < pyramids-1) {
            for (int i = 0; i < kpt2.size(); i++) {
                kpt1_top[i] /= pyramid_scale;
                kpt2[i] /= pyramid_scale;
            }
        }
        OpticalFlowSingleLevel(pyr1[l], pyr2[l], kpt1_top, kpt2, success, path_size, max_iters, inverse);
    }
}
