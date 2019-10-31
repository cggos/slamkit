#include <opencv2/opencv.hpp>
#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "tum_data_rgbd.h"

using namespace std;
using namespace cv;

/**
 * single level optical flow
 * @param [in] img1 the first image
 * @param [in] img2 the second image
 * @param [in] kp1 keypoints in img1
 * @param [in|out] kp2 keypoints in img2, if empty, use initial guess in kp1
 * @param [out] success true if a keypoint is tracked successfully
 * @param [in] inverse use inverse formulation?
 */
void OpticalFlowSingleLevel(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const vector <cv::Point2f> &kpt1,
        vector <cv::Point2f> &kpt2,
        vector<unsigned char> &success,
        bool inverse = false
);

/**
 * multi level optical flow, scale of pyramid is set to 2 by default
 * the image pyramid will be create inside the function
 * @param [in] img1 the first pyramid
 * @param [in] img2 the second pyramid
 * @param [in] kp1 keypoints in img1
 * @param [out] kp2 keypoints in img2
 * @param [out] success true if a keypoint is tracked successfully
 * @param [in] inverse set true to enable inverse formulation
 */
void OpticalFlowMultiLevel(
        const vector <cv::Mat> &pyr1,
        const vector <cv::Mat> &pyr2,
        const vector <cv::Point2f> &kpt1,
        vector <cv::Point2f> &kpt2,
        vector<unsigned char> &success,
        bool inverse = true
);

/**
 * get a gray scale value from reference image (bi-linear interpolated)
 * @param img
 * @param x
 * @param y
 * @return
 */
inline float GetPixelValue(const cv::Mat &img, float x, float y) {
    uchar *data = &img.data[int(y) * img.step + int(x)];
    float xx = x - floor(x);
    float yy = y - floor(y);
    return float(
            (1 - xx) * (1 - yy) * data[0] +
            xx * (1 - yy) * data[1] +
            (1 - xx) * yy * data[img.step] +
            xx * yy * data[img.step + 1]
    );
}


int main(int argc, char **argv) {

    Mat img1, img2;

    cg::TUMDataRGBD tum_data_rgbd("/home/cg/projects/datasets/rgbd_dataset_freiburg1_xyz/", 1);
    {
        vector<cv::Mat> colorImgs;
        const int count = 10;
        for (int i = 0; i < count; ++i) {
            cv::Mat img_color;
            if (!tum_data_rgbd.get_rgb(img_color)) {
                std::cerr << "get_rgb failed!" << std::endl;
                return -1;
            }
            colorImgs.push_back(img_color);
        }

        cv::cvtColor(colorImgs[0], img1, COLOR_BGR2GRAY);
        cv::cvtColor(colorImgs[3], img2, COLOR_BGR2GRAY);
    }

    // key points, using GFTT here.
    vector<KeyPoint> kp1;
    Ptr<GFTTDetector> detector = GFTTDetector::create(500, 0.01, 20); // maximum 500 keypoints
    detector->detect(img1, kp1);

    vector<Point2f> cv_pt1;
    for (auto &kp: kp1)
        cv_pt1.push_back(kp.pt);

    // now lets track these key points in the second image

    /// first use single level LK in the validation picture
    vector<Point2f> kp2_single;
    vector<unsigned char> success_single;
    OpticalFlowSingleLevel(img1, img2, cv_pt1, kp2_single, success_single);

    /// then test multi-level LK
    // create pyramids
    double scale = 1.0;
    vector<Mat> pyr1, pyr2; // image pyramids
    Mat tmp1, tmp2;
    for (int i = 0; i < 4; i++) {
        cv::resize(img1, tmp1, Size(img1.cols * scale, img1.rows * scale));
        pyr1.push_back(tmp1);
        cv::resize(img2, tmp2, Size(img2.cols * scale, img2.rows * scale));
        pyr2.push_back(tmp2);
        scale *= 0.5;
    }

    vector<Point2f> kp2_multi;
    vector<unsigned char> success_multi;
    OpticalFlowMultiLevel(pyr1, pyr2, cv_pt1, kp2_multi, success_multi);

    /// use opencv's flow for validation
    vector<Point2f> cv_pt2_single;
    vector<uchar> status_single;
    vector<float> error_single;
    cv::calcOpticalFlowPyrLK(img1, img2, cv_pt1, cv_pt2_single, status_single, error_single, cv::Size(8, 8));

    /// use opencv's multi-level pyramids lk optical flow
    std::vector<cv::Mat> img1_pyramid, img2_pyramid;
    cv::buildOpticalFlowPyramid(img1, img1_pyramid, cv::Size(15, 15), 3, true, cv::BORDER_REFLECT_101, cv::BORDER_CONSTANT, false);
    cv::buildOpticalFlowPyramid(img2, img2_pyramid, cv::Size(15, 15), 3, true, cv::BORDER_REFLECT_101, cv::BORDER_CONSTANT, false);

    vector<Point2f> cv_pt2_multi;
    vector<uchar> status_multi;
    cv::calcOpticalFlowPyrLK(
            img1_pyramid, img2_pyramid, cv_pt1, cv_pt2_multi, status_multi, cv::noArray(), cv::Size(15, 15), 3,
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01));


    // plot the differences of those functions
    Mat img2_single;
    cv::cvtColor(img2, img2_single, CV_GRAY2BGR);
    for (int i = 0; i < kp2_single.size(); i++) {
        if (success_single[i]) {
            cv::circle(img2_single, kp2_single[i], 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_single, kp1[i].pt, kp2_single[i], cv::Scalar(0, 250, 0));
        }
    }

    Mat img2_multi;
    cv::cvtColor(img2, img2_multi, CV_GRAY2BGR);
    for (int i = 0; i < kp2_multi.size(); i++) {
        if (success_multi[i]) {
            cv::circle(img2_multi, kp2_multi[i], 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_multi, kp1[i].pt, kp2_multi[i], cv::Scalar(0, 250, 0));
        }
    }

    Mat cv_img2_sigle;
    cv::cvtColor(img2, cv_img2_sigle, CV_GRAY2BGR);
    for (int i = 0; i < cv_pt2_single.size(); i++) {
        if (status_single[i]) {
            cv::circle(cv_img2_sigle, cv_pt2_single[i], 2, cv::Scalar(0, 250, 0), 2);
            cv::line(cv_img2_sigle, cv_pt1[i], cv_pt2_single[i], cv::Scalar(0, 250, 0));
        }
    }

    Mat cv_img2_multi;
    cv::cvtColor(img2, cv_img2_multi, CV_GRAY2BGR);
    for (int i = 0; i < cv_pt2_multi.size(); i++) {
        if (status_multi[i]) {
            cv::circle(cv_img2_multi, cv_pt2_multi[i], 2, cv::Scalar(0, 250, 0), 2);
            cv::line(cv_img2_multi, cv_pt1[i], cv_pt2_multi[i], cv::Scalar(0, 250, 0));
        }
    }

    cv::imshow("tracked single level", img2_single);
    cv::imshow("tracked multi level", img2_multi);
    cv::imshow("tracked by opencv lk single", cv_img2_sigle);
    cv::imshow("tracked by opencv lk multi", cv_img2_multi);
    while (cv::waitKey(0) != 32);

    return 0;
}

void OpticalFlowSingleLevel(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const vector<cv::Point2f> &kpt1,
        vector<cv::Point2f> &kpt2,
        vector<unsigned char> &success,
        bool inverse
) {
    // parameters
    int half_patch_size = 4;
    int iterations = 10;
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
            for (int x = -half_patch_size; x < half_patch_size; x++)
                for (int y = -half_patch_size; y < half_patch_size; y++) {

                    double error = 0;
                    Eigen::Vector2d J;  // Jacobian

                    float xf = kpt.x + x;
                    float yf = kpt.y + y;

                    if (inverse == false) {
                        // Forward Jacobian
                        J[0] = (GetPixelValue(img2, xf+dx+1, yf+dy  ) - GetPixelValue(img2, xf+dx-1, yf+dy  )) / 2;
                        J[1] = (GetPixelValue(img2, xf+dx,   yf+dy+1) - GetPixelValue(img2, xf+dx,   yf+dy-1)) / 2;
                    } else {
                        // Inverse Jacobian
                        // NOTE this J does not change when dx, dy is updated, so we can store it and only compute error
                        J[0] = (GetPixelValue(img1, xf+1, yf  ) - GetPixelValue(img1, xf-1, yf  )) / 2;
                        J[1] = (GetPixelValue(img1, xf,   yf+1) - GetPixelValue(img1, xf,   yf-1)) / 2;
                    }

                    // compute H, b and set cost;
                    error = GetPixelValue(img2, xf+dx, yf+dy) - GetPixelValue(img1, xf, yf);
                    H +=  J * J.transpose();
                    b += -J.transpose() * error;
                    cost += error * error;
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
                cout << "cost increased: " << cost << ", " << lastCost << endl;
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
        bool inverse) {

    // parameters
    int pyramids = pyr1.size();

    double pyramid_scale = pyr1[1].cols / (double)pyr1[0].cols;

    // coarse-to-fine LK tracking in pyramids
    size_t size_kp1 = kpt1.size();
    vector<cv::Point2f> kpt1_top;
    kpt1_top.reserve(size_kp1);
    for (int i = 0; i<size_kp1; i++) {
        cv::Point2f kpt = kpt1[i];
        kpt *= std::pow(pyramid_scale, pyramids-1);
        kpt1_top.push_back(kpt);
    }
    for (int l=pyramids-1; l>=0; l--) {
        if(l < pyramids-1) {
            for (int i = 0; i < kpt2.size(); i++) {
                kpt1_top[i] /= pyramid_scale;
                kpt2[i]     /= pyramid_scale;
            }
        }
        OpticalFlowSingleLevel(pyr1[l], pyr2[l], kpt1_top, kpt2, success, inverse);
    }
}
