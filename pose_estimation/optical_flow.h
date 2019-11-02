//
// Created by cg on 11/2/19.
//

#ifndef VO1_OPTICAL_FLOW_H
#define VO1_OPTICAL_FLOW_H

#include <opencv2/opencv.hpp>

using namespace std;

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
        int path_size = 7,
        int max_iters = 10,
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
        int path_size = 7,
        int max_iters = 10,
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

#endif //VO1_OPTICAL_FLOW_H
