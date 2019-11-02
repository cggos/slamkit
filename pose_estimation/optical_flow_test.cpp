#include "optical_flow.h"

#include <string>
#include <fstream>

#include "tum_data_rgbd.h"

using namespace std;
using namespace cv;

int main(int argc, char **argv) {
    std::ofstream out_file("out_file.txt");

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
    std::vector<cv::Mat> pyr1, pyr2; // image pyramids
    cv::Mat tmp1, tmp2;
    for (int i = 0; i < 4; i++) {
        if(i == 0) {
            pyr1.push_back(img1);
            pyr2.push_back(img2);
            continue;
        }
        cv::pyrDown(pyr1[i-1], tmp1, pyr1[i-1].size() / 2);
        pyr1.push_back(tmp1);
        cv::pyrDown(pyr2[i-1], tmp2, pyr2[i-1].size() / 2);
        pyr2.push_back(tmp2);
    }

    vector<Point2f> kp2_multi;
    vector<unsigned char> success_multi;
    OpticalFlowMultiLevel(pyr1, pyr2, cv_pt1, kp2_multi, success_multi, 15, 30);

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


    out_file << "OpticalFlowMultiLevel <--> calcOpticalFlowPyrLK" << std::endl;
    for(int i=0; i<status_multi.size();  ++i) {
        out_file << kp2_multi[i] << ", " << (int)success_multi[i] << " <--> "
                 << cv_pt2_multi[i] << ", " << (int)status_multi[i] << std::endl;
    }
    out_file.close();


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
