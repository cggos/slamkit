#include <iostream>
#include <fstream>
#include <list>
#include <vector>
#include <chrono>
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>

#include "tum_data_rgbd.h"

int main( int argc, char** argv )
{
    cg::TUMDataRGBD tum_data_rgbd("/home/cg/dev_sdb/datasets/TUM/RGBD-SLAM-Dataset/rgbd_dataset_freiburg2_desk/", 1);

    vector<cv::Mat> colorImgs, depthImgs;
    const int count = 100;
    for (int i = 0; i < count; ++i) {
        cv::Mat img_color;
        cv::Mat img_depth;
        if (!tum_data_rgbd.get_rgb_depth(img_color, img_depth)) {
            std::cerr << "get_rgb_depth failed!" << std::endl;
            return -1;
        }
        colorImgs.push_back(img_color);
        depthImgs.push_back(img_depth);
    }

    list<cv::Point2f> keypoints;      // 因为要删除跟踪失败的点，使用list
    cv::Mat color, depth, last_color;

    for (int index = 0; index < count; index++) {
        color = colorImgs[index];
        depth = depthImgs[index];

        if (index == 0) {
            // 对第一帧提取FAST特征点
            vector<cv::KeyPoint> kps;
            cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create();
            detector->detect(color, kps);
            for (auto kp:kps)
                keypoints.push_back(kp.pt);
            last_color = color;
            continue;
        }
        if (color.data == nullptr || depth.data == nullptr)
            continue;

        // 对其他帧用LK跟踪特征点
        vector<cv::Point2f> next_keypoints;
        vector<cv::Point2f> prev_keypoints;
        for (auto kp:keypoints)
            prev_keypoints.push_back(kp);
        vector<unsigned char> status;
        vector<float> error;
        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        cv::calcOpticalFlowPyrLK(last_color, color, prev_keypoints, next_keypoints, status, error);
        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
        cout << "LK Flow use time：" << time_used.count() << " seconds." << endl;

        // 把跟丢的点删掉
        int i = 0;
        for (auto iter = keypoints.begin(); iter != keypoints.end(); i++) {
            if (status[i] == 0) {
                iter = keypoints.erase(iter);
                continue;
            }
            *iter = next_keypoints[i];
            iter++;
        }
        cout << "tracked keypoints: " << keypoints.size() << endl;
        if (keypoints.size() == 0) {
            cout << "all keypoints are lost." << endl;
            break;
        }

        // 画出 keypoints
        cv::Mat img_show = color.clone();
        for (auto kp:keypoints)
            cv::circle(img_show, kp, 10, cv::Scalar(0, 240, 0), 1);
        cv::imshow("corners", img_show);
        cv::waitKey(0);
        last_color = color;
    }
    
    return 0;
}
