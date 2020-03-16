#include <iostream>

#include "pose_estimation.h"

#include "tum_data_rgbd.h"

using namespace std;
using namespace cv;

int main ( int argc, char** argv )
{
    Mat img_1, img_2, img_d1, img_d2;
    Mat K;
    float depth_scale = 1.f;

    cg::TUMDataRGBD tum_data_rgbd("/home/cg/dev_sdb/datasets/TUM/RGBD-SLAM-Dataset/rgbd_dataset_freiburg2_desk/", 1);
    {
        vector<cv::Mat> colorImgs, depthImgs;
        const int count = 25;
        for (int i = 0; i < count; ++i) {
            cv::Mat img_color, img_depth;
            tum_data_rgbd.get_rgb_depth(img_color, img_depth);
            colorImgs.push_back(img_color);
            depthImgs.push_back(img_depth);
        }

        img_1  = colorImgs[0];
        img_2  = colorImgs[22];
        img_d1 = depthImgs[0];
        img_d2 = depthImgs[22];

        tum_data_rgbd.getK(K);
        depth_scale = tum_data_rgbd.depth_scale_;
    }

    vector<KeyPoint> kpts_1, kpts_2;
    vector<DMatch> matches;
    find_feature_matches ( img_1, img_2, kpts_1, kpts_2, matches );
    cout<<"一共找到了"<<matches.size() <<"组匹配点"<<endl;

    vector<Point3f> pts1, pts2;
    for ( DMatch m:matches ) {
        int x1 = int(kpts_1[m.queryIdx].pt.x);
        int y1 = int(kpts_1[m.queryIdx].pt.y);
        int x2 = int(kpts_2[m.trainIdx].pt.x);
        int y2 = int(kpts_2[m.trainIdx].pt.y);
        ushort d1 = img_d1.ptr<unsigned short>(y1)[x1];
        ushort d2 = img_d2.ptr<unsigned short>(y2)[x2];
        if (d1 == 0 || d2 == 0)   // bad depth
            continue;
        float dd1 = float(d1) / depth_scale;
        float dd2 = float(d2) / depth_scale;

        Point2d p1 = pixel2cam(kpts_1[m.queryIdx].pt, K);
        Point2d p2 = pixel2cam(kpts_2[m.trainIdx].pt, K);

        pts1.push_back(Point3f(p1.x * dd1, p1.y * dd1, dd1));
        pts2.push_back(Point3f(p2.x * dd2, p2.y * dd2, dd2));
    }

    cout<<"3d-3d pairs: "<<pts1.size() <<endl;

    Eigen::Matrix3d m3R;
    Eigen::Vector3d v3t;
    pose_estimation_3d3d ( pts1, pts2, m3R, v3t); // ICP via SVD

    cout<<"ICP via SVD results: "<<endl;
    cout<<"R = "<<m3R<<endl;
    cout<<"t = "<<v3t<<endl;

    cout<<"calling bundle adjustment"<<endl;
    Mat R, t;
    R = (Mat_<double>(3, 3) <<
                            m3R(0, 0), m3R(0, 1), m3R(0, 2),
                            m3R(1, 0), m3R(1, 1), m3R(1, 2),
                            m3R(2, 0), m3R(2, 1), m3R(2, 2)
    );
    t = (Mat_<double>(3, 1) << v3t(0, 0), v3t(1, 0), v3t(2, 0));
    bundle_adjustment_3d3d( pts1, pts2, R, t );

    // verify p1 = R*p2 + t
    Mat p1;
    for ( int i=0; i<5; i++ ) {
        cout << "p1 = " << pts1[i] << endl;
        cout << "p2 = " << pts2[i] << endl;
        p1 = R * (Mat_<double>(3, 1) << pts2[i].x, pts2[i].y, pts2[i].z) + t;
        cout << "(R*p2+t) = \n" << p1 << endl;
        cout << endl;
    }
}


