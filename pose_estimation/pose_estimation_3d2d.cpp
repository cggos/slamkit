#include <iostream>

#include "pose_estimation.h"

using namespace std;
using namespace cv;

#include "tum_data_rgbd.h"

int main ( int argc, char** argv )
{
    Mat img_1, img_2, img_d;
    Mat K;
    float depth_scale = 1.f;

    cg::TUMDataRGBD tum_data_rgbd("/home/cg/dev_sdb/datasets/TUM/RGBD-SLAM-Dataset/rgbd_dataset_freiburg2_desk/", 1);
    {
        vector<cv::Mat> colorImgs, depthImgs;
        const int count = 25;
        for (int i = 0; i < count; ++i) {
            cv::Mat img_color, img_depth;
            if (!tum_data_rgbd.get_rgb_depth(img_color, img_depth)) {
                std::cerr << "get_rgb failed!" << std::endl;
                return -1;
            }
            colorImgs.push_back(img_color);
            depthImgs.push_back(img_depth);
        }
        img_1 = colorImgs[1];
        img_2 = colorImgs[20];
        img_d = depthImgs[1];

        tum_data_rgbd.getK(K);
        depth_scale = tum_data_rgbd.depth_scale_;
    }

    vector<KeyPoint> kpts_1, kpts_2;
    vector<DMatch> matches;
    find_feature_matches ( img_1, img_2, kpts_1, kpts_2, matches );
    cout<<"一共找到了"<<matches.size() <<"组匹配点"<<endl;

    vector<Point3f> pts_3d;
    vector<Point2f> pts_2d;
    for ( DMatch m : matches ) {
        int x1 = int(kpts_1[m.queryIdx].pt.x);
        int y1 = int(kpts_1[m.queryIdx].pt.y);
        ushort d = img_d.ptr<unsigned short>(y1)[x1];
        if (d == 0)   // bad depth
            continue;
        float dd = d / depth_scale;

        Point2d p1 = pixel2cam(kpts_1[m.queryIdx].pt, K);

        pts_3d.push_back(Point3f(p1.x * dd, p1.y * dd, dd));
        pts_2d.push_back(kpts_2[m.trainIdx].pt);
    }

    cout<<"3d-2d pairs: "<<pts_3d.size() <<endl;

    Mat om, t;
    solvePnP ( pts_3d, pts_2d, K, Mat(), om, t, false, SOLVEPNP_EPNP );
    Mat R;
    cv::Rodrigues ( om, R ); // rotation vector to rotation matrix

    cout<<"R="<<endl<<R<<endl;
    cout<<"t="<<endl<<t<<endl;

    cout<<"calling bundle adjustment"<<endl;

    bundle_adjustment_3d2d ( pts_3d, pts_2d, K, R, t );
}
