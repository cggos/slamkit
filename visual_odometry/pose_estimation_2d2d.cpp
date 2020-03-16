#include <iostream>

#include "pose_estimation.h"

#include "tum_data_rgbd.h"

using namespace std;

int main ( int argc, char** argv )
{
    Mat img_1, img_2, img_d;
    Mat K;

    cg::TUMDataRGBD tum_data_rgbd("/home/cg/dev_sdb/datasets/TUM/RGBD-SLAM-Dataset/rgbd_dataset_freiburg2_desk/", 1);
    {
        vector<cv::Mat> colorImgs;
        const int count = 25;
        for (int i = 0; i < count; ++i) {
            cv::Mat img_color;
            if (!tum_data_rgbd.get_rgb(img_color)) {
                std::cerr << "get_rgb failed!" << std::endl;
                return -1;
            }
            colorImgs.push_back(img_color);
        }

        img_1 = colorImgs[0];
        img_2 = colorImgs[20];

        tum_data_rgbd.getK(K);
    }

    vector<KeyPoint> kpts_1, kpts_2;
    vector<DMatch> matches;
    find_feature_matches ( img_1, img_2, kpts_1, kpts_2, matches );
    cout <<"一共找到了" << matches.size() << "组匹配点" << endl;

    Mat R,t;
    pose_estimation_2d2d ( kpts_1, kpts_2, matches, K, R, t );

    // 验证 E=t^R*scale
    Mat t_x = ( Mat_<double> ( 3,3 ) <<
                0,                      -t.at<double> ( 2,0 ),     t.at<double> ( 1,0 ),
                t.at<double> ( 2,0 ),    0,                       -t.at<double> ( 0,0 ),
               -t.at<double> ( 1.0 ),    t.at<double> ( 0,0 ),     0 );

    cout << "t^R=" << endl << t_x*R << endl;

    // 验证 epipolar constraint
    for ( DMatch m: matches ) {
        Point2d pt1 = pixel2cam(kpts_1[m.queryIdx].pt, K);
        Mat y1 = (Mat_<double>(3, 1) << pt1.x, pt1.y, 1);

        Point2d pt2 = pixel2cam(kpts_2[m.trainIdx].pt, K);
        Mat y2 = (Mat_<double>(3, 1) << pt2.x, pt2.y, 1);

        Mat d = y2.t() * t_x * R * y1;
        cout << "epipolar constraint = " << d << endl;
    }

    // 三角化
    vector<Point3d> points;
    triangulation( kpts_1, kpts_2, matches, R, t, K, points );

    //-- 验证三角化点与特征点的重投影关系
    std::cout << std::endl;
    for ( int i=0; i<matches.size(); i++ ) {
        Point2d pt1_cam = pixel2cam(kpts_1[matches[i].queryIdx].pt, K);
        Point2d pt1_cam_3d(points[i].x / points[i].z, points[i].y / points[i].z);
        cout << "point in the first camera frame: " << pt1_cam << endl;
        cout << "point projected from 3D " << pt1_cam_3d << ", d=" << points[i].z << endl;

        // 第二个图
        Point2f pt2_cam = pixel2cam(kpts_2[matches[i].trainIdx].pt, K);
        Mat pt2_trans = R * (Mat_<double>(3, 1) << points[i].x, points[i].y, points[i].z) + t;
        pt2_trans /= pt2_trans.at<double>(2, 0);
        cout << "point in the second camera frame: " << pt2_cam << endl;
        cout << "point reprojected from second frame: " << pt2_trans.t() << endl;

        cout << endl;
    }

    return 0;
}
