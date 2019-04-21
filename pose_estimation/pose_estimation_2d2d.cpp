#include <iostream>

#include "pose_estimation.h"

using namespace std;

/****************************************************
 * 2D-2D的特征匹配估计相机运动
 * **************************************************/

// 相机内参,TUM Freiburg2
double fx = 520.9;
double fy = 521.0;
double cx = 325.1;
double cy = 249.7;

int main ( int argc, char** argv )
{
    if ( argc != 3 ) {
        cout << "usage: pose_estimation_2d2d img1 img2" << endl;
        return 1;
    }

    Mat img_1 = imread ( argv[1], CV_LOAD_IMAGE_COLOR );
    Mat img_2 = imread ( argv[2], CV_LOAD_IMAGE_COLOR );

    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches ( img_1, img_2, keypoints_1, keypoints_2, matches );
    cout <<"一共找到了" << matches.size() << "组匹配点" << endl;

    Mat R,t;
    Mat K = ( Mat_<double> (3,3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0 );
    pose_estimation_2d2d ( keypoints_1, keypoints_2, matches, K, R, t );

    //-- 验证 E=t^R*scale
    Mat t_x = ( Mat_<double> ( 3,3 ) <<
                0,                      -t.at<double> ( 2,0 ),     t.at<double> ( 1,0 ),
                t.at<double> ( 2,0 ),    0,                       -t.at<double> ( 0,0 ),
               -t.at<double> ( 1.0 ),    t.at<double> ( 0,0 ),     0 );

    cout << "t^R=" << endl << t_x*R << endl;

    //-- 验证 对极约束
    for ( DMatch m: matches ) {

        Point2d pt1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
        Mat y1 = (Mat_<double>(3, 1) << pt1.x, pt1.y, 1);

        Point2d pt2 = pixel2cam(keypoints_2[m.trainIdx].pt, K);
        Mat y2 = (Mat_<double>(3, 1) << pt2.x, pt2.y, 1);

        Mat d = y2.t() * t_x * R * y1;
        cout << "epipolar constraint = " << d << endl;
    }

    return 0;
}
