#include <iostream>
#include <fstream>
#include <list>
#include <vector>
#include <chrono>
#include <ctime>
#include <climits>

#include "pose_estimation.h"

#include "tum_data_rgbd.h"

using namespace std;
using namespace cv;

int main ( int argc, char** argv )
{
    cg::TUMDataRGBD tum_data_rgbd("/home/cg/dev_sdb/datasets/TUM/RGBD-SLAM-Dataset/rgbd_dataset_freiburg2_desk/", 1);

    vector<cv::Mat> colorImgs, depthImgs;
    const int count = 10;
    for(int i=0; i<count; ++i) {
        cv::Mat img_color;
        cv::Mat img_depth;
        if (!tum_data_rgbd.get_rgb_depth(img_color, img_depth)) {
            std::cerr << "get_rgb_depth failed!" << std::endl;
            return -1;
        }
        colorImgs.push_back(img_color);
        depthImgs.push_back(img_depth);
    }

    float depth_scale = tum_data_rgbd.depth_scale_;

    Eigen::Matrix3f K;
    tum_data_rgbd.getK(K);

    Eigen::Isometry3d Tcw = Eigen::Isometry3d::Identity();

    vector<Measurement> measurements;
    cv::Mat prev_color, color, depth;
    for ( int index=0; index<count; index++ )
    {
        cout<<"*********** loop "<<index<<" ************"<<endl;
        color = colorImgs[index];
        depth = depthImgs[index];
        if ( color.data==nullptr || depth.data==nullptr )
            continue;
        cv::Mat gray;
        cv::cvtColor ( color, gray, cv::COLOR_BGR2GRAY );
        if ( index ==0 ) {
            // select the pixels with high gradiants
            for (int x = 10; x < gray.cols - 10; x++) {
                for (int y = 10; y < gray.rows - 10; y++) {
                    Eigen::Vector2d delta(
                            gray.ptr<uchar>(y)[x + 1] - gray.ptr<uchar>(y)[x - 1],
                            gray.ptr<uchar>(y + 1)[x] - gray.ptr<uchar>(y - 1)[x]
                    );
                    if (delta.norm() < 50)
                        continue;
                    ushort d = depth.ptr<ushort>(y)[x];
                    if (d == 0)
                        continue;
                    Eigen::Vector3d p3d = project2Dto3D(x, y, d, K, depth_scale);
                    float grayscale = float(gray.ptr<uchar>(y)[x]);
                    measurements.push_back(Measurement(p3d, grayscale));
                }
            }
            prev_color = color.clone();
            cout << "add total " << measurements.size() << " measurements." << endl;
            continue;
        }

        // 使用直接法计算相机运动
        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        pose_estimation_direct ( measurements, &gray, K, Tcw );
        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> ( t2-t1 );
        cout<<"direct method costs time: "<<time_used.count() <<" seconds."<<endl;
        cout<<"Tcw="<<Tcw.matrix() <<endl;

        // plot the feature points
        cv::Mat img_show ( color.rows*2, color.cols, CV_8UC3 );
        prev_color.copyTo ( img_show ( cv::Rect ( 0,0,color.cols, color.rows ) ) );
        color.copyTo ( img_show ( cv::Rect ( 0,color.rows,color.cols, color.rows ) ) );
        for ( Measurement m:measurements )
        {
            if ( rand() > RAND_MAX/5 )
                continue;
            Eigen::Vector3d p = m.pos_world;
            Eigen::Vector2d pixel_prev = project3Dto2D ( p ( 0,0 ), p ( 1,0 ), p ( 2,0 ), K);
            Eigen::Vector3d p2 = Tcw*m.pos_world;
            Eigen::Vector2d pixel_now = project3Dto2D ( p2 ( 0,0 ), p2 ( 1,0 ), p2 ( 2,0 ), K);
            if ( pixel_now(0,0)<0 || pixel_now(0,0)>=color.cols || pixel_now(1,0)<0 || pixel_now(1,0)>=color.rows )
                continue;

            float b = 0;
            float g = 250;
            float r = 0;
            img_show.ptr<uchar>( pixel_prev(1,0) )[int(pixel_prev(0,0))*3] = b;
            img_show.ptr<uchar>( pixel_prev(1,0) )[int(pixel_prev(0,0))*3+1] = g;
            img_show.ptr<uchar>( pixel_prev(1,0) )[int(pixel_prev(0,0))*3+2] = r;

            img_show.ptr<uchar>( pixel_now(1,0)+color.rows )[int(pixel_now(0,0))*3] = b;
            img_show.ptr<uchar>( pixel_now(1,0)+color.rows )[int(pixel_now(0,0))*3+1] = g;
            img_show.ptr<uchar>( pixel_now(1,0)+color.rows )[int(pixel_now(0,0))*3+2] = r;
            cv::circle ( img_show, cv::Point2d ( pixel_prev ( 0,0 ), pixel_prev ( 1,0 ) ), 4, cv::Scalar ( b,g,r ), 2 );
            cv::circle ( img_show, cv::Point2d ( pixel_now ( 0,0 ), pixel_now ( 1,0 ) +color.rows ), 4, cv::Scalar ( b,g,r ), 2 );
        }
        cv::imshow ( "result", img_show );
        cv::waitKey ( 0 );

    }
    return 0;
}


