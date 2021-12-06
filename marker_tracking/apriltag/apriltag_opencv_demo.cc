/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.
This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the Regents of The University of Michigan.
*/

#include <math.h>
#include <pangolin/pangolin.h>
#include <sophus/se3.h>

#include <Eigen/Core>
#include <boost/format.hpp>
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "apriltag/apriltag_pose.h"

extern "C" {
#include "apriltag/apriltag.h"
#include "apriltag/common/getopt.h"
#include "apriltag/tag36h11.h"
}

// #include <g2o/core/base_vertex.h>
// #include <g2o/core/base_unary_edge.h>
// #include <g2o/core/block_solver.h>
// #include <g2o/core/optimization_algorithm_levenberg.h>
// #include <g2o/types/sba/types_six_dof_expmap.h>
// #include <g2o/solvers/dense/linear_solver_dense.h>
// #include <g2o/core/robust_kernel.h>
// #include <g2o/core/robust_kernel_impl.h>

Eigen::Matrix3d cam_R;
Eigen::Vector3d cam_t;
Eigen::Vector3d t_last, t_curr;
double update_t = 0;

typedef std::vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3> > VecSE3;
typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > VecVec3d;
typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > VecVec2d;
// apriltag 4-corner 3d points and 2d pixel
VecVec3d tagpoints;
VecVec2d tagpixels;

double fx = 847.225332;
double fy = 850.953794;
double cx = 336.154365;
double cy = 275.840396;

#define RATIO 0.4  // orb

// // g2o vertex that use sophus::SE3 as pose
// class VertexSophus : public g2o::BaseVertex<6, Sophus::SE3> {
// public:
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW

//     VertexSophus() {}
//     ~VertexSophus() {}

//     bool read(std::istream &is) {}
//     bool write(std::ostream &os) const {}

//     virtual void setToOriginImpl() {
//         _estimate = Sophus::SE3();
//     }

//     virtual void oplusImpl(const double *update_) {
//         Eigen::Map<const Eigen::Matrix<double, 6, 1>> update(update_);
//         setEstimate(Sophus::SE3::exp(update) * estimate());
//     }
// };

// class EdgeProjectXYZ2UVPoseOnly: public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexSophus >
// {
// public:
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW

//     virtual void computeError();
//     virtual void linearizeOplus();

//     virtual bool read( std::istream& in ){}
//     virtual bool write(std::ostream& os) const {};

//     Eigen::Vector3d point_;
// };

// void EdgeProjectXYZ2UVPoseOnly::computeError()
// {
//     // compute projection error ...
//     const VertexSophus *vertexTcw = static_cast<const VertexSophus*>( vertex(0) );
//     Eigen::Vector3d p_c = vertexTcw->estimate()*point_;
//     Eigen::Vector2d p_uv = Eigen::Vector2d (
//         fx * p_c ( 0,0 ) / p_c ( 2,0 ) + cx,
//         fy * p_c ( 1,0 ) / p_c ( 2,0 ) + cy
//     );

//     _error = _measurement - p_uv;
// }

// void EdgeProjectXYZ2UVPoseOnly::linearizeOplus()
// {
//     const VertexSophus* vertexTcw = static_cast<const VertexSophus* >( vertex(0) );
//     Eigen::Vector3d xyz_trans = vertexTcw->estimate()*point_;
//     double x = xyz_trans[0];
//     double y = xyz_trans[1];
//     double z = xyz_trans[2];
//     double z_2 = z*z;

//     _jacobianOplusXi ( 0,0 ) =  x*y/z_2 *fx;
//     _jacobianOplusXi ( 0,1 ) = - ( 1+ ( x*x/z_2 ) ) *fx;
//     _jacobianOplusXi ( 0,2 ) = y/z * fx;
//     _jacobianOplusXi ( 0,3 ) = -1./z *fx;
//     _jacobianOplusXi ( 0,4 ) = 0;
//     _jacobianOplusXi ( 0,5 ) = x/z_2 *fx;

//     _jacobianOplusXi ( 1,0 ) = ( 1+y*y/z_2 ) *fy;
//     _jacobianOplusXi ( 1,1 ) = -x*y/z_2 *fy;
//     _jacobianOplusXi ( 1,2 ) = -x/z *fy;
//     _jacobianOplusXi ( 1,3 ) = 0;
//     _jacobianOplusXi ( 1,4 ) = -1./z *fy;
//     _jacobianOplusXi ( 1,5 ) = y/z_2 *fy;
// }

// plot the poses and points for you, need pangolin
void Draw(const VecSE3 &poses, const VecVec3d &points, const apriltag_detection_info_t &info);

void triangulation(
    const std::vector<cv::KeyPoint> &keypoint_1,
    const std::vector<cv::KeyPoint> &keypoint_2,
    const std::vector<cv::DMatch> &matches,
    cv::Mat T1, cv::Mat T2,
    std::vector<cv::Point3d> &points);

int main(int argc, char *argv[]) {
    getopt_t *getopt = getopt_create();

    getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
    getopt_add_bool(getopt, 'd', "debug", 1, "Enable debugging output (slow)");
    getopt_add_bool(getopt, 'q', "quiet", 0, "Reduce output");
    getopt_add_string(getopt, 'f', "family", "tag36h11", "Tag family to use");
    getopt_add_int(getopt, 't', "threads", "1", "Use this many CPU threads");
    getopt_add_double(getopt, 'x', "decimate", "2.0", "Decimate input image by this factor");
    getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input");
    getopt_add_bool(getopt, '0', "refine-edges", 1, "Spend more time trying to align edges of tags");

    if (!getopt_parse(getopt, argc, argv, 1) ||
        getopt_get_bool(getopt, "help")) {
        printf("Usage: %s [options]\n", argv[0]);
        getopt_do_usage(getopt);
        exit(0);
    }

    // Initialize camera
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Couldn't open video capture device" << std::endl;
        return -1;
    }

    // Initialize tag detector with options
    apriltag_family_t *tf = NULL;
    const char *famname = getopt_get_string(getopt, "family");
    if (!strcmp(famname, "tag36h11")) {
        tf = tag36h11_create();
    } else {
        printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
        exit(-1);
    }

    apriltag_detection_info_t info;
    info.tagsize = 14;  //cm
    info.fx = fx;
    info.fy = fy;
    info.cx = cx;
    info.cy = cy;

    double scale = info.tagsize / 2.0;  // apriltag 4-corner 3d points

    tagpoints.push_back(Eigen::Vector3d(-scale, scale, 0));
    tagpoints.push_back(Eigen::Vector3d(scale, scale, 0));
    tagpoints.push_back(Eigen::Vector3d(scale, -scale, 0));
    tagpoints.push_back(Eigen::Vector3d(-scale, -scale, 0));

    double camera_matrix[] =
        {
            info.fx, 0.0, info.cx,
            0.0, info.fy, info.cy,
            0.0, 0.0, 1.0};
    double dist_coeff[] = {0.115580, 0.108171, 0.008395, 0.004415};

    cv::Mat m_camera_matrix = cv::Mat(3, 3, CV_64FC1, camera_matrix).clone();
    cv::Mat m_dist_coeff = cv::Mat(1, 4, CV_64FC1, dist_coeff).clone();

    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = getopt_get_double(getopt, "decimate");
    td->quad_sigma = getopt_get_double(getopt, "blur");
    td->nthreads = getopt_get_int(getopt, "threads");
    td->debug = getopt_get_bool(getopt, "debug");
    td->refine_edges = getopt_get_bool(getopt, "refine-edges");

    cv::Mat frame, gray;
    VecSE3 poses;
    VecVec3d points;
    std::vector<cv::Point3d> points_cv;

    cv::Mat last_img, cur_img;
    cv::Mat T1, T2;
    // cv::Mat T1 = (cv::Mat_<double> (3,4) <<
    //     1,0,0,0,
    //     0,1,0,0,
    //     0,0,1,0);
    // cv::Mat T2 = (cv::Mat_<double> (3,4) <<
    //     R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0,0),
    //     R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1,0),
    //     R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2,0)
    //     );
    bool initial = false;
    int limit_min_times = 0;
    while (true) {
        cap >> frame;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // Make an image_u8_t header for the Mat data
        image_u8_t im = {.width = gray.cols,
                         .height = gray.rows,
                         .stride = gray.cols,
                         .buf = gray.data};

        zarray_t *detections = apriltag_detector_detect(td, &im);
        // cout << zarray_size(detections) << " tags detected" << endl;

        // Draw detection outlines
        for (int i = 0; i < zarray_size(detections); i++) {
            limit_min_times++;

            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            info.det = det;
            apriltag_pose_t pose;
            double err = estimate_tag_pose(&info, &pose);
            double wx, wy, wz;

            double scale = 107.0 / 220.0;  //depth scale = realvalue / measurement = 107cm/220

            wx = pose.t->data[0] * scale;
            wy = pose.t->data[1] * scale;
            wz = pose.t->data[2] * scale;
            // cout << wx <<"  "<< wy << " " << wz << endl;

            if (!initial) {
                last_img = gray.clone();
                cur_img = gray.clone();
                T1 = (cv::Mat_<double>(3, 4) << pose.R->data[0], pose.R->data[1], pose.R->data[2], pose.t->data[0],
                      pose.R->data[3], pose.R->data[4], pose.R->data[5], pose.t->data[1],
                      pose.R->data[6], pose.R->data[7], pose.R->data[8], pose.t->data[2]);
                T2 = (cv::Mat_<double>(3, 4) << pose.R->data[0], pose.R->data[1], pose.R->data[2], pose.t->data[0],
                      pose.R->data[3], pose.R->data[4], pose.R->data[5], pose.t->data[1],
                      pose.R->data[6], pose.R->data[7], pose.R->data[8], pose.t->data[2]);
                // initial = true;
                if (limit_min_times > 10) {
                    initial = true;
                }
            } else {
                // std::cout << "initial is ok!!!!" << "\n";
                cur_img = gray.clone();
                T2 = (cv::Mat_<double>(3, 4) << pose.R->data[0], pose.R->data[1], pose.R->data[2], pose.t->data[0],
                      pose.R->data[3], pose.R->data[4], pose.R->data[5], pose.t->data[1],
                      pose.R->data[6], pose.R->data[7], pose.R->data[8], pose.t->data[2]);
            }
            // std::cout << "initial:" << initial << "||" << T1.size() << "||" << T2.size() << "\n";
            // std::cout << "last_img" << last_img.size() << " cur_img" << cur_img.size() << "\n";

            double depth = sqrt(wx * wx + wy * wy + wz * wz);

            // cout << "depth = " << depth << endl;
            // cout << "R size:" << pose.R->nrows <<" "<< pose.R->ncols << endl;  //3*3
            // cout << "R:" << endl;
            // cout << pose.R->data[0] <<" "<< pose.R->data[1] << " " << pose.R->data[2] << endl;  //3*3
            // cout << pose.R->data[3] <<" "<< pose.R->data[4] << " " << pose.R->data[5] << endl;  //3*3
            // cout << pose.R->data[6] <<" "<< pose.R->data[7] << " " << pose.R->data[8] << endl;  //3*3

            cv::line(frame, cv::Point(det->p[0][0], det->p[0][1]),
                     cv::Point(det->p[1][0], det->p[1][1]),
                     cv::Scalar(0xff, 0xff, 0xff), 2);
            cv::line(frame, cv::Point(det->p[0][0], det->p[0][1]),
                     cv::Point(det->p[3][0], det->p[3][1]),
                     cv::Scalar(0xff, 0xff, 0xff), 2);
            cv::line(frame, cv::Point(det->p[1][0], det->p[1][1]),
                     cv::Point(det->p[2][0], det->p[2][1]),
                     cv::Scalar(0xff, 0xff, 0xff), 2);
            cv::line(frame, cv::Point(det->p[2][0], det->p[2][1]),
                     cv::Point(det->p[3][0], det->p[3][1]),
                     cv::Scalar(0xff, 0xff, 0xff), 2);

            cv::circle(frame, cv::Point(det->p[0][0], det->p[0][1]),
                       2,
                       cv::Scalar(0xff, 0xff, 0), 2);

            std::stringstream ss;
            ss << det->id;
            cv::String text = ss.str();
            int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
            double fontscale = 1.0;
            int baseline;
            cv::Size textsize = getTextSize(text, fontface, fontscale, 2,
                                            &baseline);
            putText(frame, text, cv::Point(det->c[0] - textsize.width / 2, det->c[1] + textsize.height / 2),
                    fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 2);

            cam_t << wx, wy, wz;
            cam_R << pose.R->data[0], pose.R->data[1], pose.R->data[2],
                pose.R->data[3], pose.R->data[4], pose.R->data[5],
                pose.R->data[6], pose.R->data[7], pose.R->data[8];

            // three-dimensional cube test (cudePoints)
            std::vector<cv::Point3f> cubePoints;
            cubePoints.push_back(cv::Point3f(-0.5, -0.5, 0.0));
            cubePoints.push_back(cv::Point3f(0.5, -0.5, 0.0));
            cubePoints.push_back(cv::Point3f(0.5, 0.5, 0.0));
            cubePoints.push_back(cv::Point3f(-0.5, 0.5, 0.0));
            cubePoints.push_back(cv::Point3f(-0.5, -0.5, 1.0));
            cubePoints.push_back(cv::Point3f(0.5, -0.5, 1.0));
            cubePoints.push_back(cv::Point3f(0.5, 0.5, 1.0));
            cubePoints.push_back(cv::Point3f(-0.5, 0.5, 1.0));

            std::vector<cv::Point2f> imagePoints;
            bool solvePnP_isok = false;
            cv::Mat cv_cam_R, cv_cam_t;
            if (solvePnP_isok) {
                cv::Point3f corners_3d[] =
                    {
                        cv::Point3f(-0.5f, -0.5f, 0),
                        cv::Point3f(-0.5f, 0.5f, 0),
                        cv::Point3f(0.5f, 0.5f, 0),
                        cv::Point3f(0.5f, -0.5f, 0)};
                std::vector<cv::Point3f> m_corners_3d = std::vector<cv::Point3f>(corners_3d, corners_3d + 4);
                std::vector<cv::Point2f> m_corners;
                m_corners.push_back(cv::Point2f(det->p[0][0], det->p[0][1]));
                m_corners.push_back(cv::Point2f(det->p[1][0], det->p[1][1]));
                m_corners.push_back(cv::Point2f(det->p[2][0], det->p[2][1]));
                m_corners.push_back(cv::Point2f(det->p[3][0], det->p[3][1]));

                cv::Mat rot_vec;
                cv::solvePnP(m_corners_3d, m_corners, m_camera_matrix, m_dist_coeff, rot_vec, cv_cam_t);
                cv::Rodrigues(rot_vec, cv_cam_R);
                cv::projectPoints(cubePoints, cv_cam_R, cv_cam_t, m_camera_matrix, m_dist_coeff, imagePoints);
            } else {
                for (auto &p : cubePoints) {
                    p = p * 5;
                }
                cv::eigen2cv(cam_R, cv_cam_R);
                cv::eigen2cv(cam_t, cv_cam_t);
                cv::projectPoints(cubePoints, cv_cam_R, cv_cam_t, m_camera_matrix, m_dist_coeff, imagePoints);
            }

            // draw cube lines
            cv::line(frame, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 0xff), 2);
            cv::line(frame, imagePoints[1], imagePoints[2], cv::Scalar(0, 0, 0xff), 2);
            cv::line(frame, imagePoints[2], imagePoints[3], cv::Scalar(0, 0, 0xff), 2);
            cv::line(frame, imagePoints[3], imagePoints[0], cv::Scalar(0, 0, 0xff), 2);

            cv::line(frame, imagePoints[4], imagePoints[5], cv::Scalar(0xff, 0, 0), 2);
            cv::line(frame, imagePoints[5], imagePoints[6], cv::Scalar(0xff, 0, 0), 2);
            cv::line(frame, imagePoints[6], imagePoints[7], cv::Scalar(0xff, 0, 0), 2);
            cv::line(frame, imagePoints[7], imagePoints[4], cv::Scalar(0xff, 0, 0), 2);

            cv::line(frame, imagePoints[0], imagePoints[4], cv::Scalar(0, 0xff, 0), 2);
            cv::line(frame, imagePoints[1], imagePoints[5], cv::Scalar(0, 0xff, 0), 2);
            cv::line(frame, imagePoints[2], imagePoints[6], cv::Scalar(0, 0xff, 0), 2);
            cv::line(frame, imagePoints[3], imagePoints[7], cv::Scalar(0, 0xff, 0), 2);

            // Note that every variable that we compute is proportional to the scale factor of H.
            // or pnp, cv::solvePnP  cv::Rodrigues

            double H00 = MATD_EL(det->H, 0, 0);
            double H01 = MATD_EL(det->H, 0, 1);
            double H02 = MATD_EL(det->H, 0, 2);
            double H10 = MATD_EL(det->H, 1, 0);
            double H11 = MATD_EL(det->H, 1, 1);
            double H12 = MATD_EL(det->H, 1, 2);
            double H20 = MATD_EL(det->H, 2, 0);
            double H21 = MATD_EL(det->H, 2, 1);
            double H22 = MATD_EL(det->H, 2, 2);
            // cout << "H = " << H00 << " " << H01 << endl;

            for (int i = 0; i < 4; i++) {
                tagpixels.push_back(Eigen::Vector2d(det->p[i][0], det->p[i][1]));
            }
            Sophus::SE3 SE3_Rt(cam_R, cam_t);
            poses.push_back(SE3_Rt);

            // last_img = gray.clone();
            // cur_img = gray.clone();

            if (initial) {
                // ORB Features
                std::vector<cv::KeyPoint> keypoints_sence, keypoints_obj;
                cv::Mat descriptors_box, descriptors_sence;
                cv::Ptr<cv::ORB> detector = cv::ORB::create();

                detector->detectAndCompute(last_img, cv::Mat(), keypoints_sence, descriptors_sence);
                detector->detectAndCompute(cur_img, cv::Mat(), keypoints_obj, descriptors_box);
                std::vector<cv::DMatch> matches;
                // 初始化flann匹配
                // cv::Ptr<cv::FlannBasedMatcher> matcher = cv::FlannBasedMatcher::create(); // default is bad, using local sensitive hash(LSH)
                cv::Ptr<cv::DescriptorMatcher> matcher = cv::makePtr<cv::FlannBasedMatcher>(cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));
                matcher->match(descriptors_box, descriptors_sence, matches);
                // 发现匹配
                std::vector<cv::DMatch> goodMatches;

                float maxdist = 0;
                for (unsigned int i = 0; i < matches.size(); ++i) {
                    // printf("dist : %.2f \n", matches[i].distance);
                    maxdist = cv::max(maxdist, matches[i].distance);
                }
                for (unsigned int i = 0; i < matches.size(); ++i) {
                    if (matches[i].distance < maxdist * RATIO)
                        goodMatches.push_back(matches[i]);
                }

                // cv::Mat dst;
                // cv::drawMatches(cur_img, keypoints_obj, last_img, keypoints_sence, goodMatches, dst);
                // cv::imshow("output", dst);
                // cv::waitKey(0);
                // cv::Mat R_cv = (cv::Mat_<double> (3,3) <<
                //     pose.R->data[0], pose.R->data[1], pose.R->data[2],
                //     pose.R->data[3], pose.R->data[4], pose.R->data[5],
                //     pose.R->data[6], pose.R->data[7], pose.R->data[8]);

                // cv::Mat t_cv = (cv::Mat_<double> (3,1) <<
                //     wx, wy, wz);

                // std::cout << R_cv << "\n";
                // std::cout << t_cv << "\n";
                // std::cout << "keypoints_sence:" << keypoints_sence.size() << "\n";
                // std::cout << "keypoints_obj:" << keypoints_obj.size() << "\n";
                // std::cout << "T1:" << T1 << "\n\n";
                // std::cout << "T2:" << T2 << "\n";
                // printf("total match points : %d || goodMatches: %d\n", matches.size(), goodMatches.size());

                // 绘制关键点
                // cv::Mat keypoint_img;
                cv::drawKeypoints(frame, keypoints_obj, frame, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
                // cv::imshow("KeyPoints Image", keypoint_img);

                t_last << T1.at<double>(0, 3), T1.at<double>(1, 3), T1.at<double>(2, 3);
                t_curr << T2.at<double>(0, 3), T2.at<double>(1, 3), T2.at<double>(2, 3);
                Eigen::Vector3d t_tmp = t_last - t_curr;
                update_t = fabsf(t_tmp[0]) + fabsf(t_tmp[1]) + fabsf(t_tmp[2]);
                std::cout << "update_t:" << update_t << "\n";
                // std::cout << "t_last:" << t_last << "\n";
                // std::cout << "t_curr:" << t_curr << "\n";
                if (goodMatches.size() > 0 && update_t > 10) {
                    // std::cout << "keypoints_sence:" << keypoints_sence.size() << "\n";
                    // std::cout << "keypoints_obj:" << keypoints_obj.size() << "\n";
                    // std::cout << "T1:" << T1 << "\n";
                    // std::cout << "T2:" << T2 << "\n";
                    triangulation(keypoints_sence, keypoints_obj, goodMatches, T1, T2, points_cv);
                }
            }
        }
        if (update_t > 10) {
            last_img = cur_img.clone();
            T1 = T2.clone();
        }

        apriltag_detections_destroy(detections);

        imshow("Tag Detections", frame);

        if (cv::waitKey(30) == 'q')
            break;
    }
    // // using bundle adjustment to optimize the pose
    // typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,2>> Block;
    // Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
    // Block* solver_ptr = new Block( linearSolver );
    // g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
    // g2o::SparseOptimizer optimizer;
    // optimizer.setAlgorithm ( solver );

    // // add pose vertices
    // for( int j = 0; j < poses.size(); j++ ){
    //     VertexSophus* vertexTcw = new VertexSophus();
    //     vertexTcw->setEstimate( poses[j] );
    //     vertexTcw->setId( j );
    //     optimizer.addVertex( vertexTcw );
    // }

    // // edges
    // for( int c = 0; c < poses.size(); c++ )
    //     for ( int i=0; i<4; i++ ){
    //         // 3D -> 2D projection
    //         EdgeProjectXYZ2UVPoseOnly* edge = new EdgeProjectXYZ2UVPoseOnly();
    //         // edge->setId ( i );
    //         edge->setVertex ( 0, dynamic_cast<VertexSophus*>(optimizer.vertex(c)) );
    //         edge->point_ = tagpoints[i];
    //         edge->setMeasurement ( tagpixels[c*4 + i] );
    //         edge->setInformation ( Eigen::Matrix2d::Identity() );
    //         optimizer.addEdge ( edge );
    //     }

    // optimizer.initializeOptimization();
    // optimizer.optimize ( 10 );

    // // fetch data from the optimizer
    // for(int c = 0; c < poses.size(); c++){
    //     // Eigen::Vector3d Pw = dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(p))->estimate();
    //     // points[p] = Pw;
    //     Sophus::SE3 Tcw = dynamic_cast<VertexSophus*>( optimizer.vertex(c) )->estimate();
    //     poses[c] = Tcw;
    // }

    for (int pt_indx = 0; pt_indx < points_cv.size(); pt_indx++) {
        points.push_back(Eigen::Vector3d(points_cv[pt_indx].x, points_cv[pt_indx].y, points_cv[pt_indx].z));
    }
    // points.push_back( Eigen::Vector3d ( -scale,  scale, 0 ) );
    // points.push_back( Eigen::Vector3d (  scale,  scale, 0 ) );
    // points.push_back( Eigen::Vector3d (  scale, -scale, 0 ) );
    // points.push_back( Eigen::Vector3d ( -scale, -scale, 0 ) );

    Draw(poses, points, info);
    apriltag_detector_destroy(td);

    if (!strcmp(famname, "tag36h11")) {
        tag36h11_destroy(tf);
    }

    getopt_destroy(getopt);

    return 0;
}

cv::Point2d pixel2cam(const cv::Point2d &p, const cv::Mat &K) {
    return cv::Point2d(
        (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
        (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1));
}

void triangulation(
    const std::vector<cv::KeyPoint> &keypoint_1,
    const std::vector<cv::KeyPoint> &keypoint_2,
    const std::vector<cv::DMatch> &matches,
    cv::Mat T1, cv::Mat T2,
    std::vector<cv::Point3d> &points) {
    // cv::Mat T1 = (cv::Mat_<double> (3,4) <<
    //     1,0,0,0,
    //     0,1,0,0,
    //     0,0,1,0);
    // cv::Mat T2 = (cv::Mat_<double> (3,4) <<
    //     R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0,0),
    //     R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1,0),
    //     R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2,0)
    //     );

    // info.fx,    0.0,       info.cx,
    // 0.0,        info.fy,   info.cy,
    // 0.0,        0.0,       1.0
    cv::Mat K = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

    std::vector<cv::Point2d> pts_1, pts_2;
    for (cv::DMatch m : matches) {
        // 将像素坐标转换至相机坐标
        pts_1.push_back(pixel2cam(keypoint_1[m.queryIdx].pt, K));
        pts_2.push_back(pixel2cam(keypoint_2[m.trainIdx].pt, K));
    }

    cv::Mat pts_4d;
    cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);

    // 转换成非齐次坐标
    for (int i = 0; i < pts_4d.cols; i++) {
        cv::Mat x = pts_4d.col(i);

        if (x.at<float>(3, 0) < 1.0) continue;

        x /= x.at<float>(3, 0);  // 归一化
        cv::Point3d p(
            x.at<float>(0, 0),
            x.at<float>(1, 0),
            x.at<float>(2, 0));
        points.push_back(p);
    }
}

void Draw(const VecSE3 &poses, const VecVec3d &points, const apriltag_detection_info_t &info) {
    if (poses.empty() || points.empty()) {
        std::cerr << "parameter is empty!" << std::endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0));

    pangolin::View &d_cam = pangolin::CreateDisplay()
                                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
                                .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

        // intrinsics
        float fx = info.fx;
        float fy = info.fy;
        float cx = info.cx;
        float cy = info.cy;

        // draw poses
        float sz = 0.1;
        int width = 640, height = 480;
        for (auto &Tcw : poses) {
            glPushMatrix();
            Sophus::Matrix4f m = Tcw.inverse().matrix().cast<float>();
            glMultMatrixf((GLfloat *)m.data());
            glColor3f(1, 0, 0);
            glLineWidth(2);
            glBegin(GL_LINES);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
            glEnd();
            glPopMatrix();
        }

        // points
        glPointSize(2);
        glBegin(GL_POINTS);
        for (size_t i = 0; i < points.size(); i++) {
            glColor3f(0.0, 1, 0);
            glVertex3d(points[i][0], points[i][1], points[i][2]);
        }
        glEnd();

        pangolin::FinishFrame();
        usleep(5000);  // sleep 5 ms
    }
}
