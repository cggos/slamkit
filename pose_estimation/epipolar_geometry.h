#ifndef _EXTRA_H_
#define _EXTRA_H_

#include <Eigen/Core>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace Eigen;
using namespace cv;

cv::Mat find_essential_mat_cv(InputArray _points1, InputArray _points2, double focal, Point2d pp);

void decompose_essential_mat_cv(InputArray _E, OutputArray _R1, OutputArray _R2, OutputArray _t);

void decompose_essential_mat(const Matrix3d &E, Matrix3d &R1, Matrix3d &R2, Vector3d &t1, Vector3d &t2);

int recover_pose_cv(InputArray E, InputArray _points1, InputArray _points2, OutputArray _R,
                     OutputArray _t, double focal, Point2d pp=Point2d(0, 0), InputOutputArray _mask=noArray());

#endif