#ifndef ARUCO_DETECTION_H
#define ARUCO_DETECTION_H

#include <opencv2/aruco.hpp>

void aruco_detect(const cv::Mat &img_gray, cv::Mat &img_out) {
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners, rejects;

  cv::aruco::detectMarkers(img_gray, cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250), corners, ids,
                           cv::aruco::DetectorParameters::create(), rejects);
  if (ids.size() > 0) cv::aruco::drawDetectedMarkers(img_out, corners, ids);

  // Logitech C270
  double fx = 869.429260;
  double fy = 875.089417;
  double cx = 293.636922;
  double cy = 273.367566;
  double k1 = 0.097962;
  double k2 = 0.164297;
  double p1 = 0.005676;
  double p2 = -0.010076;
  cv::Mat camK = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
  cv::Mat camD = (cv::Mat_<double>(4, 1) << k1, k2, p1, p2);

  std::vector<cv::Vec3d> rvecs, tvecs;
  cv::aruco::estimatePoseSingleMarkers(corners, 0.11, camK, camD, rvecs, tvecs);

  for (int i = 0; i < ids.size(); ++i) cv::aruco::drawAxis(img_out, camK, camD, rvecs[i], tvecs[i], 0.1);
}

#endif  // ARUCO_DETECTION_H