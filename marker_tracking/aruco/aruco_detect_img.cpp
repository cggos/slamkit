
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "aruco_detection.h"

int main() {
  cv::Mat img_in, img_gray, img_out;

  img_in = cv::imread("../data/imgin_dict6x6_250_sz110mm.png");

  img_in.copyTo(img_out);
  cv::cvtColor(img_in, img_gray, CV_BGR2GRAY);

  aruco_detect(img_gray, img_out);

  cv::imshow("out", img_out);
  cv::waitKey();

  return 0;
}