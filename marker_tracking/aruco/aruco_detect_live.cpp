
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "aruco_detection.h"

int main() {
  cv::VideoCapture inputVideo;
  inputVideo.open(0);
  while (inputVideo.grab()) {
    cv::Mat img_in, img_gray, img_out;
    inputVideo.retrieve(img_in);

    img_in.copyTo(img_out);
    cv::cvtColor(img_in, img_gray, CV_BGR2GRAY);

    aruco_detect(img_gray, img_out);

    cv::imshow("out", img_out);
    char key = (char)cv::waitKey(30);
    if (key == 27) break;
  }

  return 0;
}