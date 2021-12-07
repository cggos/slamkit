
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "aruco_detection.h"

int main() {
  // cv::Mat markerImage;
  // cv::aruco::drawMarker(dictionary, 23, 200, markerImage, 1);
  // cv::imwrite("marker23.png", markerImage);

  // cv::Mat inputImage;
  // inputImage = cv::imread("marker23.png");

  // std::vector<int> markerIds;
  // std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
  // cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
  // cv::aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

  // cv::Mat outputImage = inputImage.clone();
  // cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

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