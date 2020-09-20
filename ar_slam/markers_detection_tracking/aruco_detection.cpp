#include <opencv2/aruco.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int main() {
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

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
    inputVideo.open(4);
    while (inputVideo.grab()) {
        cv::Mat image, imageOut;
        inputVideo.retrieve(image);
        image.copyTo(imageOut);
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(image, dictionary, corners, ids);
        // // if at least one marker detected
        // if (ids.size() > 0)
        //     cv::aruco::drawDetectedMarkers(imageOut, corners, ids);

        cv::Mat cameraMatrix, distCoeffs;
        cameraMatrix = (cv::Mat_<float>(3, 3)) << 847.225332, 0.000000, 336.154365, 0.000000, 850.953794, 275.840396, 0.000000, 0.000000, 1.000000;
        distCoeffs = (cv::Mat_<float>(4, 1)) << 0.115580, 0.108171, 0.008395, 0.004415;

        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);

        image.copyTo(imageOut);
        for (int i = 0; i < rvecs.size(); ++i) {
            auto rvec = rvecs[i];
            auto tvec = tvecs[i];
            cv::aruco::drawAxis(imageOut, cameraMatrix, distCoeffs, rvec, tvec, 0.1);
        }

        cv::imshow("out", imageOut);
        char key = (char)cv::waitKey(30);
        if (key == 27)
            break;
    }

    return 0;
}