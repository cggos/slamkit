#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <iostream>
#include <ctype.h>
#include <algorithm>
#include <iterator>
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>

using namespace cv;
using namespace std;

#define MAX_FRAME 1000
#define MIN_NUM_FEAT 2000

void feature_tracking(Mat img_1, Mat img_2, vector<Point2f>& points1, vector<Point2f>& points2, vector<uchar>& status) {
    vector<float> err;
    calcOpticalFlowPyrLK(img_1, img_2, points1, points2, status, err,
                         Size(21, 21), 3, TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.01), 0, 0.001);

    //getting rid of points for which the KLT tracking failed or those who have gone outside the frame
    int indexCorrection = 0;
    for (int i = 0; i < status.size(); i++) {
        Point2f pt = points2.at(i - indexCorrection);
        if ((status.at(i) == 0) || (pt.x < 0) || (pt.y < 0)) {
            if ((pt.x < 0) || (pt.y < 0)) {
                status.at(i) = 0;
            }
            points1.erase(points1.begin() + (i - indexCorrection));
            points2.erase(points2.begin() + (i - indexCorrection));
            indexCorrection++;
        }
    }
}

void feature_detection(Mat img_1, vector<Point2f>& points1) {
    vector<KeyPoint> keypoints_1;
    FAST(img_1, keypoints_1, 20, true);
    KeyPoint::convert(keypoints_1, points1, vector<int>());
}

double get_absoluteScale(int frame_id, int sequence_id, double z_cal) {
    string line;
    int i = 0;
    ifstream myfile("/home/cg/projects/datasets/kitti/dataset/poses/00.txt");
    double x = 0, y = 0, z = 0;
    double x_prev, y_prev, z_prev;
    if (myfile.is_open()) {
        while ((getline(myfile, line)) && (i <= frame_id)) {
            z_prev = z;
            x_prev = x;
            y_prev = y;
            std::istringstream in(line);
            //cout << line << '\n';
            for (int j = 0; j < 12; j++) {
                in >> z;
                if (j == 7) y = z;
                if (j == 3) x = z;
            }

            i++;
        }
        myfile.close();
    } else {
        cout << "Unable to open file";
        return 0;
    }

    return sqrt((x - x_prev) * (x - x_prev) + (y - y_prev) * (y - y_prev) + (z - z_prev) * (z - z_prev));
}

int main( int argc, char** argv ) {
    Mat R_f, t_f; // the final rotation and tranlation vectors containing the

    double scale = 1.00;
    char filename1[200];
    char filename2[200];
    sprintf(filename1, "/home/cg/projects/datasets/kitti/image_0/%06d.png", 0);
    sprintf(filename2, "/home/cg/projects/datasets/kitti/image_0/%06d.png", 1);

    //read the first two frames from the dataset
    Mat img_1 = imread(filename1, 0);
    Mat img_2 = imread(filename2, 0);

    if (!img_1.data || !img_2.data) {
        std::cout << " --(!) Error reading images " << std::endl;
        return -1;
    }

    // feature detection, tracking
    vector<Point2f> points1, points2;        //vectors to store the coordinates of the feature points
    feature_detection(img_1, points1);        //detect features in img_1
    vector<uchar> status;
    feature_tracking(img_1, img_2, points1, points2, status); //track those features to img_2

    //TODO: add a fucntion to load these values directly from KITTI's calib files
    // WARNING: different sequences in the KITTI VO dataset have different intrinsic/extrinsic parameters
    double focal = 718.8560;
    cv::Point2d pp(607.1928, 185.2157);
    Mat E, R, t, mask;
    E = findEssentialMat(points2, points1, focal, pp, RANSAC, 0.999, 1.0, mask);
    recoverPose(E, points2, points1, R, t, focal, pp, mask);

    Mat prevImage = img_2;
    Mat currImage;
    vector<Point2f> prevFeatures = points2;
    vector<Point2f> currFeatures;

    R_f = R.clone();
    t_f = t.clone();

    namedWindow("Road facing camera", WINDOW_AUTOSIZE);
    namedWindow("Trajectory", WINDOW_AUTOSIZE);

    Mat traj = Mat::zeros(600, 600, CV_8UC3);

    char filename[100];
    for (int numFrame = 2; numFrame < MAX_FRAME; numFrame++) {
        sprintf(filename, "/home/cg/projects/datasets/kitti/image_0/%06d.png", numFrame);
        currImage = imread(filename, 0);

        vector<uchar> status;
        feature_tracking(prevImage, currImage, prevFeatures, currFeatures, status);

        E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.999, 1.0, mask);
        recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask);

        Mat prevPts(2, prevFeatures.size(), CV_64F), currPts(2, currFeatures.size(), CV_64F);

        //this (x,y) combination makes sense as observed from the source code of triangulatePoints on GitHub
        for (int i = 0; i < prevFeatures.size(); i++) {
            prevPts.at<double>(0, i) = prevFeatures.at(i).x;
            prevPts.at<double>(1, i) = prevFeatures.at(i).y;

            currPts.at<double>(0, i) = currFeatures.at(i).x;
            currPts.at<double>(1, i) = currFeatures.at(i).y;
        }

        scale = get_absoluteScale(numFrame, 0, t.at<double>(2));

        if ((scale > 0.1) && (t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1))) {
            t_f = t_f + scale * (R_f * t);
            R_f = R * R_f;

        } else {
            //cout << "scale below 0.1, or incorrect translation" << endl;
        }

        // a redetection is triggered in case the number of feautres being trakced go below a particular threshold
        if (prevFeatures.size() < MIN_NUM_FEAT) {
            //cout << "Number of tracked features reduced to " << prevFeatures.size() << endl;
            //cout << "trigerring redection" << endl;
            feature_detection(prevImage, prevFeatures);
            feature_tracking(prevImage, currImage, prevFeatures, currFeatures, status);
        }

        prevImage = currImage.clone();
        prevFeatures = currFeatures;

        int x = int(t_f.at<double>(0)) + 300;
        int y = int(t_f.at<double>(2)) + 100;
        circle(traj, Point(x, y), 1, CV_RGB(255, 0, 0), 2);

        rectangle(traj, Point(10, 30), Point(550, 50), CV_RGB(0, 0, 0), CV_FILLED);
        char text[100];
        sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2));
        putText(traj, text, cv::Point(10, 50), FONT_HERSHEY_PLAIN, 1, Scalar::all(255), 1, 8);

        imshow("Road facing camera", currImage);
        imshow("Trajectory", traj);

        waitKey(1);
    }

    getchar();

    return 0;
}