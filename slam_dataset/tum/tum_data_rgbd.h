#ifndef TUM_DATASET_RGBD_H
#define TUM_DATASET_RGBD_H

#include <iostream>
#include <fstream>

#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace cg {

    struct CamIntrinsic {
        double cx;
        double cy;
        double fx;
        double fy;
    };

    class TUMDataRGBD { // rgbd_dataset_freiburg1_xyz

    public:
        TUMDataRGBD() : data_dir_("./") {
            open_file();
            set_cam_k(0);
        }

        TUMDataRGBD(std::string data_dir, int ntype=0) : data_dir_(data_dir) {
            open_file();
            set_cam_k(ntype);
        }

        ~TUMDataRGBD() {
            close_file();
        }

    public:
        bool get_rgb_depth(cv::Mat &img_color, cv::Mat &img_depth) {

            std::string rgb_time, rgb_file, depth_time, depth_file;
            file_associate_ >> rgb_time >> rgb_file >> depth_time >> depth_file;

            std::cout << rgb_file << "\n" << depth_file << std::endl;

            img_color = cv::imread(data_dir_ + "/" + rgb_file, CV_LOAD_IMAGE_COLOR);
            if(img_color.empty()) {
                std::cerr << "imread img_color failed!" << std::endl;
                return false;
            }
            img_depth = cv::imread(data_dir_ + "/" + depth_file, CV_LOAD_IMAGE_UNCHANGED);
            if(img_depth.empty()) {
                std::cerr << "imread img_depth failed!" << std::endl;
                return false;
            }

            return true;
        }

        bool get_rgb_depth_pose(cv::Mat &img_color, cv::Mat &img_depth, Eigen::Isometry3d &pose) {

            std::string rgb_time, rgb_file, depth_time, depth_file, pose_time;
            file_associate_ >> rgb_time >> rgb_file >> depth_time >> depth_file;

            std::cout << rgb_file << "\n" << depth_file << std::endl;

            img_color = cv::imread(data_dir_ + "/" + rgb_file, CV_LOAD_IMAGE_COLOR);
            if(img_color.empty()) {
                std::cerr << "imread img_color failed!" << std::endl;
                return false;
            }
            img_depth = cv::imread(data_dir_ + "/" + depth_file, CV_LOAD_IMAGE_UNCHANGED);
            if(img_depth.empty()) {
                std::cerr << "imread img_depth failed!" << std::endl;
                return false;
            }

            file_associate_ >> pose_time;
            double data[7] = {0};
            for (int i = 0; i < 7; i++) {
                file_associate_ >> data[i];
            }
            Eigen::Quaterniond q(data[6], data[3], data[4], data[5]);
            pose = Eigen::Isometry3d(q);
            pose.pretranslate(Eigen::Vector3d(data[0], data[1], data[2]));

            return true;
        }

        bool get_rgb(cv::Mat &img_color) {
            std::string rgb_file, time_rgb;
            file_rgb_ >> time_rgb >> rgb_file;
            img_color = cv::imread(data_dir_ + "/" + rgb_file, CV_LOAD_IMAGE_COLOR);
            if(img_color.empty()) {
                std::cerr << "imread img_color failed!" << std::endl;
                return false;
            }
            return true;
        }

        bool get_depth(cv::Mat &img_depth) {
            std::string depth_file, time_depth;
            file_depth_ >> time_depth >> depth_file;
            img_depth = cv::imread(data_dir_ + "/" + depth_file, CV_LOAD_IMAGE_UNCHANGED);
            if(img_depth.empty()) {
                std::cerr << "imread img_depth failed!" << std::endl;
                return false;
            }
            return true;
        }

        bool getK(Eigen::Matrix3f &K) {
            K <<    cam_k_.fx, 0.f, cam_k_.cx,
                    0.f, cam_k_.fy, cam_k_.cy,
                    0.f, 0.f, 1.0f;
            return true;
        }

        bool getK(cv::Mat &K) {
            K = (cv::Mat_<double> ( 3,3 ) <<
                                         cam_k_.fx, 0.f, cam_k_.cx,
                                         0.f, cam_k_.fy, cam_k_.cy,
                                         0.f, 0.f, 1.0f);
        }

    private:
        void open_file() {
            file_associate_.open(data_dir_ + "/" + "associate.txt");
            if (!file_associate_.is_open()) {
                std::cerr << "open file failed!" << std::endl;
                return;
            }
            file_rgb_.open(data_dir_ + "/" + "rgb.txt");
            if (!file_rgb_.is_open()) {
                std::cerr << "open file failed!" << std::endl;
                return;
            }
            std::string rgb_line;
            std::getline(file_rgb_,rgb_line);
            while(rgb_line.find('#')!=std::string::npos) {
                std::getline(file_rgb_, rgb_line);
            }
            file_depth_.open(data_dir_ + "/" + "depth.txt");
            if (!file_depth_.is_open()) {
                std::cerr << "open file failed!" << std::endl;
                return;
            }
            std::string depth_line;
            std::getline(file_depth_,depth_line);
            while(depth_line.find('#')!=std::string::npos) {
                std::getline(file_depth_, depth_line);
            }
        }

        void close_file() {
            if (file_associate_.is_open())
                file_associate_.close();
            if (file_rgb_.is_open())
                file_rgb_.close();
            if (file_depth_.is_open())
                file_depth_.close();
        }

        void set_cam_k(int ntype) {
            switch(ntype) {
                case 0: // Freiburg 1 RGB
                    cam_k_.cx = 318.6;
                    cam_k_.cy = 255.3;
                    cam_k_.fx = 517.3;
                    cam_k_.fy = 516.5;
                    break;
                case 1: // Freiburg 2 RGB
                    cam_k_.cx = 325.1;
                    cam_k_.cy = 249.7;
                    cam_k_.fx = 520.9;
                    cam_k_.fy = 521.0;
                    break;
            }
        }

    public:
        CamIntrinsic cam_k_;
        const float depth_scale_ = 5000.f;

    private:
        std::string data_dir_;
        std::ifstream file_associate_;
        std::ifstream file_rgb_;
        std::ifstream file_depth_;
    };
}

#endif //TUM_DATASET_RGBD_H
