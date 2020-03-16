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
            open_associate_file();
            set_cam_k();
        }

        TUMDataRGBD(std::string data_dir) : data_dir_(data_dir) {
            open_associate_file();
            set_cam_k();
        }

        ~TUMDataRGBD() {
            close_associate_file();
        }

    public:
        bool get_rgb_depth_pose(cv::Mat &img_color, cv::Mat &img_depth, Eigen::Isometry3d &pose) {

            std::string rgb_time, rgb_file, depth_time, depth_file, pose_time;
            file_associate_ >> rgb_time >> rgb_file >> depth_time >> depth_file;

            std::cout << rgb_file << "\n" << depth_file << std::endl;

            img_color = cv::imread(data_dir_ + "/" + rgb_file);
            if(img_color.empty()) {
                std::cerr << "imread img_color failed!" << std::endl;
                return false;
            }
            img_depth = cv::imread(data_dir_ + "/" + depth_file);
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

    private:
        void open_associate_file() {
            file_associate_.open(data_dir_ + "/" + "associate_with_groundtruth.txt");
            if (!file_associate_.is_open()) {
                std::cerr << "open file failed!" << std::endl;
                return;
            }
        }

        void close_associate_file() {
            if (file_associate_.is_open())
                file_associate_.close();
        }

        void set_cam_k() {
            cam_k_.cx = 318.6;
            cam_k_.cy = 255.3;
            cam_k_.fx = 517.3;
            cam_k_.fy = 516.5;
        }

    public:
        CamIntrinsic cam_k_;
        const double depth_scale_ = 5000.0;

    private:
        std::string data_dir_;
        std::ifstream file_associate_;
    };
}

#endif //TUM_DATASET_RGBD_H
