//
// Created by cg on 4/27/19.
//

#ifndef VO1_TRAJECTORY_H
#define VO1_TRAJECTORY_H

#include <vector>
#include <fstream>
#include <iostream>
#include <unistd.h>

#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <pangolin/pangolin.h>

namespace cg {
    void draw_trajectory(vector <Sophus::SE3d> posesA, vector <Sophus::SE3d> posesB, Sophus::SE3d se3AB) {
        if (posesA.empty()) {
            cerr << "Trajectory is empty!" << endl;
            return;
        }

        // create pangolin window and plot the trajectory
        pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
                pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
        );

        pangolin::View &d_cam = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
                .SetHandler(new pangolin::Handler3D(s_cam));


        while (pangolin::ShouldQuit() == false) {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            d_cam.Activate(s_cam);
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

            glLineWidth(2);
            for (size_t i = 0; i < posesA.size() - 1; i++) {
                glColor3f(1.0f, 0.0f, 0.0f);
                glBegin(GL_LINES);
                auto p1 = posesA[i], p2 = posesA[i + 1];
                glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
                glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
                glEnd();
            }

            for (size_t i = 0; i < posesB.size() - 1; i++) {
                glColor3f(0.0f, 0.0f, 1.0f);
                glBegin(GL_LINES);
                auto p1 = se3AB * posesB[i], p2 = se3AB * posesB[i + 1];
                glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
                glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
                glEnd();
            }

            pangolin::FinishFrame();
            usleep(5000);   // sleep 5 ms
        }
    }
}

#endif //VO1_TRAJECTORY_H
