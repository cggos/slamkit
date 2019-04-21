#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <unistd.h>

using namespace std;

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;

#include <sophus/se3.h>
#include <pangolin/pangolin.h>

typedef vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;

string strFileCompare = "../icp_compare.txt";

void GetCenterOfMass3D(const std::vector<Vector3d> &vvPts, Vector3d &center);
void ICP_SVD(const std::vector<Vector3d> &ptsA, const std::vector<Vector3d> &ptsB, Sophus::SE3 &se3AfromB);
void DrawTrajectory(vector<Sophus::SE3> , vector<Sophus::SE3>, Sophus::SE3);

int main()
{
    ifstream fileCompare(strFileCompare);
    if(!fileCompare.is_open()){
        std::cout << "icp_compare.txt open FAILED!" << std::endl;
       return -1; 
    }
    Eigen::Vector3d v3T;
    Eigen::Quaterniond q;
    Sophus::SE3 se3;
    std::vector<Vector3d> vv3Te;
    std::vector<Vector3d> vv3Tg;
    std::vector<Sophus::SE3> vse3_e;
    std::vector<Sophus::SE3> vse3_g;
    double time_e, tx_e, ty_e, tz_e, qx_e, qy_e, qz_e, qw_e;
    double time_g, tx_g, ty_g, tz_g, qx_g, qy_g, qz_g, qw_g;
    while(!fileCompare.eof()){
        if(fileCompare.fail())
            break;
        fileCompare >> time_e >> tx_e >> ty_e >> tz_e >> qx_e >> qy_e >> qz_e >> qw_e
                    >> time_g >> tx_g >> ty_g >> tz_g >> qx_g >> qy_g >> qz_g >> qw_g;
        
        v3T = Eigen::Vector3d(tx_e,ty_e,tz_e);  
        q   = Eigen::Quaterniond(qw_e, qx_e, qy_e, qz_e);
        
        vv3Te.push_back(v3T);
        se3.translation() = v3T;
        se3.so3() = Sophus::SO3(q);
        vse3_e.push_back(se3);
        
        v3T = Eigen::Vector3d(tx_g,ty_g,tz_g);  
        q   = Eigen::Quaterniond(qw_g, qx_g, qy_g, qz_g);
        
        vv3Tg.push_back(v3T);
        se3.translation() = v3T;
        se3.so3() = Sophus::SO3(q);
        vse3_g.push_back(se3);
    }

    Sophus::SE3 se3_ge;
    ICP_SVD(vv3Tg, vv3Te, se3_ge);
    
    DrawTrajectory(vse3_g, vse3_e, se3_ge);

    return 0;
}

void GetCenterOfMass3D(const std::vector<Vector3d> &vvPts, Vector3d &center)
{
	auto nSize = static_cast<unsigned int>(vvPts.size());
	if(nSize < 1)
	    return;
	for(unsigned int i=0; i<nSize; i++)
	{
	    center += vvPts[i];
	}
	center /= nSize;
}

void ICP_SVD(const std::vector<Vector3d> &ptsA, const std::vector<Vector3d> &ptsB, Sophus::SE3 &se3AfromB)
{
    auto nSizeA = static_cast<unsigned int>(ptsA.size());
    auto nSizeB = static_cast<unsigned int>(ptsB.size());

    if(nSizeA==0 || nSizeA != nSizeB)
        return;

    // center of mass
    Vector3d ptCenterA = Vector3d::Zero();
    Vector3d ptCenterB = Vector3d::Zero();
    GetCenterOfMass3D(ptsA, ptCenterA);
    GetCenterOfMass3D(ptsB, ptCenterB);

    //compute W
    Matrix3d W = Matrix3d::Zero();
    for(unsigned int i=0; i<nSizeA; i++)
    {
        W += (ptsA[i]-ptCenterA) * (ptsB[i]-ptCenterB).transpose();
    }

    double determinantW = W(0,0)*W(1,1)*W(2,2) + W(0,1)*W(1,2)*W(2,0) + W(0,2)*W(1,0)*W(2,1) -
	               (W(0,0)*W(1,2)*W(2,1) + W(0,1)*W(1,0)*W(2,2) + W(0,2)*W(1,1)*W(2,0));
    assert(determinantW>1e-8);

    //svd decomposition
    JacobiSVD<MatrixXd> svd(W, ComputeThinU | ComputeThinV);
    Matrix3d m3U = svd.matrixU();
    Matrix3d m3V = svd.matrixV();

    Matrix3d m3R = m3U * m3V.transpose();
    Vector3d v3T = ptCenterA-m3R*ptCenterB;

    //compose SE3
    se3AfromB = Sophus::SE3(m3R, v3T);
}

/*******************************************************************************************/
void DrawTrajectory(vector<Sophus::SE3> posesA, vector<Sophus::SE3> posesB, Sophus::SE3 se3AB) {
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
