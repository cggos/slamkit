#include "pose_estimation.h"

#include <iostream>

Point2d pixel2cam ( const Point2d& p, const Mat& K )
{
    return Point2d
            (
                    ( p.x - K.at<double> ( 0,2 ) ) / K.at<double> ( 0,0 ),
                    ( p.y - K.at<double> ( 1,2 ) ) / K.at<double> ( 1,1 )
            );
}

void find_feature_matches (
        const Mat& img_1, const Mat& img_2,
        std::vector<KeyPoint>& keypoints_1,
        std::vector<KeyPoint>& keypoints_2,
        std::vector< DMatch >& matches )
{

    Ptr<FeatureDetector> detector = ORB::create();
    detector->detect ( img_1, keypoints_1 );
    detector->detect ( img_2, keypoints_2 );

    Mat descriptors_1, descriptors_2;
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );

    std::vector<DMatch> match;
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create ( "BruteForce-Hamming" );
    matcher->match ( descriptors_1, descriptors_2, match );

    double min_dist=10000, max_dist=0;

    for ( int i = 0; i < descriptors_1.rows; i++ ) {
        double dist = match[i].distance;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;
    }

    printf ( "-- Max dist : %f \n", max_dist );
    printf ( "-- Min dist : %f \n", min_dist );

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for ( int i = 0; i < descriptors_1.rows; i++ ) {
        if (match[i].distance <= max(2 * min_dist, 30.0))
            matches.push_back(match[i]);
    }
}

void triangulation (
        const vector< KeyPoint >& keypoint_1,
        const vector< KeyPoint >& keypoint_2,
        const std::vector< DMatch >& matches,
        const Mat& R, const Mat& t,
        const Mat& K,
        vector< Point3d >& points )
{
    Mat T1 = (Mat_<float> (3,4) <<
                                1,0,0,0,
                                0,1,0,0,
                                0,0,1,0);
    Mat T2 = (Mat_<float> (3,4) <<
                                R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0,0),
                                R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1,0),
                                R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2,0)
    );

    vector<Point2f> pts_1, pts_2;
    for ( DMatch m:matches ) {
        pts_1.push_back(pixel2cam(keypoint_1[m.queryIdx].pt, K));
        pts_2.push_back(pixel2cam(keypoint_2[m.trainIdx].pt, K));
    }

    Mat pts_4d;
    cv::triangulatePoints( T1, T2, pts_1, pts_2, pts_4d );

    for ( int i=0; i<pts_4d.cols; i++ ) {
        Mat x = pts_4d.col(i);
        x /= x.at<float>(3, 0);
        Point3d p(
                x.at<float>(0, 0),
                x.at<float>(1, 0),
                x.at<float>(2, 0)
        );
        points.push_back(p);
    }
}

bool pose_estimation_direct(
        const vector<Measurement>& measurements, cv::Mat* gray, Eigen::Matrix3f& K, Eigen::Isometry3d& Tcw) {

    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 1> > Block;  // 求解的向量是6＊1的
    Block::LinearSolverType *linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
    Block *solver_ptr = new Block(std::unique_ptr<Block::LinearSolverType>(linearSolver));

    // g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr ); // G-N
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<Block>(solver_ptr)); // L-M

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    g2o::VertexSE3Expmap *pose = new g2o::VertexSE3Expmap();
    pose->setEstimate(g2o::SE3Quat(Tcw.rotation(), Tcw.translation()));
    pose->setId(0);
    optimizer.addVertex(pose);

    int id = 1;
    for (Measurement m: measurements) {
        EdgeSE3ProjectDirect *edge = new EdgeSE3ProjectDirect(m.pos_world, K(0, 0), K(1, 1), K(0, 2), K(1, 2), gray);
        edge->setVertex(0, pose);
        edge->setMeasurement(m.grayscale);
        edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
        edge->setId(id++);
        optimizer.addEdge(edge);
    }
    cout << "edges in graph: " << optimizer.edges().size() << endl;
    optimizer.initializeOptimization();
    optimizer.optimize(30);
    Tcw = pose->estimate();
}

void pose_estimation_2d2d (
        std::vector<KeyPoint> keypoints_1,
        std::vector<KeyPoint> keypoints_2,
        std::vector< DMatch > matches,
        const Mat& K,
        Mat& R, Mat& t )
{
    vector<Point2f> pts1;
    vector<Point2f> pts2;
    for ( int i = 0; i < ( int ) matches.size(); i++ ) {
        pts1.push_back(keypoints_1[matches[i].queryIdx].pt);
        pts2.push_back(keypoints_2[matches[i].trainIdx].pt);
    }

    Mat matF = findFundamentalMat(pts1, pts2, CV_FM_8POINT);
    cout << "fundamental_matrix is " << endl << matF << endl;

    double cx = K.at<double>(0,2);
    double cy = K.at<double>(1,2);
    double fx = K.at<double>(0,0);
    Mat matE = findEssentialMat(pts1, pts2, fx, Point2d(cx,cy));
    cout << "essential_matrix is " << endl << matE << endl;

    Mat matH = findHomography(pts1, pts2, RANSAC, 3);
    cout << "homography_matrix is " << endl << matH << endl;

    recoverPose(matE, pts1, pts2, R, t, fx, Point2d(cx,cy));
    cout << "R is " << endl << R << endl;
    cout << "t is " << endl << t << endl;
}

void pose_estimation_3d3d (
        const std::vector<cv::Point3f> &pts1,
        const std::vector<cv::Point3f> &pts2,
        Eigen::Matrix3d& R, Eigen::Vector3d& t) {

    cv::Point3f p1, p2;     // center of mass
    int N = pts1.size();
    for (int i = 0; i < N; i++) {
        p1 += pts1[i];
        p2 += pts2[i];
    }
    p1 /= N;
    p2 /= N;

    std::vector<cv::Point3f> q1(N), q2(N); // remove the center
    for (int i = 0; i < N; i++) {
        q1[i] = pts1[i] - p1;
        q2[i] = pts2[i] - p2;
    }

    // compute q1*q2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (int i = 0; i < N; i++) {
        Eigen::Vector3d v3q1 = Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z);
        Eigen::Vector3d v3q2 = Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z);
        W += v3q1 * v3q2.transpose();
    }
    cout << "W=\n" << W << endl;

    double determinantW = W(0,0)*W(1,1)*W(2,2) + W(0,1)*W(1,2)*W(2,0) + W(0,2)*W(1,0)*W(2,1) -
                          (W(0,0)*W(1,2)*W(2,1) + W(0,1)*W(1,0)*W(2,2) + W(0,2)*W(1,1)*W(2,0));
    assert(determinantW>1e-8);

    // SVD on W
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    cout << "U=\n" << U << endl;
    cout << "V=\n" << V << endl;

    R = U * (V.transpose());
    t = Eigen::Vector3d(p1.x, p1.y, p1.z) - R * Eigen::Vector3d(p2.x, p2.y, p2.z);
}

void bundle_adjustment_3d2d (
        const vector< Point3f > points_3d,
        const vector< Point2f > points_2d,
        const Mat& K,
        Mat& R, Mat& t ) {
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3> > Block;  // pose 维度为 6, landmark 维度为 3
    Block::LinearSolverType *linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>(); // g2o::LinearSolverCSparse ???
    Block *solver_ptr = new Block(std::unique_ptr<Block::LinearSolverType>(linearSolver));

    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<Block>(solver_ptr));

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    // vertex
    g2o::VertexSE3Expmap *pose = new g2o::VertexSE3Expmap(); // camera pose
    Eigen::Matrix3d R_mat;
    R_mat <<
            R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
            R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
            R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2);
    pose->setId(0);
    pose->setEstimate(g2o::SE3Quat(
            R_mat,
            Eigen::Vector3d(t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0))
    ));
    optimizer.addVertex(pose);

    int index = 1;
    for (const Point3f p:points_3d)   // landmarks
    {
        g2o::VertexSBAPointXYZ *point = new g2o::VertexSBAPointXYZ();
        point->setId(index++);
        point->setEstimate(Eigen::Vector3d(p.x, p.y, p.z));
        point->setMarginalized(true); // g2o 中必须设置 marg
        optimizer.addVertex(point);
    }

    // parameter: camera intrinsics
    g2o::CameraParameters *camera =
            new g2o::CameraParameters(K.at<double>(0, 0), Eigen::Vector2d(K.at<double>(0, 2), K.at<double>(1, 2)), 0);
    camera->setId(0);
    optimizer.addParameter(camera);

    // edges
    index = 1;
    for (const Point2f p:points_2d) {
        g2o::EdgeProjectXYZ2UV *edge = new g2o::EdgeProjectXYZ2UV();
        edge->setId(index);
        edge->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ *> ( optimizer.vertex(index)));
        edge->setVertex(1, pose);
        edge->setMeasurement(Eigen::Vector2d(p.x, p.y));
        edge->setParameterId(0, 0);
        edge->setInformation(Eigen::Matrix2d::Identity());
        optimizer.addEdge(edge);
        index++;
    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(100);

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "optimization costs time: " << time_used.count() << " seconds." << endl;

    cout << endl << "after optimization:" << endl;
    cout << "T=" << endl << Eigen::Isometry3d(pose->estimate()).matrix() << endl;
}

void bundle_adjustment_3d3d (
        const vector< Point3f >& pts1,
        const vector< Point3f >& pts2,
        Mat& R, Mat& t )
{
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // pose维度为 6, landmark 维度为 3
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<Block::PoseMatrixType>(); // 线性方程求解器
    Block* solver_ptr = new Block( std::unique_ptr<Block::LinearSolverType>(linearSolver) );      // 矩阵块求解器

    g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( std::unique_ptr<Block>(solver_ptr) );

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm( solver );

    // vertex
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
    pose->setId(0);
    pose->setEstimate(g2o::SE3Quat(Eigen::Matrix3d::Identity(), Eigen::Vector3d( 0,0,0 )));
    optimizer.addVertex( pose );

    // edges
    int index = 1;
    vector<EdgeProjectXYZRGBDPoseOnly*> edges;
    for ( size_t i=0; i<pts1.size(); i++ ) {
        EdgeProjectXYZRGBDPoseOnly *edge =
                new EdgeProjectXYZRGBDPoseOnly(Eigen::Vector3d(pts2[i].x, pts2[i].y, pts2[i].z));
        edge->setId(index);
        edge->setVertex(0, dynamic_cast<g2o::VertexSE3Expmap *> (pose));
        edge->setMeasurement(Eigen::Vector3d(pts1[i].x, pts1[i].y, pts1[i].z));
        edge->setInformation(Eigen::Matrix3d::Identity() * 1e4);
        optimizer.addEdge(edge);
        index++;
        edges.push_back(edge);
    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

    optimizer.setVerbose( true );
    optimizer.initializeOptimization();
    optimizer.optimize(10);

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
    cout<<"optimization costs time: "<<time_used.count()<<" seconds."<<endl;

    cout<<endl<<"after optimization:"<<endl;
    cout<<"T="<<endl<<Eigen::Isometry3d( pose->estimate() ).matrix()<<endl;

}