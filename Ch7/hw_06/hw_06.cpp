/*
 * @Author: Divenire
 * @Date: 2021-10-18 10:47:03
 * @LastEditors: Divenire
 * @LastEditTime: 2021-12-27 21:34:14
 * @Description:  书后习题6对应程序，将第一个相机的观测路标设置未优化变量纳入优化。
 */

#include "hw_06.h"

using namespace std;
using namespace cv;

void find_feature_matches(
    const Mat &img_1, const Mat &img_2,
    std::vector<KeyPoint> &keypoints_1,
    std::vector<KeyPoint> &keypoints_2,
    std::vector<DMatch> &matches);

// 像素坐标转相机归一化坐标
Point2d pixel2cam(const Point2d &p, const Mat &K);

// BA by g2o
typedef vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;
typedef vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVector3d;

void bundleAdjustmentG2O(
    const vector<Point3d> &points1_3d,
    const vector<Point2d> &points1_2d,
    const vector<Point2d> &points2_2d,
    const Mat &K,
    g2o::SE3Quat &pose);

void bundleAdjustmentSelfG2O(
    const vector<Point3d> &points1_3d,
    const vector<Point2d> &points1_2d,
    const vector<Point2d> &points2_2d,
    const Mat &K,
    Sophus::SE3d &pose);

string img1_path = "../data/1.png";
string img2_path = "../data/2.png";
string depth1_path = "../data/1_depth.png";
string depth2_path = "../data/2_depth.png";

int main(int argc, char **argv)
{
    //-- 读取图像
    Mat img_1 = imread(img1_path);
    Mat img_2 = imread(img2_path);

    assert(img_1.data && img_2.data && "Can not load images!");

    //-- 特征点提取与匹配
    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
    cout << "一共找到了" << matches.size() << "组匹配点" << endl;

    // 读取第一幅图像的深度信息
    Mat d1 = imread(depth1_path); // 深度图为16位无符号数，单通道图像

    // 相机的内参矩阵
    Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

    // 匹配好的3D-2D点对
    vector<Point3d> pts_3d;
    vector<Point2d> pts_2d;

    // match point in frame 1
    vector<Point2d> pts1_2d;

    for (DMatch m : matches)
    {
        //提取第一张图像的匹配点对的深度
        ushort d = d1.ptr<unsigned short>(int(keypoints_1[m.queryIdx].pt.y))[int(keypoints_1[m.queryIdx].pt.x)];
        if (d == 0) // bad depth
            continue;
        // 单位转换
        float dd = d / 5000.0;
        // Step1:将第一张图像匹配好的特征点坐标转换到归一化平面上
        Point2d p1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
        // Steps2:根据深度信息转换为空间点的3维坐标 图片1的坐标系下(也可以理解为世界坐标系下的空间点坐标)
        pts_3d.push_back(Point3d(p1.x * dd, p1.y * dd, dd));

        // 存储与第一张图像相匹配的第二张图像中对应的2d坐标
        pts_2d.push_back(keypoints_2[m.trainIdx].pt);
        // store keypoint coordinate in frame 1
        pts1_2d.push_back(keypoints_1[m.queryIdx].pt);
    }

    //输出3d-2d匹配的点对数量
    cout << "3d-2d pairs: " << pts_3d.size() << endl;

    // ******************** 使用g2o将pnp建模为图优化问题，使用内置的类型求解BA **************************

    cout << "calling bundle adjustment by g2o" << endl;
    g2o::SE3Quat pose_g2o;

    auto t1 = chrono::steady_clock::now();
    bundleAdjustmentG2O(pts_3d, pts1_2d, pts_2d, K, pose_g2o);
    auto t2 = chrono::steady_clock::now();
    auto time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);

    cout << "solve pnp by g2o cost time: " << time_used.count() << " seconds." << endl;

    // ******************** 使用g2o将pnp建模为图优化问题，使用自定义的类型求解BA **************************

    cout << "calling bundle adjustment by selfg2o" << endl;
    Sophus::SE3d pose_selfg2o;

    t1 = chrono::steady_clock::now();
    bundleAdjustmentSelfG2O(pts_3d, pts1_2d, pts_2d, K, pose_selfg2o);
    t2 = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);

    cout << "solve pnp by selfg2o cost time: " << time_used.count() << " seconds." << endl;

    //

    return 0;
}

void bundleAdjustmentSelfG2O(
    const vector<Point3d> &points1_3d,
    const vector<Point2d> &points1_2d,
    const vector<Point2d> &points2_2d,
    const Mat &K,
    Sophus::SE3d &pose)
{
    // 构建图优化，先设定g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType;           // pose is 6, landmark is 3
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType; // 线性求解器类型

    // 梯度下降方法，可以从GN, LM, DogLeg 中选
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer; // 图模型
    optimizer.setAlgorithm(solver); // 设置求解器
    optimizer.setVerbose(true);     // 打开调试输出

    // 往图中添加顶点
    // （1）添加位姿节点 第一帧作为世界坐标系（不优化） 同时也是相机坐标系
    int PoseVertexNUm = 2; // only 2 frame
    int curPoseIndx = 0;
    for (; curPoseIndx < PoseVertexNUm; curPoseIndx++)
    {
        auto pose = new VertexSE3(); //SE3
        if (curPoseIndx == 0)
            pose->setFixed(true); //fix first  frame
        pose->setId(curPoseIndx);
        pose->setEstimate(Sophus::SE3d()); //origin
        optimizer.addVertex(pose);
    }

    // （2）添加路标节点
    for (unsigned int i = 0; i < points1_3d.size(); ++i)
    {
        auto Point = new VertexLandmark();
        Point->setId(i + 2);
        Point->setMarginalized(true);
        Point->setEstimate(Eigen::Vector3d(points1_3d[i].x, points1_3d[i].y, points1_3d[i].z));
        optimizer.addVertex(Point);
    }

    // param 相机内参 这儿认为fx = fy
    auto *camera = new g2o::CameraParameters(
        K.at<double>(0, 0), Eigen::Vector2d(K.at<double>(0, 2), K.at<double>(1, 2)), 0);
    camera->setId(0);
    optimizer.addParameter(camera);

    // 加入边edge
    // (1) frame 1 landmark to 2d
    for (unsigned int i = 0; i < points1_2d.size(); ++i)
    {
        auto edge = new EdgeProjection();
        // binary edge
        edge->setVertex(0, dynamic_cast<VertexLandmark *>(optimizer.vertex(2 + i))); //landmark in frame 1
        edge->setVertex(1, dynamic_cast<VertexSE3 *>(optimizer.vertex(0)));          // frame 1
        edge->setMeasurement(Eigen::Vector2d(points1_2d[i].x, points1_2d[i].y));
        edge->setParameterId(0, 0);
        edge->setInformation(Eigen::Matrix2d::Identity());
        optimizer.addEdge(edge);
    }

    // (2) landmark to 2d in frame2
    for (unsigned int i = 0; i < points2_2d.size(); ++i)
    {
        auto edge = new EdgeProjection();
        // binary edge
        edge->setVertex(0, dynamic_cast<VertexLandmark *>(optimizer.vertex(2 + i))); //landmark in frame 1
        edge->setVertex(1, dynamic_cast<VertexSE3 *>(optimizer.vertex(1)));          // frame 1
        edge->setMeasurement(Eigen::Vector2d(points2_2d[i].x, points2_2d[i].y));
        edge->setParameterId(0, 0);
        edge->setInformation(Eigen::Matrix2d::Identity());
        optimizer.addEdge(edge);
    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

    // 打开调试输出
    optimizer.setVerbose(true);
    // 初始化优化
    optimizer.initializeOptimization();
    // 最大迭代次数10
    optimizer.optimize(10);

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);

    cout << "optimization costs time: " << time_used.count() << " seconds." << endl;
    // 输出优化值
    pose = dynamic_cast<VertexSE3 *>(optimizer.vertex(1))->estimate();
    cout << "pose estimated by g2o =\n"
         << pose.matrix() << endl;
}

// **********************  g2o求解BA问题 使用自带的顶点和边类型**********************
void bundleAdjustmentG2O(
    const vector<Point3d> &points1_3d,
    const vector<Point2d> &points1_2d,
    const vector<Point2d> &points2_2d,
    const Mat &K,
    g2o::SE3Quat &pose)
{
    // 构建图优化，先设定g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType;           // pose is 6, landmark is 3
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType; // 线性求解器类型

    // 梯度下降方法，可以从GN, LM, DogLeg 中选
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer; // 图模型
    optimizer.setAlgorithm(solver); // 设置求解器
    optimizer.setVerbose(true);     // 打开调试输出

    // 往图中添加顶点
    // （1）添加位姿节点 第一帧作为世界坐标系（不优化） 同时也是相机坐标系
    int PoseVertexNUm = 2; // only 2 frame
    int curPoseIndx = 0;
    for (; curPoseIndx < PoseVertexNUm; curPoseIndx++)
    {
        auto v_pose = new g2o::VertexSE3Expmap(); //SE3
        if (curPoseIndx == 0)
            v_pose->setFixed(true); //fix first  frame
        v_pose->setId(curPoseIndx);
        v_pose->setEstimate(g2o::SE3Quat()); //origin
        optimizer.addVertex(v_pose);
    }

    // （2）添加路标节点
    for (unsigned int i = 0; i < points1_3d.size(); ++i)
    {
        auto Point = new g2o::VertexSBAPointXYZ();
        Point->setId(i + 2);
        Point->setMarginalized(true);
        Point->setEstimate(Eigen::Vector3d(points1_3d[i].x, points1_3d[i].y, points1_3d[i].z));
        optimizer.addVertex(Point);
    }

    // param
    g2o::CameraParameters *camera = new g2o::CameraParameters(
        K.at<double>(0, 0), Eigen::Vector2d(K.at<double>(0, 2), K.at<double>(1, 2)), 0);
    camera->setId(0);
    optimizer.addParameter(camera);

    // 加入边edge
    // (1) frame 1 landmark to 2d
    for (unsigned int i = 0; i < points1_2d.size(); ++i)
    {
        auto edge = new g2o::EdgeProjectXYZ2UV();
        // binary edge
        edge->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(2 + i))); //landmark in frame 1
        edge->setVertex(1, dynamic_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(0)));       // frame 1
        edge->setMeasurement(Eigen::Vector2d(points1_2d[i].x, points1_2d[i].y));
        edge->setParameterId(0, 0);
        edge->setInformation(Eigen::Matrix2d::Identity());
        optimizer.addEdge(edge);
    }

    // (2) landmark to 2d in frame2
    for (unsigned int i = 0; i < points2_2d.size(); ++i)
    {
        auto edge = new g2o::EdgeProjectXYZ2UV();
        // binary edge
        edge->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(2 + i))); //landmark in frame 1
        edge->setVertex(1, dynamic_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(1)));       // frame 1
        edge->setMeasurement(Eigen::Vector2d(points2_2d[i].x, points2_2d[i].y));
        edge->setParameterId(0, 0);
        edge->setInformation(Eigen::Matrix2d::Identity());
        optimizer.addEdge(edge);
    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

    // 打开调试输出
    optimizer.setVerbose(true);
    // 初始化优化
    optimizer.initializeOptimization();
    // 最大迭代次数10
    optimizer.optimize(10);

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);

    cout << "optimization costs time: " << time_used.count() << " seconds." << endl;
    pose = dynamic_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(1))->estimate();
    cout << "pose estimated by g2o =\n"
         << pose << endl;
}

// **********************  寻找特征点以及匹配关系 **********************
void find_feature_matches(const Mat &img_1, const Mat &img_2,
                          std::vector<KeyPoint> &keypoints_1,
                          std::vector<KeyPoint> &keypoints_2,
                          std::vector<DMatch> &matches)
{
    //-- 初始化
    Mat descriptors_1, descriptors_2;
    // used in OpenCV3
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect(img_1, keypoints_1);
    detector->detect(img_2, keypoints_2);

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute(img_1, keypoints_1, descriptors_1);
    descriptor->compute(img_2, keypoints_2, descriptors_2);

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
    vector<DMatch> match;
    matcher->match(descriptors_1, descriptors_2, match);

    //-- 第四步:匹配点对筛选
    double min_dist = 10000, max_dist = 0;

    //找出所有匹配之间的最小距离和最大距离, 即最相似的和最不相似的两组点之间的距离
    for (int i = 0; i < descriptors_1.rows; i++)
    {
        double dist = match[i].distance;
        if (dist < min_dist)
            min_dist = dist;
        if (dist > max_dist)
            max_dist = dist;
    }

    printf("-- Max dist : %f \n", max_dist);
    printf("-- Min dist : %f \n", min_dist);

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for (int i = 0; i < descriptors_1.rows; i++)
    {
        if (match[i].distance <= max(2 * min_dist, 30.0))
        {
            matches.push_back(match[i]);
        }
    }
}

// **********************  像素坐标转换到归一化平面上 **********************
Point2d pixel2cam(const Point2d &p, const Mat &K)
{
    return {
        (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
        (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)};
}