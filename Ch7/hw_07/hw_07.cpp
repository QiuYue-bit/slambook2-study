/*
 * @Author: Divenire
 * @Date: 2021-10-18 10:47:07
 * @LastEditors: Divenire
 * @LastEditTime: 2021-10-19 22:13:29
 * @Description:
 *
 */


#include "hw_07.h"

string img1_path = "../data/1.png"; /* NOLINT */
string img2_path = "../data/2.png";/* NOLINT */
string depth1_path = "../data/1_depth.png";/* NOLINT */
string depth2_path = "../data/2_depth.png";/* NOLINT */

int main(int argc, char **argv)
{

    //-- 读取图像
    Mat img_1 = imread(img1_path);
    Mat img_2 = imread(img2_path);

    // 寻找特征点及匹配关系
    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
    cout << "一共找到了" << matches.size() << "组匹配点" << endl;

    // 建立3D点
    Mat depth1 = imread(depth1_path); // 深度图为16位无符号数，单通道图像
    Mat depth2 = imread(depth2_path); // 深度图为16位无符号数，单通道图像
    Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    vector<Eigen::Vector3d> pts1, pts2;

    // 对于每一对匹配
    for (DMatch m : matches)
    {
        // 提取特征点二点深度信息
        ushort d1 = depth1.ptr<unsigned short>(int(keypoints_1[m.queryIdx].pt.y))[int(keypoints_1[m.queryIdx].pt.x)];
        ushort d2 = depth2.ptr<unsigned short>(int(keypoints_2[m.trainIdx].pt.y))[int(keypoints_2[m.trainIdx].pt.x)];

        // bad depth
        if (d1 == 0 || d2 == 0)
            continue;

        // 像素坐标转换到相机归一化坐标
        Point2d p1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
        Point2d p2 = pixel2cam(keypoints_2[m.trainIdx].pt, K);

        // 图像深度转换
        double dd1 = (d1) / 5000.0;
        double dd2 = (d2) / 5000.0;

        // 保存三维点对的匹配关系
        pts1.emplace_back(p1.x * dd1, p1.y * dd1, dd1);
        pts2.emplace_back(p2.x * dd2, p2.y * dd2, dd2);
    }

    // 输出匹配好的3维坐标点对
    cout << "3d-3d pairs: " << pts1.size() << endl;


    Eigen::Matrix3d R_u;
    Eigen::Vector3d t_u;
    cout << "================================calling bundle adjustment" << endl;
    // BA求解3D点对匹配问题
    auto t1 = chrono::steady_clock::now();
    bundleAdjustment(pts1, pts2, R_u, t_u);
    auto t2 = chrono::steady_clock::now();
    auto time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "bundleAdjustment costs time: " << time_used.count() << " seconds." << endl;

    // verify p2 = R * p1 + t
    // 采取5个点验证求取的R，t是否正确
    Sophus::SE3d T(R_u,t_u);
    double error;
    for (int i = 0; i < pts1.size(); i++)
    {
        Eigen::Vector3d temp;
        Eigen::Vector3d p1(pts1[i]);
        Eigen::Vector3d p2(pts2[i]);
        // world p project to current p T*p1
        //
        temp= T*p1 - p2;
//        cout<<"temp"<<temp<<endl;
        error += temp.dot(temp);
    }
    cout<<"full ba error"<<error;
}



// ************************ Bundle Adjustment优化 ************************

void bundleAdjustment(
        vector<Eigen::Vector3d> &pts1,
        const vector<Eigen::Vector3d> &pts2,
        Eigen::Matrix3d &R,
        Eigen::Vector3d &t)
{
    // 构建图优化，先设定g2o
    typedef g2o::BlockSolverX BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType; // 线性求解器类型
    // 梯度下降方法，可以从GN, LM, DogLeg 中选
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer; // 图模型
    optimizer.setAlgorithm(solver); // 设置求解器
    optimizer.setVerbose(true);     // 打开调试输出


    // 一个顶点，待优化的变量为姿态
    auto *pose = new VertexPose(); // camera pose
    pose->setId(0);
    pose->setEstimate(Sophus::SE3d());
    optimizer.addVertex(pose);

    // 添加地图点
    for (int i = 0;i<pts1.size();++i) {
        auto *landmark = new VertexLandmark();
        landmark->setId(i+1);
        landmark->setMarginalized(true);
        landmark->setEstimate(pts1[i]);
        optimizer.addVertex(landmark);
    }

    // 添加误差边
    vector<EdgeProjectXYZRGBDPoseAndPoint*> edges;
    for (int i = 0;i<pts1.size();++i)
    {
        auto *edge = new EdgeProjectXYZRGBDPoseAndPoint();

        edge->setVertex(0, dynamic_cast<VertexPose*>(optimizer.vertex(0)));
        edge->setVertex(1,dynamic_cast<VertexLandmark*>(optimizer.vertex(1+i)));
        // 第二幅图像的点作为两侧
        edge->setMeasurement(pts2[i]);
        // 设置信息矩阵
        edge->setInformation(Eigen::Matrix3d::Identity());
        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        edge->setRobustKernel(rk);
        optimizer.addEdge(edge);
        edges.push_back(edge);
    }

    optimizer.initializeOptimization();
    // 最大迭代次数设置为10
    optimizer.optimize(10);


    cout << endl
         << "after optimization:" << endl;
    cout << "T=\n"
         << pose->estimate().matrix() << endl;

    // convert to cv::Mat
    R = pose->estimate().rotationMatrix();
    t = pose->estimate().translation();

    for (int i = 0; i < pts1.size(); ++i)
    {
        pts1[i] = dynamic_cast<VertexLandmark*>(optimizer.vertex(1+i))->estimate();
    }

//    for ( auto e:edges )
//    {
//        e->computeError();
//        cout<<"e->error"<<e->error();
//        cout<<"error = "<<e->chi2()<<endl;
//    }

}


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

Point2d pixel2cam(const Point2d &p, const Mat &K)
{
    return {
            (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
            (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)};
}

void pose_estimation_3d3d(const vector<Point3f> &pts1,
                          const vector<Point3f> &pts2,
                          Mat &R, Mat &t)
{
    Point3f p1, p2; // center of mass
    auto N = (int)pts1.size();

    //计算质心
    for (int i = 0; i < N; i++)
    {
        p1 += pts1[i];
        p2 += pts2[i];
    }
    p1 = Point3f(p1 / N);
    p2 = Point3f(p2 / N);

    // 计算每个点去质心后的坐标
    vector<Point3f> q1(N), q2(N); // remove the center
    for (int i = 0; i < N; i++)
    {
        q1[i] = pts1[i] - p1;
        q2[i] = pts2[i] - p2;
    }

    // compute q1*q2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (int i = 0; i < N; i++)
    {
        W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
    }
    cout << "W=" << W << endl;

    // SVD on W
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Eigen::Matrix3d& U = svd.matrixU();
    const Eigen::Matrix3d& V = svd.matrixV();

    cout << "U=" << U << endl;
    cout << "V=" << V << endl;

    // 根据W的SVD分解结果，计算出旋转矩阵R
    Eigen::Matrix3d R_ = U * (V.transpose());
    if (R_.determinant() < 0)
    {
        R_ = -R_;
    }
    Eigen::Vector3d t_ = Eigen::Vector3d(p1.x, p1.y, p1.z) - R_ * Eigen::Vector3d(p2.x, p2.y, p2.z);

    // convert to cv::Mat
    R = (Mat_<double>(3, 3) << R_(0, 0), R_(0, 1), R_(0, 2),
            R_(1, 0), R_(1, 1), R_(1, 2),
            R_(2, 0), R_(2, 1), R_(2, 2));
    t = (Mat_<double>(3, 1) << t_(0, 0), t_(1, 0), t_(2, 0));
}


