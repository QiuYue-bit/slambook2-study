/*
 * @Author: Divenire
 * @Date: 2021-10-18 10:47:03
 * @LastEditors: Divenire
 * @LastEditTime: 2021-10-19 17:16:08
 * @Description: 3d-2d姿态估计器 
 */

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <sophus/se3.hpp>
#include <chrono>

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
    const VecVector3d &points_3d,
    const VecVector2d &points_2d,
    const Mat &K,
    Sophus::SE3d &pose);

// BA by gauss-newton
void bundleAdjustmentGaussNewton(
    const VecVector3d &points_3d,
    const VecVector2d &points_2d,
    const Mat &K,
    Sophus::SE3d &pose);

string img1_path = "../data/1.png";
string img2_path = "../data/2.png";
string depth1_path = "../data/1_depth.png";
string depth2_path = "../data/2_depth.png";

Mat R , r, t;

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
  vector<Point3f> pts_3d;
  vector<Point2f> pts_2d;

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
    pts_3d.push_back(Point3f(p1.x * dd, p1.y * dd, dd));

    // 存储与第一张图像相匹配的第二张图像中对应的2d坐标
    pts_2d.push_back(keypoints_2[m.trainIdx].pt);
  }

  //输出3d-2d匹配的点对数量
  cout << "3d-2d pairs: " << pts_3d.size() << endl;


  // ******************** OPENCV求解PNP问题 **************************
  cout << "calling OPENCV to solve pnp" << endl;

  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  // 调用OpenCV 的 PnP 求解，可选择EPNP，DLS等方法
  solvePnP(pts_3d, pts_2d, K, Mat(), r, t, false);
  // r为旋转向量形式，用Rodrigues公式转换为旋转矩阵形式
  cv::Rodrigues(r, R);
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "========solve pnp in opencv cost time: " << time_used.count() << " seconds." << endl;

  cout << "Opencv solve R= " << endl
       << R << endl;
  cout << "Opencv solve t= " << endl
       << t << endl;


  // ******************** 高斯牛顿法建模为BA求解pnp问题 **************************
  // 内存对齐？
  VecVector3d pts_3d_eigen;
  VecVector2d pts_2d_eigen;
  for (size_t i = 0; i < pts_3d.size(); ++i)
  {
    pts_3d_eigen.push_back(Eigen::Vector3d(pts_3d[i].x, pts_3d[i].y, pts_3d[i].z));
    pts_2d_eigen.push_back(Eigen::Vector2d(pts_2d[i].x, pts_2d[i].y));
  }

  cout << "calling bundle adjustment by gauss newton" << endl;
  // 使用高斯牛顿求解的SE3
  Sophus::SE3d pose_gn;
  t1 = chrono::steady_clock::now();
  bundleAdjustmentGaussNewton(pts_3d_eigen, pts_2d_eigen, K, pose_gn);
  t2 = chrono::steady_clock::now();
  time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "solve pnp by gauss newton cost time: " << time_used.count() << " seconds." << endl;

  // ******************** 使用g2o将pnp建模为图优化问题，求解BA **************************

  cout << "calling bundle adjustment by g2o" << endl;
  Sophus::SE3d pose_g2o;
  t1 = chrono::steady_clock::now();
  bundleAdjustmentG2O(pts_3d_eigen, pts_2d_eigen, K, pose_g2o);
  t2 = chrono::steady_clock::now();
  time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "solve pnp by g2o cost time: " << time_used.count() << " seconds." << endl;
  return 0;
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

  //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
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
  return Point2d(
      (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
      (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1));
}

// **********************  手写高斯牛顿法求解BA问题 **********************
void bundleAdjustmentGaussNewton(
    const VecVector3d &points_3d,
    const VecVector2d &points_2d,
    const Mat &K,
    Sophus::SE3d &pose)
{
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  // 最大迭代次数10
  const int iterations = 10;
  // 代价和上一次的代价
  double cost = 0, lastCost = 0;
  // 内参
  double fx = K.at<double>(0, 0);
  double fy = K.at<double>(1, 1);
  double cx = K.at<double>(0, 2);
  double cy = K.at<double>(1, 2);

  for (int iter = 0; iter < iterations; iter++)
  {
    Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
    Vector6d b = Vector6d::Zero();

    cost = 0;

    // compute cost
    for (int i = 0; i < points_3d.size(); i++)
    {
      // 相机坐标系下的空间点位置
      Eigen::Vector3d pc = pose * points_3d[i];
      // 逆深度，后续用于计算雅可比，提前存储，减少计算量
      double inv_z = 1.0 / pc[2];
      double inv_z2 = inv_z * inv_z;
      //相机坐标系下的空间点位置,投影到归一化平面
      Eigen::Vector2d proj(fx * pc[0] / pc[2] + cx, fy * pc[1] / pc[2] + cy);
      //观测值减去预测值
      Eigen::Vector2d e = points_2d[i] - proj;

      // 计算代价函数
      cost += e.squaredNorm();

      // 误差关于位姿的雅可比矩阵
      Eigen::Matrix<double, 2, 6> J;
      J << -fx * inv_z,
          0,
          fx * pc[0] * inv_z2,
          fx * pc[0] * pc[1] * inv_z2,
          -fx - fx * pc[0] * pc[0] * inv_z2,
          fx * pc[1] * inv_z,
          0,
          -fy * inv_z,
          fy * pc[1] * inv_z2,
          fy + fy * pc[1] * pc[1] * inv_z2,
          -fy * pc[0] * pc[1] * inv_z2,
          -fy * pc[0] * inv_z;

      // 计算Hessin 和 b
      H += J.transpose() * J;
      b += -J.transpose() * e;
    }

    // ldlt求解，得到迭代步长，李代数形式
    Vector6d dx;
    dx = H.ldlt().solve(b);

    if (isnan(dx[0]))
    {
      cout << "result is nan!" << endl;
      break;
    }

    // 代价函数增加了- -
    if (iter > 0 && cost >= lastCost)
    {
      // cost increase, update is not good
      cout << "cost: " << cost << ", last cost: " << lastCost << endl;
      break;
    }

    // update your estimation
    pose = Sophus::SE3d::exp(dx) * pose;
    lastCost = cost;

    cout << "iteration " << iter << " cost=" << std::setprecision(12) << cost << endl;
    if (dx.norm() < 1e-6)
    {
      // converge

      break;
    }
  }

  cout << "pose by g-n: \n"
       << pose.matrix() << endl;
}

// **********************  g2o求解BA问题 **********************

/// 顶点，也就是待优化的SE3变量
class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  // 原点
  virtual void setToOriginImpl() override
  {
    _estimate = Sophus::SE3d();
  }

  /// 定义SE3上的加法(更新)
  virtual void oplusImpl(const double *update) override
  {
    Eigen::Matrix<double, 6, 1> update_eigen;
    update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
    _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
  }

  virtual bool read(istream &in) override {}

  virtual bool write(ostream &out) const override {}
};

// 边，观测点减去预测点的坐标的值
class EdgeProjection : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPose>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  EdgeProjection(const Eigen::Vector3d &pos, const Eigen::Matrix3d &K) : _pos3d(pos), _K(K) {}

  virtual void computeError() override
  {
    const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
    Sophus::SE3d T = v->estimate();

    // 转换到像素坐标下
    Eigen::Vector3d pos_pixel = _K * (T * _pos3d);
    pos_pixel /= pos_pixel[2];
    // 取前两维进行计算
    _error = _measurement - pos_pixel.head<2>();
  }

  // 雅可比矩阵
  virtual void linearizeOplus() override
  {
    const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
    Sophus::SE3d T = v->estimate();
    Eigen::Vector3d pos_cam = T * _pos3d;
    double fx = _K(0, 0);
    double fy = _K(1, 1);
    double cx = _K(0, 2);
    double cy = _K(1, 2);
    double X = pos_cam[0];
    double Y = pos_cam[1];
    double Z = pos_cam[2];
    double Z2 = Z * Z;

    _jacobianOplusXi
        << -fx / Z,
        0, fx * X / Z2, fx * X * Y / Z2, -fx - fx * X * X / Z2, fx * Y / Z,
        0, -fy / Z, fy * Y / (Z * Z), fy + fy * Y * Y / Z2, -fy * X * Y / Z2, -fy * X / Z;
  }

  virtual bool read(istream &in) override {}

  virtual bool write(ostream &out) const override {}

private:
  Eigen::Vector3d _pos3d;
  Eigen::Matrix3d _K;
};



void bundleAdjustmentG2O(
    const VecVector3d &points_3d,
    const VecVector2d &points_2d,
    const Mat &K,
    Sophus::SE3d &pose)
{

  // 构建图优化，先设定g2o
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType;           // pose is 6, landmark is 3
  typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType; // 线性求解器类型

  // 梯度下降方法，可以从GN, LM, DogLeg 中选
  auto solver = new g2o::OptimizationAlgorithmGaussNewton(
      g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
  g2o::SparseOptimizer optimizer; // 图模型
  optimizer.setAlgorithm(solver); // 设置求解器
  optimizer.setVerbose(true);     // 打开调试输出

  // 往图中添加顶点，因为只有一个待优化变量T，只有一个顶点
  VertexPose *vertex_pose = new VertexPose(); // camera vertex_pose
  vertex_pose->setId(0);

  // 设置初始值为I
  Eigen::Matrix3d Rotation ;
  Eigen::Vector3d translation;
  for(int i=0;i<3;++i)
  {
      for(int j=0;j<3;++j)
      {
          Rotation(i,j)=R.at<double>(i,j);
      }
      translation(i)=R.at<double>(3,i);
  }
//  cout<<R.at<double>(0,1);
    cout<<"init T";
    Sophus::SE3d T(Rotation,translation);
    cout<<T.rotationMatrix()<<endl;
  vertex_pose->setEstimate(T);

  optimizer.addVertex(vertex_pose);

  // 相机内参矩阵
  Eigen::Matrix3d K_eigen;
  K_eigen << K.at<double>(0, 0), K.at<double>(0, 1), K.at<double>(0, 2),
      K.at<double>(1, 0), K.at<double>(1, 1), K.at<double>(1, 2),
      K.at<double>(2, 0), K.at<double>(2, 1), K.at<double>(2, 2);

  // 往图中添加误差项(边)
  int index = 1;
  for (size_t i = 0; i < points_2d.size(); ++i)
  {
    auto p2d = points_2d[i];
    auto p3d = points_3d[i];
    EdgeProjection *edge = new EdgeProjection(p3d, K_eigen);
    edge->setId(index);
    edge->setVertex(0, vertex_pose);
    // 添加测量
    edge->setMeasurement(p2d);
    // 测量的信息矩阵
    edge->setInformation(Eigen::Matrix2d::Identity());
    optimizer.addEdge(edge);
    index++;
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
  cout << "pose estimated by g2o =\n"
       << vertex_pose->estimate().matrix() << endl;

  // 输出优化值
  pose = vertex_pose->estimate();
  
}
