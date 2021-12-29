//
// Created by divenire on 12/27/21.
//

#ifndef MAIN_HW_07_H
#define MAIN_HW_07_H

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <chrono>
#include <sophus/se3.hpp>
#include <utility>

using namespace std;
using namespace cv;


/// 顶点
// (1)位姿顶点
class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    // 原点
    void setToOriginImpl() override
    {
        _estimate = Sophus::SE3d();
    }

    /// 定义SE上的加法
    void oplusImpl(const double *update) override
    {
        Eigen::Matrix<double, 6, 1> update_eigen;
        update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
        /// left multiplication on SE3
        _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
    }

    bool read(istream &in) override { return false; }

    bool write(ostream &out) const override { return false; }
};

// (2) 路标点 XYZ
class VertexLandmark : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    // 原点
    void setToOriginImpl() override
    {
        _estimate.fill(0);
    }

    void oplusImpl(const double *update) override
    {
        Eigen::Vector3d v(update);
        _estimate += v;
    }

    bool read(istream &in) override { return false;}

    bool write(ostream &out) const override {return false;}
};


/// g2o edge 一元边，误差维度为3
class EdgeProjectXYZRGBDPoseAndPoint : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, VertexPose,VertexLandmark>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;


    // 计算误差 测量减估计乘点
    void computeError() override
    {
        const auto *pose = dynamic_cast<const VertexPose *>(_vertices[0]);
        const auto *landmark = dynamic_cast<const VertexLandmark*>(_vertices[1]);
        const Eigen::Vector3d est = pose->estimate()*landmark->estimate();

        // 量测是第二帧看到的地图点
        // error = p' - Tp
        _error = _measurement - est;
    }

    // 误差雅可比矩阵
    void linearizeOplus() override
    {
        const auto *pose = dynamic_cast<VertexPose *>(_vertices[0]);
        const auto *landmark = dynamic_cast<const VertexLandmark*>(_vertices[1]);

        Sophus::SE3d T = pose->estimate();
        Eigen::Vector3d xyz_trans = T * landmark->estimate();
        // p187-7.44 平移在前 旋转在后

        // jacobian of T
        _jacobianOplusXi.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity();
        _jacobianOplusXi.block<3, 3>(0, 3)= Sophus::SO3d::hat(xyz_trans);

        // jacobian of p
        _jacobianOplusXj = -T.rotationMatrix();
    }

    bool read(istream &in) override { return false; }

    bool write(ostream &out) const override { return false; }

protected:
    Eigen::Vector3d _point;
};


// 寻找特征以及匹配关系
void find_feature_matches(
        const Mat &img_1, const Mat &img_2,
        std::vector<KeyPoint> &keypoints_1,
        std::vector<KeyPoint> &keypoints_2,
        std::vector<DMatch> &matches);

// 像素坐标转相机归一化坐标
Point2d pixel2cam(const Point2d &p, const Mat &K);

// 根据3D点对估计姿态和运动
void pose_estimation_3d3d(
        const vector<Point3f> &pts1,
        const vector<Point3f> &pts2,
        Mat &R, Mat &t);

// BA
void bundleAdjustment(
        vector<Eigen::Vector3d> &pts1,
        const vector<Eigen::Vector3d> &pts2,
        Eigen::Matrix3d &R,
        Eigen::Vector3d &t);

#endif //MAIN_HW_07_H
