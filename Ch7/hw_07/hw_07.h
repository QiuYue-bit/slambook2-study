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
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <chrono>
#include <sophus/se3.hpp>

using namespace std;
using namespace cv;


/// vertex and edges used in g2o ba
class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    // 原点
    virtual void setToOriginImpl() override
    {
        _estimate = Sophus::SE3d();
    }

    /// 定义SE上的加法
    virtual void oplusImpl(const double *update) override
    {
        Eigen::Matrix<double, 6, 1> update_eigen;
        update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
        _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
    }

    virtual bool read(istream &in) override { return 0; }

    virtual bool write(ostream &out) const override { return 0; }
};

/// g2o edge 一元边，误差维度为3
class EdgeProjectXYZRGBDPoseOnly : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexPose>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeProjectXYZRGBDPoseOnly(const Eigen::Vector3d &point) : _point(point) {}

    // 计算误差 测量减估计乘点
    virtual void computeError() override
    {
        const VertexPose *pose = static_cast<const VertexPose *>(_vertices[0]);
        _error = _measurement - pose->estimate() * _point;
    }

    // 误差雅可比矩阵
    virtual void linearizeOplus() override
    {
        VertexPose *pose = static_cast<VertexPose *>(_vertices[0]);
        Sophus::SE3d T = pose->estimate();
        Eigen::Vector3d xyz_trans = T * _point;
        _jacobianOplusXi.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity();
        _jacobianOplusXi.block<3, 3>(0, 3) = Sophus::SO3d::hat(xyz_trans);
    }

    bool read(istream &in) { return 0; }

    bool write(ostream &out) const { return 0; }

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
        const vector<Eigen::Vector3d> &pts1,
        const vector<Eigen::Vector3d> &pts2,
        Eigen::Matrix3d &R,
        Eigen::Vector3d &t);

#endif //MAIN_HW_07_H
