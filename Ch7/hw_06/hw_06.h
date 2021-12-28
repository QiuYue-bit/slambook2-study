//
// Created by divenire on 12/27/21.
//

#ifndef MAIN_HW_06_H
#define MAIN_HW_06_H

#include <Eigen/Core>
#include <sophus/se3.hpp>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <chrono>

using namespace std;

/// 顶点
// (1) 路标点 XYZ
class VertexLandmark : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    // 原点
    virtual void setToOriginImpl() override
    {
        _estimate.fill(0);
    }

    /// 定义SE3上的加法(更新)
    virtual void oplusImpl(const double *update) override
    {
        Eigen::Vector3d v(update);
        _estimate += v;
    }

    virtual bool read(istream &in) override {}

    virtual bool write(ostream &out) const override {}
};

// (2) 相机位姿
class VertexSE3 : public g2o::BaseVertex<6, Sophus::SE3d>
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
        // 左乘模型
        _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
        updateCache();
    }

    virtual bool read(istream &in) override {}

    virtual bool write(ostream &out) const override {}
};

/// Edge 二元边，一端连接Landmark 一端连接位姿SE3
class EdgeProjection : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, VertexLandmark, VertexSE3>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeProjection()
    {
        _cam = 0;
        resizeParameters(1);
        installParameter(_cam, 0);
    }

    virtual void computeError() override
    {
        const VertexLandmark *vi = dynamic_cast<const VertexLandmark *>(_vertices[0]);
        const VertexSE3 *vj = dynamic_cast<const VertexSE3 *>(_vertices[1]);
        const g2o::CameraParameters *cam = dynamic_cast<const g2o::CameraParameters *>(parameter(0));
        _error = measurement() - cam->cam_map(vj->estimate() * (vi->estimate()));
    }

    // 雅可比矩阵
    virtual void linearizeOplus() override
    {
        const VertexLandmark *vi = dynamic_cast<const VertexLandmark *>(_vertices[0]);
        const VertexSE3 *vj = dynamic_cast<const VertexSE3 *>(_vertices[1]);
        const g2o::CameraParameters *cam = dynamic_cast<const g2o::CameraParameters *>(parameter(0));
        // 第二帧观测到的地图点
        Eigen::Vector3d xyz = vi->estimate();
        Eigen::Vector3d xyz_trans = vj->estimate() * xyz;
        number_t x = xyz_trans[0];
        number_t y = xyz_trans[1];
        number_t z = xyz_trans[2];
        number_t z_2 = z * z;

        // part of Jacobian
        // 这儿认为fx和fy是相同的
        Eigen::Matrix<number_t, 2, 3> tmp;
        tmp(0, 0) = cam->focal_length;
        tmp(0, 1) = 0;
        tmp(0, 2) = -x / z * cam->focal_length;

        tmp(1, 0) = 0;
        tmp(1, 1) = cam->focal_length;
        tmp(1, 2) = -y / z * cam->focal_length;

        _jacobianOplusXi = -1. / z * tmp * vj->estimate().rotationMatrix();

        // 误差关于位姿的导数 SE3的定义是平移在前 旋转在后
        _jacobianOplusXj(0, 0) = -1. / z * cam->focal_length;
        _jacobianOplusXj(0, 1) = 0;
        _jacobianOplusXj(0, 2) = x / z_2 * cam->focal_length;
        _jacobianOplusXj(0, 3) = x * y / z_2 * cam->focal_length;
        _jacobianOplusXj(0, 4) = -(1 + (x * x / z_2)) * cam->focal_length;
        _jacobianOplusXj(0, 5) = y / z * cam->focal_length;

        _jacobianOplusXj(1, 0) = 0;
        _jacobianOplusXj(1, 1) = -1. / z * cam->focal_length;
        _jacobianOplusXj(1, 2) = y / z_2 * cam->focal_length;
        _jacobianOplusXj(1, 3) = (1 + y * y / z_2) * cam->focal_length;
        _jacobianOplusXj(1, 4) = -x * y / z_2 * cam->focal_length;
        _jacobianOplusXj(1, 5) = -x / z * cam->focal_length;
    }

    virtual bool read(istream &in) override {}

    virtual bool write(ostream &out) const override {}

private:
    g2o::CameraParameters *_cam;
};

#endif //MAIN_HW_06_H
