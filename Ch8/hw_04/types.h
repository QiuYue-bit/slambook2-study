//
// Created by divenire on 12/31/21.
//

#ifndef MAIN_TYPES_H
#define MAIN_TYPES_H

#include <iostream>
#include <fstream>
#include <list>
#include <utility>
#include <vector>
#include <chrono>
#include <ctime>
#include <climits>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <sophus/se3.hpp>

using namespace std;
using namespace g2o;
using namespace Eigen;


class VertexSE3 : public g2o::BaseVertex<6, Sophus::SE3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    // 原点
    void setToOriginImpl() override
    {
        _estimate = Sophus::SE3d();
    }

    /// 定义SE3上的加法(更新)
    void oplusImpl(const double *update) override
    {
        Eigen::Matrix<double, 6, 1> update_eigen;
        update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
        // 左乘模型
        _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
        updateCache();
    }

    bool read(istream &in) override {}

    bool write(ostream &out) const override {}
};

/// Edge 一元边，一端连接SE3，误差维度为1
class EdgeProjection : public g2o::BaseUnaryEdge<1, double, VertexSE3>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeProjection(Eigen::Vector3d point, double fx, double fy, double cx, double cy, cv::Mat* image)
    :p_world_(std::move(point)),cx_(cx),cy_(cy),fx_(fx),fy_(fy),image_(image)
    {}

    void computeError() override
    {
        // 根据当前的位姿及相机内参，将帧1的地图点投影到当前帧
        const auto v = dynamic_cast<VertexSE3*>(_vertices[0]);
        Eigen::Vector3d pointLocal = v->estimate() * p_world_;
        // 投影到像素平面上
        double x = pointLocal[0]/pointLocal[2] * fx_ +cx_;
        double y = pointLocal[1]/pointLocal[2] * fy_ +cy_;
        // check x,y is in the image
        // 光度误差计算
        if ( x-4<0 || ( x+4 ) >image_->cols || ( y-4 ) <0 || ( y+4 ) >image_->rows )
        {
            _error ( 0,0 ) = 0.0;
            this->setLevel ( 1 );
        }
        else
        {
            _error ( 0,0 ) = GetPixelValue ( x,y ) - _measurement;
        }
    }

    // 雅可比矩阵
    void linearizeOplus() override
    {
        // 判断该边是否需要优化
        if(level() == 1)
        {
            //  投影点不在图像范围内，误差为0，直接雅克比矩阵设置未0 就好了
            // 雅克比是1*6的向量
            _jacobianOplusXi = Eigen::Matrix<double,1,6>::Zero();
            return;
        }
        else
        {
            // 书上P220
            const auto SE3 = dynamic_cast<VertexSE3*>(_vertices[0]);
            Eigen::Vector3d PointLocal = SE3->estimate()*p_world_;

            // 数据准备
            double x = PointLocal[0];
            double y = PointLocal[1];
            double invz = 1.0 / PointLocal[2];
            double invz_2 = invz *invz;
            double u = x*fx_*invz + cx_;
            double v = y*fy_*invz + cy_;

            // 像素关于位姿的梯度
            Eigen::Matrix<double,2,6> jacobian_uv_ksai;
            jacobian_uv_ksai ( 0,3 ) = - x*y*invz_2 *fx_;
            jacobian_uv_ksai ( 0,4 ) = ( 1+ ( x*x*invz_2 ) ) *fx_;
            jacobian_uv_ksai ( 0,5 ) = - y*invz *fx_;
            jacobian_uv_ksai ( 0,0 ) = invz *fx_;
            jacobian_uv_ksai ( 0,1 ) = 0;
            jacobian_uv_ksai ( 0,2 ) = -x*invz_2 *fx_;

            jacobian_uv_ksai ( 1,3 ) = - ( 1+y*y*invz_2 ) *fy_;
            jacobian_uv_ksai ( 1,4 ) = x*y*invz_2 *fy_;
            jacobian_uv_ksai ( 1,5 ) = x*invz *fy_;
            jacobian_uv_ksai ( 1,0 ) = 0;
            jacobian_uv_ksai ( 1,1 ) = invz *fy_;
            jacobian_uv_ksai ( 1,2 ) = -y*invz_2 *fy_;

            //  像素梯度
            Eigen::Matrix<double, 1, 2> jacobian_pixel_uv;
            jacobian_pixel_uv ( 0,0 ) = ( GetPixelValue( u+1,v )-GetPixelValue ( u-1,v ) ) *0.5;
            jacobian_pixel_uv ( 0,1 ) = ( GetPixelValue ( u,v+1 )-GetPixelValue ( u,v-1 ) ) *0.5;

            // 误差相对于李代数的雅克比矩阵
            _jacobianOplusXi = jacobian_pixel_uv*jacobian_uv_ksai;

        }
    }

    bool read(istream &in) override {}

    bool write(ostream &out) const override {}

protected:
    inline double GetPixelValue(double x ,double y)
    {
        uchar * data = & image_->data[ int ( y ) * image_->step + int ( x ) ];
        // 取x 和 y 的小数部分
        double xx = x - floor ( x );
        double yy = y - floor ( y );
        // 插值
        return double (
                ( 1-xx ) * ( 1-yy ) * data[0] +
                xx* ( 1-yy ) * data[1] +
                ( 1-xx ) *yy*data[ image_->step ] +
                xx*yy*data[image_->step+1]
        );
    }

private:
    Eigen::Vector3d p_world_;   // 3D point in world frame
    double cx_=0, cy_=0, fx_=0, fy_=0; // Camera intrinsics
    cv::Mat* image_=nullptr;    // reference image
};

#endif //MAIN_TYPES_H
