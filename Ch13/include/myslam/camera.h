#pragma once

#ifndef MYSLAM_CAMERA_H
#define MYSLAM_CAMERA_H

#include "myslam/common_include.h"

namespace myslam
{

    class Camera
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Camera> Ptr;

        // 相机内参
        double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0,
               baseline_ = 0;

        // 外参
        // 相对于左目的外参
        // T_CxC0
        SE3 pose_;     // extrinsic
        SE3 pose_inv_; // inverse of extrinsics

        Camera();

        Camera(double fx, double fy, double cx, double cy, double baseline,
               const SE3 &pose)
            : fx_(fx), fy_(fy), cx_(cx), cy_(cy), baseline_(baseline), pose_(pose)
        {
            pose_inv_ = pose_.inverse();
        }

        // 获取相机的外参
        SE3 pose() const { return pose_; }

        // 获取相机的内参
        Mat33 K() const
        {
            Mat33 k;
            k << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;
            return k;
        }

        // *世界系下的三维点 P_w
        // *虚拟相机坐标系 C_0   左目相机坐标系C_1 左目相机坐标系C_2
        // *左目和右目相对于虚拟相机坐标系得外参为 pose_

        // 将世界系下的三维点 P_w 投影到相机坐标系 C_x
        Vec3 world2camera(const Vec3 &p_w, const SE3 &T_c_w);

        // 将世界系下的三维点 P_w 投影相机像素平面上 u v
        Vec2 world2pixel(const Vec3 &p_w, const SE3 &T_c_w);

        // 相机坐标系下观测到得三维点 P_Cx 变换到世界系下 P_w
        Vec3 camera2world(const Vec3 &p_c, const SE3 &T_c_w);

        // 相机坐标系下观测到得三维点 P_Cx
        // 投影到相机像素平面下，得到像素坐标 u,v
        Vec2 camera2pixel(const Vec3 &p_c);

        // 像素坐标 u,v 反投影回 相机下的归一化平面上depth 默认为1
        // 或者投影回任意得深度下
        Vec3 pixel2camera(const Vec2 &p_p, double depth = 1);

        // 像素uv根据外参，投影回三维点坐标 深度为1或者人为指定
        Vec3 pixel2world(const Vec2 &p_p, const SE3 &T_c_w, double depth = 1);
    };
}

#endif