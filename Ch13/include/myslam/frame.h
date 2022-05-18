#pragma once

#ifndef MYSLAM_FRAME_H
#define MYSLAM_FRAME_H

#include "myslam/common_include.h"

namespace myslam
{

    // forward declare
    struct MapPoint; //这里为什么要申明MapPoint
    struct Feature;

    /**
     * 帧
     * 每一帧分配独立id，关键帧分配关键帧ID
     */

    struct Frame
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        //无论Frame还是Feature还是MapPoint，后面都是在用智能指针类型在构建对象，所以这里有这句智能指针定义
        typedef std::shared_ptr<Frame> Ptr;

        unsigned long id_ = 0;          // 帧的ID号
        unsigned long keyframe_id_ = 0; // 关键帧的ID号
        bool is_keyframe_ = false;      // 该帧是不是关键帧

        SE3 pose_; // 该帧的位姿

        int cost_time; // 处理该帧消耗的时间，单位ms

        // 位姿的数据锁，这个位姿可能同时被多个线程读写
        // 因此读写的时候要加锁
        std::mutex pose_mutex_;

        cv::Mat left_img_, right_img_; // 双目图像，左目，右目

        // 每帧图像 左目和右目提取到的特征，用vector存储
        std::vector<std::shared_ptr<Feature>> features_left_;
        std::vector<std::shared_ptr<Feature>> features_right_;

        //构造函数
        Frame() {}

        // 返回当前帧的位姿
        // unique_lock自动上锁，自动解锁
        SE3 Pose()
        {
            //
            std::unique_lock<std::mutex> lck(pose_mutex_);
            return pose_;
        }

        // 上锁设置位姿
        void SetPose(const SE3 &pose)
        {
            std::unique_lock<std::mutex> lck(pose_mutex_);
            pose_ = pose;
        }

        /// 设置关键帧并分配并键帧id
        void SetKeyFrame();

        /// 工厂构建模式，分配id
        static std::shared_ptr<Frame> CreateFrame();
    };
}

#endif // MYSLAM_FRAME_H
