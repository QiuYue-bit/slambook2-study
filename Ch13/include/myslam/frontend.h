#pragma once
#ifndef MYSLAM_FRONTEND_H
#define MYSLAM_FRONTEND_H

#include <opencv2/features2d.hpp>

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/map.h"
#include "myslam/camera.h"
#include "myslam/viewer.h"

namespace myslam
{
    // 定义一个枚举类
    enum class FrontendStatus
    {
        INITING,
        TRACKING_GOOD,
        TRACKING_BAD,
        LOST
    };

    class Frontend
    {

        // 外部接口函数
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Frontend> Ptr;

        // 初始化构造函数
        // 创建特征提取器，从配置文件中读取特征点数量阈值
        Frontend();

        // 对frame进行处理
        // 初始化、跟踪、或者reset
        // Frame::Ptr 是取 Frame类中的Ptr
        // 等价于 AddFrame(std::shared_ptr<Frame> frame)
        bool AddFrame(Frame::Ptr frame);

        // 设置指向地图对象的指针
        void SetMap(Map::Ptr map) { map_ = map; }

        // 设置指向后端对象的指针
        void SetBackend(std::shared_ptr<Backend> backend) { backend_ = backend; }

        // 设置指向显示线程的指针
        void SetViewer(std::shared_ptr<Viewer> viewer) { viewer_ = viewer; }

        // 返回当前追踪的状态
        FrontendStatus GetStatus() const { return status_; }

        // 设置相机的内参和外参
        // 只在初始化的时候用到
        void SetCameras(Camera::Ptr left, Camera::Ptr right)
        { //这里应该是为了设定相机参数
            camera_left_ = left;
            camera_right_ = right;
        }

    private:
        // status_是个枚举类，指示当前跟踪的情况
        FrontendStatus status_ = FrontendStatus::INITING;

        Frame::Ptr current_frame_ = nullptr; // 当前帧
        Frame::Ptr last_frame_ = nullptr;    // 上一帧

        Camera::Ptr camera_left_ = nullptr;  // 左侧相机，参数情况
        Camera::Ptr camera_right_ = nullptr; // 右侧相机，参数情况

        Map::Ptr map_ = nullptr; //指向地图类

        std::shared_ptr<Backend> backend_ = nullptr; // 指向后端对象的指针
        std::shared_ptr<Viewer> viewer_ = nullptr;   // 指向显示线程的指针

        // 当前帧与上一帧的相对运动，用于估计当前帧pose初值
        SE3 relative_motion_;

        // params
        int num_features_ = 200;             //没用上
        int num_features_init_ = 100;        // 初始化需要提取的特征点的阈值
        int num_features_tracking_ = 50;     // 跟踪情况的阈值，大于这个值认为跟踪情况良好
        int num_features_tracking_bad_ = 20; // 跟踪情况的阈值，小于num_features_tracking_大于这个值认为是bad

        int tracking_inliers_ = 0;                  // 当前帧用于估计位姿的内点数目
        int num_features_needed_for_keyframe_ = 80; // 决定关键帧的内点数目阈值，小于这个值就要创建新的关键帧

        cv::Ptr<cv::GFTTDetector> gftt_; // 特征检测器，在初始化的时候九确定了，我觉得可以设置为静态变量的

        /**
         * Track in normal mode
         * @return true if success
         */
        bool Track();

        /**
         * Reset when lost
         * @return true if success
         */
        bool Reset();

        /**
         * Track with last frame
         * @return num of tracked points
         */
        int TrackLastFrame();

        /**
         * estimate current frame's pose
         * @return num of inliers
         */
        int EstimateCurrentPose();

        /**
         * set current frame as a keyframe and insert it into backend
         * @return true if success
         */
        bool InsertKeyframe();

        /**
         * Try init the frontend with stereo images saved in current_frame_
         * @return true if success
         */
        bool StereoInit();

        /**
         * Detect features in left image in current_frame_
         * keypoints will be saved in current_frame_
         * @return
         */
        int DetectFeatures();

        /**
         * Find the corresponding features in right image of current_frame_
         * @return num of features found
         */
        int FindFeaturesInRight();

        /**
         * Build the initial map with single image
         * @return true if succeed
         */
        bool BuildInitMap();

        /**
         * Triangulate the 2D points in current frame
         * @return num of triangulated points
         */
        int TriangulateNewPoints();

        /**
         * Set the features in keyframe as new observation of the map points
         */
        void SetObservationsForKeyFrame();
    };
}

#endif
