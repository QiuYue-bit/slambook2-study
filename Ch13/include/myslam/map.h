/***
 * @Author       : Yue
 * @Date         : 2022-05-12 15:50:57
 * @LastEditTime : 2022-05-18 09:39:32
 * @LastEditors  : Yue
 * @Description  :
 * @FilePath     : /Ch13/include/myslam/map.h
 * @佛祖保佑 BUG FREE
 */
#pragma once
#ifndef MAP_H
#define MAP_H

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/mappoint.h"

namespace myslam
{

    /**
     * @brief 地图
     * 和地图的交互：前端调用InsertKeyframe和InsertMapPoint插入新帧和地图点，后端维护地图的结构，判定outlier/剔除等等
     */
    class Map
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Map> Ptr;

        // 无序容器，将地图点和地图点的ID关联起来，方便查找
        // Key - value 结构
        typedef std::unordered_map<unsigned long, MapPoint::Ptr> LandmarksType;

                // 无序容器，将关键帧和关键帧的ID关联起来，方便查找
        typedef std::unordered_map<unsigned long, Frame::Ptr> KeyframesType;

        Map() {}

        /// 增加一个关键帧
        void InsertKeyFrame(Frame::Ptr frame);

        /// 增加一个地图顶点
        void InsertMapPoint(MapPoint::Ptr map_point);

        /// 获取所有地图点
        LandmarksType GetAllMapPoints()
        {
            std::unique_lock<std::mutex> lck(data_mutex_); //防止和InsertKeyFrame,CleanMap,RemoveOldKeyframe等函数冲突
            return landmarks_;
        }

        /// 获取所有关键帧
        KeyframesType GetAllKeyFrames()
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return keyframes_;
        }

        /// 获取激活地图点
        LandmarksType GetActiveMapPoints()
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return active_landmarks_;
        }

        /// 获取激活关键帧
        KeyframesType GetActiveKeyFrames()
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return active_keyframes_;
        }

        /// 清理map中观测数量为零的点
        void CleanMap();

    private:
        // 在滑窗中移除旧的关键帧
        void RemoveOldKeyframe();

        std::mutex data_mutex_;

        LandmarksType landmarks_;        // all landmarks

        LandmarksType active_landmarks_; // active landmarks

        KeyframesType keyframes_;        // all key-frames
        
        KeyframesType active_keyframes_; // all key-frames


        Frame::Ptr current_frame_ = nullptr;

        // settings
        int num_active_keyframes_ = 7; // 激活的关键帧数量,这里的激活就是一个窗口概念
    };

}

#endif