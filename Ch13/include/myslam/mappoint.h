/***
 * @Author       : Yue
 * @Date         : 2022-05-12 15:50:57
 * @LastEditTime : 2022-05-18 09:39:58
 * @LastEditors  : Yue
 * @Description  :
 * @FilePath     : /Ch13/include/myslam/mappoint.h
 * @佛祖保佑 BUG FREE
 */
#pragma once
#ifndef MYSLAM_MAPPOINT_H
#define MYSLAM_MAPPOINT_H

#include "myslam/common_include.h"
#include "myslam/feature.h"

namespace myslam
{

    /**
     * 路标点类
     * 特征点在三角化之后形成路标点
     */
    struct MapPoint
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<MapPoint> Ptr;

        // 地图点的ID号
        unsigned long id_ = 0; // ID

        // 没用上这个
        bool is_outlier_ = false;

        // 地图点的坐标
        Vec3 pos_ = Vec3::Zero();

        std::mutex data_mutex_;

        // 地图点被观测到的次数
        // 也就是与之相关的特征点数量
        int observed_times_ = 0;

        // List结构,用来获取地图点观测所对应的特征
        // List 在任何位置插入和删除都很快
        //
        std::list<std::weak_ptr<Feature>> observations_;

        MapPoint() {}

        MapPoint(long id, Vec3 position);

        // 返回地图点的坐标
        // ? 没必要啊你这之前的pos_都是Public的直接访问就可以
        Vec3 Pos()
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return pos_;
        }

        // 设置地图点的坐标
        void SetPos(const Vec3 &pos)
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            pos_ = pos;
        };

        // 建立地图点的观测
        void AddObservation(std::shared_ptr<Feature> feature)
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            observations_.push_back(feature);
            observed_times_++;
        }

        // 移除地图点的观测
        void RemoveObservation(std::shared_ptr<Feature> feat);

        // 返回某个地图点的所有观测
        std::list<std::weak_ptr<Feature>> GetObs()
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return observations_;
        }

        // factory function
        static MapPoint::Ptr CreateNewMappoint();
    };
}

#endif // MYSLAM_MAPPOINT_H