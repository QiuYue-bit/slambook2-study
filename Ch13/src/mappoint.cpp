/***
 * @Author       : Yue
 * @Date         : 2022-05-12 15:50:57
 * @LastEditTime : 2022-05-15 19:38:52
 * @LastEditors  : Yue
 * @Description  :
 * @FilePath     : /Ch13/src/mappoint.cpp
 * @佛祖保佑 BUG FREE
 */
///针对mappoint.h中涉及的各类函数进行定义实现

#include "myslam/mappoint.h"
#include "myslam/feature.h"

namespace myslam
{

    MapPoint::MapPoint(long id, Vec3 position) : id_(id), pos_(position) {}

    MapPoint::Ptr MapPoint::CreateNewMappoint()
    {
        static long factory_id = 0;
        MapPoint::Ptr new_mappoint(new MapPoint);
        new_mappoint->id_ = factory_id++;
        return new_mappoint;
    }

    void MapPoint::RemoveObservation(std::shared_ptr<Feature> feat)
    {
        std::unique_lock<std::mutex> lck(data_mutex_);

        // 一个地图点可能有多个观测
        // 换句话说,一个地图点对应了很多个特征点feat1,feat2,feat3
        // 现在要在观测关系中移除这个地图点与feat的关系
        for (auto iter = observations_.begin(); iter != observations_.end();
             iter++)
        {
            if (iter->lock() == feat)
            {
                // observations_是个Lists 能够比较方便的删除元素
                // 地图点从特征观测中删除iter指向的特征
                // todo 写个程序或者看看这个erase的效果
                observations_.erase(iter);

                // iter此时和feat指向的是同一个特征对象，对地图点而言，这个特征不再是它的观测，
                //  那么对特征而言，这个地图点也不再是它对应的地图点，也需要对feature对象删除map_point_
                feat->map_point_.reset();

                // 这个地图点被观测到的次数--
                observed_times_--;

                // 删完了就退出去
                break;
            }
        }
    }

}
