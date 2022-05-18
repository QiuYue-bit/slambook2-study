/***
 * @Author       : Yue
 * @Date         : 2022-05-12 15:50:57
 * @LastEditTime : 2022-05-17 22:12:06
 * @LastEditors  : Yue
 * @Description  :
 * @FilePath     : /Ch13/include/myslam/backend.h
 * @佛祖保佑 BUG FREE
 */

#ifndef MYSLAM_BACKEND_H
#define MYSLAM_BACKEND_H

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/map.h"
#include "myslam/camera.h"

namespace myslam
{

    class Backend
    {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Backend> Ptr;

        /// 构造函数中启动优化线程并挂起
        Backend();

        // 设置左右相机的内参和外参
        void SetCameras(Camera::Ptr left, Camera::Ptr right)
        {
            cam_left_ = left;
            cam_right_ = right;
        }

        /// 设置地图
        void SetMap(std::shared_ptr<Map> map) { map_ = map; }

        /// 触发地图更新，启动优化
        void UpdateMap();

        /// 关闭后端线程
        void Stop();

    private:
        /// 后端线程
        void BackendLoop();

        /// 对给定关键帧和路标点进行优化
        void Optimize(Map::KeyframesType &keyframes, Map::LandmarksType &landmarks);

        // 指向地图类的指针
        std::shared_ptr<Map> map_;

        // 后端优化线程绑定
        std::thread backend_thread_;

        // 锁
        std::mutex data_mutex_;

        // map_update_ 条件对象
        // 如果地图中插入了新的关键帧
        // 就让后端运行一波
        std::condition_variable map_update_;

        // 原子变量 不用上锁
        std::atomic<bool> backend_running_;

        // 左右目相机的参数
        Camera::Ptr cam_left_ = nullptr, cam_right_ = nullptr;
    };
}

#endif // MYSLAM_BACKEND_H
