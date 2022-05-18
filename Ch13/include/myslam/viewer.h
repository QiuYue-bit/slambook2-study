/*** 
 * @Author       : Yue
 * @Date         : 2022-05-12 15:50:57
 * @LastEditTime : 2022-05-18 14:42:44
 * @LastEditors  : Yue
 * @Description  : 
 * @FilePath     : /Ch13/include/myslam/viewer.h
 * @佛祖保佑 BUG FREE
 */
#ifndef MYSLAM_VIEWER_H
#define MYSLAM_VIEWER_H

#include <thread>
#include <pangolin/pangolin.h>

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/map.h"


//接下来进入可视化环节，通过Pangolin将视觉里程计中的帧,点和行进路径可视化


namespace myslam{

    class Viewer
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Viewer> Ptr;

        // Viewer 构造函数，同时开启显示线程
        Viewer();

        // 显示线程是用来显示地图的
        // 这儿就设置一下地图类的指针，后续会用到
        void SetMap(Map::Ptr map) { map_ = map; } 

        // 设置标志位，退出线程的while循环
        void Close();

        // 把当前帧设为显示线程中最新的帧
        void AddCurrentFrame(Frame::Ptr current_frame);

        // 更新地图
        void UpdateMap();


    private:

        // 
        void ThreadLoop();

        // 画出关键帧
        void DrawFrame(Frame::Ptr frame, const float* color);  

        //画出地图点
        void DrawMapPoints();

        // 设置pangolin的视角跟随当前帧
        void FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera);//视角跟随当前帧

        // 将特征点绘制在图像上
        cv::Mat PlotFrameImage();

        // 当前帧的指针
        Frame::Ptr current_frame_ = nullptr;

        // 指向地图的指针，用于绘制关键帧和地图点
        Map::Ptr map_ = nullptr;

        // 显示线程
        std::thread viewer_thread_;
        bool viewer_running_ = true;

        // 
        std::unordered_map<unsigned long, Frame::Ptr> active_keyframes_;
        std::unordered_map<unsigned long, MapPoint::Ptr> active_landmarks_;

        // 显示线程要
        std::mutex viewer_data_mutex_;
    };

}

#endif
