/*** 
 * @Author       : Yue
 * @Date         : 2022-05-12 15:50:57
 * @LastEditTime : 2022-05-17 19:35:13
 * @LastEditors  : Yue
 * @Description  : 
 * @FilePath     : /Ch13/include/myslam/visual_odometry.h
 * @佛祖保佑 BUG FREE
 */
#pragma once
#ifndef MYSLAM_VISUAL_ODOMETRY_H
#define MYSLAM_VISUAL_ODOMETRY_H

#include "myslam/backend.h"
#include "myslam/common_include.h"
#include "myslam/dataset.h"
#include "myslam/frontend.h"
#include "myslam/viewer.h"

namespace myslam
{

    class VisualOdometry
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<VisualOdometry> Ptr;

        /// constructor with config file
        VisualOdometry(std::string &config_path);

        //初始化
        bool Init();

        //在特定数据集上启动VO
        void Run();

        //在数据集图像序列上步进
        bool Step();

        //获取前端当前的跟踪情况
        // 好、坏、跟丢
        FrontendStatus GetFrontendStatus() const { return frontend_->GetStatus(); }

        // 图片的缩放系数
        static double img_resize_;

    private:
        bool inited_ = false;
        std::string config_file_path_;

        // 前段、后端、地图类、可视化器对象的指针
        Frontend::Ptr frontend_ = nullptr;
        Backend::Ptr backend_ = nullptr;
        Map::Ptr map_ = nullptr;
        Viewer::Ptr viewer_ = nullptr;



        // dataset
        Dataset::Ptr dataset_ = nullptr;
    };
}

#endif // MYSLAM_VISUAL_ODOMETRY_H
