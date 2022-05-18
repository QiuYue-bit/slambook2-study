/***
 * @Author       : Yue
 * @Date         : 2022-05-12 15:50:57
 * @LastEditTime : 2022-05-18 09:36:20
 * @LastEditors  : Yue
 * @Description  :
 * @FilePath     : /Ch13/include/myslam/dataset.h
 * @佛祖保佑 BUG FREE
 */
#ifndef MYSLAM_DATASET_H
#define MYSLAM_DATASET_H
#include "myslam/camera.h"
#include "myslam/common_include.h"
#include "myslam/frame.h"

///数据集操作类

namespace myslam
{

    class Dataset
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW; //内存位数对齐，反正加上就对了吧

        typedef std::shared_ptr<Dataset> Ptr;

        //构造函数,对dataset_path_赋值
        Dataset(const std::string &dataset_path);

        // 初始化，返回是否成功
        // 根据KITTI的calib文件，读取相机的内参和外参
        bool Init();

        // 读取图像，对图像进行缩放，同时创建帧类
        Frame::Ptr NextFrame();

        // 返回指定ID相机的内参
        Camera::Ptr GetCamera(int camera_id) const
        {
            return cameras_.at(camera_id);
        }

    private:
        std::string dataset_path_;
        int current_image_index_ = 0;

        std::vector<Camera::Ptr> cameras_;
    };

}

#endif
