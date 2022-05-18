/*** 
 * @Author       : Yue
 * @Date         : 2022-05-12 15:50:57
 * @LastEditTime : 2022-05-18 16:01:27
 * @LastEditors  : Yue
 * @Description  : 
 * @FilePath     : /Ch13/src/config.cpp
 * @佛祖保佑 BUG FREE
 */
#include "myslam/config.h"

namespace myslam
{

    // 静态成员变量，在类外定义
    // 一个指向类的指针
    std::shared_ptr<Config> Config::config_ = nullptr;


    
    bool Config::SetParameterFile(const std::string &filename)
    {
        // 如果指向Config类的指针还没有初始化，进行一下初始化

        // 类的成员函数可以直接使用类的静态成员
        if (config_ == nullptr)

            // 类的构造函数是私有的，这儿可以进行new操作是因为，这里是类成员函数的内部
            config_ = std::shared_ptr<Config>(new Config());

        // 根据参数文件路径，返回文件句柄
        config_->file_ = cv::FileStorage(filename.c_str(), cv::FileStorage::READ);

        // 尝试打开文件
        if (config_->file_.isOpened() == false)
        {
            LOG(ERROR) << "parameter file " << filename << " does not exist.";
            config_->file_.release();
            return false;
        }

        // 打开成功，返回true
        return true;
    }

    //
    Config::~Config()
    {
        if (file_.isOpened())
            file_.release();
    }


}
