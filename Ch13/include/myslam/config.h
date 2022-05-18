/*** 
 * @Author       : Yue
 * @Date         : 2022-05-12 15:50:57
 * @LastEditTime : 2022-05-17 16:13:11
 * @LastEditors  : Yue
 * @Description  : 
 * @FilePath     : /Ch13/include/myslam/config.h
 * @佛祖保佑 BUG FREE
 */
#pragma once
#ifndef MYSLAM_CONFIG_H
#define MYSLAM_CONFIG_H

#include "myslam/common_include.h"

// Config类，这个类从名称上就很容易看出来是读取配置文件的，我们会有一个yaml文件或者其他类型的文件用来存放一些系统的超参数
// Config类提供了统一的函数用来确定配置文件的位置，并从中读取相应参数。
// Config类的行为是确定的，因此成员函数都是静态变量，不需要去实例化
namespace myslam
{

    class Config
    {
    private:
		
        //类的构造函数一般是public的，但是也可以是private的。构造函数为私有的类有这样的特点：
        //<1>不能实例化：因为实例化时类外部无法访问其内部的私有的构造函数；
        //<2>不能继承：同<1>；
        Config() {}
		
        // 类的静态对象 config_指针指向类的对象
		// 单例设计模式，在SetParameterFile成员函数中被初始化
        static std::shared_ptr<Config> config_;

        // 参数文件的句柄
        cv::FileStorage file_;



    public:
        ~Config(); // close the file when deconstructing

        // set a new config file
		// 静态成员函数 设置
        static bool SetParameterFile(const std::string &filename);


        //静态成员函数主要为了调用方便，不需要生成对象就能调用，它跟类的实例无关，只跟类有关，不需要this指针。
        //从OOA/OOD的角度考虑，一切不需要实例化就可以有确定行为方式的函数都应该设计成静态的。
        template <typename T> 
        static T Get(const std::string &key)
        {
            return T(Config::config_->file_[key]);
        }
    };

}

#endif
