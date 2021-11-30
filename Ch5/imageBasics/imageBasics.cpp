/*
 * @Author: Divenire
 * @Date: 2021-09-23 15:05:38
 * @LastEditors: Divenire
 * @LastEditTime: 2021-09-23 16:47:08
 * @Description: 
 *  OPENCV的基本操作
 *  1.图像的读取，显示
 *  2.图像深拷贝、浅拷贝、
 *  3.遍历像素的几种方法
 *  在彩色图像中，图像数据缓冲区的前 3 字节表示左上角像素的三个通道的值，接下来的 3字节表示第 1 行的第 2 个像素，
    以此类推（注意 OpenCV 默认的通道次序为 BGR）。一个宽 W高 H 的图像所需的内存块大小为 W×H×3 uchars。不过出于性能上的考虑，
    我们会用几个额外的像素来填补行的长度。这是因为，如果行数是某个数字（例如 8）的整数倍，图像处理的性能可能会提高，
    因此最好根据内存配置情况将数据对齐。所以并不一定每行最后一个元素后边一定是下一行的第一个元素！
 */
    
#include <iostream>
#include <chrono>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;

string image_path = "../imageBasics/pictrues/ubuntu.png";
string image_path1 = "/home/divenire/Divenire_ws/dataset/KITTI/dataset/sequences/00/image_0/000001.png";


// 三种访问的方法
void method1(cv::Mat image);
void method2(cv::Mat image);
void method3(cv::Mat image);

int main(int argc, char **argv) 
{
  

//    输出一下OPENCV的版本
    cout << CV_VERSION << endl;

    cv::Mat image = cv::imread(image_path);

//    图像数据不存在,可能是文件不存在
    if (image.data == nullptr)
    {
        cerr << "文件" << image_path << "不存在." << endl;
        return 0;
    }

    // 文件顺利读取, 首先输出一些基本信息
    cout << "图像宽为" << image.cols << ",高为" << image.rows << ",通道数为" << image.channels() << endl;
    cv::imshow("image", image);      // 用cv::imshow显示图像
    cv::waitKey(0);                  // 暂停程序,等待一个按键输入

    // 判断image的类型
    if (image.type() != CV_8UC1 && image.type() != CV_8UC3)
    {
        // 图像类型不符合要求
        cout << "请输入一张彩色图或灰度图." << endl;
        return 0;
    }

    // 关于 cv::Mat 的拷贝
    // 直接赋值并不会拷贝数据，仅仅复制了一个引用
    cv::Mat image_another = image;
    // 修改 image_another 会导致 image 发生变化
    image_another(cv::Rect(0, 0, 100, 100)).setTo(0); // 将左上角100*100的块置零
    cv::imshow("image", image);
    cv::waitKey(0);

    // 使用clone函数来拷贝数据
    cv::Mat image_clone = image.clone();
    image_clone(cv::Rect(0, 0, 100, 100)).setTo(255);
    cv::imshow("image_clone", image_clone);
    cv::waitKey(0);

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    method2(image_another);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double >>(t2 - t1);
    cout << "遍历图像用时：" << time_used.count() << " 秒。" << endl;


    cv::imshow("image_method", image_another);
    cv::waitKey(0);


    // 对于图像还有很多基本的操作,如剪切,旋转,缩放等,限于篇幅就不一一介绍了,请参看OpenCV官方文档查询每个函数的调用方法.
    cv::destroyAllWindows();
    return 0;
}


//---------------------------------------【方法一】----------------------------------------------
//		一个点一个点读取 可以实现随机访问 速度最慢
//-------------------------------------------------------------------------------------------------
void method1(cv::Mat image)
{
    for (int i = 0; i < image.rows; i++)
    {
        for (int j = 0; j < image.cols; j++)
        {
            //Do anything you want
            if (image.channels() == 1)
            {
                //读取灰度图像的点
                image.at<uchar>(i, j) = 255;
            } else if (image.channels() == 3)
            {
                //读取彩色图像点的0,1,2通道
                image.at<cv::Vec3b>(i, j)[0] = 255;
                image.at<cv::Vec3b>(i, j)[1] = 255;
                image.at<cv::Vec3b>(i, j)[2] = 255;
            }
        }
    }
}

//---------------------------------------【方法二】----------------------------------------------
//		指针遍历 最快的方法
//-------------------------------------------------------------------------------------------------
void method2(cv::Mat image)
{
    int image_rows = image.rows;//行数
    int elementCount = image.cols * image.channels();//每行元素数

    // 在彩色图像中，图像数据缓冲区的前 3 字节表示左上角像素的三个通道的值，接下来的 3字节表示第 1 行的第 2 个像素，
    // 以此类推（注意 OpenCV 默认的通道次序为 BGR）。一个宽 W高 H 的图像所需的内存块大小为 W×H×3 uchars。不过出于性能上的考虑，
    // 我们会用几个额外的像素来填补行的长度。这是因为，如果行数是某个数字（例如 8）的整数倍，图像处理的性能可能会提高，
    // 因此最好根据内存配置情况将数据对齐。所以并不一定每行最后一个元素后边一定是下一行的第一个元素！

    //如果图像没有经过填补 内存是连续排序
    if (image.isContinuous())
    {
        //把整幅图像看成一行 一维数组
        elementCount = elementCount * image_rows;
        image_rows = 1;
    }

    for (int i = 0; i < image_rows; i++)
    {
        uchar *linehead = image.ptr<uchar>(i);//每行的起始地址
        for (int j = 0; j < elementCount; j++)
        {
            //遍历元素 Do anything you want
            linehead[j] = 255;
        }
    }
}

//---------------------------------------【方法三】----------------------------------------------
//		迭代器
//-------------------------------------------------------------------------------------------------
void method3(cv::Mat image)
{
    if (image.channels() == 3)
    {
        Mat_<Vec3b>::iterator it = image.begin<Vec3b>();
        Mat_<Vec3b>::iterator itend = image.end<Vec3b>();
        for (; it != itend; it++)
        {
            (*it)[0] = 255;
            (*it)[1] = 255;
            (*it)[2] = 255;
        }
    } else if (image.channels() == 1)
    {
        Mat_<uchar>::iterator it = image.begin<uchar>();
        Mat_<uchar>::iterator itend = image.end<uchar>();
        for (; it != itend; it++)
        {
            (*it) = 255;
        }
    }
}