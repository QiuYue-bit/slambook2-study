/*
 * @Author: Divenire
 * @Date: 2022-05-17
 * @LastEditors: Divenire
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

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <memory>

using namespace std;
using namespace cv;

string image_path = "../data/ubuntu.png";


class pictrure
{
    public:
        cv::Mat img_;
};

std::shared_ptr<pictrure> readImage()
{
    auto image_left =
            cv::imread(image_path,
                       cv::IMREAD_GRAYSCALE);

    if(image_left.data == nullptr)
        return nullptr;

    std::shared_ptr<pictrure> Ptr(new pictrure);

    Ptr->img_ = image_left;

    if(&image_left != &Ptr->img_)
    {
        cout<<"this is a deep copy"<<endl;
        cout<<" &image_left is :"<<&image_left<<endl;
        cout<<" &Ptr->img_ :"<<&Ptr->img_<<endl;
    }

    return Ptr;

}


int main()
{

    auto ptr = readImage();

    if(ptr)
    {
        imshow("test",ptr->img_);
        cv::waitKey(0);
        cout<<"show ";
    }

    else
        cout << "nullptr"<<endl;



    return 0;
}
