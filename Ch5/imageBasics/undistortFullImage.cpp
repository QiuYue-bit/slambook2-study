#include "iostream"
#include "vector"
#include "opencv4/opencv2/opencv.hpp"
#include <string>

using namespace std;



string imagePath = "../imageBasics/pictrues/distorted.png";// 请确保路径正确
// 畸变参数
double k1 = -0.28340811, k2 = 0.07395907, p1 = 0.00019359, p2 = 1.76187114e-05;
// 内参
double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375;

//    计算去畸变后的图像边界
static cv::Mat ComputeImageBounds(const cv::Mat& image);
float mnMinX = 0;//左上和左下横坐标最小的
float mnMaxX = 0;//右上和右下横坐标最大的
float mnMinY = 0;//左上和右上纵坐标最小的
float mnMaxY = 0;//左下和右下纵坐标最小的

//    计算去畸变的点
static cv::Point2f computeUndistortedPoint(int u,int v);

int main(int argc, char **argv)
{
     // 图像是灰度图，CV_8UC1
    cv::Mat image = cv::imread(imagePath, 0);  
    //    图像数据不存在,可能是文件不存在
    if (image.data == nullptr)
    {
        cerr << "文件" << imagePath << "不存在." << endl;
        return 0;
    }
    // 文件顺利读取, 首先输出一些基本信息
    cout << "图像宽为" << image.cols << ",高为" << image.rows << ",通道数为" << image.channels() << endl;
    cv::imshow("image", image);      // 用cv::imshow显示原图像

    int rows = image.rows, cols = image.cols;
    cv::Mat image_undistort = ComputeImageBounds(image);

    //去畸变处理
    for (int v = 0; v < rows; ++v)
    {
        for (int u = 0; u < cols; ++u)
        {
            // 对原图的每一个点计算去畸变后的坐标
            cv::Point2f uv_Undistorted= computeUndistortedPoint(u,v);
//          转换到新坐标系下
            uv_Undistorted.x -= mnMinX;
            uv_Undistorted.y -= mnMinY;
            image_undistort.at<uchar>((int)uv_Undistorted.y,(int)uv_Undistorted.x) = image.at<uchar>(v,u);
        }
    }

    // 画图去畸变前后图像
    cv::imshow("undistorted", image_undistort);

    // 保存
    string outfile = "../imageBasics/image_undistort.png";
    cv::imwrite(outfile,image_undistort);
    cv::waitKey();
    cv::destroyAllWindows();
    return 0;
}

/*
 *     输入为未去畸变的点坐标,输出位对应的去畸变后的点坐标
 */
static cv::Point2f computeUndistortedPoint(int u,int v)
{

//            计算未发生畸变时，uv对应的归一化平面的xy坐标
    double x=((double)u-cx)/fx,y = ((double)v-cy)/fy;
    double r=sqrt(x*x + y*y);

//            计算去畸变后的XY坐标
    double x_undistorted = x/(1 + k1*pow(r,2)+k2*pow(r,4)) +2*p1*x*y+p2*(r * r +2*x*x);
    double y_undistorted = y/(1 + k1*pow(r,2)+k2*pow(r,4)) +2*p2*x*y+p1*(r * r +2*y*y);


//            投影回像素坐标
    double u_undistorted = fx*x_undistorted + cx;
    double v_undistorted = fy*y_undistorted + cy;

    return cv::Point2f(u_undistorted,v_undistorted);
}
static cv::Mat ComputeImageBounds(const cv::Mat& image)
{

    //  4个点的uv坐标
    vector<cv::Point2f> vertex;
    cv::Mat mat(4,2,CV_32F);
    //左上
    vertex.emplace_back(0.0,0.0);
    //右上
    vertex.emplace_back(image.cols,0.0);
    //左下
    vertex.emplace_back(0.0,image.rows);
    //右下
    vertex.emplace_back(image.cols,image.rows);

    for (int i = 0; i < 4; ++i)
    {
        vertex[i] = computeUndistortedPoint(vertex[i].x,vertex[i].y);
    }

     mnMinX = min(vertex[0].x,vertex[2].x);//左上和左下横坐标最小的
     mnMaxX = max(vertex[1].x,vertex[3].x);//右上和右下横坐标最大的
     mnMinY = min(vertex[0].y,vertex[1].y);//左上和右上纵坐标最小的
     mnMaxY = max(vertex[2].y,vertex[3].y);//左下和右下纵坐标最小的
    cout<<mnMinX<<endl<<mnMaxX<<endl<<mnMinY<<endl<<mnMaxY<<endl;

    int new_rows = (int)(mnMaxY-mnMinY);
    int new_cols = (int)(mnMaxX-mnMinX);
    cout<<"New rows "<<new_rows<<endl<<"New cols "<<new_cols<<endl;

    return cv::Mat(new_rows,new_cols, CV_8UC1);
}
