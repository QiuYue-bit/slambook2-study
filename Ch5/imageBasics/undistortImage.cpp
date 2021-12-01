/*
 * @Author: Divenire
 * @Date: 2021-09-23 15:05:38
 * @LastEditors: Divenire
 * @LastEditTime: 2021-12-01 21:57:39
 * @Description: file content
 */

#include <opencv2/opencv.hpp>
#include <string>

using namespace std;
using namespace cv;

string image_file = "../data//distorted.png"; // 请确保路径正确

// 双线性插值
uchar Bilinear_Inter(double u, double v, const Mat &image);

// 本程序实现去畸变部分的代码。尽管我们可以调用OpenCV的去畸变，但自己实现一遍有助于理解。
// 畸变参数
double k1 = -0.28340811, k2 = 0.07395907, p1 = 0.00019359, p2 = 1.76187114e-05;
// 内参
double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375;

int main(int argc, char **argv)
{

  cv::Mat image = cv::imread(image_file, 0); // 图像是灰度图，CV_8UC1
  int rows = image.rows, cols = image.cols;
  cv::Mat image_undistort = cv::Mat(rows, cols, CV_8UC1); // 去畸变以后的图
      // 判断图像文件是否正确读取
  if (image.data == nullptr)
  { //数据不存在,可能是文件不存在
    cerr << "文件" << argv[1] << "不存在." << endl;
    return 0;
  }

  // 计算去畸变后图像的内容
  for (int v = 0; v < rows; v++)
  {
    for (int u = 0; u < cols; u++)
    {
      // 按照公式，计算点(u,v)对应到畸变图像中的坐标(u_distorted, v_distorted)
      double x = (u - cx) / fx, y = (v - cy) / fy;
      double r = sqrt(x * x + y * y);
      double x_distorted = x * (1 + k1 * r * r + k2 * r * r * r * r) + 2 * p1 * x * y + p2 * (r * r + 2 * x * x);
      double y_distorted = y * (1 + k1 * r * r + k2 * r * r * r * r) + p1 * (r * r + 2 * y * y) + 2 * p2 * x * y;
      double u_distorted = fx * x_distorted + cx;
      double v_distorted = fy * y_distorted + cy;

      // 赋值 (最近邻插值)
      if (u_distorted >= 0 && v_distorted >= 0 && u_distorted < cols && v_distorted < rows)
      {
        image_undistort.at<uchar>(v, u) = Bilinear_Inter(u_distorted, v_distorted, image);
      }
      else
      {
        image_undistort.at<uchar>(v, u) = 0;
      }
    }
  }

  // 画图去畸变后图像
  cv::imshow("distorted", image);
  cv::imshow("undistorted", image_undistort);
  cv::waitKey();
  return 0;
}

uchar Bilinear_Inter(double u, double v, const Mat &image)
{
  if ((int)u >= 0 && (int)u <= image.cols - 2)
    if ((int)v >= 0 && (int)v <= image.rows - 2)
    {
      uchar
          LT = image.at<uchar>((int)v, (int)u),
          LB = image.at<uchar>((int)v + 1, (int)u),
          RT = image.at<uchar>((int)v, (int)u + 1),
          RB = image.at<uchar>((int)v + 1, (int)u + 1);

      double dLx, dTy, dRx, dBy;
      dLx = u - (int)u;
      dRx = (int)u - u + 1;
      dTy = v - (int)v;
      dBy = (int)v - v + 1;
      uchar TP = dLx * RT + dRx * LT;

      uchar TB = dLx * RB + dRx * LB;
      return TP * dBy + TB * dTy;
    }
      return 0;
}
