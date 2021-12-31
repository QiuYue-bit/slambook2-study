//
// Created by divenire on 12/31/21.
//

#ifndef MAIN_COMMON_H
#define MAIN_COMMON_H

#include <iostream>
#include <fstream>
#include <list>
#include <utility>
#include <vector>
#include <chrono>
#include <ctime>
#include <climits>

#include <opencv2/core/core.hpp>

// 相机内参
double cx = 325.5;
double cy = 253.5;
double fx = 518.0;
double fy = 519.0;
double depth_scale = 1000.0;


class Measurement
{
public:
    Measurement(Eigen::Vector3d p,double g)
    : pos_world_(std::move(p)),grayscale_(g)
    {}

    Eigen::Vector3d Get3dcorrdinate() const {return pos_world_;}
    double Getgrayscale() const {return grayscale_;}

private:
    Eigen::Vector3d pos_world_;
    double grayscale_;

};

inline Eigen::Vector3d project2Dto3D ( int x, int y, int d, double fx, double fy, double cx, double cy, double scale )
{
    double zz = double ( d ) /scale;
    double xx = zz* ( x-cx ) /fx;
    double yy = zz* ( y-cy ) /fy;
    return Eigen::Vector3d ( xx, yy, zz );
}

inline Eigen::Vector2d project3Dto2D ( double x, double y, double z, double fx, double fy, double cx, double cy )
{
    double u = fx*x/z+cx;
    double v = fy*y/z+cy;
    return Eigen::Vector2d ( u,v );
}

/// 绘图1 连线
inline bool DrawRelevantPointsWithLines(const Measurement& measurements, cv::Mat& image, const Sophus::SE3d& Tcw )
{
    int rows = image.rows;
    int half_cols =image.cols/2;

    // 随机颜色
    float b = 255*float ( rand() ) /RAND_MAX;
    float g = 255*float ( rand() ) /RAND_MAX;
    float r = 255*float ( rand() ) /RAND_MAX;

    //
    Eigen::Vector3d p = measurements.Get3dcorrdinate();
    Eigen::Vector2d pixel_ref = project3Dto2D ( p ( 0,0 ), p ( 1,0 ), p ( 2,0 ), fx, fy, cx, cy );

    Eigen::Vector3d p2 = Tcw*measurements.Get3dcorrdinate();
    Eigen::Vector2d pixel_cur = project3Dto2D ( p2 ( 0,0 ), p2 ( 1,0 ), p2 ( 2,0 ), fx, fy, cx, cy );

    // 越界了就直接错误
    if ( pixel_cur(0)<0 || pixel_cur(0)>=half_cols || pixel_cur(1)<0 || pixel_cur(1)>=rows)
        return false;

    //对应的特征连线 与 标记
    cv::circle ( image, cv::Point2d ( pixel_ref ( 0,0 ), pixel_ref ( 1,0 ) ), 2, cv::Scalar ( b,g,r ), 1 );
    cv::circle ( image, cv::Point2d ( pixel_cur ( 0,0 ) + half_cols, pixel_cur ( 1,0 )  ), 2, cv::Scalar ( b,g,r ), 1 );
    cv::line ( image, cv::Point2d ( pixel_ref ( 0,0 ), pixel_ref ( 1,0 ) ), cv::Point2d ( pixel_cur ( 0,0 )+half_cols, pixel_cur ( 1,0 ) ), cv::Scalar ( b,g,r ), 1 );

    return true;
}
/// 绘图2 不连线
/// 绘图1 连线
inline bool DrawRelevantPointsWithoutLines(const Measurement& measurements, cv::Mat& image, const Sophus::SE3d& Tcw )
{
    int rows = image.rows;
    int half_cols =image.cols/2;

    // 随机颜色
    float b = 255*float ( rand() ) /RAND_MAX;
    float g = 255*float ( rand() ) /RAND_MAX;
    float r = 255*float ( rand() ) /RAND_MAX;

    //
    Eigen::Vector3d p = measurements.Get3dcorrdinate();
    Eigen::Vector2d pixel_ref = project3Dto2D ( p ( 0,0 ), p ( 1,0 ), p ( 2,0 ), fx, fy, cx, cy );

    Eigen::Vector3d p2 = Tcw*measurements.Get3dcorrdinate();
    Eigen::Vector2d pixel_cur = project3Dto2D ( p2 ( 0,0 ), p2 ( 1,0 ), p2 ( 2,0 ), fx, fy, cx, cy );

    // 越界了就直接错误
    if ( pixel_cur(0)<0 || pixel_cur(0)>=half_cols || pixel_cur(1)<0 || pixel_cur(1)>=rows)
        return false;

    //对应的特征连线 与 标记
    cv::circle ( image, cv::Point2d ( pixel_ref ( 0,0 ), pixel_ref ( 1,0 ) ), 2, cv::Scalar ( b,g,r ), 1 );
    cv::circle ( image, cv::Point2d ( pixel_cur ( 0,0 ) + half_cols, pixel_cur ( 1,0 )  ), 2, cv::Scalar ( b,g,r ), 1 );
//    cv::line ( image, cv::Point2d ( pixel_ref ( 0,0 ), pixel_ref ( 1,0 ) ), cv::Point2d ( pixel_cur ( 0,0 )+half_cols, pixel_cur ( 1,0 ) ), cv::Scalar ( b,g,r ), 1 );

    return true;
}



#endif //MAIN_COMMON_H
