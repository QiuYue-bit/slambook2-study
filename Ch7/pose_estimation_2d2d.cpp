/*
 * @Author: Divenire
 * @Date: 2021-10-18 10:46:59
 * @LastEditors: Divenire
 * @LastEditTime: 2021-10-18 22:02:56
 * @Description: 本程序演示了如何使用2D-2D的特征匹配估计相机运动
 *               输入图像，输出特征点以及特征点的匹配关系
 *               根据特征点及特征点的匹配情况计算E，F,H矩阵来恢复旋转和平移。
 *
 */

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;

// 输入图像，输出特征点以及特征点的匹配关系
void find_feature_matches(
    const Mat &img_1, const Mat &img_2,
    std::vector<KeyPoint> &keypoints_1,
    std::vector<KeyPoint> &keypoints_2,
    std::vector<DMatch> &matches);

// 根据特征点及特征点的匹配情况计算E，F,H矩阵来恢复旋转和平移。
void pose_estimation_2d2d(
    std::vector<KeyPoint> keypoints_1,
    std::vector<KeyPoint> keypoints_2,
    std::vector<DMatch> matches,
    Mat &R, Mat &t);

// 像素坐标转相机归一化坐标
Point2d pixel2cam(const Point2d &p, const Mat &K);

int main(int argc, char **argv)
{

  string img1_path = "../1.png";
  string img2_path = "../2.png";

  //-- 读取图像
  Mat img_1 = imread(img1_path);
  Mat img_2 = imread(img2_path);
  assert(img_1.data && img_2.data && "Can not load images!");

  vector<KeyPoint> keypoints_1, keypoints_2;
  vector<DMatch> matches;
  find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
  cout << "一共找到了" << matches.size() << "组匹配点" << endl;

  //-- 估计两张图像间运动
  Mat R, t;
  pose_estimation_2d2d(keypoints_1, keypoints_2, matches, R, t);

  //-- 验证E=t^R*scale
  Mat t_x =
      (Mat_<double>(3, 3) << 0, -t.at<double>(2, 0), t.at<double>(1, 0),
       t.at<double>(2, 0), 0, -t.at<double>(0, 0),
       -t.at<double>(1, 0), t.at<double>(0, 0), 0);

  cout << "t^R=" << endl
       << t_x * R << endl;

  //-- 验证对极约束,也就是验证x^t_2 * E * x_1 = 0
  Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

  vector<DMatch> err_matches;
  for (DMatch m : matches) {
      Point2d pt1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
      Mat y1 = (Mat_<double>(3, 1) << pt1.x, pt1.y, 1);
      Point2d pt2 = pixel2cam(keypoints_2[m.trainIdx].pt, K);
      Mat y2 = (Mat_<double>(3, 1) << pt2.x, pt2.y, 1);
      Mat d = y2.t() * t_x * R * y1;
      cout << "epipolar constraint = " << d.at<double>(0) << endl;

      //误差太大的，就删除看看坐标是不是误匹配
      if (abs(d.at<double>(0)) > 0.003)
      {
          err_matches.push_back(m);
      }
  }
    cv::Mat image_show;
    cv::drawMatches(img_1, keypoints_1, img_2, keypoints_2, err_matches, image_show);
    cv::imshow("err_matches", image_show);
    cv::waitKey(0);
  return 0;
}

void find_feature_matches(const Mat &img_1, const Mat &img_2,
                          std::vector<KeyPoint> &keypoints_1,
                          std::vector<KeyPoint> &keypoints_2,
                          std::vector<DMatch> &matches)
{
  //-- 初始化
  Mat descriptors_1, descriptors_2;
  // used in OpenCV3
  Ptr<FeatureDetector> detector = ORB::create();
  Ptr<DescriptorExtractor> descriptor = ORB::create();
  //-- 第一步:检测 Oriented FAST 角点位置
  detector->detect(img_1, keypoints_1);
  detector->detect(img_2, keypoints_2);

  //-- 第二步:根据角点位置计算 BRIEF 描述子
  descriptor->compute(img_1, keypoints_1, descriptors_1);
  descriptor->compute(img_2, keypoints_2, descriptors_2);

  //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
  Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
  vector<DMatch> match;
  matcher->match(descriptors_1, descriptors_2, match);

  //-- 第四步:匹配点对筛选
  double min_dist = 10000, max_dist = 0;

  //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
  for (int i = 0; i < descriptors_1.rows; i++)
  {
    double dist = match[i].distance;
    if (dist < min_dist)
      min_dist = dist;
    if (dist > max_dist)
      max_dist = dist;
  }

  printf("-- Max dist : %f \n", max_dist);
  printf("-- Min dist : %f \n", min_dist);

  //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
  for (int i = 0; i < descriptors_1.rows; i++)
  {
    if (match[i].distance <= max(2 * min_dist, 30.0))
    {
      matches.push_back(match[i]);
    }
  }
}

Point2d pixel2cam(const Point2d &p, const Mat &K)
{
  return Point2d(
      (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
      (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1));
}

void pose_estimation_2d2d(std::vector<KeyPoint> keypoints_1,
                          std::vector<KeyPoint> keypoints_2,
                          std::vector<DMatch> matches,
                          Mat &R, Mat &t)
{
  // 相机内参,TUM Freiburg2
  Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

  //-- 把匹配点转换为vector<Point2f>的形式
  vector<Point2f> points1;
  vector<Point2f> points2;

  for (int i = 0; i < (int)matches.size(); i++)
  {
    points1.push_back(keypoints_1[matches[i].queryIdx].pt);
    points2.push_back(keypoints_2[matches[i].trainIdx].pt);
  }

  //-- 计算基础矩阵
  Mat fundamental_matrix;
  fundamental_matrix = findFundamentalMat(points1, points2, FM_8POINT);
  cout << "fundamental_matrix is " << endl
       << fundamental_matrix << endl;

  //-- 计算本质矩阵
  Point2d principal_point(325.1, 249.7); //相机光心, TUM dataset标定值
  double focal_length = 521;             //相机焦距, TUM dataset标定值
  Mat essential_matrix;
  essential_matrix = findEssentialMat(points1, points2, focal_length, principal_point);
  cout << "essential_matrix is " << endl
       << essential_matrix << endl;

  //-- 计算单应矩阵
  //-- 但是本例中场景不是平面，单应矩阵意义不大
  Mat homography_matrix;
  homography_matrix = findHomography(points1, points2, RANSAC, 3);
  cout << "homography_matrix is " << endl
       << homography_matrix << endl;

  //-- 从本质矩阵中恢复旋转和平移信息.
  // 此函数仅在Opencv3中提供
  recoverPose(essential_matrix, points1, points2, R, t, focal_length, principal_point);
  cout << "R is " << endl
       << R << endl;
  cout << "t is " << endl
       << t << endl;
}
