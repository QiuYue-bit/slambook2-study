/*** 
 * @Author       : Yue
 * @Date         : 2022-05-12 15:50:57
 * @LastEditTime : 2022-05-17 21:36:24
 * @LastEditors  : Yue
 * @Description  : 
 * @FilePath     : /Ch13/include/myslam/algorithm.h
 * @佛祖保佑 BUG FREE
 */
#ifndef MYSLAM_ALGORITHM_H
#define MYSLAM_ALGORITHM_H

// algorithms used in myslam
#include "myslam/common_include.h"

/// algorithm.h文件主要实现了一个三角化函数，其实个人感觉三角化算法不单独用头文件和cpp文件来做也是可行的

namespace myslam
{

    inline bool triangulation(const std::vector<SE3> &poses, const std::vector<Vec3> points, Vec3 &pt_world)
    {

        MatXX A(2 * poses.size(), 4);
        VecX b(2 * poses.size());
        b.setZero();

        // 构建A矩阵，见Onenote-《三角化》
        for (size_t i = 0; i < poses.size(); ++i)
        {
            auto m = poses[i].matrix3x4();
            A.block<1, 4>(2 * i, 0) = points[i][0] * m.row(2) - m.row(0);
            A.block<1, 4>(2 * i + 1, 0) = points[i][1] * m.row(2) - m.row(1);
        }

        // 矩阵A的维度是2m*4的，有4个奇异值，取最小奇异值对应的特征向量就是世界坐标
        // 将齐次坐标最后一维化成1,再取前三维，就是世界坐标

        auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
        pt_world = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();

        if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2)
        {
            // 确保A矩阵的零空间是1维的，也就是解的有效性判断
            // 如果最后两个奇异值特别接近，说明这个A矩阵的零空间是两维的，解的质量是过关的，这个解是可靠的，是不存在歧义的。
            return true;
        }
        else
        {
            // 解质量不好，就是说可能存在另一个特征向量对应的特征值和这个我们选出来的解特征向量对应的特征值大小比较接近
            // ，在最小化这件事情上它们两个解的效果应该差不多，这个时候就存在歧义了，我们的SVD解就显得不可靠了，就得放弃
            return false;
        }

        return false;
    }

    // converters
    inline Vec2 toVec2(const cv::Point2f p) { return Vec2(p.x, p.y); }
}

#endif // MYSLAM_ALGORITHM_H