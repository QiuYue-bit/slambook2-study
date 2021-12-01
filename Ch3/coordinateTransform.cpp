/*
 * @Author: Divenire
 * @Date: 2021-09-04 13:00:59
 * @LastEditors: Divenire
 * @LastEditTime: 2021-12-01 16:11:37
 * @Description: 小萝卜坐标转换程序
 */

#include <iostream>
using namespace std;


// Eigen 核心部分
#include "Eigen/Core"
#include "Eigen/Geometry"
using namespace Eigen;

int main()
{

//    初值设置。
    Quaterniond q1(0.55,0.3,0.2,0.2),q2(-0.1,0.3,-0.7,0.2);
    Vector3d t1(0.7,1.1,0.2),t2(-0.1,0.4,0.8);
    Vector3d pos1(0.5,-0.1,0.2);

//    四元素归一化。
    q1.normalize();
    q2.normalize();

//    转换为齐次坐标。
    Isometry3d t1w(q1),t2w(q2);
    t1w.pretranslate(t1);
    t2w.pretranslate(t2);

//    小萝卜二号在世界坐标系下的坐标。
    Vector3d p2 = t2w * t1w.inverse() * pos1;
    std::cout << "====================================================" << std::endl;
    std::cout << "\t\t\t\t转换后的坐标为" 
    <<p2(0) <<" "
    <<p2(1) <<" "
    <<p2(2) <<" "
    << std::endl;

    return 0;
}

