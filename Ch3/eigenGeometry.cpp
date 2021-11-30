/*
 * @Author: Divenire
 * @Date: 2021-09-04 13:00:59
 * @LastEditors: Divenire
 * @LastEditTime: 2021-09-22 17:11:00
 * @Description: 
 * 该程序主要演示一下基本的Eigen的几何模块使用，
 * 包括 旋转矩阵、角轴变换 ，visualizeGeometry中也有一些变换
 */

#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace std;


int main(int argc, char **argv)
{

    // Eigen/Geometry 模块提供了各种旋转和平移的表示
    // 3D 旋转矩阵直接使用 Matrix3d 或 Matrix3f
    Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Identity();

    ///旋转向量

    // 旋转向量使用 AngleAxis, 它底层不直接是Matrix，但运算可以当作矩阵（因为重载了运算符）
    Eigen::Vector3d v1(1,1,0);
    Eigen::AngleAxisd rotationVector(M_PI / 4,Eigen::Vector3d::UnitZ()); //沿着Z轴逆时针旋转45度
//    设置cout的数据精度
    cout.precision(3);

    // 也可以直接赋值
    rotationMatrix = rotationVector.toRotationMatrix();
    cout << "rotation matrix =\n" << rotationMatrix << endl;   //用matrix()转换成矩阵


    // 用 AngleAxis 可以进行坐标变换
    Eigen::Vector3d v2 = rotationVector*v1;
    std::cout<<"沿着Z轴逆时针旋转45度后："<<v2.transpose()<<std::endl;

    // 或者用旋转矩阵
    v2 = rotationMatrix*v1;
    std::cout<<"沿着Z轴逆时针旋转45度后："<<v2.transpose()<<std::endl;


    // 欧拉角: 可以将旋转矩阵直接转换成欧拉角
    Eigen::Vector3d eulerAngles = rotationMatrix.eulerAngles(2,1,0);// ZYX顺序，即yaw-pitch-roll的顺序输出
//    输出为弧度制 45度对应0.785=Pi/4
    cout << "yaw pitch roll = " << eulerAngles.transpose() << endl;

    /// 欧式变换

    // 欧氏变换矩阵使用 Eigen::Isometry
    /*
     * [R t]
     * [0 1]
     */
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    // 按照rotation_vector进行旋转
    T.rotate(rotationVector);
    // 把平移向量设成(1,3,4)
    T.pretranslate(Eigen::Vector3d(1, 3, 4));
    cout<<"欧式变换 \n"<<T.matrix()<<endl<<"欧式变换的旋转矩阵 \n"<<T.rotation()<<endl<<"欧式变换的平移部分 \n"<<T.translation().transpose()<<endl;

    // 用变换矩阵进行坐标变换   相当于R*v+t
    Eigen::Vector3d v_transformed = T * v1;
    cout << "v1 tranformed = " << v_transformed.transpose() << endl;

    /// 对于仿射和射影变换，使用 Eigen::Affine3d 和 Eigen::Projective3d 即可，略


    /// 四元数
    // 可以直接把AngleAxis赋值给四元数，反之亦然
    Eigen::Quaterniond q = Eigen::Quaterniond(rotationVector);
    cout << "quaternion from rotation vector = " << q.coeffs().transpose()
         << endl;
    // 请注意coeffs的顺序是(x,y,z,w),w为实部，前三者为虚部

    // 也可以把旋转矩阵赋给它
    q = Eigen::Quaterniond(rotationMatrix);
    cout << "quaternion from rotation matrix = " << q.coeffs().transpose() << endl;
    // 使用四元数旋转一个向量，使用重载的乘法即可
    v2 = q * v1; // 注意数学上是qvq^{-1}
    cout << "(1,0,0) after rotation = " << v2.transpose() << endl;
    // 用常规向量乘法表示，则应该如下计算
    cout << "should be equal to " << (q * Eigen::Quaterniond(0, 1, 0, 0) * q.inverse()).coeffs().transpose() << endl;
    return 0;
}
