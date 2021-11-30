/*
 * @Author: Divenire
 * @Date: 2021-09-04 13:00:59
 * @LastEditors: Divenire
 * @LastEditTime: 2021-09-22 17:10:23
 * @Description:  
 *  该程序主要演示一下基本的Eigen数据类型以及常见的使用方法
 *  矩阵元素操作，矩阵块和特征值
 */

#include "iostream"
//Eigen核心部分
#include "Eigen/Core"
#include "Eigen/Dense"

int main()
{

    /*********** 内置类型 ***********/

    std::cout<<"内置类型"<<std::endl;
    // Eigen 中所有向量和矩阵都是Eigen::Matrix，它是一个模板类。它的前三个参数为：数据类型，行，列
    // 声明一个2*3的float矩阵
    Eigen::Matrix<float, 2, 3> matrix_23f;

    // 同时，Eigen 通过 typedef 提供了许多内置类型，不过底层仍是Eigen::Matrix
    // 例如 Vector3d 实质上是 Eigen::Matrix<double, 3, 1>，即三维向量
    // Matrix3d 实质上是 Eigen::Matrix<double, 3, 3>
    Eigen::Vector3d v_3d;
    // 这是一样的
    Eigen::Matrix<double, 3, 1> vd_3d;

    //单位矩阵
    Eigen::Matrix<double,3,3> mi=Eigen::Matrix3d::Identity(3,3);
    std::cout<<"Eigen::Matrix3d::Identity(3,3) \n"<<mi<<std::endl;


    ///动态矩阵
    std::cout<<"动态矩阵"<<std::endl;

    // 如果不确定矩阵大小，可以使用动态大小的矩阵
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix_dynamic;
    // 更简单的
    Eigen::MatrixXd matrix_x;
    // 这种类型还有很多，我们不一一列举

    ///输入数据 初始化
    std::cout<<"输入数据"<<std::endl;
    // 下面是对Eigen阵的操作
    matrix_23f << 1, 2, 3, 4, 5, 6;
    // 输出
    std::cout << "matrix 2x3 from 1->6 : \n" << matrix_23f<<std::endl;

    /// 访问矩阵的元素
    std::cout<<"访问矩阵的元素"<<std::endl;
    // 用()访问矩阵中的元素
    std::cout << "print matrix 2x3: \n";
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 3; j++) std::cout << matrix_23f(i, j) << "\t";
        std::cout << std::endl;
    }

    /****************习题*******************/

    //提取矩阵的一部分的子矩阵，从一个很大的矩阵中，提取出右上角的2x2矩阵，然后赋值为I
    // 生成随机矩阵和随机向量
    #define MATRIX_SIZE 6
    Eigen::Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN
            = Eigen::MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    std::cout<<"-------------习题-------------"<<std::endl;
    std::cout<<matrix_NN<<std::endl<<std::endl;
//    Method 1 位操作
    for (int i = MATRIX_SIZE-2; i < MATRIX_SIZE; i++)
    {
        for (int j = 0; j < 2; j++)
            if(i==j)
                matrix_NN(i,j) = 1;
    }
//    Method 2 块操作
    matrix_NN.block<2,2>(0,4) = Eigen::Matrix2d::Identity(2,2);


    std::cout<<matrix_NN<<std::endl;
    std::cout<<"-------------习题-------------"<<std::endl;
    /// 矩阵运算

    // 矩阵和向量相乘（实际上仍是矩阵和矩阵）
    v_3d<<1,2,3;
    vd_3d<<4,5,6;

    // 但是在Eigen里你不能混合两种不同类型的矩阵，像这样是错的
//    Eigen::Matrix<double, 2, 1> result_wrong_type = matrix_23f * v_3d;
    // 应该显式转换
    Eigen::Matrix<double,2,1> result = matrix_23f.cast<double>() * v_3d;

    // 同样你不能搞错矩阵的维度
    // 试着取消下面的注释，看看Eigen会报什么错
//   Eigen::Matrix<double, 2, 3> result_wrong_dimension = matrix_23.cast<double>() * v_3d;
//    Eigen::Matrix<double, 2, 1> result_right_dimension = matrix_23.cast<double>() * v_3d;


    // 四则运算就不演示了，直接用+-*/即可。
    Eigen::Matrix<double,3,3> matrix_33 = Eigen::Matrix3d::Random();      // 随机数矩阵
    std::cout << "random matrix: \n" << matrix_33 << std::endl;
    std::cout << "transpose: \n" << matrix_33.transpose() << std::endl;      // 转置
    std::cout << "sum: " << matrix_33.sum() << std::endl;            // 各元素和
    std::cout << "trace: " << matrix_33.trace() << std::endl;          // 迹
    std::cout << "times 10: \n" << 10 * matrix_33 << std::endl;               // 数乘
    std::cout << "inverse: \n" << matrix_33.inverse() << std::endl;        // 逆
    std::cout << "det: " << matrix_33.determinant() << std::endl;    // 行列式

    /// 特征值
    // 实对称矩阵可以保证对角化成功
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(matrix_33 * matrix_33.transpose());
    std::cout << "Eigen values = \n" << eigen_solver.eigenvalues() << std::endl;
    std::cout << "Eigen vectors = \n" << eigen_solver.eigenvectors() << std::endl;

    
}
