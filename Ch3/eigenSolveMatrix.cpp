/*
 * @Author: Divenire
 * @Date: 2021-09-04 13:00:59
 * @LastEditors: Divenire
 * @LastEditTime: 2021-09-22 17:24:38
 * @Description:  
 * 
 * 该程序主要演示一下使用Eigen求解线性方程组的几种办法,并对耗时进行对比.
 * 不同解法在精度和效率上是有区别的。
 * https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html
 * https://eigen.tuxfamily.org/dox/group__DenseDecompositionBenchmark.html
 * 
 */


#include "iostream"
//Eigen核心部分
#include "Eigen/Core"
#include "Eigen/Dense"

#define MATRIX_SIZE 100

int main()
{
    /// 生成随机矩阵和随机向量
    Eigen::Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN
            = Eigen::MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    // 保证半正定
    matrix_NN = matrix_NN * matrix_NN.transpose(); 
    Eigen::Matrix<double, MATRIX_SIZE, 1> v_Nd = Eigen::MatrixXd::Random(MATRIX_SIZE, 1);

    /// 一般的线性方程求解

    /**************矩阵直接求逆计算******************/
    clock_t time_stt = clock(); // 计时
    Eigen::Matrix<double,MATRIX_SIZE,1> x = matrix_NN.inverse() * v_Nd;
    std::cout << "time of normal inverse is "
         << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << std::endl;
    // std::cout << "x = " << x.transpose() << std::endl;

    /*************QR分解******************/
    time_stt = clock(); // 计时
    x = matrix_NN.colPivHouseholderQr().solve(v_Nd);    //12.877ms
//    x = matrix_NN.householderQr().solve(v_Nd);      //  15.464ms
//    x = matrix_NN.fullPivHouseholderQr().solve(v_Nd); //25.963ms
    std::cout << "time of Qr decomposition is "
              << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << std::endl;
    // std::cout << "x = " << x.transpose() << std::endl;

    /**************Cholesky LDLT 分解******************/
    time_stt = clock(); // 计时
    x = matrix_NN.ldlt().solve(v_Nd);    //12.877ms
    std::cout << "time of ldlt decomposition is "
              << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << std::endl;
    // std::cout << "x = " << x.transpose() << std::endl;

}