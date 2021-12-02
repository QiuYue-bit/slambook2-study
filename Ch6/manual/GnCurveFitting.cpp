/*
 * @Author: divenire
 * @Date: 2021-10-15 13:46:43
 * @LastEditTime: 2021-12-02 15:06:51
 * @LastEditors: Divenire
 * @Description: 
                solve time cost = 2.80578 ms 
                true model: a:1 b:2     c:1
                estimated model: a:0.890912     b:2.1719        c:0.943629
 */
#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

int main(int argc, char **argv)
{
    double ar = 1.0, br = 2.0, cr = 1.0;  // 真实参数值
    double ae = 2.0, be = -1.0, ce = 5.0; // 估计参数值
    int N = 100;                          // 数据点
    double w_sigma = 1.0;                 // 噪声Sigma值
    double inv_sigma = 1.0 / w_sigma;
    cv::RNG rng; // OpenCV随机数产生器

    // 产生模拟数据
    vector<double> x_data, y_data;
    for (int i = 0; i < N; i++)
    {
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma * w_sigma));
    }

    // 最大迭代次数
    int iterations = 10;

    // 本次迭代的cost和上一次迭代的cost
    double cost = 0, lastCost = 0;

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    for (int iter = 0; iter < iterations; iter++)
    {

        Matrix3d H = Matrix3d::Zero(); // Hessian = J^T W^{-1} J in Gauss-Newton
        Vector3d b = Vector3d::Zero(); // H * delta_x = b
        cost = 0;

        for (int i = 0; i < N; i++)
        {
            // 计算数据点误差
            double xi = x_data[i], yi = y_data[i]; // 第i个数据点
            double error = yi - exp(ae * xi * xi + be * xi + ce);

            // 计算当前估计点处的雅克比矩阵
            Vector3d J;
            J[0] = -xi * xi * exp(ae * xi * xi + be * xi + ce); // de/da
            J[1] = -xi * exp(ae * xi * xi + be * xi + ce);      // de/db
            J[2] = -exp(ae * xi * xi + be * xi + ce);           // de/dc

            // 海森矩阵计算
            H += J * J.transpose();

            // 书上的g(x)
            b += -error * J;
            
            // 代价计算
            cost += error * error;
        }

        // ldlt方法求解线性方程 Hx=b
        Vector3d dx = H.ldlt().solve(b);

        if (isnan(dx[0]))
        {
            cout << "result is nan!" << endl;
            break;
        }

        //判断代价函数是否减小，如果迭代后代价函数反而增大了，就退出。
        if (iter > 0 && cost >= lastCost)
        {
            cout << "cost: " << cost << ">= last cost: " << lastCost << ", break." << endl;
            break;
        }

        // 更新估计值
        ae += dx[0];
        be += dx[1];
        ce += dx[2];

        lastCost = cost;
    
        cout <<"iteration= "<<iter
        << "\t total cost: " << cost 
        << "\t update: " << dx.transpose() 
        << "\t estimated params: " << ae << "," << be << "," << ce << endl;
    }

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "solve time cost = " << time_used.count()*1000 << " ms " << endl;

    // 输出优化值
    cout << "true model: " <<"a:"<< ar<<"\t"<<
                             "b:"<<br<<"\t"<<
                             "c:"<<cr << endl;

    cout << "estimated model: " <<"a:"<<ae<<"\t"<<
                                  "b:"<<be<<"\t"<<
                                  "c:"<<ce<< endl;

    return 0;
}
