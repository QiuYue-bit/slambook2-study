/*
 * @Author: Divenire
 * @Date: 2021-10-15 13:46:43
 * @LastEditors: Divenire
 * @LastEditTime: 2021-12-02 14:20:56
 * @Description:
                曲线拟合问题。
                用不同的迭代算法和核函数对结果进行测试。
 */

#include <iostream>
#include <opencv2/core/core.hpp>
#include <cmath>
#include <chrono>

#include "types.h"

using namespace std;
using namespace g2o;


// 顶点类实体
CurveFittingEdge::CurveFittingEdge(double x) : g2o::BaseUnaryEdge<1, double, CurveFittingVertex>()
{
    _x = x;
}

bool CurveFittingEdge::write(std::ostream &os) const
{
    return false;
}

bool CurveFittingEdge::read(std::istream &is) 
{
    return false;
}

// 边类实体
CurveFittingVertex::CurveFittingVertex(): g2o::BaseVertex<3, Eigen::Vector3d>()
{

}

bool CurveFittingVertex::write(std::ostream &os) const
{
    return false;
}

bool CurveFittingVertex::read(std::istream &is) 
{
    return false;
}



int main(int argc, char **argv)
{
    double ar = 1.0, br = 2.0, cr = 1.0;  // 真实参数值
    double ae = 2.0, be = -1.0, ce = 5.0; // 估计参数值
    int N = 100;                          // 数据点
    double w_sigma = 1.0;                 // 噪声Sigma值
    double inv_sigma = 1.0 / w_sigma;
    cv::RNG rng; // OpenCV随机数产生器

    vector<double> x_data, y_data; // 数据
    for (int i = 0; i < N; i++)
    {
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma * w_sigma));
    }

    // Pose的维度为3 ，Landmark的维度为1。换句话说状态变量增量的维度为3，观测数据的维度为1.
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>> BlockSolverType;

    // 线性求解器类型 设为为 dense cholesky分解法
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

    // 优化方法选择高斯牛顿法
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));

    // 图模型参数设置
    g2o::SparseOptimizer optimizer; // 图模型
    optimizer.setAlgorithm(solver); // 设置求解器
    optimizer.setVerbose(true);     // 打开调试输出

    // 往图中增加顶点
    cout << "Optimization: Adding Vertex  ... ";
    CurveFittingVertex *v = new CurveFittingVertex;
    // 设置初始估计值
    v->setEstimate(Eigen::Vector3d(ae, be, ce));
    // 顶点的id号
    v->setId(0);
    optimizer.addVertex(v);
    cerr << "done." << endl;


    // 往图中增加边
    cout << "Optimization: Adding Edge  ... ";
    for (int i = 0; i < N; i++)
    {
        CurveFittingEdge *edge = new CurveFittingEdge(x_data[i]);
        
        // 设置边的ID，其实可以不不用设置
        // edge->setId(i);

        // 设置连接的顶点,由于是一元边，只有一个顶点。
        edge->setVertex(0, v);

        // 设置边的观测数据                                                                   
        edge->setMeasurement(y_data[i]);      
           
        // 设置信息矩阵:协方差的逆                                           
        edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1 / (w_sigma * w_sigma)); 

        //robust fuction
        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        edge->setRobustKernel(rk);

        optimizer.addEdge(edge);
    }
    cerr << "done." << endl;


    // 执行优化
    cout << "start optimization" << endl;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    
    // 优化器初始化，为H，b等矩阵分配内存块，计算零元素等
    optimizer.initializeOptimization();

    // 开始优化，最大迭代次数10次
    optimizer.optimize(10);

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "solve time cost = " << time_used.count()*1000 << " ms. " << endl;

    // 输出优化值
    Eigen::Vector3d abc_estimate = v->estimate();
    cout << "true model: " <<"a:"<< ar<<"\t"<<
                            "b:"<<br<<"\t"<<
                            "c:"<<cr << endl;

    cout << "estimated model: " <<"a:"<< abc_estimate(0,0)<<"\t"<<
                                  "b:"<<abc_estimate(1,0)<<"\t"<<
                                  "c:"<<abc_estimate(2,0) << endl;


    return 0;
}