/*
 * @Author: Divenire
 * @Date: 2021-10-16 13:44:47
 * @LastEditors: Divenire
 * @LastEditTime: 2021-10-16 15:35:09
 * @Description: Ceres使用计算雅克比的方式拟合曲线
 *               在我的机器上I7-8750h 耗时0.26ms
 *               对比可以基本看出来解析法是最快的，比自动微分快了20倍
 */
#include <iostream>
#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>
#include <chrono>
using namespace std;

// A CostFunction implementing analytically derivatives for the
// function f(x) = 10 - x.
class QuadraticCostFunction
    : public ceres::SizedCostFunction<1,3> //残差维度，代估计参数的维度
{

private:
    double x;
    double y;

public:
    QuadraticCostFunction(double _x, double _y) : x(_x), y(_y) {}
    virtual ~QuadraticCostFunction() {}

    virtual bool Evaluate(double const *const *parameters,
                          double *residuals,
                          double **jacobians) const
    {
        double a = parameters[0][0];
        double b = parameters[0][1];
        double c = parameters[0][2];
        // f(x) =  y-ceres::exp(a * x * x +b * x + c);
        residuals[0] = y-ceres::exp(a * x * x +b * x + c);

        if (jacobians != NULL && jacobians[0] != NULL)
        {
            jacobians[0][0] = - ceres::exp(a * x * x + b * x + c)*x*x;
            jacobians[0][1] = - ceres::exp(a * x * x + b * x + c)*x;
            jacobians[0][2] = - ceres::exp(a * x * x + b * x + c);
        }

        return true;
    }
};

int main()
{
    double ar = 3.0, br = 2.0, cr = 1.0;  // 真实参数值
    double ae = 2.0, be = -1.0, ce = 5.0; // 估计参数值
    int N = 100;                          // 100个数据点
    double w_sigma = 1.0;                 // 噪声Sigma值
    double inv_sigma = 1.0 / w_sigma;

    cv::RNG rng; // OpenCV随机数产生器

    // 仿真噪声数据
    vector<double> x_data, y_data; // 数据
    for (int i = 0; i < N; i++)
    {
        // x的范围限制在0-1 因为随机点的模为-1~1
        double x = i / (double)N;
        x_data.push_back(x);
        y_data.push_back(exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma * w_sigma));
    }

    //  参数块
    double abc[3] = {ae, be, ce};

    // 构建最小二乘问题
    ceres::Problem problem;

    //反复使用AddResidualBlock方法（逐个散点，反复N次）
    //将每个点的残差累计求和构建最小二乘优化式
    //不使用核函数，待优化参数是abc
    for (int i = 0; i < N; ++i)
    {
        // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
        ceres::CostFunction *cost_function =
            new QuadraticCostFunction(x_data[i], y_data[i]);  
        problem.AddResidualBlock(
            cost_function,
            nullptr,
            abc);
    }

    // 配置求解器
    ceres::Solver::Options options; //这里有许多配置可以填
    // 使用DENSE_NORMAL_CHOLESKY进行增量方程的求解
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY; //增量方程求解的方式
    // 打印每一步的日志到cout
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;            // 优化信息
    ceres::Solve(options, &problem, &summary); // 开始优化
    cout << summary.BriefReport() << endl;     // 输出Ceres迭代报告

    // 输出估计结果
    cout << "estimated a,b,c = ";
    for (auto a : abc)
        cout << a << " ";
}