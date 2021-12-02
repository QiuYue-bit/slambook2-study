/*
 * @Author: Divenire
 * @Date: 2021-10-15 13:46:43
 * @LastEditors: Divenire
 * @LastEditTime: 2021-10-16 15:33:49
 * @Description: Ceres自动求导拟合曲线
 *                  在我的机器上I7-8750h 耗时6.17ms
 */
#include <iostream>
#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>
#include <chrono>
using namespace std;

//代价函数的计算模型
struct CURVE_FITTING_COST
{
    //点 x和 y
    const double _x, _y;

    //    结构体初始化函数 把x赋值给_x
    CURVE_FITTING_COST(double x, double y) : _x(x), _y(y){}


    template <typename T>
    // 重载运算符
    bool operator()(
        const T *const abc, //输入参数是3维的
        T *residual) const{
        // 残差公式 : y-exp(ax^2 + bx +c)
        residual[0] = T(_y) - ceres::exp(abc[0] * T(_x) * T(_x) + abc[1] * T(_x) + abc[2]);
        return true;
    }

};

int main()
{
    double ar = 3.0, br = 2.0, cr = 1.0;    // 真实参数值
    double ae = 2.0, be = -1.0, ce = 5.0;   // 估计参数值
    int N = 100;                            // 100个数据点
    double w_sigma = 1.0;                   // 噪声Sigma值
    double inv_sigma = 1.0 / w_sigma;       


    cv::RNG rng;                            // OpenCV随机数产生器

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
    for (int i = 0; i < N; ++i)
    {
        // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
        ceres::CostFunction *cost_function =
            new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
                new CURVE_FITTING_COST(x_data[i], y_data[i]));

        problem.AddResidualBlock(
            cost_function, 
            nullptr, 
            abc
        );
    }

    // 配置求解器
    ceres::Solver::Options options; //这里有许多配置可以填
    // 使用DENSE_NORMAL_CHOLESKY进行增量方程的求解
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY; //增量方程求解的方式
    // 打印每一步的日志到cout
    options.minimizer_progress_to_stdout = true;


    ceres::Solver::Summary summary;                // 优化信息
    ceres::Solve(options, &problem, &summary);     // 开始优化
    cout << summary.BriefReport() << endl;         // 输出Ceres迭代报告

    // 输出估计结果
    cout << "estimated a,b,c = ";
    for(auto a:abc)
        cout<<a<<" ";
    cout << endl;
}