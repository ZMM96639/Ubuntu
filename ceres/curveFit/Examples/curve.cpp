
#include <iostream>
#include <opencv2/core/core.hpp>

#include "Costfunc.hpp"

int main(int argc, char **argv)
{
    double a = 1.0, b = 2.0, c = 1.0; // 真实参数值
    int n = 100;                      // 数据点
    double w_sigma = 1.0;             // 噪声Sigma值
    cv::RNG rng;                      // OpenCV随机数产生器
    double abc[3]{0, 0, 0};           // abc参数的估计值

    std::vector<double> x_data, y_data;

    std::cout << "generating data: " << std::endl;
    for (int i = 0; i < n; i++)
    {
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(exp(a * x * x + b * x + c) + rng.gaussian(w_sigma));

        std::cout << x_data[i] << " " << y_data[i] << std::endl;
    }

    // 构建最小二乘问题
    ceres::Problem problem;
    for (int i = 0; i < n; i++)
    {
        auto solver = new Costfunc(x_data[i], y_data[i]);

        // 向问题中添加误差项
        // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
        // 核函数，这里不使用，为空
        // 待估计参数
        problem.AddResidualBlock(new ceres::AutoDiffCostFunction<Costfunc, 1, 3>(solver), nullptr, abc);
    }

    // 配置求解器
    ceres::Solver::Options option;
    option.linear_solver_type = ceres::DENSE_QR; // 增量方程如何求解
    option.minimizer_progress_to_stdout = true;  // 输出到 cout

    ceres::Solver::Summary summary;
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    ceres::Solve(option, &problem, &summary);

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);

    std::cout << "solve time cost = " << time_used.count() << " seconds. " << std::endl;

    // 输出结果
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "estimated a,b,c = ";
    for (auto a : abc)
    {
        std::cout << a << " ";
    }

    std::cout << std::endl;

    return 0;
}