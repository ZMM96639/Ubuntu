
#include <iostream>
#include "BundleAdjustment.hpp"

int main(int argc, char **agrv)
{
    double a = 1.0, b = 2.0, c = 1.0; // 真实参数值
    int n = 100;                      // 数据点
    double w_sigma = 1.0;             // 噪声Sigma值
    cv::RNG rgn;                      // OpenCV随机数产生器
    double abc[3]{0, 0, 0};           // abc参数的估计值

    std::vector<double> x_data, y_data; // 数据

    std::cout << "generating data:" << std::endl;
    for (int i = 0; i < n; ++i)
    {
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(exp(a * x * x + b * x + c) + rgn.gaussian(w_sigma));
        std::cout << "(x_data[" << std::to_string(i) << "],"
                  << "y_data[" << std::to_string(i) << "]): ("
                  << x_data[i] << "," << y_data[i] << ")" << std::endl;
    }

    BundleAdjustment::bundleAdjustmentG2O(x_data, y_data, n, w_sigma);
    
    return 0;
}