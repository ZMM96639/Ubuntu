
#pragma once

#include <ceres/ceres.h>

// 使用Ceres求解非线性优化问题，一共分为三个部分：
// 1. 第一部分：构建cost fuction，即代价函数，也就是寻优的目标式。这个部分需要使用仿函数（functor）这一技巧来实现.
//    做法是定义一个cost function的结构体，在结构体内重载（）运算符(这一部分会涉及到具体的函数形式，即通常的误惨差求解函数).
// 2. 第二部分：通过上一步的代价函数构建待求解的优化问题。
// 3. 第三部分：配置求解器参数并求解问题，这个步骤就是设置方程怎么求解、求解过程是否输出等，然后调用一下Solve方法.

struct Costfunc
{
    Costfunc(double x, double y) : m_x(x), m_y(y) {}

    // abc: 模型参数，有3维; residual: 残差
    template <typename T>
    bool operator()(const T *const abc, T *residual) const
    {
        residual[0] = T(m_y) - ceres::exp(abc[0] * T(m_x) * T(m_x) + abc[1] * T(m_x) + abc[2]); // y-exp(ax^2+bx+c)
        return true;
    }

private:
    const double m_x, m_y;
};
