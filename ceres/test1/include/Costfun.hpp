

#pragma once

#include <vector>

#include "ceres/ceres.h"

// myCostfun namespace
namespace myCostfun
{
    // tempalte overload operator ()
    struct Costfun
    {
        template <typename T>
        bool operator()(const T *const x, T *residual) const
        {
            residual[0] = 10.0 - x[0];
            return true;
        }
    };

    struct F1
    {
        template <typename T>
        bool operator()(const T *const x1, const T *const x2, T *residual) const
        {
            // f1 = x1 + 10 * x2;
            residual[0] = x1[0] + 10.0 * x2[0];
            return true;
        }
    };

    struct F2
    {
        template <typename T>
        bool operator()(const T *const x3, const T *const x4, T *residual) const
        {
            // f2 = sqrt(5) (x3 - x4)
            residual[0] = sqrt(5.0) * (x3[0] - x4[0]);
            return true;
        }
    };

    struct F3
    {
        template <typename T>
        bool operator()(const T *const x2, const T *const x3, T *residual) const
        {
            // f3 = (x2 - 2 x3)^2
            residual[0] = (x2[0] - 2.0 * x3[0]) * (x2[0] - 2.0 * x3[0]);
            return true;
        }
    };

    struct F4
    {
        template <typename T>
        bool operator()(const T *const x1, const T *const x4, T *residual) const
        {
            residual[0] = sqrt(10.0) * (x1[0] - x4[0]) * (x1[0] - x4[0]);
            return true;
        }
    };

    struct ExponentialResidual
    {
        ExponentialResidual(double x, double y) : x_(x), y_(y) {}
        template <typename T>
        bool operator()(const T *const m, const T *const c, T *residual) const
        {
            residual[0] = y_ - exp(m[0] * x_ + c[0]);
            return true;
        }

    private:
        const double x_;
        const double y_;
    };
}
