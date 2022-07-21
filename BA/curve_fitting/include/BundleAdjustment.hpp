
#pragma once

#include <sophus/se3.hpp>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>

namespace BundleAdjustment
{
    // BA by g2o
    void bundleAdjustmentG2O(const std::vector<double> &x_data,
                             const std::vector<double> &y_data,
                             int n, double sigma);
}
