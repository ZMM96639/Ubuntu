
#pragma once

#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <opencv2/core/core.hpp>

namespace BundleAdjustment
{
    using VecVector3d = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;
    using VecVector2d = std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>;

    void optimilizationUsingG2O(const VecVector3d &pt_3d,
                                const VecVector2d &pt_2d,
                                const cv::Mat &K, Sophus::SE3d &pose,
                                const int interations);

    void optimilizationUsingGaussNewton();
}