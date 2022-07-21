
#pragma once

#include <sophus/se3.hpp>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>

namespace BundleAdjustment
{
    // BA by g2o
    typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;

    typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVector3d;

    void bundleAdjustmentGaussNewton(
        const VecVector3d &points_3d,
        const VecVector2d &points_2d,
        const cv::Mat &K,
        Sophus::SE3d &pose,
        const int &iterations = 10);

    void bundleAdjustmentG2O(
        const VecVector3d &points_3d,
        const VecVector2d &points_2d,
        const cv::Mat &K,
        Sophus::SE3d &pose);

    cv::Point2d pixel2cam(const cv::Point2d &p, const cv::Mat &K);
}
