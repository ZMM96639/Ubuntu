
#pragma once

#include "g2o/core/base_unary_edge.h"
#include "PoseVertex.hpp"

namespace Pose
{
    class PoseEdge : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, Pose::PoseVertex>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        PoseEdge(const Eigen::Vector3d &pose, const Eigen::Matrix3d &K);

        virtual void computeError() override;
        virtual void linearizeOplus() override;

        virtual bool read(std::istream &in) override;
        virtual bool write(std::ostream &out) const override;

        PoseEdge &operator=(const PoseEdge &) = delete;
        PoseEdge(const PoseEdge &) = delete;

    private:
        Eigen::Vector3d m_pose3d;
        Eigen::Matrix3d m_K;
    };
}