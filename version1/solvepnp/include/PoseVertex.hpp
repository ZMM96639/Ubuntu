
#pragma once

#include <iostream>
#include <Eigen/Core>
#include <sophus/se3.hpp>
#include "g2o/core/base_vertex.h"

namespace Pose
{
    class PoseVertex : public g2o::BaseVertex<6, Sophus::SE3d>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        virtual void setToOriginImpl() override;
        virtual void oplusImpl(const double *update) override;

        virtual bool read(std::istream &in) override;
        virtual bool write(std::ostream &out) const override;

        PoseVertex &operator=(const PoseVertex &) = delete;
    };
}