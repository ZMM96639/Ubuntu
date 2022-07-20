

#pragma once

#include <iostream>
#include <sophus/se3.hpp>
#include <g2o/core/base_vertex.h>

class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual void setToOriginImpl() override;

    /// left multiplication on SE3
    virtual void oplusImpl(const double *update) override;

    virtual bool read(std::istream &in) override;

    virtual bool write(std::ostream &out) const override;
};