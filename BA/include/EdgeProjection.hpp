

#pragma once

#include <g2o/core/base_unary_edge.h>
#include "VertexPose.hpp"

class VertexPose;

class EdgeProjection : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPose>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeProjection(const Eigen::Vector3d &pos, const Eigen::Matrix3d &K);

    virtual void computeError() override;

    virtual void linearizeOplus() override;

    virtual bool read(std::istream &in) override;

    virtual bool write(std::ostream &out) const override;

private:
    Eigen::Vector3d _pos3d;
    Eigen::Matrix3d _K;
};
