

#pragma once

#include <g2o/core/base_unary_edge.h>
#include "VertexPose.hpp"

class VertexPose;

class EdgeProjection : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPose>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW; // 表示在利用Eigen库的数据结构时new的时候 需要对齐，所以加入EIGEN特有的宏定义即可实现

    EdgeProjection(const Eigen::Vector3d &pos, const Eigen::Matrix3d &K);

    virtual void computeError() override;

    virtual void linearizeOplus() override;

    // 存盘和读盘：留空
    virtual bool read(std::istream &in) override;

    virtual bool write(std::ostream &out) const override;

private:
    Eigen::Vector3d _pos3d;
    Eigen::Matrix3d _K;
};
