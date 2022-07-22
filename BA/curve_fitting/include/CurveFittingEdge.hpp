

#pragma once

#include <Eigen/Core>

#include "CurveFittingVertex.hpp"
#include "g2o/core/base_unary_edge.h"

namespace CurveFitting
{
    // 误差模型 模板参数：D 观测值维度，类型 连接顶点类型
    class CurveFittingEdge : public g2o::BaseUnaryEdge<1, double, CurveFittingVertex>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW; // 表示在利用Eigen库的数据结构时new的时候 需要对齐，所以加入EIGEN特有的宏定义即可实现

        CurveFittingEdge() = default;
        CurveFittingEdge(double x);

        ~CurveFittingEdge() = default;

    public:
        // 计算曲线模型误差
        void computeError();

        virtual bool read(std::istream &in) override;
        virtual bool write(std::ostream &out) const override;

    private:
        CurveFittingEdge &operator=(const CurveFittingEdge &) = delete;
        CurveFittingEdge(const CurveFittingEdge &) = delete;

    public:
        double _x;
    };
}