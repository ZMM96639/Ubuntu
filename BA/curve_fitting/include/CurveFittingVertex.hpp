
#pragma once

#include <iostream>
#include <Eigen/Core>
#include "g2o/core/base_vertex.h"

namespace CurveFitting
{
    // 曲线模型的顶点，模板参数：优化变量维度和数据类型
    class CurveFittingVertex : public g2o::BaseVertex<3, Eigen::Vector3d>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW; // 表示在利用Eigen库的数据结构时new的时候 需要对齐，所以加入EIGEN特有的宏定义即可实现

        CurveFittingVertex() = default;
        ~CurveFittingVertex() = default;

    public:
        virtual void setToOriginImpl() override;               // 重置
        virtual void oplusImpl(const double *update) override; // 更新

        // 存盘和读盘：留空
        virtual bool read(std::istream &in) override;
        virtual bool write(std::ostream &out) const override;

    private:
        CurveFittingVertex &operator=(const CurveFittingVertex &) = delete;
        CurveFittingVertex(const CurveFittingVertex &) = delete;
    };

}