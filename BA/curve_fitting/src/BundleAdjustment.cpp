
#pragma once

#include <iostream>
#include <chrono>
#include <algorithm>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/linear_solver_dense.h>
#include <g2o/solvers/linear_solver_eigen.h>
#include <g2o/core/optimization_algorithm_dogleg.h>       // invalid    it need to look for reason
#include <g2o/core/optimization_algorithm_levenberg.h>    // valid
#include <g2o/core/optimization_algorithm_gauss_newton.h> // invalid    it need to look for reason

#include "BundleAdjustment.hpp"
#include "CurveFittingVertex.hpp"
#include "CurveFittingEdge.hpp"

namespace BundleAdjustment
{
    void bundleAdjustmentG2O(
        const std::vector<double> &x_data,
        const std::vector<double> &y_data,
        int n, double sigma)
    {
        // 第一种解决方式: 将普通指针强制转换成智能指针 需要注意的是 转化之后 原来的普通指针指向的内容会有变化.
        // 普通指针可以强制转换成智能指针，方式是通过智能指针的一个构造函数来实现的.
        // Block(std::unique_ptr<Block::LinearSolverType>(linearSolver));

        // 这里面就是将linearSolver普通指针作为参数用智能指针构造一个临时的对象，此时原来的普通指针就无效了，一定不要再次用那个指针了，否则会有意想不到的错误，如果还想保留原来的指针.
        // 第二种方式 定义的时候就直接用智能指针就好，但是就如第二种解决方案那样，也会遇到类型转换的问题.
        // g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>());

        // 构建图优化，先设定g2o
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>> Block;                                // 每个误差项优化变量维度为3，误差值维度为1
        Block::LinearSolverType *linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>(); // 线性方程求解器
        Block *solver_ptr = new Block(linearSolver);                                                 // 矩阵块求解器

        // 梯度下降方法，从GN, LM, DogLeg 中选
        auto solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

        g2o::SparseOptimizer optimizer; // 图模型
        optimizer.setAlgorithm(solver); // 设置求解器
        optimizer.setVerbose(true);     // 打开调试输出

        // 往图中增加顶点
        CurveFitting::CurveFittingVertex *v = new CurveFitting::CurveFittingVertex();
        v->setEstimate(Eigen::Vector3d(0, 0, 0));
        v->setId(0);
        optimizer.addVertex(v);

        // 往图中增加边
        for (int i = 0; i < n; ++i)
        {
            CurveFitting::CurveFittingEdge *edge = new CurveFitting::CurveFittingEdge(x_data[i]);
            edge->setId(i);
            edge->setVertex(0, v);                                                               // 设置连接的顶点
            edge->setMeasurement(y_data[i]);                                                     // 观测数值
            edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1 / (sigma * sigma)); // 信息矩阵：协方差矩阵之逆
            optimizer.addEdge(edge);
        }

        // 执行优化
        std::cout << "start optimization" << std::endl;
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        optimizer.initializeOptimization();
        optimizer.optimize(100);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);

        std::cout << "solver time cost = " << time_used.count() << " second." << std::endl;

        // 输出优化值
        Eigen::Vector3d abc_estimate = v->estimate();
        std::cout << "estimated model: " << abc_estimate.transpose() << std::endl;
    }

} // namespace BundleAdjustment