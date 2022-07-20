

#include <iostream>
#include <chrono>
#include <algorithm>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/linear_solver_dense.h>
#include <g2o/solvers/linear_solver_eigen.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>

#include "BundleAdjustment.hpp"
#include "EdgeProjection.hpp"

namespace BundleAdjustment
{
    void bundleAdjustmentGaussNewton(
        const VecVector3d &points_3d,
        const VecVector2d &points_2d,
        const cv::Mat &K,
        Sophus::SE3d &pose)
    {
        typedef Eigen::Matrix<double, 6, 1> Vector6d;
        const int iterations = 10;
        double cost = 0, lastCost = 0;
        double fx = K.at<double>(0, 0);
        double fy = K.at<double>(1, 1);
        double cx = K.at<double>(0, 2);
        double cy = K.at<double>(1, 2);
        for (int iter = 0; iter < iterations; iter++)
        {
            Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
            Vector6d b = Vector6d::Zero();
            cost = 0;
            // compute cost
            for (int num = 0; num < points_3d.size(); num++)
            {
                Eigen::Vector3d pc = pose * points_3d[num]; // coordinate:world to camera
                double inv_z = 1.0 / pc[2];
                double inv_z2 = inv_z * inv_z;
                Eigen::Vector2d proj(fx * pc[0] / pc[2] + cx, fy * pc[1] / pc[2] + cy);
                Eigen::Vector2d e = points_2d[num] - proj;
                cost += e.squaredNorm();
                Eigen::Matrix<double, 2, 6> J;
                J << -fx * inv_z,
                    0,
                    fx * pc[0] * inv_z2,
                    fx * pc[0] * pc[1] * inv_z2,
                    -fx - fx * pc[0] * pc[0] * inv_z2,
                    fx * pc[1] * inv_z,
                    0,
                    -fy * inv_z,
                    fy * pc[1] * inv_z2,
                    fy + fy * pc[1] * pc[1] * inv_z2,
                    -fy * pc[0] * pc[1] * inv_z2,
                    -fy * pc[0] * inv_z;
                H += J.transpose() * J;
                b += -J.transpose() * e;
            }
            Vector6d dx;
            dx = H.ldlt().solve(b);
            if (std::isnan(dx[0]))
            {
                std::cout << "result is nan!" << std::endl;
                break;
            }
            if (iter > 0 && cost >= lastCost)
            {
                // cost increase, update is not good
                std::cout << "cost: " << cost << ", last cost: " << lastCost << std::endl;
                break;
            }
            // update your estimation
            pose = Sophus::SE3d::exp(dx) * pose;
            lastCost = cost;

            std::cout << "iteration " << iter << " cost=" << std::setprecision(12) << cost << std::endl;
            if (dx.norm() < 1e-6)
            {
                // converge
                break;
            }
        }
        std::cout << "pose by g-n: \n"
                  << pose.matrix() << std::endl;
    }

    void bundleAdjustmentG2O(
        const VecVector3d &points_3d,
        const VecVector2d &points_2d,
        const cv::Mat &K,
        Sophus::SE3d &pose)
    {
        // 初始化g2o
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> Block; // pose维度为 6, landmark 维度为 3

        Block::LinearSolverType *linearSolver = new g2o::LinearSolverEigen<Block::PoseMatrixType>(); // 线性方程求解器

        Block *solver_ptr = new Block(linearSolver); // 矩阵块求解器

        // 梯度下降方法，可以从GN, LM, DogLeg 中选
        auto solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

        g2o::SparseOptimizer optimizer; // 图模型

        optimizer.setAlgorithm(solver); // 设置求解器
        optimizer.setVerbose(true);     // 打开调试输出

        // vertex // camera vertex_pose
        VertexPose *vertex_pose = new VertexPose();

        vertex_pose->setId(0);
        vertex_pose->setEstimate(Sophus::SE3d());
        optimizer.addVertex(vertex_pose);

        // K
        Eigen::Matrix3d K_eigen;

        K_eigen << K.at<double>(0, 0), K.at<double>(0, 1), K.at<double>(0, 2),
            K.at<double>(1, 0), K.at<double>(1, 1), K.at<double>(1, 2),
            K.at<double>(2, 0), K.at<double>(2, 1), K.at<double>(2, 2);

        // edges
        int index = 8;
        for (size_t i = 0; i < points_2d.size(); ++i)
        {
            auto p2d = points_2d[i];
            auto p3d = points_3d[i];

            EdgeProjection *edge = new EdgeProjection(p3d, K_eigen);

            edge->setId(index);
            edge->setVertex(0, vertex_pose);
            edge->setMeasurement(p2d);
            edge->setInformation(Eigen::Matrix2d::Identity());

            optimizer.addEdge(edge);
            index++;
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        optimizer.setVerbose(true);
        optimizer.initializeOptimization();
        optimizer.optimize(10);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);

        std::cout << "optimization costs time: " << time_used.count() << " seconds." << std::endl;

        std::cout << "pose estimated by g2o =\n"
                  << vertex_pose->estimate().matrix() << std::endl;

        pose = vertex_pose->estimate();
    }

    // 像素坐标转相机归一化坐标
    cv::Point2d pixel2cam(const cv::Point2d &p, const cv::Mat &K)
    {
        return cv::Point2d((p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
                           (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1));
    }

} // namespace BundleAdjustment
