

#include "PoseEdge.hpp"
#include "PoseEdge.hpp"
#include "BundleAdjustment.hpp"

#include "g2o/core/block_solver.h"
#include "g2o/solvers/linear_solver_dense.h"
#include "g2o/solvers/linear_solver_eigen.h"

#include "g2o/core/optimization_algorithm_dogleg.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"

void BundleAdjustment::optimilizationUsingG2O(const VecVector3d &pt_3d,
                                              const VecVector2d &pt_2d,
                                              const cv::Mat &K, Sophus::SE3d &pose,
                                              const int interations)
{
    if (pt_3d.size() != pt_2d.size() || K.empty())
    {
        std::cout << "Please check the input!" << std::endl;
        exit(0);
    }

    using Block = g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>>;
    Block::LinearSolverType *linearsolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
    Block *solver_ptr = new Block(linearsolver);

    auto solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    Pose::PoseVertex *vertex = new Pose::PoseVertex();
    vertex->setId(0);
    vertex->setEstimate(Sophus::SE3d());
    optimizer.addVertex(vertex);

    Eigen::Matrix3d eigen_K;
    eigen_K << K.at<double>(0, 0), K.at<double>(0, 1), K.at<double>(0, 2),
        K.at<double>(1, 0), K.at<double>(1, 1), K.at<double>(1, 2),
        K.at<double>(2, 0), K.at<double>(2, 1), K.at<double>(2, 2);

    for (int i = 0; i < pt_3d.size(); i++)
    {
        Pose::PoseEdge *edge = new Pose::PoseEdge(pt_3d[i], eigen_K);
        edge->setId(i);
        edge->setVertex(0, vertex);
        edge->setMeasurement(pt_2d[i]);
        edge->setInformation(Eigen::Matrix2d::Identity());
        optimizer.addEdge(edge);
    }

    optimizer.initializeOptimization();
    optimizer.optimize(interations);

    std::cout << "pose estimated by g2o = \n"
              << vertex->estimate().matrix()
              << std::endl;

    pose = vertex->estimate();
}

void BundleAdjustment::optimilizationUsingGaussNewton()
{
}