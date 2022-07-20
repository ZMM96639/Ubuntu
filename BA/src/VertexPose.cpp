

/// vertex and edges used in g2o ba

#include "VertexPose.hpp"

void VertexPose::setToOriginImpl()
{
    _estimate = Sophus::SE3d();
}

/// left multiplication on SE3
void VertexPose::oplusImpl(const double *update)
{
    Eigen::Matrix<double, 6, 1> update_eigen;
    update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
    _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
}

bool VertexPose::read(std::istream &in)
{
    return true;
}

bool VertexPose::write(std::ostream &out) const
{
    return true;
}