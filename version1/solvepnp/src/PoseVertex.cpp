
#include "PoseVertex.hpp"

void Pose::PoseVertex::setToOriginImpl()
{
    _estimate = Sophus::SE3d();
}

void Pose::PoseVertex::oplusImpl(const double *update)
{
    Eigen::Matrix<double, 6, 1> update_eigen;
    update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
    _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
}

bool Pose::PoseVertex::read(std::istream &in)
{
    return true;
}

bool Pose::PoseVertex::write(std::ostream &out) const
{
    return true;
}