

#include "PoseEdge.hpp"

Pose::PoseEdge::PoseEdge(const Eigen::Vector3d &pose, const Eigen::Matrix3d &K) : m_pose3d(pose), m_K(K) {}

void Pose::PoseEdge::computeError()
{
    const Pose::PoseVertex *v = static_cast<const Pose::PoseVertex *>(_vertices[0]);
    const Sophus::SE3d T = v->estimate();

    Eigen::Vector3d pose_pixel = m_K * (T * m_pose3d);
    pose_pixel /= pose_pixel[2];

    _error = _measurement - pose_pixel.head<2>();
}

void Pose::PoseEdge::linearizeOplus()
{
    const Pose::PoseVertex *v = static_cast<const Pose::PoseVertex *>(_vertices[0]);
    const Sophus::SE3d T = v->estimate();

    Eigen::Vector3d pose_cam = T * m_pose3d;

    double fx = m_K(0, 0);
    double fy = m_K(1, 1);
    double cx = m_K(0, 2);
    double cy = m_K(1, 2);

    double X = pose_cam[0];
    double Y = pose_cam[1];
    double Z = pose_cam[2];
    double Z2 = Z * Z;

    _jacobianOplusXi << -fx / Z, 0, fx * X / Z2, fx * X * Y / Z2, -fx - fx * X * X / Z2, fx * Y / Z,
        0, -fy * Z, fy * Y / Z2, fy + fy * Y * Y / Z2, -fy * X * Y / Z2, -fy * X / Z;
}

bool Pose::PoseEdge::read(std::istream &in)
{
    return true;
}

bool Pose::PoseEdge::write(std::ostream &out) const
{
    return true;
}