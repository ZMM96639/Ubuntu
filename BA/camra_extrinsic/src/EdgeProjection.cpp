

#include "EdgeProjection.hpp"

EdgeProjection::EdgeProjection(const Eigen::Vector3d &pos, const Eigen::Matrix3d &K) : _pos3d(pos), _K(K) {}

void EdgeProjection::computeError()
{
    const VertexPose *v = static_cast<const VertexPose *>(_vertices[0]);
    Sophus::SE3d T = v->estimate();
    Eigen::Vector3d pos_pixel = _K * (T * _pos3d); // T: 世界坐标系至相机坐标间的变换; _pos3d: 世界坐标系下的3d点
    pos_pixel /= pos_pixel[2];                     // 齐次坐标转换为非齐次坐标
    _error = _measurement - pos_pixel.head<2>();
}

void EdgeProjection::linearizeOplus()
{
    const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
    Sophus::SE3d T = v->estimate();
    Eigen::Vector3d pos_cam = T * _pos3d;
    double fx = _K(0, 0);
    double fy = _K(1, 1);
    double cx = _K(0, 2);
    double cy = _K(1, 2);
    double X = pos_cam[0];
    double Y = pos_cam[1];
    double Z = pos_cam[2];
    double Z2 = Z * Z;

    // 雅克比矩阵见书p187 公式7.46
    _jacobianOplusXi
        << -fx / Z,
        0, fx * X / Z2, fx * X * Y / Z2, -fx - fx * X * X / Z2, fx * Y / Z,
        0, -fy / Z, fy * Y / (Z * Z), fy + fy * Y * Y / Z2, -fy * X * Y / Z2, -fy * X / Z;
}

bool EdgeProjection::read(std::istream &in)
{
    return true;
}

bool EdgeProjection::write(std::ostream &out) const
{
    return true;
}
