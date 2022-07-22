

#pragma once

// 对于用g2o来进行优化的话，首先要定义顶点和边的模板
// g2o::BaseVertex<D, T>
// D: 描述顶点的状态量(可以理解为优化变量); T: 顶点类型

// VertexSE2 : public BaseVertex<3, SE2>  (2D pose Vertex, (x,y,theta))
// VertexSE3 : public BaseVertex<6, Isometry3>  (6d vector (x,y,z,qx,qy,qz) (note that we leave out the w part of the quaternion))
// VertexPointXY : public BaseVertex<2, Vector2>
// VertexPointXYZ : public BaseVertex<3, Vector3>
// VertexSBAPointXYZ : public BaseVertex<3, Vector3>

/* SE3 Vertex parameterized internally with a transformation matrix and externally with its exponential map */

// VertexSE3Expmap : public BaseVertex<6, SE3Quat>

/* SBACam Vertex, (x,y,z,qw,qx,qy,qz),(x,y,z,qx,qy,qz) (note that we leave out the w part of the quaternion.
   qw is assumed to be positive, otherwise there is an ambiguity in qx,qy,qz as a rotation  */

// VertexCam : public BaseVertex<6, SBACam>

/* Sim3 Vertex, (x,y,z,qw,qx,qy,qz),7d vector,(x,y,z,qx,qy,qz) (note that we leave out the w part of the quaternion. */

// VertexSim3Expmap : public BaseVertex<7, Sim3>

class myVertex : public g2o::BaseVertex<Dim, Type>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW; // 表示在利用Eigen库的数据结构时new的时候需要对齐，所以加入EIGEN特有的宏定义即可实现.

    myVertex() = default;
    ~myVertex() = default;

    virtual void setToOriginImpl() override; // 输入优化变量初始值.
    virtual void oplusImpl(const double *update) override; // 顶点的更新函数: 主要用于优化过程中增量 △x 的计算.

    // 存盘和读盘：留空
    virtual bool read(std::istream &in) override;
    virtual bool write(std::ostream &out) const override;

private:
    myVertex &operator=(const myVertex &) = delete;
    myVertex(const myVertex &) = delete;
};
