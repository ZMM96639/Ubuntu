

#pragma once

// 定义边模板 边也就是误差，二维 并且把顶点也放进去
class myEdge : public g2o::BaseBinaryEdge<errorDim, errorType, Vertex1Type, Vertex2Type>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    myEdge() = default;
    ~myEdge() = default;

    virtual void computeError() override;
    virtual void linearizeOplus() override; // 计算雅克比矩阵，这个函数是可选的，如果给出了则进行解析求导，不给则进行数值求导

    // 存盘和读盘：留空
    virtual bool read(std::istream &in) override;
    virtual bool write(std::ostream &out) override;

private:
    myEdge &operator=(const myEdge &) = delete;
    myEdge(const myEdge &) = delete;

private:
    // data
};
