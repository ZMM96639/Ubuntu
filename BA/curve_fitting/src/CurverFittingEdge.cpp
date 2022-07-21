

#include "CurveFittingEdge.hpp"

namespace CurveFitting
{
    CurveFittingEdge::CurveFittingEdge(double x) : BaseUnaryEdge(), _x(x) {}

    void CurveFittingEdge::computeError()
    {
        const CurveFittingVertex *v = static_cast<const CurveFittingVertex *>(_vertices[0]);
        const Eigen::Vector3d abc = v->estimate();

        //这里的error是1x1的矩阵，因为误差项就是1个 _measurement是测量值yi
        _error(0, 0) = _measurement - std::exp(abc(0, 0) * _x * _x + abc(1, 0) * _x + abc(2, 0));
    }

    bool CurveFittingEdge::read(std::istream &in)
    {
        return true;
    }

    bool CurveFittingEdge::write(std::ostream &out) const
    {
        return true;
    }
}