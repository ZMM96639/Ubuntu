

#include "CurveFittingEdge.hpp"

namespace CurveFitting
{
    CurveFittingEdge::CurveFittingEdge(double x) : BaseUnaryEdge(), _x(x) {}

    void CurveFittingEdge::computeError()
    {
        const CurveFittingVertex *v = static_cast<const CurveFittingVertex *>(_vertices[0]);
        const Eigen::Vector3d abc = v->estimate();
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