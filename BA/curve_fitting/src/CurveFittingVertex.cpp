

#include "CurveFittingVertex.hpp"

namespace CurveFitting
{

    void CurveFittingVertex::setToOriginImpl()
    {
        _estimate << 0, 0, 0;
    }

    void CurveFittingVertex::oplusImpl(const double *update)
    {
        _estimate += Eigen::Vector3d(update);
    }

    bool CurveFittingVertex::read(std::istream &in)
    {
        return true;
    }

    bool CurveFittingVertex::write(std::ostream &out) const
    {
        return true;
    }

}