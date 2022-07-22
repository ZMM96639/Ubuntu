
#include "Edge.hpp"

void myEdge::computeError()
{
    _error = _measurement - Something;
}

void myEdge::linearizeOplus()
{
    _jacobianOplusXi(pose, pose) = Something;
    _jacobianOplusXj(pose, pose) = Something;
}