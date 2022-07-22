
#include "Vertex.hpp"

void myVertex::setToOriginImpl()
{
    _estimate = Type();
}

void myVertex::oplusImpl(const double *update)
{
    _estimate += /* update */;
}

bool myVertex::read(std::istream &is)
{
    return true;
}

bool myVertex::write(std::ostream &os) const
{
    return true; 
}
