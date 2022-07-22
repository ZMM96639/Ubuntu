
#pragma once
#include <map>

#include <pdal/util/ProgramArgs.hpp>
#include <pdal/Filter.hpp>
#include <pdal/Streamable.hpp>
#include <array>
#include <string>

#include "pose.h"

namespace pdal
{

    class PDAL_DLL EulerConvert : public Filter, public Streamable
    {
    public:
        EulerConvert() : Filter() {}
        std::string getName() const;

    private:
        virtual void addArgs(ProgramArgs &args);
        virtual void initialize();
        virtual void addDimensions(PointLayoutPtr layout);
        virtual void prepared(PointTableRef table);
        virtual bool processOne(PointRef &point);
        virtual void filter(PointView &view);

        void eulerAngleToRotationMatrix(PointRef &point);
        void rotationMatrixToEulerAngle(PointRef &point);

        EulerConvert &operator=(const EulerConvert &); // not implemented
        EulerConvert(const EulerConvert &);            // not implemented

    private:
        std::string m_conversion;
        std::string m_order;

        std::string m_x_angle_dimension;
        std::string m_y_angle_dimension;
        std::string m_z_angle_dimension;
        
        StringList m_matrix_dimensions;
        std::string m_angle_unit;
 
        std::map<std::string, Dimension::Id> m_nameIds;


    };

} // namespace pdal
