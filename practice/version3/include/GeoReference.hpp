
#pragma once

#include <pdal/util/ProgramArgs.hpp>
#include <pdal/Filter.hpp>
#include <pdal/Streamable.hpp>
#include <array>
#include <string>

#include "isometry3d_interpolation_buffer.h"
#include "ogr_conversions.h"
#include "pose.h"
namespace pdal
{

    class PDAL_DLL GeoReference : public Filter, public Streamable
    {
    public:
        GeoReference() : Filter() {}
        std::string getName() const;

    private:
        virtual void addArgs(ProgramArgs &args);
        virtual void initialize();
        virtual void prepared(PointTableRef table);
        virtual bool processOne(PointRef &point);
        virtual void filter(PointView &view);
        virtual void addDimensions(PointLayoutPtr layout);

        GeoReference &operator=(const GeoReference &); // not implemented
        GeoReference(const GeoReference &);            // not implemented
    private:
        bool initIsometry3dBufferFromFile();
       

    private:
   
        std::string m_tragictory_filename;
        StringList m_matrix_dimensions;
        bool m_skip;        //whether point that look up gps_time failure

        dilu::mapping::Isometry3dInterpolationBufferWithGeo m_buffer;
    };

} // namespace pdal
