

#pragma once

#include <pdal/Filter.hpp>
#include <pdal/Streamable.hpp>

#include "pose.h"

namespace pdal
{

    class PDAL_DLL Pose6DTransformation : public Filter, public Streamable
    {
    public:
        Pose6DTransformation() : Filter() {}
        std::string getName() const;

    private:
        virtual void addArgs(ProgramArgs &args);
        virtual void initialize();
        virtual void addDimensions(PointLayoutPtr layout);
        virtual void prepared(PointTableRef table);
        virtual bool processOne(PointRef &point);
        virtual void filter(PointView &view);

        Pose6DTransformation &operator=(const Pose6DTransformation &) = delete; // not implemented
        Pose6DTransformation(const Pose6DTransformation &) = delete;            // not implemented

    private:
        std::vector<double> m_arg_matrix_vals;
        std::string m_corrospondence;

        std::string m_order;
        double m_x_angle;
        double m_y_angle;
        double m_z_angle;
        std::string m_angle_unit;
        double m_translation_x;
        double m_translation_y;
        double m_translation_z;

        StringList m_matrix_dimensions;

        bool m_invert;

        std::string m_multi_order; // left or right

        bool m_invert_result;

        Eigen::Isometry3d m_matrix;
        std::map<std::string, Dimension::Id> m_nameIds;
    };

} // namespace pdal
