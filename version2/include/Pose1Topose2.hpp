

#pragma once

#include <pdal/Filter.hpp>
#include <pdal/Streamable.hpp>

#include "pose.h"
#include <map>

namespace pdal
{
    class PDAL_DLL Pose1Topose2 : public Filter
    {
    public:
        Pose1Topose2() : Filter() {}
        std::string getName() const;

    private:
        virtual void addArgs(ProgramArgs &args) override;
        virtual void initialize() override;

        virtual void addDimensions(PointLayoutPtr layout) override;
        virtual void processOne(PointRef &point);
        virtual void prepared(PointTableRef table) override;

        virtual void filter(PointView &view) override;

        void dataInput(std::vector<double> &matrix_vals, Eigen::Isometry3d &matrix, bool invert);

        Pose1Topose2 &operator=(const Pose1Topose2 &) = delete;
        Pose1Topose2(const Pose1Topose2 &) = delete;

    private:
        std::string m_data_outfile;
        std::vector<double> m_arg_groundtruth_vals;
        bool m_invert;

        Eigen::Isometry3d m_groundtruth;
        Eigen::Isometry3d m_error;

        StringList m_matrix_dimensions;
        std::string m_angle_unit;

        double x_axisd_angle = 0;
        double y_axisd_angle = 0;
        double z_axisd_angle = 0;

        std::map<std::string, Dimension::Id> m_nameIds;
    };
} // namespace pdal
