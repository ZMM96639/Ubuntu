#pragma once

#include <pdal/Filter.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>

#include "pose.h"

namespace pdal
{
    class PDAL_DLL Err_reprojection : public Filter
    {
    public:
        Err_reprojection() : Filter() {}
        std::string getName() const;

    private:
        virtual void addArgs(ProgramArgs &args);
        virtual void initialize();
        virtual void addDimensions(PointLayoutPtr layout);
        virtual void prepared(PointTableRef table);
        virtual void filter(PointView &view);

        void dataInput(std::vector<double> &arg_matrix, Eigen::Matrix3d &matrix);
        void dataInput(std::vector<double> &arg_matrix, Eigen::Vector3d &matrix);

        Err_reprojection &operator=(const Err_reprojection &) = delete; // not implemented
        Err_reprojection(const Err_reprojection &) = delete;            // not implemented

    private:
        std::string m_image_file;
        std::string m_image_outfile;

        std::vector<double> m_arg_camera_intrinsic;
        std::vector<double> m_arg_lidarTocamera_rotation;
        std::vector<double> m_arg_lidarTocamera_translation;
        Eigen::Matrix3d m_camera_intrinsic;
        Eigen::Isometry3d trans_cam_veh;

        int m_image_width;
        int m_image_height;
    };

} // namespace pdal
