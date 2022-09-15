

#pragma once

#include <pdal/Filter.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>

#include "pose.h"

namespace pdal
{
    class PDAL_DLL PcdprojectToImages : public Filter
    {
    public:
        PcdprojectToImages() : Filter() {}
        std::string getName() const;

    private:
        virtual void addArgs(ProgramArgs &args);
        virtual void initialize();
        virtual void addDimensions(PointLayoutPtr layout);
        virtual void prepared(PointTableRef table);
        virtual void filter(PointView &view);

        void dataInput(std::vector<double> &arg_matrix, Eigen::Matrix3d &matrix);
        void dataInput(std::vector<double> &arg_matrix, Eigen::Vector3d &matrix);

        PcdprojectToImages &operator=(const PcdprojectToImages &) = delete;
        PcdprojectToImages(const PcdprojectToImages &) = delete;

    private:
        std::string m_img_file, m_img_outfile;
        std::vector<double> m_arg_camera_intrinsic;
        std::vector<double> m_arg_lidarTocamera_rotation;
        std::vector<double> m_arg_lidarTocamera_translation;

        std::map<std::string, Dimension::Id> m_nameIds;

        Eigen::Matrix3d m_camera_intrinsic;
        Eigen::Isometry3d trans_cam_veh;

        int m_img_width;
        int m_img_height;
    };
}