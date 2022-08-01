

#pragma once

#include <pdal/Filter.hpp>

#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>

#include "pose.h"

namespace pdal
{
    class PDAL_DLL DrawpointpairOnImage : public Filter
    {
    public:
        DrawpointpairOnImage() : Filter() {}
        std::string getName() const;

    private:
        virtual void addArgs(ProgramArgs &args);
        virtual void initialize();
        virtual void addDimensions(PointLayoutPtr layout);
        virtual void prepared(PointTableRef table);
        virtual void filter(PointView &view);

        void dataInput(std::vector<double> &arg_matrix, Eigen::Matrix3d &matrix);
        void dataInput(std::vector<double> &arg_matrix, Eigen::Vector3d &matrix);
        void dataInput(const std::string &arg_matrix, std::vector<Eigen::Vector2d> &matrix);

        void dataOutput(const std::string &arg_matrix, std::vector<Eigen::Vector3d> &matrix);

        DrawpointpairOnImage &operator=(const DrawpointpairOnImage &) = delete;
        DrawpointpairOnImage(const DrawpointpairOnImage &) = delete;

    private:
        std::string m_img_file, m_img_outfile;

        std::string m_ppoint_file;
        std::string m_ppoint_outfile;

        std::vector<double> m_arg_camera_intrinsic;
        std::vector<double> m_arg_lidarTocamera_rotation;
        std::vector<double> m_arg_lidarTocamera_translation;

        Eigen::Matrix3d m_camera_intrinsic;
        Eigen::Isometry3d m_trans_cam_veh;

        std::vector<Eigen::Vector2d> m_ppoint, Err_rep;
        std::vector<Eigen::Vector3d> m_outppoint;

        int m_img_width;
        int m_img_height;
    };
}