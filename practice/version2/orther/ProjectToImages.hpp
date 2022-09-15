#pragma once

#include <pdal/Filter.hpp>
#include <pdal/KDIndex.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>

#include <algorithm>
#include "pose.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <utility>
#include <chrono>
namespace pdal
{
  
    class PDAL_DLL ProjectToImages : public Filter
    {
    public:
        ProjectToImages() : Filter()
        {
        }
        std::string getName() const;

    private:
        virtual void addArgs(ProgramArgs &args);
        virtual void initialize();
        virtual void addDimensions(PointLayoutPtr layout);
        virtual void prepared(PointTableRef table);
        virtual void filter(PointView &view);

        ProjectToImages &operator=(const ProjectToImages &); // not implemented
        ProjectToImages(const ProjectToImages &);            // not implemented


    private:

        
        // std::string m_cam_filename; //cam_file
        std::string m_image_file;    //image_dir
        std::string m_image_outfile; 
        // std::string m_imagename_fieldname; 
      
        // StringList m_rotation_fieldnames;
        // double m_radius;
        std::vector<double> m_arg_camera_intrinsic;
        Eigen::Matrix3d m_camera_intrinsic;
        // double m_camera_front;
        // int m_buffer_size;

        int m_image_width;
        int m_image_height;
  
        // std::vector<CameraPose> m_camera_poses;
        // PointViewPtr m_camera_view;
        // KD2Index *m_kdi_ptr;

        // std::vector<std::pair<std::string, cv::Mat>> m_imageurl_mats;
    };

} // namespace pdal
