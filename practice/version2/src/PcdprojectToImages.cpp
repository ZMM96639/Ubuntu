
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "PcdprojectToImages.hpp"

namespace pdal
{
    static PluginInfo const s_info{
        "filters.pcdprojecttoimages",
        "project pointcloud point to image pixel",
        "http://pdal.io/stages/filters.pcdprojecttoimages.html"};

    CREATE_SHARED_STAGE(PcdprojectToImages, s_info)

    std::string PcdprojectToImages::getName() const
    {
        return s_info.name;
    }

    void PcdprojectToImages::addArgs(ProgramArgs &args)
    {
        args.add("image_file", "image_file path", m_img_file).setPositional();
        args.add("image_outfile", "image_outfile path", m_img_outfile).setPositional();

        args.add("camera_intrinsic", "camera 3x3 intrinsic", m_arg_camera_intrinsic).setPositional();
        args.add("lidarTocamera_rotation", "lidarTocamera 3x3 rotation", m_arg_lidarTocamera_rotation).setPositional();
        args.add("lidarTocamera_translation", "lidarTocamera 3x1 translation", m_arg_lidarTocamera_translation).setPositional();

        args.add("image_width", "image width", m_img_width).setPositional();
        args.add("image_height", "image height", m_img_height).setPositional();
    }

    void PcdprojectToImages::initialize()
    {
        // initialize camera intrinsic matrix
        dataInput(m_arg_camera_intrinsic, m_camera_intrinsic);

        // initialize camera extrinsic matrix
        Eigen::Matrix3d m_lidarTocamera_rotation;
        dataInput(m_arg_lidarTocamera_rotation, m_lidarTocamera_rotation);

        Eigen::Vector3d m_lidarTocamera_translation;
        dataInput(m_arg_lidarTocamera_translation, m_lidarTocamera_translation);

        trans_cam_veh = dilu::mapping::translationAndRotationMatrixToIsometry3d(m_lidarTocamera_translation, m_lidarTocamera_rotation);
    }

    void PcdprojectToImages::addDimensions(PointLayoutPtr layout)
    {
    }

    void PcdprojectToImages::prepared(PointTableRef table)
    {
        PointLayoutPtr layout(table.layout());

        if (layout->findDim("X") == Dimension::Id::Unknown ||
            layout->findDim("Y") == Dimension::Id::Unknown ||
            layout->findDim("Z") == Dimension::Id::Unknown)
        {
            throwError("Dimension X Y Z does not all exist.");
        }

        m_nameIds.insert(std::make_pair("X", Dimension::Id::X));
        m_nameIds.insert(std::make_pair("Y", Dimension::Id::Y));
        m_nameIds.insert(std::make_pair("Z", Dimension::Id::Z));
    }

    void PcdprojectToImages::filter(PointView &view)
    {
        cv::Mat image = cv::imread(m_img_file.c_str(), cv::IMREAD_COLOR);
        for (PointId id = 0; id < view.size(); ++id)
        {
            PointRef point(view, id);

            Eigen::Vector3d veh_pt(point.getFieldAs<double>(m_nameIds["X"]),
                                   point.getFieldAs<double>(m_nameIds["Y"]),
                                   point.getFieldAs<double>(m_nameIds["Z"]));

            Eigen::Vector3d camera_pt = trans_cam_veh * veh_pt;
            Eigen::Vector3d img_pt = (m_camera_intrinsic * camera_pt) / camera_pt[2];

            if (img_pt[0] <= 0 || img_pt[1] <= 0 || img_pt[0] >= m_img_width || img_pt[1] >= m_img_height)
            {
                continue;
            }

            image.at<cv::Vec3b>(img_pt[1], img_pt[0])[0] = 0;
            image.at<cv::Vec3b>(img_pt[1], img_pt[0])[1] = 0;
            image.at<cv::Vec3b>(img_pt[1], img_pt[0])[2] = 255;

            cv::circle(image, cv::Point(img_pt[0], img_pt[1]), 1, cv::Scalar(0, 255, 0), 0.5, CV_AA, 0);

            cv::imwrite(m_img_outfile, image);
        }
    }

    void PcdprojectToImages::dataInput(std::vector<double> &arg_matrix, Eigen::Matrix3d &matrix)
    {
        if (arg_matrix.size() != 9)
        {
            throwError(" ' matrix ' option must contain 9 digits.");
        }

        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                int k = i * 3 + j;
                matrix(i, j) = arg_matrix[k];
            }
        }
    }

    void PcdprojectToImages::dataInput(std::vector<double> &arg_matrix, Eigen::Vector3d &matrix)
    {
        for (int i = 0; i < 3; i++)
        {
            matrix(i) = arg_matrix[i];
        }
    }
} // namespace pdal
