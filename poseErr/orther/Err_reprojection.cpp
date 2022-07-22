

#include "Err_reprojection.hpp"
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// TODO:目前为吉利标注测试临时版本，后面需把cam_lidar参数提出来。
namespace pdal
{

    static PluginInfo const s_info{
        "filters.err_reprojection",
        "project pointcloud point to image pixel",
        "http://pdal.io/stages/filters.ProjectToImages.html"};

    CREATE_SHARED_STAGE(Err_reprojection, s_info)

    std::string Err_reprojection::getName() const
    {
        return s_info.name;
    }

    void Err_reprojection::addArgs(ProgramArgs &args)
    {

        args.add("image_file", "image file", m_image_file).setPositional();
        args.add("image_outfile", "out file", m_image_outfile).setPositional();

        args.add("camera_intrinsic", "camera 3x3 intrinsic", m_arg_camera_intrinsic).setPositional();
        args.add("lidarTocamera_rotation", "lidarTocamera 3x3 rotation", m_arg_lidarTocamera_rotation).setPositional();
        args.add("lidarTocamera_translation", "lidarTocamera 3x1 translation", m_arg_lidarTocamera_translation).setPositional();

        args.add("image_width", "image height", m_image_width).setPositional();
        args.add("image_height", "image width", m_image_height).setPositional();
    }

    void Err_reprojection::initialize()
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

    void Err_reprojection::addDimensions(PointLayoutPtr layout)
    {
    }

    void Err_reprojection::prepared(PointTableRef table)
    {
        PointLayoutPtr layout(table.layout());

        if (layout->findDim("X") == Dimension::Id::Unknown ||
            layout->findDim("Y") == Dimension::Id::Unknown ||
            layout->findDim("Z") == Dimension::Id::Unknown)
            throwError("Dimension X Y Z does not all exist.");
    }

    void Err_reprojection::filter(PointView &view)
    {

        cv::Mat image = cv::imread(m_image_file.c_str(), cv::IMREAD_COLOR);

        for (PointId id = 0; id < view.size(); ++id)
        {
            PointRef point(view, id);

            Eigen::Vector3d veh_pt(point.getFieldAs<double>(pdal::Dimension::Id::X),
                                   point.getFieldAs<double>(pdal::Dimension::Id::Y),
                                   point.getFieldAs<double>(pdal::Dimension::Id::Z));

            // caluate camera frame pt
            Eigen::Vector3d camera_pt = trans_cam_veh * veh_pt;

            // projection camera point to image pt
            Eigen::Vector3d image_pt = (m_camera_intrinsic * camera_pt) / camera_pt[2];

            // judge  pt wheather on image screen
            // if (image_pt[0] <= 0 || image_pt[1] <= 0 || image_pt[0] >= m_image_width || image_pt[1] >= m_image_height)
            // {
            //     continue;
            // }

            // image.at<cv::Vec3b>(image_pt[1], image_pt[0])[0] = 0;   // b
            // image.at<cv::Vec3b>(image_pt[1], image_pt[0])[1] = 255; // g
            // image.at<cv::Vec3b>(image_pt[1], image_pt[0])[2] = 0;   // r
            // cv::circle(image, cv::Point(image_pt[0], image_pt[1]), 10, cv::Scalar(0, 255, 0), 2, CV_AA, 0);
        }

        cv::imwrite(m_image_outfile.c_str(), image);
    }

    void Err_reprojection::dataInput(std::vector<double> &arg_matrix, Eigen::Matrix3d &matrix)
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

    void Err_reprojection::dataInput(std::vector<double> &arg_matrix, Eigen::Vector3d &matrix)
    {
        for (int i = 0; i < 3; i++)
        {
            matrix(i) = arg_matrix[i];
        }
    }

} // namespace pdal
