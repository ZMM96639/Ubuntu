

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui.hpp>

#include "DrawpointpairOnImage.hpp"

namespace pdal
{
    static PluginInfo const s_info{
        "filters.drawpointpaironimage",
        "drawpointpair on image",
        "http://pdal.io/stages/filters.drawpointpaironimage.html"};

    CREATE_SHARED_STAGE(DrawpointpairOnImage, s_info)

    std::string DrawpointpairOnImage::getName() const
    {
        return s_info.name;
    }

    void DrawpointpairOnImage::addArgs(ProgramArgs &args)
    {
        args.add("image_file", "image_file path", m_img_file).setPositional();
        args.add("image_outfile", "image_outfile path", m_img_outfile).setPositional();

        args.add("camera_intrinsic", "camera 3x3 intrinsic", m_arg_camera_intrinsic).setPositional();
        args.add("lidarTocamera_rotation", "lidarTocamera 3x3 rotation", m_arg_lidarTocamera_rotation).setPositional();
        args.add("lidarTocamera_translation", "lidarTocamera 3x1 translation", m_arg_lidarTocamera_translation).setPositional();

        args.add("pixel_dimension", "pixel coordinate along x and y direction", m_pixel_dimensions);

        args.add("image_width", "image width", m_img_width).setPositional();
        args.add("image_height", "image height", m_img_height).setPositional();
    }

    void DrawpointpairOnImage::initialize()
    {
        // initialize camera intrinsic matrix
        dataInput(m_arg_camera_intrinsic, m_camera_intrinsic);

        // initialize camera extrinsic matrix
        Eigen::Matrix3d m_lidarTocamera_rotation;
        dataInput(m_arg_lidarTocamera_rotation, m_lidarTocamera_rotation);

        Eigen::Vector3d m_lidarTocamera_translation;
        dataInput(m_arg_lidarTocamera_translation, m_lidarTocamera_translation);

        m_trans_cam_veh = dilu::mapping::translationAndRotationMatrixToIsometry3d(m_lidarTocamera_translation, m_lidarTocamera_rotation);
    }

    void DrawpointpairOnImage::addDimensions(PointLayoutPtr layout)
    {
        for (auto &dim_name : m_pixel_dimensions)
        {
            Dimension::Type dim_type = Dimension::Type::Double;
            Dimension::Id dim_id = layout->registerOrAssignDim(dim_name, dim_type);
        }
    }

    void DrawpointpairOnImage::prepared(PointTableRef table)
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

        for (auto &dim_name : m_pixel_dimensions)
        {
            Dimension::Id dim_id = layout->findDim(dim_name);
            if (dim_id == Dimension::Id::Unknown)
            {
                throwError("pixel coordinate " + dim_name + " does not exist!");
            }
            m_nameIds.insert(std::make_pair(dim_name, dim_id));
        }
    }

    void DrawpointpairOnImage::processOne(PointRef &point) {}

    void DrawpointpairOnImage::filter(PointView &view)
    {
        cv::Mat image = cv::imread(m_img_file.c_str(), cv::IMREAD_COLOR);

        for (PointId id = 0; id < view.size(); ++id)
        {
            PointRef point(view, id);

            Eigen::Vector3d veh_pt(point.getFieldAs<double>(m_nameIds["X"]),
                                   point.getFieldAs<double>(m_nameIds["Y"]),
                                   point.getFieldAs<double>(m_nameIds["Z"]));

            Eigen::Vector2d veh_pp(point.getFieldAs<double>(m_nameIds["u"]),
                                   point.getFieldAs<double>(m_nameIds["v"]));

            Eigen::Vector3d camera_pt = m_trans_cam_veh * veh_pt;
            Eigen::Vector3d img_pt = (m_camera_intrinsic * camera_pt) / camera_pt[2];

            double theta_u = veh_pp[0] - img_pt[0];
            double theta_v = veh_pp[1] - img_pt[1];

            point.setField(m_nameIds["theta_u"], theta_u);
            point.setField(m_nameIds["theta_v"], theta_v);

            if (img_pt[0] <= 0 || img_pt[1] <= 0 || img_pt[0] >= m_img_width || img_pt[1] >= m_img_height)
            {
                continue;
            }

            image.at<cv::Vec3b>(img_pt[1], img_pt[0])[0] = 0;
            image.at<cv::Vec3b>(img_pt[1], img_pt[0])[1] = 0;
            image.at<cv::Vec3b>(img_pt[1], img_pt[0])[2] = 255;

            image.at<cv::Vec3b>(veh_pp[1], veh_pp[0])[0] = 0;
            image.at<cv::Vec3b>(veh_pp[1], veh_pp[0])[1] = 255;
            image.at<cv::Vec3b>(veh_pp[1], veh_pp[0])[2] = 0;

            cv::circle(image, cv::Point(img_pt[0], img_pt[1]), 8, cv::Scalar(0, 0, 255), 1, CV_AA, 0);
            cv::circle(image, cv::Point(veh_pp[0], veh_pp[1]), 8, cv::Scalar(0, 255, 0), 1, CV_AA, 0);
        }

        cv::imwrite(m_img_outfile, image);
    }

    void DrawpointpairOnImage::dataInput(std::vector<double> &arg_matrix, Eigen::Matrix3d &matrix)
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

    void DrawpointpairOnImage::dataInput(std::vector<double> &arg_matrix, Eigen::Vector3d &matrix)
    {
        if (arg_matrix.size() != 3)
        {
            throwError(" ' matrix ' option must contain 3 digits.");
        }
        for (int i = 0; i < 3; i++)
        {
            matrix(i) = arg_matrix[i];
        }
    }

    void DrawpointpairOnImage::dataInput(const std::string &arg_matrix, std::vector<Eigen::Vector2d> &matrix)
    {
        std::ifstream f(arg_matrix.c_str());
        while (!f.eof())
        {
            std::string s;
            getline(f, s);
            if (!s.empty())
            {
                std::stringstream ss;
                ss << s;
                Eigen::Vector2d vec2d;
                ss >> vec2d[0] >> vec2d[1];
                matrix.push_back(vec2d);
            }
        }
    }

    void DrawpointpairOnImage::dataOutput(const std::string &arg_matrix, std::vector<Eigen::Vector3d> &matrix)
    {
        std::fstream fs(arg_matrix.c_str(), std::ios::out);
        for (int i = 0; i < matrix.size(); i++)
        {
            for (int j = 0; j < 2; j++)
            {
                fs << matrix[i][j] << " ";
            }
            fs << std::endl;
        }
    }

} // namespace pdal
