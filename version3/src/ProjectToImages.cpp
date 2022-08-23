#include "ProjectToImages.hpp"

// TODO:目前为吉利标注测试临时版本，后面需把cam_lidar参数提出来。
namespace pdal
{

    static PluginInfo const s_info{
        "filters.projecttoimages",
        "project pointcloud point to image pixel",
        "http://pdal.io/stages/filters.ProjectToImages.html"};

    CREATE_SHARED_STAGE(ProjectToImages, s_info)

    std::string ProjectToImages::getName() const
    {
        return s_info.name;
    }

    void ProjectToImages::addArgs(ProgramArgs &args)
    {

        args.add("image_file", "image file  ", m_image_file).setPositional();
        args.add("image_outfile", "out  file  ", m_image_outfile).setPositional();

        args.add("camera_intrinsic", "camera 3*3 intrinsic", m_arg_camera_intrinsic).setPositional();
        args.add("image_width", " image height ", m_image_width).setPositional();

        args.add("image_height", " image width  ", m_image_height).setPositional();
    }

    void ProjectToImages::initialize()
    {

        // initialize camera intrinsic matrix
        if (m_arg_camera_intrinsic.size() != 9)
            throwError(" ' camera_intrinsic ' option must contain 9 digits.");
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
            {
                int k = i * 3 + j;
                m_camera_intrinsic(i, j) = m_arg_camera_intrinsic[k];
            }
    }

    void ProjectToImages::addDimensions(PointLayoutPtr layout)
    {
    }

    void ProjectToImages::prepared(PointTableRef table)
    {
        PointLayoutPtr layout(table.layout());

        if (layout->findDim("X") == Dimension::Id::Unknown ||
            layout->findDim("Y") == Dimension::Id::Unknown ||
            layout->findDim("Z") == Dimension::Id::Unknown)
            throwError("Dimension X Y Z does not all exist.");
    }

    void ProjectToImages::filter(PointView &view)
    {

        cv::Mat image = cv::imread(m_image_file.c_str(), cv::IMREAD_COLOR);

        for (PointId id = 0; id < view.size(); ++id)
        {
            PointRef point(view, id);

            // Eigen::Vector3d lidar_pt(-5, -5, -0.51);

            // //lidar -veh
            // Eigen::Vector3d translation_lidar_veh(3.9787, -0.247104, 0.663931);
            // Eigen::Matrix3d rotation_lidar_veh;
            // rotation_lidar_veh << 0.0498362, -0.998757, 8.00672e-05, 0.998757, 0.0498363, 0.000576385, -0.000579659, 5.12429e-05, 1.0;
            // Eigen::Isometry3d trans_lidar_veh = dilu::mapping::translationAndRotationMatrixToIsometry3d(translation_lidar_veh, rotation_lidar_veh);

            // //veh pt
            // Eigen::Vector3d veh_pt=trans_lidar_veh*lidar_pt;

            // lidar -veh
            //  Eigen::Vector3d translation_lidar_veh(3.9787, -0.247104, 0.663931);
            //  Eigen::Matrix3d rotation_lidar_veh;
            //  rotation_lidar_veh << 0.0498362, -0.998757, 8.00672e-05, 0.998757, 0.0498363, 0.000576385, -0.000579659, 5.12429e-05, 1.0;
            //  Eigen::Isometry3d trans_lidar_veh = dilu::mapping::translationAndRotationMatrixToIsometry3d(translation_lidar_veh, rotation_lidar_veh);

            Eigen::Vector3d veh_pt(point.getFieldAs<double>(pdal::Dimension::Id::X),
                                   point.getFieldAs<double>(pdal::Dimension::Id::Y),
                                   point.getFieldAs<double>(pdal::Dimension::Id::Z));

            //  Eigen::Vector3d veh_pt(-13.81,-0.26,0.01);

            Eigen::Vector3d translation_cam_veh(-0.55105289999999996, 0.85853689999999006, -5.4233028999999995);
            Eigen::Matrix3d rotation_cam_veh;
            rotation_cam_veh << -0.122404, 0.991573, -0.042429, -0.056600, -0.049655, -0.997161, -0.990865, -0.119655, 0.062201;
            Eigen::Isometry3d trans_cam_veh = dilu::mapping::translationAndRotationMatrixToIsometry3d(translation_cam_veh, rotation_cam_veh);

            // caluate camera frame pt
            Eigen::Vector3d camera_pt = trans_cam_veh * veh_pt;

            // projection camera point to image pt
            Eigen::Vector3d image_pt = (m_camera_intrinsic * camera_pt) / camera_pt[2];

            // judge  pt wheather on image screen
            if (image_pt[0] <= 0 || image_pt[1] <= 0 || image_pt[0] >= m_image_width || image_pt[1] >= m_image_height)
            {
                continue;
            }

            image.at<cv::Vec3b>(image_pt[1], image_pt[0])[0] = 0;   // b
            image.at<cv::Vec3b>(image_pt[1], image_pt[0])[1] = 255; // g
            image.at<cv::Vec3b>(image_pt[1], image_pt[0])[2] = 0;   // r
            // cv::circle(image, cv::Point( image_pt[0],image_pt[1]), 10, cv::Scalar(0, 255, 0), 1, CV_AA, 0);
        }

        cv::imwrite(m_image_outfile.c_str(), image);
    }

} // namespace pdal
