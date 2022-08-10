
#include <iostream>
#include "Pose1Topose2.hpp"

namespace pdal
{
    static PluginInfo const s_info{
        "filters.pose1topose2",
        "6-dimensional data multiplication 4x4 matrix",
        "http://pdal.io/stages/filters.pose1topose2.html"};

    CREATE_SHARED_STAGE(Pose1Topose2, s_info)

    std::string Pose1Topose2::getName() const
    {
        return s_info.name;
    }

    void Pose1Topose2::addArgs(ProgramArgs &args)
    {
        args.add("image_outfile", "out file", m_data_outfile);
        args.add("groundtruth", "multiplier 4x4 matrix values，contains 3x3 rotation matrix and translation vector ", m_arg_groundtruth_vals);
        args.add("invert", "Apply inverse transformation", m_invert, false);

        args.add("matrix_dimensions", "rotation matrix3D dimension names ", m_matrix_dimensions, {"rot11,rot12,rot13,rot21,rot22,rot23,rot31,rot32,rot33"});
        args.add("angle_unit", " axis rotation angle unit ，type：'degree' or 'radian' ，default：'degree' ", m_angle_unit, "degree");
    }

    void Pose1Topose2::initialize()
    {
        dataInput(m_arg_groundtruth_vals, m_groundtruth, true);
    }

    void Pose1Topose2::addDimensions(PointLayoutPtr layout)
    {
        for (auto &dim_name : m_matrix_dimensions)
        {
            Dimension::Type dim_type = Dimension::Type::Double;
            Dimension::Id dim_id = layout->registerOrAssignDim(dim_name, dim_type);
        }
    }

    void Pose1Topose2::prepared(PointTableRef table)
    {
        PointLayoutPtr layout(table.layout());

        // Check the dimension whether exists
        if (layout->findDim("X") == Dimension::Id::Unknown ||
            layout->findDim("Y") == Dimension::Id::Unknown ||
            layout->findDim("Z") == Dimension::Id::Unknown)
            throwError("Dimension X Y Z does not all exist.");

        m_nameIds.insert(std::make_pair("X", Dimension::Id::X));
        m_nameIds.insert(std::make_pair("Y", Dimension::Id::Y));
        m_nameIds.insert(std::make_pair("Z", Dimension::Id::Z));

        for (auto &dim_name : m_matrix_dimensions)
        {
            Dimension::Id dim_id = layout->findDim(dim_name);

            if (dim_id == Dimension::Id::Unknown)
            {
                throwError("Matrix dimension " + dim_name + " does not exist.");
            }
            m_nameIds.insert(std::make_pair(dim_name, dim_id));
        }
    }

    void Pose1Topose2::processOne(PointRef &point)
    {
        // fetch translation and euler angle  from point
        double x = point.getFieldAs<double>(m_nameIds["X"]);
        double y = point.getFieldAs<double>(m_nameIds["Y"]);
        double z = point.getFieldAs<double>(m_nameIds["Z"]);
        Eigen::Vector3d tranlation_(x, y, z);

        // construct a rotation matrix
        Eigen::Matrix3d rotation_;
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                int k = i * 3 + j;
                rotation_(i, j) = point.getFieldAs<double>(m_nameIds[m_matrix_dimensions[k]]);
            }
        }

        Eigen::Isometry3d transform = dilu::mapping::translationAndRotationMatrixToIsometry3d(tranlation_, rotation_);

        m_error = m_groundtruth * transform;

        Eigen::Matrix3d rotation;
        int r = 0, c = 0;
        for (int i = 0; i < 12; i++)
        {
            if (i == 3 || i == 7 || i == 11)
            {
                c++;
                r = 0;
                continue;
            }
            rotation(r, c) = m_error.data()[i];
            r++;
        }

        dilu::mapping::Matrix3dToEularAngle(rotation, "+X+Y+Z", &x_axisd_angle, &y_axisd_angle, &z_axisd_angle);

        if (m_angle_unit == "degree")
        {
            x_axisd_angle = x_axisd_angle * 180 / M_PI;
            y_axisd_angle = y_axisd_angle * 180 / M_PI;
            z_axisd_angle = z_axisd_angle * 180 / M_PI;
        }

        std::cout << "Err_matrix 4x4:" << std::endl;
        std::cout << m_error.matrix() << std::endl;

        std::cout << "/***********/" << std::endl;
        std::cout << "Err_euler:" << std::endl;
        std::cout << "(x_axisd, y_axisd z_axisd): "
                  << "(" << x_axisd_angle << "," << y_axisd_angle << "," << z_axisd_angle
                  << ") (units: degree)"
                  << std::endl;
        std::cout << "Err: " << sqrt((x_axisd_angle * x_axisd_angle + y_axisd_angle * y_axisd_angle + z_axisd_angle * z_axisd_angle))
                  << " degree"
                  << std::endl;

        std::cout << "/***********/" << std::endl;
        std::cout << "Err_tranlation:" << std::endl
                  << "(x,y,z): "
                  << "("
                  << m_error.translation()[0] << "," << m_error.translation()[1] << "," << m_error.translation()[2]
                  << ") (units: cm)"
                  << std::endl;
        std::cout << "Err: " << sqrt((m_error.translation()[0] * m_error.translation()[0] + m_error.translation()[1] * m_error.translation()[1] + m_error.translation()[2] * m_error.translation()[2]))
                  << " cm"
                  << std::endl;
    }

    void Pose1Topose2::filter(PointView &view)
    {
        PointRef point(view, 0);
        for (PointId id = 0; id < view.size(); ++id)
        {
            point.setPointId(id);
            processOne(point);
        }
    }

    void Pose1Topose2::dataInput(std::vector<double> &matrix_vals, Eigen::Isometry3d &matrix, bool invert)
    {
        // check option matrix size & feach data from 'm_arg_matrix_vals'
        if (matrix_vals.size() != 16)
            throwError(" 'matrix' option must contain 16 digits.");
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                int k = i * 4 + j;
                matrix(i, j) = matrix_vals[k];
            }
        }
        if (invert)
            matrix = matrix.inverse();
    }
}