

#include "EulerConvert.hpp"

namespace pdal
{

    static PluginInfo const s_info{
        "filters.eulerconvert",
        "Convert  attitude  between 'eulerAngle' and 'rotation matrix3d' ",
        "http://pdal.io/stages/filters.eulerconvert.html"};

    CREATE_SHARED_STAGE(EulerConvert, s_info)

    std::string EulerConvert::getName() const
    {
        return s_info.name;
    }

    void EulerConvert::addArgs(ProgramArgs &args)
    {
        args.add("conversion", "attitude conversion type: 'euler2matrix' or 'matrix2euler' ",
                 m_conversion);

        args.add("order", "axis rotation angle order and sign. (ps: the positive and negative direciton of the angle conforms to the right-hand screw rule ) ", m_order);

        args.add("x_angle_dimension", "x-axis  rotation angle dimension name", m_x_angle_dimension);
        args.add("y_angle_dimension", "y-axis  rotation angle dimension name", m_y_angle_dimension);
        args.add("z_angle_dimension", "z-axis  rotation angle dimension name", m_z_angle_dimension);

        args.add("matrix_dimensions", "rotation matrix3D dimension names ", m_matrix_dimensions, {"rot11,rot22,rot33,rot21,rot22,rot23,rot31,rot32,rot33"});

        args.add("angle_unit", " axis rotation angle unit ，type：'degree' or 'radian' ，default：'degree' ", m_angle_unit, "degree");
    }

    void EulerConvert::initialize()
    {
        //check attitude conversion type
        if (Utils::iequals(m_conversion, "euler2matrix"))
            m_conversion = "euler2matrix";
        else if (Utils::iequals(m_conversion, "matrix2euler"))
            m_conversion = "matrix2euler";
        else
            throwError("Invalid conversion type.");

        //check  angle unit type
        if (Utils::iequals(m_angle_unit, "degree"))
            m_angle_unit = "degree";
        else if (Utils::iequals(m_angle_unit, "radian"))
            m_angle_unit = "radian";
        else
            throwError("Invalid angle unit.");

        //check matrix_dimensions size
        if (m_matrix_dimensions.size() != 9)
            throwError(" 'matrix_dimensions' option must contain 9 dimension name.");

        //check rotation order type
        if (Utils::iequals(m_order, "ROS"))
            m_order = "+Z+Y+X";
        else if (Utils::iequals(m_order, "IE"))
            m_order = "-Z+X+Y";
        else
            m_order = Utils::toupper(m_order);

        if (m_order.size() != 6)
            throwError("Invalid rotation order ." + m_order);
        for (int i = 0; i < m_order.size(); i++)
        {
            if (i % 2 == 0 && m_order[i] != '-' && m_order[i] != '+')
                throwError("Invalid rotation order ." + m_order);
            else if (i % 2 == 1 && m_order[i] != 'X' && m_order[i] != 'Y' && m_order[i] != 'Z')
                throwError("Invalid rotation order ." + m_order);
        }
    }

    void EulerConvert::addDimensions(PointLayoutPtr layout)
    {
        if (m_conversion == "euler2matrix")
        {
            for (auto &dim_name : m_matrix_dimensions)
            {
                Dimension::Type dim_type = Dimension::Type::Double;
                Dimension::Id dim_id = layout->registerOrAssignDim(dim_name, dim_type);
            }
        }
        else if (m_conversion == "matrix2euler")
        {
            Dimension::Type dim_type = Dimension::Type::Double;

            Dimension::Id angle_x_dim_id = layout->registerOrAssignDim(m_x_angle_dimension, dim_type);
            Dimension::Id angle_y_dim_id = layout->registerOrAssignDim(m_y_angle_dimension, dim_type);
            Dimension::Id angle_z_dim_id = layout->registerOrAssignDim(m_z_angle_dimension, dim_type);
        }
    }

    void EulerConvert::prepared(PointTableRef table)
    {
        if (m_conversion == "euler2matrix")
        {

            for (auto &dim_name : m_matrix_dimensions)
            {
                Dimension::Id dim_id = table.layout()->findDim(dim_name);
                m_nameIds.insert(make_pair(dim_name, dim_id));
            }

            Dimension::Id angle_x_dim_id = table.layout()->findDim(m_x_angle_dimension);
            Dimension::Id angle_y_dim_id = table.layout()->findDim(m_y_angle_dimension);
            Dimension::Id angle_z_dim_id = table.layout()->findDim(m_z_angle_dimension);

            if (angle_x_dim_id == Dimension::Id::Unknown || angle_y_dim_id == Dimension::Id::Unknown || angle_z_dim_id == Dimension::Id::Unknown)
                throwError("Axis angle dimension not all exist.");

            m_nameIds.insert(make_pair(m_x_angle_dimension, angle_x_dim_id));
            m_nameIds.insert(make_pair(m_y_angle_dimension, angle_y_dim_id));
            m_nameIds.insert(make_pair(m_z_angle_dimension, angle_z_dim_id));
        }

        else if (m_conversion == "matrix2euler")
        {
            for (auto &dim_name : m_matrix_dimensions)
            {
                Dimension::Id dim_id = table.layout()->findDim(dim_name);
                if (dim_id == Dimension::Id::Unknown)
                    throwError("Matrix dimension " + dim_name + " does not exist.");
                m_nameIds.insert(make_pair(dim_name, dim_id));
            }

            Dimension::Id angle_x_dim_id = table.layout()->findDim(m_x_angle_dimension);
            Dimension::Id angle_y_dim_id = table.layout()->findDim(m_y_angle_dimension);
            Dimension::Id angle_z_dim_id = table.layout()->findDim(m_z_angle_dimension);
            m_nameIds.insert(make_pair(m_x_angle_dimension, angle_x_dim_id));
            m_nameIds.insert(make_pair(m_y_angle_dimension, angle_y_dim_id));
            m_nameIds.insert(make_pair(m_z_angle_dimension, angle_z_dim_id));
        }
    }

    void EulerConvert::eulerAngleToRotationMatrix(PointRef &point)
    {

        double x_axisd_angle = point.getFieldAs<double>(m_nameIds[m_x_angle_dimension]);
        double y_axisd_angle = point.getFieldAs<double>(m_nameIds[m_y_angle_dimension]);
        double z_axisd_angle = point.getFieldAs<double>(m_nameIds[m_z_angle_dimension]);
        if (m_angle_unit == "degree")
        {
            x_axisd_angle = x_axisd_angle * M_PI / 180;
            y_axisd_angle = y_axisd_angle * M_PI / 180;
            z_axisd_angle = z_axisd_angle * M_PI / 180;
        }

        Eigen::Matrix3d rotation = dilu::mapping::EularAngleToMatrix3d(x_axisd_angle, y_axisd_angle, z_axisd_angle, m_order);

        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
            {
                int k = i * 3 + j;
                point.setField(m_nameIds[m_matrix_dimensions[k]], rotation(i, j));
            }
    }

    void EulerConvert::rotationMatrixToEulerAngle(PointRef &point)
    {

        Eigen::Matrix3d rotation;
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                int k = i * 3 + j;
                rotation(i, j) = point.getFieldAs<double>(m_nameIds[m_matrix_dimensions[k]]);
            }
        }
        double x_axisd_angle, y_axisd_angle, z_axisd_angle;

        dilu::mapping::Matrix3dToEularAngle(rotation, m_order, &x_axisd_angle, &y_axisd_angle, &z_axisd_angle);
        if (m_angle_unit == "degree")
        {
             x_axisd_angle = x_axisd_angle * 180 / M_PI;
             y_axisd_angle = y_axisd_angle * 180 / M_PI;
             z_axisd_angle = z_axisd_angle * 180 / M_PI;
        }


    }

    bool EulerConvert::processOne(PointRef &point)
    {
        if (m_conversion == "euler2matrix")
            eulerAngleToRotationMatrix(point);
        else if (m_conversion == "matrix2euler")
            rotationMatrixToEulerAngle(point);

        return true;
    }

    void EulerConvert::filter(PointView &view)
    {

        PointRef point(view, 0);
        for (PointId id = 0; id < view.size(); ++id)
        {
            point.setPointId(id);
            processOne(point);
        }
    }

} // namespace pdal
