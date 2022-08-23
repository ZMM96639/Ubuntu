

#include "Pose6DTransformation.hpp"
#include "pose.h"
namespace pdal
{

    static PluginInfo const s_info{
        "filters.pose6dtransformation",
        "6-dimensional data multiplication 4x4 matrix",
        "http://pdal.io/stages/filters.pose6dtransformation.html"};

    CREATE_SHARED_STAGE(Pose6DTransformation, s_info)

    std::string Pose6DTransformation::getName() const
    {
        return s_info.name;
    }

    void Pose6DTransformation::addArgs(ProgramArgs &args)
    {
        args.add("matrix", "multiplier 4x4 matrix values，contains 3x3 rotation matrix and translation vector ", m_arg_matrix_vals);
        args.add("corrospondence", "Matrix  4x4 matrix values or EulerAngle ", m_corrospondence);

        // corrospondence equal euler
        args.add("order", "euler angle order and sign. (ps: the positive and negative direciton of the angle conforms to the right-hand screw rule ) ", m_order);
        args.add("x_angle", "axis x rotation angle ", m_x_angle);
        args.add("y_angle", "axis y rotation angle ", m_y_angle);
        args.add("z_angle", "axis z rotation angle ", m_z_angle);
        args.add("angle_unit", " axis rotation angle unit ，type：'degree' or 'radian' ，default：'degree' ", m_angle_unit, "degree");
        args.add("translation_x", "axis x translation vector ,unit(m)", m_translation_x);
        args.add("translation_y", "axis y translation vector ,unit(m)", m_translation_y);
        args.add("translation_z", "axis z translation vector ,unit(m)", m_translation_z);

        // corrospondence equal matrix
        args.add("matrix_dimensions", "rotation matrix3D  dimension names ", m_matrix_dimensions, {"rot11,rot22,rot33,rot21,rot22,rot23,rot31,rot32,rot33"});
        args.add("invert", "Apply inverse transformation", m_invert, false);

        //add new arg ,order to parse kitti dataset
        args.add("multi_order", "when corrospondence equal matrix, the option 'matrix' multiplication order ,left or right ,default right ", m_multi_order, "right");
        args.add("invert_result", "whether invert result Isometry3d  ", m_invert_result, false);
    }

    void Pose6DTransformation::initialize()
    {

        //check option matrix size & feach data from 'm_arg_matrix_vals'
        if (m_arg_matrix_vals.size() != 16 && Utils::iequals(m_corrospondence, "matrix"))
            throwError(" 'matrix' option must contain 16 digits.");

        if (Utils::iequals(m_corrospondence, "matrix"))
        {
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    int k = i * 4 + j;
                    m_matrix(i, j) = m_arg_matrix_vals[k];
                }
            }
            if (m_invert)
                m_matrix = m_matrix.inverse();
        }
        else if (Utils::iequals(m_corrospondence, "euler"))
        {

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

            double x_axisd_angle = m_angle_unit == "degree" ? m_x_angle * M_PI / 180 : m_x_angle;
            double y_axisd_angle = m_angle_unit == "degree" ? m_y_angle * M_PI / 180 : m_y_angle;
            double z_axisd_angle = m_angle_unit == "degree" ? m_z_angle * M_PI / 180 : m_z_angle;

            Eigen::Matrix3d rotation = dilu::mapping::EularAngleToMatrix3d(x_axisd_angle, y_axisd_angle, z_axisd_angle, m_order);
            Eigen::Vector3d translation(m_translation_x, m_translation_y, m_translation_z);
            m_matrix = dilu::mapping::translationAndRotationMatrixToIsometry3d(translation, rotation);
        }
        else
            throwError("not support corrospondence " + m_corrospondence + " type");
    }

    void Pose6DTransformation::addDimensions(PointLayoutPtr layout)
    {
    }

    void Pose6DTransformation::prepared(PointTableRef table)
    {
        //Check the dimension whether exists
        if (table.layout()->findDim("X") == Dimension::Id::Unknown ||
            table.layout()->findDim("Y") == Dimension::Id::Unknown ||
            table.layout()->findDim("Z") == Dimension::Id::Unknown)
            throwError("Dimension X Y Z does not all exist.");

        m_nameIds.insert(std::make_pair("X", Dimension::Id::X));
        m_nameIds.insert(std::make_pair("Y", Dimension::Id::Y));
        m_nameIds.insert(std::make_pair("Z", Dimension::Id::Z));
        for (auto &dim_name : m_matrix_dimensions)
        {
            Dimension::Id dim_id = table.layout()->findDim(dim_name);
            if (dim_id == Dimension::Id::Unknown)
                throwError("Rotation matrix dimension " + dim_name + " does not exist.");
            m_nameIds.insert(make_pair(dim_name, dim_id));
        }
    }

    bool Pose6DTransformation::processOne(PointRef &point)
    {
        //fetch translation and euler angle  from point
        double x = point.getFieldAs<double>(m_nameIds["X"]);
        double y = point.getFieldAs<double>(m_nameIds["Y"]);
        double z = point.getFieldAs<double>(m_nameIds["Z"]);

        // According to the Euler angle rotation order and rotation direction, construct a rotation matrix
        Eigen::Vector3d translation(x, y, z);
        Eigen::Matrix3d rotation;

        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
            {
                int k = i * 3 + j;
                rotation(i, j) = point.getFieldAs<double>(m_nameIds[m_matrix_dimensions[k]]);
            }

        Eigen::Isometry3d transform;
        transform = dilu::mapping::translationAndRotationMatrixToIsometry3d(translation, rotation);

        // Matrix multiplication
        if (m_multi_order == "left")
            transform = m_matrix * transform;
        else
            transform = transform * m_matrix;

        //whether reverse
        if (m_invert_result)
            transform = transform.inverse();

        // Get rotation matrix and translation vector from 4x4 Eigen::Isometry3d matrix
        Eigen::Vector3d after_translation = transform.translation();
        Eigen::Matrix3d after_rotation = transform.rotation();

        //Update data from after transform matrix
        point.setField(m_nameIds["X"], after_translation[0]);
        point.setField(m_nameIds["Y"], after_translation[1]);
        point.setField(m_nameIds["Z"], after_translation[2]);
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
            {
                int k = i * 3 + j;
                point.setField(m_nameIds[m_matrix_dimensions[k]], after_rotation(i, j));
            }

        return true;
    }

    void Pose6DTransformation::filter(PointView &view)
    {

        PointRef point(view, 0);
        for (PointId id = 0; id < view.size(); ++id)
        {
            point.setPointId(id);
            processOne(point);
        }
    }

} // namespace pdal
