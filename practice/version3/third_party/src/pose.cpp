
#include "pose.h"

namespace dilu
{
    namespace mapping
    {

        Eigen::Isometry3d translationAndRotationMatrixToIsometry3d(const Eigen::Vector3d translation, const Eigen::Matrix3d rotation)
        {
            Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
            pose.rotate(rotation);
            pose.pretranslate(translation);

            return pose;
        }

        Eigen::Matrix3d EularAngleToMatrix3d(const double &angle_x, const double &angle_y, const double &angle_z, const std::string &order)
        {

            unsigned int position_x = order.find("X");
            unsigned int position_y = order.find("Y");
            unsigned int position_z = order.find("Z");

            bool angle_x_positive = order[position_x - 1] == '+' ? true : false;
            bool angle_y_positive = order[position_y - 1] == '+' ? true : false;
            bool angle_z_positive = order[position_z - 1] == '+' ? true : false;

            unsigned int rotation_first, rotation_second, rotation_third;

            if (position_x < position_y && position_y < position_z)
            {
                rotation_first = 0;
                rotation_second = 1;
                rotation_third = 2;
            }

            else if (position_x < position_z && position_z < position_y)
            {
                rotation_first = 0;
                rotation_second = 2;
                rotation_third = 1;
            }

            else if (position_y < position_z && position_z < position_x)
            {
                rotation_first = 1;
                rotation_second = 2;
                rotation_third = 0;
            }

            else if (position_y < position_x && position_x < position_z)
            {

                rotation_first = 1;
                rotation_second = 0;
                rotation_third = position_z;
            }

            else if (position_z < position_y && position_y < position_x)
            {
                rotation_first = 2;
                rotation_second = 1;
                rotation_third = 0;
            }

            else if (position_z < position_x && position_x < position_y)
            {
                rotation_first = 2;
                rotation_second = 0;
                rotation_third = 1;
            }
 
            return EularAngleToMatrix3d(angle_x, angle_x_positive, angle_y, angle_y_positive, angle_z, angle_z_positive, rotation_first, rotation_second, rotation_third);
        }

        Eigen::Matrix3d EularAngleToMatrix3d(const double &angle_x, const bool &angle_x_positive, const double &angle_y, const bool &angle_y_positive, const double &angle_z, const bool &angle_z_positive, const unsigned int rotation_first, const unsigned int rotation_second, const unsigned int rotation_third)
        {
            int rotation_direction_x = angle_x_positive ? 1 : -1;
            int rotation_direction_y = angle_y_positive ? 1 : -1;
            int rotation_direction_z = angle_z_positive ? 1 : -1;

            Eigen::AngleAxisd axis_x_angle(rotation_direction_x * angle_x, Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd axis_y_angle(rotation_direction_y * angle_y, Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd axis_z_angle(rotation_direction_z * angle_z, Eigen::Vector3d::UnitZ());

            //ZYX
            if (rotation_first == 2 && rotation_second == 1 && rotation_third == 0)
                return (axis_z_angle * axis_y_angle * axis_x_angle).toRotationMatrix();
            //ZXY
            else if (rotation_first == 2 && rotation_second == 0 && rotation_third == 1)
                return (axis_z_angle * axis_x_angle * axis_y_angle).toRotationMatrix();
            //YZX
            else if (rotation_first == 1 && rotation_second == 2 && rotation_third == 0)
                return (axis_y_angle * axis_z_angle * axis_x_angle).toRotationMatrix();
            //YXZ
            else if (rotation_first == 1 && rotation_second == 0 && rotation_third == 2)
                return (axis_y_angle * axis_x_angle * axis_z_angle).toRotationMatrix();
            //XYZ
            else if (rotation_first == 0 && rotation_second == 1 && rotation_third == 2)
                return (axis_x_angle * axis_y_angle * axis_z_angle).toRotationMatrix();
            //XZY
            else if (rotation_first == 0 && rotation_second == 2 && rotation_third == 1)
                return (axis_x_angle * axis_z_angle * axis_y_angle).toRotationMatrix();
        }

        void Matrix3dToEularAngle(const Eigen::Matrix3d &rotation, const std::string &order, double *angle_x, double *angle_y, double *angle_z)
        {
            unsigned int position_x = order.find("X");
            unsigned int position_y = order.find("Y");
            unsigned int position_z = order.find("Z");

            bool angle_x_positive = order[position_x - 1] == '+' ? true : false;
            bool angle_y_positive = order[position_y - 1] == '+' ? true : false;
            bool angle_z_positive = order[position_z - 1] == '+' ? true : false;

            unsigned int rotation_first, rotation_second, rotation_third;

            if (position_x < position_y && position_y < position_z)
            {
                rotation_first = 0;
                rotation_second = 1;
                rotation_third = 2;
            }

            else if (position_x < position_z && position_z < position_y)
            {
                rotation_first = 0;
                rotation_second = 2;
                rotation_third = 1;
            }

            else if (position_y < position_z && position_z < position_x)
            {
                rotation_first = 1;
                rotation_second = 2;
                rotation_third = 0;
            }

            else if (position_y < position_x && position_x < position_z)
            {

                rotation_first = 1;
                rotation_second = 0;
                rotation_third = 2;
            }

            else if (position_z < position_y && position_y < position_x)
            {
                rotation_first = 2;
                rotation_second = 1;
                rotation_third = 0;
            }

            else if (position_z < position_x && position_x < position_y)
            {
                rotation_first = 2;
                rotation_second = 0;
                rotation_third = 1;
            }
            Matrix3dToEularAngle(rotation,rotation_first,rotation_second,rotation_third,angle_x,angle_x_positive,angle_y,angle_y_positive,angle_z,angle_z_positive);
            
        }

        void Matrix3dToEularAngle(const Eigen::Matrix3d &rotation, const unsigned int &rotation_first, const unsigned int &rotation_second, const unsigned int &rotation_third, double *angle_x, const bool &angle_x_positive,
                                  double *angle_y, const bool &angle_y_positive, double *angle_z, const bool &angle_z_positive)
        {

            Eigen::Vector3d euler = rotation.eulerAngles(rotation_first, rotation_second, rotation_third);
            int rotation_direction_x = angle_x_positive ? 1 : -1;
            int rotation_direction_y = angle_y_positive ? 1 : -1;
            int rotation_direction_z = angle_z_positive ? 1 : -1;
            //ZYX
            if (rotation_first == 2 && rotation_second == 1 && rotation_third == 0)
            {
                *angle_z = euler[0] * rotation_direction_z;
                *angle_y = euler[1] * rotation_direction_y;
                *angle_x = euler[2] * rotation_direction_x;
            }
            //ZXY
            else if (rotation_first == 2 && rotation_second == 0 && rotation_third == 1)
            {
                *angle_z = euler[0] * rotation_direction_z;
                *angle_x = euler[1] * rotation_direction_x;
                *angle_y = euler[2] * rotation_direction_y;
            }
            //YZX
            else if (rotation_first == 1 && rotation_second == 2 && rotation_third == 0)
            {
                *angle_y = euler[0] * rotation_direction_y;
                *angle_z = euler[1] * rotation_direction_z;
                *angle_x = euler[2] * rotation_direction_x;
            }

            //YXZ
            else if (rotation_first == 1 && rotation_second == 0 && rotation_third == 2)
            {
                *angle_y = euler[0] * rotation_direction_y;
                *angle_x = euler[1] * rotation_direction_x;
                *angle_z = euler[2] * rotation_direction_z;
            }

            //XYZ
            else if (rotation_first == 0 && rotation_second == 1 && rotation_third == 2)
            {
                *angle_x = euler[0] * rotation_direction_x;
                *angle_y = euler[1] * rotation_direction_y;
                *angle_z = euler[2] * rotation_direction_z;
            }
            //XZY
            else if (rotation_first == 0 && rotation_second == 2 && rotation_third == 1)
            {
                *angle_x = euler[0] * rotation_direction_x;
                *angle_z = euler[1] * rotation_direction_z;
                *angle_y = euler[2] * rotation_direction_y;
            }
        }
        /*
* @brief converts heading pitch roll to rotation matrix 
* @param heading heading angle in rad 
* @param pitch pitch angle in rad·
* @param roll roll angle in rad
* @return  Eigen::Matrix3d
*/
        Eigen::Matrix3d headingPitchRollToMatrix3d(const double heading, const double pitch, const double roll)
        {
            // Eigen::AngleAxisd heading_angle(heading, Eigen::Vector3d::UnitZ());
            // Eigen::AngleAxisd pitch_angle(-pitch, Eigen::Vector3d::UnitX());
            // Eigen::AngleAxisd roll_angle(-roll, Eigen::Vector3d::UnitY());
            // return (heading_angle * pitch_angle * roll_angle).toRotationMatrix();

            double angle_x = pitch;
            double angle_y = roll;
            double angle_z = heading;

            bool angle_x_positive = true;
            bool angle_y_positive = true;
            bool angle_z_positive = false;
            return EularAngleToMatrix3d(angle_x, angle_x_positive, angle_y, angle_y_positive, angle_z, angle_z_positive, 2, 0, 1);
        }

        /*
* @brief converts rotation matrix  to heading pitch roll 
* @param  rotation rotation matrix 
* @param heading heading angle in rad 
* @param pitch pitch angle in rad
* @param roll roll angle in rad
*/
        void matrix3dToHeadingPitchRoll(const Eigen::Matrix3d &rotation, double *heading, double *pitch, double *roll)
        {

            // Eigen::Vector3d euler = rotation.eulerAngles(2, 0, 1);
            // *heading = euler[0];
            // *pitch = -euler[1];
            // *roll = -euler[2];
            double angle_x, angle_y, angle_z;
            Matrix3dToEularAngle(rotation, 2, 0, 1, &angle_x, true, &angle_y, false, &angle_z, false);
            *heading = angle_z;
            *pitch = angle_x;
            *roll = angle_y;
        }

        /*
* @brief converts ROS yaw pitch roll to rotation matrix 
* @param yaw yaw angle in rad 
* @param pitch pitch angle in rad
* @param roll roll angle in rad
* @return  Eigen::Matrix3d
*/
        Eigen::Matrix3d yawPitchRollToMatrix3d(const double yaw, const double pitch, const double roll)
        {
            // Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
            // Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
            // Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());

            // return (yaw_angle * pitch_angle * roll_angle).toRotationMatrix();
            double angle_x = roll;
            double angle_y = pitch;
            double angle_z = yaw;

            bool angle_x_positive = true;
            bool angle_y_positive = true;
            bool angle_z_positive = true;
            return EularAngleToMatrix3d(angle_x, angle_x_positive, angle_y, angle_y_positive, angle_z, angle_z_positive, 2, 1, 0);
        }

        /*
* @brief converts rotation matrix  to ros yaw pitch roll 
* @param  rotation rotation matrix 
* @param yaw yaw angle in rad 
* @param pitch pitch angle in rad
* @param roll roll angle in rad
*/
        void matrix3dToYawPitchRoll(const Eigen::Matrix3d &rotation, double *yaw, double *pitch, double *roll)
        {
            // Eigen::Vector3d euler = rotation.eulerAngles(2, 1, 0);
            // *yaw = euler[0];
            // *pitch = euler[1];
            // *roll = euler[2];

            double angle_x, angle_y, angle_z;
            Matrix3dToEularAngle(rotation, 2, 1, 0, &angle_x, true, &angle_y, true, &angle_z, true);
            *yaw = angle_z;
            *pitch = angle_y;
            *roll = angle_x;
        }

        /*
* @brief calculate rotation matrix from ecef To local enu  
* @param lon longitude in deg 
* @param lat latitude in deg
* @return  Eigen::Matrix3d
*/
        Eigen::Matrix3d getRotationMatrixEcefToEnu(const double longitude, const double latitude)
        {
            const double kDeg2Rad = M_PI / 180.0;
            double lon = longitude * kDeg2Rad;
            double lat = latitude * kDeg2Rad;

            double sin_lon = sin(lon);
            double cos_lon = cos(lon);
            double sin_lat = sin(lat);
            double cos_lat = cos(lat);

            Eigen::Matrix3d rot;
            rot << -sin_lon, cos_lon, 0,
                -sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat,
                cos_lat * cos_lon, cos_lat * sin_lon, sin_lat;
            return rot;
        }

        /*
* @brief calculate rotation matrix from local ENU To ECEF   
* @param lon longitude in deg 
* @param lat latitude in deg
* @return  Eigen::Matrix3d  rot_enu2ecef
*/
        Eigen::Matrix3d getRotationMatrixEnuToEcef(const double longitude, const double latitude)
        {
            return getRotationMatrixEcefToEnu(longitude, latitude).transpose();
        }

        /*
* @brief convert local ENU heading pitch roll to rotation matrix in ECEF frame
* @param heading heading angle in rad 
* @param pitch pitch angle in rad
* @param roll roll angle in rad
* @param lon longitude in deg 
* @param lat latitude in deg
* @return  Eigen::Matrix3d
*/
        Eigen::Matrix3d headingPitchRollToEcefRotation(const double &heading, const double &pitch, const double &roll, const double &longitude, const double &latitude)
        {
            Eigen::Matrix3d enu2body = headingPitchRollToMatrix3d(heading, pitch, roll);
            Eigen::Matrix3d ecef2enu = getRotationMatrixEcefToEnu(longitude, latitude);
            return ecef2enu * enu2body;
        }

        Eigen::Isometry3d llhhpyToEcefIsometry3d(double longitude, double latitude, double altitude, double heading, double pitch, double roll, OGRCoordinateTransformation *trans_llh_ecef)
        {
            Eigen::Vector3d translation(longitude, latitude, altitude);
            trans_llh_ecef->Transform(1, &translation[0], &translation[1], &translation[2]);

            Eigen::Matrix3d rotation = headingPitchRollToEcefRotation(heading, pitch, roll, longitude, latitude);

            Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
            pose.rotate(rotation);
            pose.pretranslate(translation);

            return pose;
        }

        // void EcefIsometry3dTollhhpy(const Eigen::Isometry3d &ecef_pose, double &longitude, double &latitude, double &altitude, double &heading, double &pitch, double &roll, OGRCoordinateTransformation *trans_ecef_llh)
        // {
        //     Eigen::Vector3d translation(ecef_pose.translation());
        //     trans_ecef_llh->Transform(1, &translation[0], &translation[1]);

        //     longitude = translation[0];
        //     latitude = translation[1];
        //     altitude = translation[2];

        //     Eigen::Matrix3d rotation = ecef_pose.rotation();
        //     dilu::mapping::matrix3dToYawPitchRoll(rotation, &heading, &pitch, &roll);

        //     return ;
        // }

        Eigen::Isometry3d llhhprToUtmIsometry3d(double longitude, double latitude, double altitude, double heading, double pitch, double roll, OGRCoordinateTransformation *trans_llh_utm)
        {

            Eigen::Vector3d translation(latitude, longitude, altitude);
            trans_llh_utm->Transform(1, &translation[0], &translation[1], &translation[2]);

            Eigen::Matrix3d rotation = headingPitchRollToMatrix3d(heading, pitch, roll);
            Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
            pose.rotate(rotation);
            pose.pretranslate(translation);

            return pose;
        }

        Eigen::Isometry3d llhyprToUtmIsometry3d(double longitude, double latitude, double altitude, double yaw, double pitch, double roll, OGRCoordinateTransformation *trans_llh_utm)
        {
            Eigen::Vector3d translation(longitude, latitude, altitude);
            trans_llh_utm->Transform(1, &translation[0], &translation[1]);

            Eigen::Matrix3d rotation = yawPitchRollToMatrix3d(yaw, pitch, roll);

            Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
            pose.rotate(rotation);
            pose.pretranslate(translation);

            return pose;
        }

        /*
* @brief convert heading pitch roll in local ENU to rotation matrix in ECEF frame
* @param lon longitude in deg 
* @param lat latitude in deg
* @return  Eigen::Matrix3d
*/
        void EcefRotationToHeadingPitchRoll(const Eigen::Matrix3d &rot, const double &longitude, const double &latitude, double *heading, double *pitch, double *roll)
        {

            Eigen::Matrix3d enu2ecef = getRotationMatrixEnuToEcef(longitude, latitude);
            Eigen::Matrix3d enu2body = enu2ecef * rot;
            matrix3dToHeadingPitchRoll(enu2body, heading, pitch, roll);
        }

        /*
* @brief spherical linear interpolation between the two rotation matrix via quaternions
* @param start first rotation matrix 
* @param end second rotation matrix
* @param t factor [0, 1]
* @return interpolated Eigen::Matrix3d
*/
        Eigen::Matrix3d slerpRotationMatrix(const Eigen::Matrix3d &start, const Eigen::Matrix3d &end, const double &t)
        {
            return Eigen::Quaterniond(start).slerp(t, Eigen::Quaterniond(end)).toRotationMatrix();
        }

        /*
* @brief linear interpolation between two Eigen::Vector3d
* @param start first Vector3d 
* @param end second Vector3d
* @param t factor [0, 1]
* @return interpolated Eigen::Vector3d
*/
        Eigen::Vector3d linearInterpolateVector3d(const Eigen::Vector3d &start, const Eigen::Vector3d &end, const double &t)
        {
            return start * t + end * (1 - t);
        }

        /*
* @brief linear interpolation between two Isometry3d, linearInterpolateVector3d and slerpRotationMatrix 
* @param start first rotation matrix 
* @param end second rotation matrix
* @param t factor [0, 1]
* @return interpolated Eigen::Isometry3d
*/
        Eigen::Isometry3d linearInterpolateIsometry3d(const Eigen::Isometry3d &start, const Eigen::Isometry3d &end, const double &t)
        {
            Eigen::Vector3d trans = linearInterpolateVector3d(start.translation(), end.translation(), t);
            Eigen::Matrix3d rot = slerpRotationMatrix(start.rotation(), end.rotation(), t);

            Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
            result.rotate(rot);
            result.pretranslate(trans);

            return result;
        }

        /*
* @brief linear interpolation between two timestamped Isometry3d
* @param start_pose first Eigen::Isometry3d 
* @param start_time timestamp of first Eigen::Isometry3d 
* @param end_pose second Eigen::Isometry3d
* @param end_time timestamp of second Eigen::Isometry3d
* @param time timestamp to look up interpolation  [start_time, end_time]
* @return interpolated Eigen::Isometry3d
*/
        Eigen::Isometry3d linearInterpolateTimestampedIsometry3d(const Eigen::Isometry3d &start_pose, const double start_time, const Eigen::Isometry3d &end_pose, const double &end_time, const double &time)
        {

            return linearInterpolateIsometry3d(start_pose, end_pose, (time - start_time) / (end_time - start_time));
        }

    } // namespace mapping
} // namespace dilu
