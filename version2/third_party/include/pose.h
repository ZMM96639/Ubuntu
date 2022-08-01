#ifndef DILU_MAPPING_POSE_H
#define DILU_MAPPING_POSE_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gdal_priv.h>
#include <ogr_core.h>
#include <ogr_api.h>
#include <ogr_feature.h>
#include <ogr_geometry.h>
#include <ogrsf_frmts.h>
#include <ogr_spatialref.h>

#include <math.h>
#include <iostream>
namespace dilu
{
	namespace mapping
	{

		Eigen::Isometry3d translationAndRotationMatrixToIsometry3d(const Eigen::Vector3d translation, const Eigen::Matrix3d rotation);

		Eigen::Matrix3d EularAngleToMatrix3d(const double &x_angle, const double &y_angle, const double &z_angle, const std::string &order);

		Eigen::Matrix3d EularAngleToMatrix3d(const double &angle_x, const bool &angle_x_closewise, const double &angle_y, const bool &angle_y_closewise, const double &angle_z, const bool &angle_z_closewise, const unsigned int rotation_first, const unsigned int rotation_second, const unsigned int rotation_third);
 
 		void Matrix3dToEularAngle(const Eigen::Matrix3d &rotation, const std::string &order, double *angle_x, double *angle_y, double *angle_z);
		
		void Matrix3dToEularAngle(const Eigen::Matrix3d &rotation, const unsigned int &rotation_first, const unsigned int &rotation_second, const unsigned int &rotation_third,
											 double *angle_x, const bool &angle_x_closewise, double *angle_y, const bool &angle_y_closewise, double *angle_z, const bool &angle_z_closewise);

		/*
* @brief converts heading pitch roll to rotation matrix 
* @param heading heading angle in rad 
* @param pitch pitch angle in rad
* @param roll roll angle in rad
* @return  Eigen::Matrix3d
*/
		Eigen::Matrix3d headingPitchRollToMatrix3d(const double heading, const double pitch, const double roll);

		/*
* @brief converts rotation matrix  to heading pitch roll 
* @param  rotation rotation matrix 
* @param heading heading angle in rad 
* @param pitch pitch angle in rad
* @param roll roll angle in rad
*/
		void matrix3dToHeadingPitchRoll(const Eigen::Matrix3d &rotation, double *heading, double *pitch, double *roll);

		/*
* @brief converts ROS yaw pitch roll to rotation matrix 
* @param yaw yaw angle in rad 
* @param pitch pitch angle in rad
* @param roll roll angle in rad
* @return  Eigen::Matrix3d
*/
		Eigen::Matrix3d yawPitchRollToMatrix3d(const double yaw, const double pitch, const double roll);

		/*
* @brief converts rotation matrix  to ros yaw pitch roll 
* @param  rotation rotation matrix 
* @param yaw yaw angle in rad 
* @param pitch pitch angle in rad
* @param roll roll angle in rad
*/
		void matrix3dToYawPitchRoll(const Eigen::Matrix3d &rotation, double *yaw, double *pitch, double *roll);

		/*
* @brief calculate rotation matrix from ecef To local enu  
* @param lon longitude in deg 
* @param lat latitude in deg
* @return  Eigen::Matrix3d
*/
		Eigen::Matrix3d getRotationMatrixEcefToEnu(const double longitude, const double latitude);

		/*
* @brief calculate rotation matrix from local ENU To ECEF   
* @param lon longitude in deg 
* @param lat latitude in deg
* @return  Eigen::Matrix3d  rot_enu2ecef
*/
		Eigen::Matrix3d getRotationMatrixEnuToEcef(const double longitude, const double latitude);

		/*
* @brief convert local ENU heading pitch roll to rotation matrix in ECEF frame
* @param heading heading angle in rad 
* @param pitch pitch angle in rad
* @param roll roll angle in rad
* @param lon longitude in deg 
* @param lat latitude in deg
* @return  Eigen::Matrix3d
*/
		Eigen::Matrix3d headingPitchRollToEcefRotation(const double &heading, const double &pitch, const double &roll, const double &longitude, const double &latitude);

		Eigen::Isometry3d llhhpyToEcefIsometry3d(double longitude, double latitude, double altitude, double heading, double pitch, double roll, OGRCoordinateTransformation *trans_llh_ecef);

		/*
* @brief convert heading pitch roll in local ENU to rotation matrix in ECEF frame
* @param lon longitude in deg 
* @param lat latitude in deg
* @return  Eigen::Matrix3d
*/
		void EcefRotationToHeadingPitchRoll(const Eigen::Matrix3d &rot, const double &longitude, const double &latitude, double *heading, double *pitch, double *roll);

		/*
* @brief spherical linear interpolation between the two rotation matrix via quaternions
* @param start first rotation matrix 
* @param end second rotation matrix
* @param t factor [0, 1]
* @return interpolated Eigen::Matrix3d
*/
		Eigen::Matrix3d slerpRotationMatrix(const Eigen::Matrix3d &start, const Eigen::Matrix3d &end, const double &t);

		/*
* @brief linear interpolation between two Eigen::Vector3d
* @param start first Vector3d 
* @param end second Vector3d
* @param t factor [0, 1]
* @return interpolated Eigen::Vector3d
*/
		Eigen::Vector3d linearInterpolateVector3d(const Eigen::Vector3d &start, const Eigen::Vector3d &end, const double &t);

		/*
* @brief linear interpolation between two Isometry3d, linearInterpolateVector3d and slerpRotationMatrix 
* @param start first rotation matrix 
* @param end second rotation matrix
* @param t factor [0, 1]
* @return interpolated Eigen::Isometry3d
*/
		Eigen::Isometry3d linearInterpolateIsometry3d(const Eigen::Isometry3d &start, const Eigen::Isometry3d &end, const double &t);

		/*
* @brief linear interpolation between two timestamped Isometry3d
* @param start_pose first Eigen::Isometry3d 
* @param start_time timestamp of first Eigen::Isometry3d 
* @param end_pose second Eigen::Isometry3d
* @param end_time timestamp of second Eigen::Isometry3d
* @param time timestamp to look up interpolation  [start_time, end_time]
* @return interpolated Eigen::Isometry3d
*/
		Eigen::Isometry3d linearInterpolateTimestampedIsometry3d(const Eigen::Isometry3d &start_pose, const double start_time, const Eigen::Isometry3d &end_pose, const double &end_time, const double &time);

		// Isometry3d.translation()
		// Isometry3d.rotation()

		/*
Eigen::Isometry3d(Eigen::Translation3d(t.translation.x, t.translation.y, t.translation.z)
			 * Eigen::Quaterniond(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z));
*/

		Eigen::Isometry3d llhhprToUtmIsometry3d(double longitude, double latitude, double altitude, double heading, double pitch, double roll, OGRCoordinateTransformation *trans_llh_utm);

		Eigen::Isometry3d llhyprToUtmIsometry3d(double longitude, double latitude, double altitude, double yaw, double pitch, double roll, OGRCoordinateTransformation *trans_llh_utm);

		void EcefIsometry3dTollhhpy(const Eigen::Isometry3d &ecef_pose, double &longitude, double &latitude, double &altitude, double &heading, double &pitch, double &roll, OGRCoordinateTransformation *trans_ecef_llh);

	} // namespace mapping
} // namespace dilu

#endif // DILU_MAPPING_POSE_H
