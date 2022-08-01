
#ifndef MAPPING_DILU_COMMON_H
#define MAPPING_DILU_COMMON_H
#include <cmath>
#include <string>

#include <stdio.h>
#include <stdlib.h>

namespace dilu
{
    namespace mapping
    {

        static inline double UnixTime2GpsTime(const double unix_time)
        {

            const int kUnixGpsEpochDelta = 315964800;
            int leap_seconds = 18;
            return unix_time + leap_seconds - kUnixGpsEpochDelta;
        }
        /*
@brief  convert gps absolute time 2 unix time
@param gps_time  gps absolute time in secs, e.g 1294883069.735
@return unix time in secs
@TODO only support > 1167264017, leap 18 secs, need to add leap seconds table to support other range    https://www.andrews.edu/~tzs/timeconv/timealgorithm.html
*/
        static inline double GpsTime2UnixTime(const double gps_time)
        {
            const int kUnixGpsEpochDelta = 315964800;
            int leap_seconds = 18;
            return gps_time + kUnixGpsEpochDelta - leap_seconds;
        }

        /*
@brief  convert  gps absolute time 2 beijing time in string format yyyymmddhhmmssdddddd year month day hour minute second subsecond(6 digits)
@param gps_time  gps absolute time in secs, e.g 1294883069.735
@return beijing time in string format
*/
        // std::string GpsTime2BeijingTime(const double gps_time)
        // {
        //   double unix_time = GpsTime2UnixTime(gps_time);
        //   double beijing_time = unix_time + 8 * 3600;

        //   std::time_t beijing_time_t(beijing_time);
        //   std::tm *beijing_time_tm = std::localtime(&beijing_time_t);

        //   char time_second_str[100];
        //   std::strftime(time_second_str, 100, "%Y%m%d%H%M%S", beijing_time_tm); /* 20200701021651 */

        //   std::string decimal_str = ToStringWithPrecision(beijing_time - int(beijing_time), 6);
        //   return std::string(time_second_str) + decimal_str.substr(2, 6);
        // }

        /*
@brief  CalculateMeridianConvergenceAngle as for Spherical Transverse Mercator, which gives quite good approximation
@param lat  point latitude in degree  
@param lon  point longitude in degree
@param lon_grid_origin grid coordinate origin longitude degree
@return gamma meridian convergence at point (degrees), positive grid north  east
*/
        static inline double CalculateMeridianConvergenceAngle(const double lat, const double lon, const double lon_grid_origin)
        {
            double lat_rad = lat * M_PI / 180.0;
            double lon_rad = lon * M_PI / 180.0;
            double lon_grid_origin_rad = lon_grid_origin * M_PI / 180.0;
            double gamma = atan(tan(lon_rad - lon_grid_origin_rad) * sin(lat_rad)) * 180.0 / M_PI;
            // double gamma = sin(lat_rad) * (lon - lon_grid_origin)
            return gamma;
        }

        /*
@brief  CalculateMeridianConvergenceAngle as for Spherical Transverse Mercator, which gives quite good approximation
@param heading  true north (degree)  
@param lat  point latitude (degree)  
@param lon  point longitude (degree)
@param lon_grid_origin grid origin longitude (degree)
@return grid yaw
*/
        static inline double Heading2GridYaw(const double heading, const double lat, const double lon, const double lon_grid_origin)
        {
            return heading - CalculateMeridianConvergenceAngle(lat, lon, lon_grid_origin);
        }

        static inline void UTMZoneToEPSGCode(const std::string &UTMZone, size_t &EPSG_Code)
        {

            int zone_num = 0;
            int i = 0;
            while (i < UTMZone.size())
            {
                if (UTMZone[i] >= '0' && UTMZone[i] <= '9')
                {
                    zone_num *= 10;
                    zone_num += UTMZone[i] - '0';
                    i++;
                }
                else
                    break;
            }
            EPSG_Code = 32600 + zone_num;
        }

    } // namespace mapping
} // namespace dilu

#endif