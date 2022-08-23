
#include "ogr_conversions.h"

namespace dilu
{
    namespace mapping
    {
        /*   
        void novatelInspvaxToGDALOGRLayer(const novatel_oem7_msgs::INSPVAX &msg, OGRLayer &layer)
        {

            std::vector<std::string> table_names{"time", "latitude", "longitude", "altitude", "heading", "pitch", "roll", "x", "y", "z", "epsg_code"};

            for (int i = 0; i < table_names.size() && layer.GetFeatureCount() == 0; i++)
            {
                std::string table_name = table_names[i];
                OGRFieldDefn field_temp(table_name.c_str(), OFTReal);
                field_temp.SetWidth(32);
                field_temp.SetPrecision(10);
                layer.CreateField(&field_temp);
            }

            OGRFeature *feature;
            feature = OGRFeature::CreateFeature(layer.GetLayerDefn());

            uint32_t utc_time_sec = (msg.nov_header.gps_week_number) * 604800 + (msg.nov_header.gps_week_milliseconds / 1000) + 315964800 - 18;
            uint32_t utc_time_nsec = (msg.nov_header.gps_week_milliseconds % 1000) * 1e+6;
            double unix_time = utc_time_sec + utc_time_nsec * 1e-9;

            feature->SetField("time", unix_time);
            feature->SetField("latitude", msg.latitude);
            feature->SetField("longitude", msg.longitude);
            feature->SetField("altitude", msg.height);
            feature->SetField("heading", msg.azimuth);
            feature->SetField("pitch", msg.pitch);
            feature->SetField("roll", msg.roll);

            double utm_x = 0;
            double utm_y = 0;
            double utm_meridian_convergence_;
            std::string utm_zone_ = "";
            RobotLocalization::NavsatConversions::LLtoUTM(msg.latitude, msg.longitude, utm_y, utm_x, utm_zone_, utm_meridian_convergence_);

            feature->SetField("x", utm_x);
            feature->SetField("y", utm_y);
            feature->SetField("z", msg.height);

            size_t epsg_code = 0;
            dilu::mapping::UTMZoneToEPSGCode(utm_zone_, epsg_code);
            feature->SetField("epsg_code", double(epsg_code));

            // OGRSpatialReference *spatialRefercence = layer.GetSpatialRef();
            // spatialRefercence->importFromEPSG(epsg_code);

            OGRPoint pt(utm_x, utm_y, msg.height);
            feature->SetGeometry(&pt);

            std::cout << layer.GetFeatureCount() << "\t" << msg.latitude << "\t" << msg.longitude << "\t" << msg.height << std::endl;

            if (layer.CreateFeature(feature) != OGRERR_NONE)
            {
                printf("Failed to create feature in shapefile.\n");
                return;
            }
            OGRFeature::DestroyFeature(feature);
        }
*/
        void isometry3dInterpolationBufferToGDALOGRLayer(const dilu::mapping::Isometry3dInterpolationBuffer &buffer, OGRLayer &layer, OGRCoordinateTransformation *trans)
        {

            std::vector<std::string> table_names{"time", "x", "y", "z", "heading", "pitch", "roll"};

            for (int i = 0; i < table_names.size() && layer.GetFeatureCount() == 0; i++)
            {
                std::string table_name = table_names[i];
                OGRFieldDefn field_temp(table_name.c_str(), OFTReal);
                field_temp.SetWidth(32);
                field_temp.SetPrecision(10);
                layer.CreateField(&field_temp);
            }

            for (size_t i = 0; i < buffer.size(); i++)
            {
                dilu::mapping::StampedIsometry3d stamped_pose = buffer.at(i);

                Eigen::Isometry3d pose = stamped_pose.pose;
                double time = stamped_pose.time;

                Eigen::Vector3d translation = pose.translation();
                Eigen::Matrix3d rotation = pose.rotation();
                double heading, pitch, roll;
                // dilu::mapping::matrix3dToYawPitchRoll(rotation, &yaw, &pitch, &roll);
                dilu::mapping::matrix3dToHeadingPitchRoll(rotation, &heading, &pitch, &roll);
                if (trans != NULL)
                {
                    trans->Transform(1, &translation[0], &translation[1], &translation[2]);
                }
                double x = translation[0];
                double y = translation[1];
                double z = translation[2];

                OGRFeature *feature = OGRFeature::CreateFeature(layer.GetLayerDefn());
                feature->SetField("time", time);
                feature->SetField("x", x);
                feature->SetField("y", y);
                feature->SetField("z", z);
                feature->SetField("heading", heading);
                feature->SetField("pitch", pitch);
                feature->SetField("roll", roll);

                OGRPoint pt(x, y, z);
                feature->SetGeometry(&pt);

                std::cout << layer.GetFeatureCount() << "-"
                          << "\t" << x << "\t" << y << "\t" << z << std::endl;

                if (layer.CreateFeature(feature) != OGRERR_NONE)
                {
                    printf("Failed to create feature in shapefile.\n");
                    return;
                }
                OGRFeature::DestroyFeature(feature);
            }
        }

        bool trajOGRLayerToIsometry3dInterpolationBuffer(OGRLayer &layer, dilu::mapping::Isometry3dInterpolationBufferWithGeo *buffer)
        {

            // layer.ResetReading();

            // OGRFeature *feature;
            // while ((feature = layer.GetNextFeature()) != NULL)
            // {

            //     double gps_time = feature->GetFieldAsDouble("GpsTime");
               
            //     double x = feature->GetFieldAsDouble("X");
            //     double y = feature->GetFieldAsDouble("Y");
            //     double z = feature->GetFieldAsDouble("Z");

            //     double rot11 = feature->GetFieldAsDouble("Rot11");
            //     double rot12 = feature->GetFieldAsDouble("Rot12");
            //     double rot13 = feature->GetFieldAsDouble("Rot13");
            //     double rot21 = feature->GetFieldAsDouble("Rot21");
            //     double rot22 = feature->GetFieldAsDouble("Rot22");
            //     double rot23 = feature->GetFieldAsDouble("Rot23");
            //     double rot31 = feature->GetFieldAsDouble("Rot31");
            //     double rot32 = feature->GetFieldAsDouble("Rot32");
            //     double rot33 = feature->GetFieldAsDouble("Rot33");

            //     Eigen::Matrix3d rotation;
            //     // rotation << rot11 << rot12 << rot13
            //     //          << rot21 << rot22 << rot23
            //     //          << rot31 << rot32 << rot33;
            //      Eigen::Vector3d translation(x,y,z);

            //     Eigen::Isometry3d pose = dilu::mapping::translationAndRotationMatrixToIsometry3d(translation,rotation);
            

            //     buffer->add(gps_time, pose);
            //     OGRFeature::DestroyFeature(feature);
            // }

            return true;
        }

       

    } // namespace mapping
} // namespace dilu
