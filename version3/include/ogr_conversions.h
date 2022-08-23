
#ifndef DILU_MAPPING_OGR_CONVERSIONS_H
#define DILU_MAPPING_OGR_CONVERSIONS_H

// #include <sensor_msgs/PointCloud2.h>

#include <gdal_priv.h>
#include <ogr_core.h>
#include <ogr_api.h>
#include <ogr_feature.h>
#include <ogr_geometry.h>
#include <ogrsf_frmts.h>
#include <ogr_spatialref.h>

// #include "novatel_oem7_msgs/INSPVAX.h"
#include  <iostream>
#include "navsat_conversions.h"
#include "dilu_common.h"
#include "isometry3d_interpolation_buffer.h"
#include "pose.h"



namespace dilu
{
        namespace mapping
        {

                // void novatelInspvaxToGDALOGRLayer(const novatel_oem7_msgs::INSPVAX &msg, OGRLayer &layer);

                void isometry3dInterpolationBufferToGDALOGRLayer(const dilu::mapping::Isometry3dInterpolationBuffer &buffer, OGRLayer &layer, OGRCoordinateTransformation *trans = NULL);

                // TODO check required fields exist, set as params
                bool trajOGRLayerToIsometry3dInterpolationBuffer( OGRLayer &layer, dilu::mapping::Isometry3dInterpolationBufferWithGeo *buffer);

       
        } // namespace dilu
} // namespace mapping

#endif // DILU_MAPPING_OGR_CONVERSIONS_H
