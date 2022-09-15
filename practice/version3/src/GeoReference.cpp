

#include "GeoReference.hpp"

namespace pdal
{

    static PluginInfo const s_info{
        "filters.georeference",
        "Transform pointcloud from local lidar coordinate to world coordinate system ",
        "http://pdal.io/stages/filters.georeference.html"};

    CREATE_SHARED_STAGE(GeoReference, s_info)

    std::string GeoReference::getName() const
    {
        return s_info.name;
    }

    void GeoReference::addArgs(ProgramArgs &args)
    {
        args.add("tragictory_filename", "input tragictory filename", m_tragictory_filename).setPositional();

        args.add("matrix_dimensions", "rotation matrix3D  dimension names ", m_matrix_dimensions, {"rot11,rot22,rot33,rot21,rot22,rot23,rot31,rot32,rot33"});

        args.add("skip", "whether skip  point that look up gps_time failure", m_skip, true);
    }

    void GeoReference::initialize()
    {
    }

    void GeoReference::addDimensions(PointLayoutPtr layout)
    {
    }

    void GeoReference::prepared(PointTableRef table)
    {

        PointLayoutPtr layout(table.layout());

        if (!initIsometry3dBufferFromFile())
        {
            throwError("read tragictory from '" + m_tragictory_filename + "'failed");
        }

        if (layout->findDim("GpsTime") == Dimension::Id::Unknown)
        {
            throwError("Dimension GpsTime does not exist.");
        }

        else if (layout->findDim("X") == Dimension::Id::Unknown || layout->findDim("Y") == Dimension::Id::Unknown || layout->findDim("Z") == Dimension::Id::Unknown)
        {
            throwError("Dimension x y z  does not all exist .");
        }
    }

    bool GeoReference::initIsometry3dBufferFromFile()
    {
        GDALAllRegister();
        CPLSetConfigOption("GDAL_FILENAME_IS_UTF8", "NO");

        char **ppszOptions = NULL;
        GDALDataset *dataset = (GDALDataset *)(GDALOpenEx(m_tragictory_filename.c_str(), GDAL_OF_VECTOR, NULL, ppszOptions, NULL));
        if (dataset == NULL || dataset->GetLayerCount() < 1)
        {
            std::cout << " dataset open fail " << std::endl;
            return false;
        }
        OGRLayer *layer = dataset->GetLayer(0);

        layer->ResetReading();

        OGRFeature *feature;
        while ((feature = layer->GetNextFeature()) != NULL)
        {
            double gps_time = feature->GetFieldAsDouble("GpsTime");
            double x = feature->GetFieldAsDouble("X");
            double y = feature->GetFieldAsDouble("Y");
            double z = feature->GetFieldAsDouble("Z");

            Eigen::Matrix3d rotation;
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                {
                    int k = i * 3 + j;
                    rotation(i, j) = feature->GetFieldAsDouble(m_matrix_dimensions[k].c_str());
                }

            Eigen::Vector3d translation(x, y, z);

            Eigen::Isometry3d pose = dilu::mapping::translationAndRotationMatrixToIsometry3d(translation, rotation);

            m_buffer.add(gps_time, pose);
            OGRFeature::DestroyFeature(feature);
        }
        std::cout << "Interpolation buffer size is " << m_buffer.size() << std::endl;
        GDALClose(dataset);

        if (m_buffer.size() == 0)
            return false;

        return true;
    }

    bool GeoReference::processOne(PointRef &point)
    {

        double time = point.getFieldAs<double>(pdal::Dimension::Id::GpsTime);
        Eigen::Vector3d pt_local(point.getFieldAs<double>(pdal::Dimension::Id::X),
                                 point.getFieldAs<double>(pdal::Dimension::Id::Y),
                                 point.getFieldAs<double>(pdal::Dimension::Id::Z));

        if (!m_buffer.has(time))
        {
            // log()->get(LogLevel::Warning)
            //     << "look up buffer time failed ! currect pt gps_time is %.9f ,buffer  min gps_time is %.9f max gps_time is %.9f,buffer\n",
            //     time, m_buffer.minTime(), m_buffer.maxTime();

            // printf("look up buffer time failed ! currect pt gps_time is %.9f ,buffer  min gps_time is %.9f max gps_time is %.9f,buffer\n", time, m_buffer.minTime(), m_buffer.maxTime());
            if (m_skip)
            {
                point.setField(pdal::Dimension::Id::X, 0);
                point.setField(pdal::Dimension::Id::Y, 0);
                point.setField(pdal::Dimension::Id::Z, 0);
                return true;
            }
            else
            {
                point.setField(pdal::Dimension::Id::X, 0);
                point.setField(pdal::Dimension::Id::Y, 0);
                point.setField(pdal::Dimension::Id::Z, 0);
                return false;
            }
        }
        else
        {

            Eigen::Vector3d pt_world = m_buffer.lookup(time) * pt_local;

            point.setField(pdal::Dimension::Id::X, pt_world[0]);
            point.setField(pdal::Dimension::Id::Y, pt_world[1]);
            point.setField(pdal::Dimension::Id::Z, pt_world[2]);

            return true;
        }
    }

    void GeoReference::filter(PointView &view)
    {
        PointRef point(view, 0);
        for (PointId id = 0; id < view.size(); ++id)
        {
            point.setPointId(id);
            if (!processOne(point))
                break;
        }
    }

} // namespace pdal
