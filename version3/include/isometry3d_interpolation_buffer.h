
#ifndef DILU_MAPPING_ISOMETRY3D_INTERPOLATION_BUFFER_H
#define DILU_MAPPING_ISOMETRY3D_INTERPOLATION_BUFFER_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
namespace dilu
{
    namespace mapping
    {
        typedef struct StampedIsometry3d_t
        {
            double time;
            Eigen::Isometry3d pose;
        } StampedIsometry3d;

        class Isometry3dInterpolationBuffer
        {
        public:
            Isometry3dInterpolationBuffer() {}

            // Adds a new Isometry3d transform to the buffer
            void add(const double time, const Eigen::Isometry3d &pose);

            void clear();

            bool has(const double time) const;

            Eigen::Isometry3d lookup(const double time) const;

            void transformBuffer(const Eigen::Isometry3d &transform);

            double minTime() const;

            double maxTime() const;

            bool empty() const;

            size_t size() const;

            StampedIsometry3d at(size_t index) const;

            void readStampPosesFormFile(const std::string &file);

        protected:
            std::vector<StampedIsometry3d> stamped_poses_;
        };

        class Isometry3dInterpolationBufferWithGeo : public Isometry3dInterpolationBuffer
        {

        public:
            int get_epsg_code()
            {
                return this->epsg_code;
            };

            void set_epsg_code(int epsg_code)
            {
                this->epsg_code = epsg_code;
            };

            void readStampPosesFormFile(const std::string &file_name);

        private:
            int epsg_code;
        };

    } // namespace mapping
} // namespace dilu

#endif // DILU_MAPPING_ISOMETRY3D_INTERPOLATION_BUFFER_H
