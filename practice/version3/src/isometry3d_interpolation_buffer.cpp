
#include "isometry3d_interpolation_buffer.h"

// #include <glog/logging.h>
#include <algorithm>

#include "pose.h"

namespace dilu
{
    namespace mapping
    {

        void Isometry3dInterpolationBuffer::add(const double time, const Eigen::Isometry3d &pose)
        {

            if (!stamped_poses_.empty())
            {

                if (time < maxTime())
                {
                    // std::cout << "Warning pose " <<time<< std::endl;
                    printf("trajectory file GpsTime not sorted  %.9f ,current maxTime is %.9f ",time,maxTime());
                    exit(-1);
                }
                // CHECK_GE(time, maxTime()) << "New transform is older than latest.";
            }
            stamped_poses_.push_back(StampedIsometry3d{time, pose});
        }

        void Isometry3dInterpolationBuffer::clear()
        {
            stamped_poses_.clear();
        }

        bool Isometry3dInterpolationBuffer::has(const double time) const
        {
            if (stamped_poses_.empty())
                return false;

            return minTime() <= time && time <= maxTime();
        }

        Eigen::Isometry3d Isometry3dInterpolationBuffer::lookup(const double time) const
        {
            //CHECK(has(time)) << "Missing transform for: " << time;

            const auto end = std::lower_bound(
                stamped_poses_.begin(), stamped_poses_.end(), time,
                [](const StampedIsometry3d &stamped_pose,
                   const double time)
                {
                    return stamped_pose.time < time;
                });

            if (end->time == time)
            {
                return end->pose;
            }

            const auto start = std::prev(end);
            return linearInterpolateTimestampedIsometry3d(start->pose, start->time, end->pose, end->time, time);
        }

        void Isometry3dInterpolationBuffer::transformBuffer(const Eigen::Isometry3d &transform)
        {
            for (size_t i = 0; i < stamped_poses_.size(); ++i)
            {
                stamped_poses_[i].pose = stamped_poses_[i].pose * transform;
            }
        }

        StampedIsometry3d Isometry3dInterpolationBuffer::at(size_t index) const
        {
            return stamped_poses_[index];
        };

        double Isometry3dInterpolationBuffer::minTime() const
        {
            //CHECK(!empty()) << "Empty buffer.";
            return stamped_poses_.front().time;
        }

        double Isometry3dInterpolationBuffer::maxTime() const
        {
            //CHECK(!empty()) << "Empty buffer.";
            return stamped_poses_.back().time;
        }

        bool Isometry3dInterpolationBuffer::empty() const
        {
            return (stamped_poses_.empty());
        }

        size_t Isometry3dInterpolationBuffer::size() const
        {
            return stamped_poses_.size();
        }

    } // namespace mapping
} // namespace dilu
