#pragma once
// cloud_info.h - Standalone replacement for lvi_sam/cloud_info.h (generated ROS message).

#include "../ros_compat.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lvi_sam {

struct cloud_info {
    using ConstPtr = std::shared_ptr<const cloud_info>;

    std_msgs::Header header;

    std::vector<int32_t> startRingIndex;
    std::vector<int32_t> endRingIndex;

    std::vector<int32_t> pointColInd;   // point column index in range image
    std::vector<float>   pointRange;    // point range

    int64_t imuAvailable  = 0;
    int64_t odomAvailable = 0;

    // Attitude for lidar odometry initialization
    float imuRollInit  = 0;
    float imuPitchInit = 0;
    float imuYawInit   = 0;

    // Odometry
    float odomX     = 0;
    float odomY     = 0;
    float odomZ     = 0;
    float odomRoll  = 0;
    float odomPitch = 0;
    float odomYaw   = 0;

    int64_t odomResetId = 0;

    // Point cloud data stored as serialized PointCloud2 (for compatibility with
    // pcl::fromROSMsg / pcl::toROSMsg used in the existing pipeline).
    sensor_msgs::PointCloud2 cloud_deskewed;
    sensor_msgs::PointCloud2 cloud_corner;
    sensor_msgs::PointCloud2 cloud_surface;
};

using cloud_infoConstPtr = std::shared_ptr<const cloud_info>;

} // namespace lvi_sam
