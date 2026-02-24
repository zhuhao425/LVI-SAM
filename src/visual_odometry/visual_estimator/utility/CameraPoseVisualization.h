#pragma once
// CameraPoseVisualization.h - Stub. Visualization removed.
#include "../../../ros_compat.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>

class CameraPoseVisualization {
public:
    std::string m_marker_ns;
    CameraPoseVisualization(float, float, float, float) {}
    void setImageBoundaryColor(float, float, float, float = 1.0) {}
    void setOpticalCenterConnectorColor(float, float, float, float = 1.0) {}
    void setScale(double) {}
    void setLineWidth(double) {}
    void add_pose(const Eigen::Vector3d&, const Eigen::Quaterniond&) {}
    void reset() {}
    void publish_by(ros::Publisher<visualization_msgs::MarkerArray>&,
                    const std_msgs::Header&) {}
    void add_edge(const Eigen::Vector3d&, const Eigen::Vector3d&) {}
    void add_loopedge(const Eigen::Vector3d&, const Eigen::Vector3d&) {}
};
