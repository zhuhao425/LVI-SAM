#pragma once
// visualization.h - Visualization removed (no ROS dependency).
// Stubs remain for compatibility with existing code that calls pub functions.

#include "../../../ros_compat.h"
#include "../estimator.h"
#include "../parameters.h"

// These publishers are defined but publish to topics with no subscribers;
// all visualization calls become no-ops.
extern ros::Publisher<nav_msgs::Odometry>   pub_odometry;
extern ros::Publisher<nav_msgs::Path>       pub_path;
extern ros::Publisher<nav_msgs::Odometry>   pub_pose;

extern int IMAGE_ROW, IMAGE_COL;

void registerPub(ros::NodeHandle &n);

void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q,
                       const Eigen::Vector3d &V, const std_msgs::Header &header,
                       const int &failureId);

void printStatistics(const Estimator &estimator, double t);

void pubOdometry(const Estimator &estimator, const std_msgs::Header &header);
void pubInitialGuess(const Estimator &estimator, const std_msgs::Header &header);
void pubKeyPoses(const Estimator &estimator, const std_msgs::Header &header);
void pubCameraPose(const Estimator &estimator, const std_msgs::Header &header);
void pubPointCloud(const Estimator &estimator, const std_msgs::Header &header);
void pubTF(const Estimator &estimator, const std_msgs::Header &header);
void pubKeyframe(const Estimator &estimator);
void pubRelocalization(const Estimator &estimator);
