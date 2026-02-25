#pragma once

#include "../ros_compat.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <ctime>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

typedef pcl::PointXYZI PointType;


class ParamServer
{
public:

    ros::NodeHandle nh;

    std::string PROJECT_NAME;
    std::string robot_id;

    std::string pointCloudTopic;
    std::string imuTopic;
    std::string odomTopic;
    std::string gpsTopic;

    bool useImuHeadingInitialization;
    bool useGpsElevation;
    float gpsCovThreshold;
    float poseCovThreshold;

    bool savePCD;
    std::string savePCDDirectory;

    int N_SCAN;
    int Horizon_SCAN;
    std::string timeField;
    int downsampleRate;

    float imuAccNoise;
    float imuGyrNoise;
    float imuAccBiasN;
    float imuGyrBiasN;
    float imuGravity;
    std::vector<double> extRotV;
    std::vector<double> extRPYV;
    std::vector<double> extTransV;
    Eigen::Matrix3d extRot;
    Eigen::Matrix3d extRPY;
    Eigen::Vector3d extTrans;
    Eigen::Quaterniond extQRPY;

    float edgeThreshold;
    float surfThreshold;
    int edgeFeatureMinValidNum;
    int surfFeatureMinValidNum;

    float odometrySurfLeafSize;
    float mappingCornerLeafSize;
    float mappingSurfLeafSize;

    float z_tollerance;
    float rotation_tollerance;

    int numberOfCores;
    double mappingProcessInterval;

    float surroundingkeyframeAddingDistThreshold;
    float surroundingkeyframeAddingAngleThreshold;
    float surroundingKeyframeDensity;
    float surroundingKeyframeSearchRadius;

    bool loopClosureEnableFlag;
    int   surroundingKeyframeSize;
    float historyKeyframeSearchRadius;
    float historyKeyframeSearchTimeDiff;
    int   historyKeyframeSearchNum;
    float historyKeyframeFitnessScore;

    float globalMapVisualizationSearchRadius;
    float globalMapVisualizationPoseDensity;
    float globalMapVisualizationLeafSize;

    ParamServer()
    {
        nh.param<std::string>("/PROJECT_NAME", PROJECT_NAME, "lvi_sam");
        nh.param<std::string>("/robot_id", robot_id, "roboat");

        nh.param<std::string>(PROJECT_NAME + "/pointCloudTopic", pointCloudTopic, "points_raw");
        nh.param<std::string>(PROJECT_NAME + "/imuTopic",        imuTopic,        "imu_correct");
        nh.param<std::string>(PROJECT_NAME + "/odomTopic",       odomTopic,       "odometry/imu");
        nh.param<std::string>(PROJECT_NAME + "/gpsTopic",        gpsTopic,        "odometry/gps");

        nh.param<bool> (PROJECT_NAME + "/useImuHeadingInitialization", useImuHeadingInitialization, false);
        nh.param<bool> (PROJECT_NAME + "/useGpsElevation",             useGpsElevation,             false);
        nh.param<float>(PROJECT_NAME + "/gpsCovThreshold",             gpsCovThreshold,             2.0f);
        nh.param<float>(PROJECT_NAME + "/poseCovThreshold",            poseCovThreshold,            25.0f);

        nh.param<bool>       (PROJECT_NAME + "/savePCD",          savePCD,          false);
        nh.param<std::string>(PROJECT_NAME + "/savePCDDirectory", savePCDDirectory, "/tmp/loam/");

        nh.param<int>        (PROJECT_NAME + "/N_SCAN",        N_SCAN,        16);
        nh.param<int>        (PROJECT_NAME + "/Horizon_SCAN",  Horizon_SCAN,  1800);
        nh.param<std::string>(PROJECT_NAME + "/timeField",     timeField,     "time");
        nh.param<int>        (PROJECT_NAME + "/downsampleRate",downsampleRate, 1);

        nh.param<float>(PROJECT_NAME + "/imuAccNoise",  imuAccNoise,  0.01f);
        nh.param<float>(PROJECT_NAME + "/imuGyrNoise",  imuGyrNoise,  0.001f);
        nh.param<float>(PROJECT_NAME + "/imuAccBiasN",  imuAccBiasN,  0.0002f);
        nh.param<float>(PROJECT_NAME + "/imuGyrBiasN",  imuGyrBiasN,  0.00003f);
        nh.param<float>(PROJECT_NAME + "/imuGravity",   imuGravity,   9.80511f);

        nh.param<std::vector<double>>(PROJECT_NAME + "/extrinsicRot",   extRotV,   std::vector<double>());
        nh.param<std::vector<double>>(PROJECT_NAME + "/extrinsicRPY",   extRPYV,   std::vector<double>());
        nh.param<std::vector<double>>(PROJECT_NAME + "/extrinsicTrans", extTransV, std::vector<double>());

        if (extRotV.size() == 9)
            extRot = Eigen::Map<const Eigen::Matrix<double,-1,-1,Eigen::RowMajor>>(extRotV.data(), 3, 3);
        else
            extRot = Eigen::Matrix3d::Identity();

        if (extRPYV.size() == 9)
            extRPY = Eigen::Map<const Eigen::Matrix<double,-1,-1,Eigen::RowMajor>>(extRPYV.data(), 3, 3);
        else
            extRPY = Eigen::Matrix3d::Identity();

        if (extTransV.size() == 3)
            extTrans = Eigen::Map<const Eigen::Matrix<double,-1,-1,Eigen::RowMajor>>(extTransV.data(), 3, 1);
        else
            extTrans = Eigen::Vector3d::Zero();

        extQRPY = Eigen::Quaterniond(extRPY);

        nh.param<float>(PROJECT_NAME + "/edgeThreshold",          edgeThreshold,          0.1f);
        nh.param<float>(PROJECT_NAME + "/surfThreshold",          surfThreshold,          0.1f);
        nh.param<int>  (PROJECT_NAME + "/edgeFeatureMinValidNum", edgeFeatureMinValidNum, 10);
        nh.param<int>  (PROJECT_NAME + "/surfFeatureMinValidNum", surfFeatureMinValidNum, 100);

        nh.param<float>(PROJECT_NAME + "/odometrySurfLeafSize",  odometrySurfLeafSize,  0.2f);
        nh.param<float>(PROJECT_NAME + "/mappingCornerLeafSize", mappingCornerLeafSize, 0.2f);
        nh.param<float>(PROJECT_NAME + "/mappingSurfLeafSize",   mappingSurfLeafSize,   0.2f);

        nh.param<float>(PROJECT_NAME + "/z_tollerance",        z_tollerance,        FLT_MAX);
        nh.param<float>(PROJECT_NAME + "/rotation_tollerance", rotation_tollerance, FLT_MAX);

        nh.param<int>   (PROJECT_NAME + "/numberOfCores",          numberOfCores,          2);
        nh.param<double>(PROJECT_NAME + "/mappingProcessInterval", mappingProcessInterval, 0.15);

        nh.param<float>(PROJECT_NAME + "/surroundingkeyframeAddingDistThreshold",  surroundingkeyframeAddingDistThreshold,  1.0f);
        nh.param<float>(PROJECT_NAME + "/surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold, 0.2f);
        nh.param<float>(PROJECT_NAME + "/surroundingKeyframeDensity",              surroundingKeyframeDensity,              1.0f);
        nh.param<float>(PROJECT_NAME + "/surroundingKeyframeSearchRadius",         surroundingKeyframeSearchRadius,         50.0f);

        nh.param<bool> (PROJECT_NAME + "/loopClosureEnableFlag",         loopClosureEnableFlag,         false);
        nh.param<int>  (PROJECT_NAME + "/surroundingKeyframeSize",       surroundingKeyframeSize,       50);
        nh.param<float>(PROJECT_NAME + "/historyKeyframeSearchRadius",   historyKeyframeSearchRadius,   10.0f);
        nh.param<float>(PROJECT_NAME + "/historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff, 30.0f);
        nh.param<int>  (PROJECT_NAME + "/historyKeyframeSearchNum",      historyKeyframeSearchNum,      25);
        nh.param<float>(PROJECT_NAME + "/historyKeyframeFitnessScore",   historyKeyframeFitnessScore,   0.3f);

        nh.param<float>(PROJECT_NAME + "/globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius, 1e3f);
        nh.param<float>(PROJECT_NAME + "/globalMapVisualizationPoseDensity",  globalMapVisualizationPoseDensity,  10.0f);
        nh.param<float>(PROJECT_NAME + "/globalMapVisualizationLeafSize",     globalMapVisualizationLeafSize,     1.0f);

        usleep(100);
    }

    sensor_msgs::Imu imuConverter(const sensor_msgs::Imu& imu_in)
    {
        sensor_msgs::Imu imu_out = imu_in;
        Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
        acc = extRot * acc;
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();
        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
        gyr = extRot * gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();
        Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x,
                                  imu_in.orientation.y, imu_in.orientation.z);
        Eigen::Quaterniond q_final = q_from * extQRPY;
        imu_out.orientation.x = q_final.x();
        imu_out.orientation.y = q_final.y();
        imu_out.orientation.z = q_final.z();
        imu_out.orientation.w = q_final.w();

        if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() +
                 q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
        {
            ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
            ros::shutdown();
        }
        return imu_out;
    }
};

template<typename T>
sensor_msgs::PointCloud2 publishCloud(ros::Publisher<sensor_msgs::PointCloud2> *thisPub,
                                      T thisCloud,
                                      ros::Time thisStamp,
                                      std::string thisFrame)
{
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp    = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub->getNumSubscribers() != 0)
        thisPub->publish(tempCloud);
    return tempCloud;
}

template<typename T>
double ROS_TIME(T msg)
{
    return msg->header.stamp.toSec();
}

template<typename T>
void imuAngular2rosAngular(sensor_msgs::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z)
{
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}

template<typename T>
void imuAccel2rosAccel(sensor_msgs::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z)
{
    *acc_x = thisImuMsg->linear_acceleration.x;
    *acc_y = thisImuMsg->linear_acceleration.y;
    *acc_z = thisImuMsg->linear_acceleration.z;
}

template<typename T>
void imuRPY2rosRPY(sensor_msgs::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw)
{
    double imuRoll, imuPitch, imuYaw;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(thisImuMsg->orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);
    *rosRoll  = imuRoll;
    *rosPitch = imuPitch;
    *rosYaw   = imuYaw;
}

float pointDistance(PointType p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}

float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y)+(p1.z-p2.z)*(p1.z-p2.z));
}
