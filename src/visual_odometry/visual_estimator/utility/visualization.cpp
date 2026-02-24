// visualization.cpp - Visualization removed. All pub functions are stubs.
#include "visualization.h"

using namespace Eigen;

// Publishers declared but publish to topics with no subscribers
ros::Publisher<nav_msgs::Odometry>   pub_odometry;
ros::Publisher<nav_msgs::Odometry>   pub_latest_odometry;
ros::Publisher<nav_msgs::Odometry>   pub_latest_odometry_ros;
ros::Publisher<nav_msgs::Path>       pub_path;
ros::Publisher<nav_msgs::Odometry>   pub_pose;

int IMAGE_ROW = 540;
int IMAGE_COL = 720;

void registerPub(ros::NodeHandle &n)
{
    pub_latest_odometry     = n.advertise<nav_msgs::Odometry>(PROJECT_NAME + "/vins/odometry/imu_propagate",     1000);
    pub_latest_odometry_ros = n.advertise<nav_msgs::Odometry>(PROJECT_NAME + "/vins/odometry/imu_propagate_ros", 1000);
    pub_path                = n.advertise<nav_msgs::Path>    (PROJECT_NAME + "/vins/odometry/path",              1000);
    pub_odometry            = n.advertise<nav_msgs::Odometry>(PROJECT_NAME + "/vins/odometry/odometry",          1000);
}

void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q,
                       const Eigen::Vector3d &V, const std_msgs::Header &header,
                       const int& failureId)
{
    // Publish imu odometry (used by imageProjection for deskew)
    nav_msgs::Odometry odometry;
    odometry.header         = header;
    odometry.header.frame_id = "vins_world";
    odometry.child_frame_id  = "vins_body";
    odometry.pose.pose.position.x    = P.x();
    odometry.pose.pose.position.y    = P.y();
    odometry.pose.pose.position.z    = P.z();
    odometry.pose.pose.orientation.x = Q.x();
    odometry.pose.pose.orientation.y = Q.y();
    odometry.pose.pose.orientation.z = Q.z();
    odometry.pose.pose.orientation.w = Q.w();
    odometry.twist.twist.linear.x    = V.x();
    odometry.twist.twist.linear.y    = V.y();
    odometry.twist.twist.linear.z    = V.z();
    pub_latest_odometry.publish(odometry);

    // Publish odometry in ROS format (with rotation changed) for lidar pipeline
    odometry.pose.covariance[0] = double(failureId);

    Eigen::Quaterniond q_odom_cam = Q;
    Eigen::Quaterniond q_cam_to_lidar(0, 1, 0, 0); // camera - lidar rotation
    Eigen::Quaterniond q_odom_ros = q_odom_cam * q_cam_to_lidar;
    odometry.pose.pose.orientation.x = q_odom_ros.x();
    odometry.pose.pose.orientation.y = q_odom_ros.y();
    odometry.pose.pose.orientation.z = q_odom_ros.z();
    odometry.pose.pose.orientation.w = q_odom_ros.w();
    pub_latest_odometry_ros.publish(odometry);
}

void printStatistics(const Estimator &estimator, double t)
{
    if (estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;
    printf("position: %f, %f, %f\r",
           estimator.Ps[WINDOW_SIZE].x(),
           estimator.Ps[WINDOW_SIZE].y(),
           estimator.Ps[WINDOW_SIZE].z());

    if (ESTIMATE_EXTRINSIC)
    {
        cv::FileStorage fs(EX_CALIB_RESULT_PATH, cv::FileStorage::WRITE);
        Eigen::Matrix3d eigen_R = estimator.ric[0];
        Eigen::Vector3d eigen_T = estimator.tic[0];
        cv::Mat cv_R, cv_T;
        cv::eigen2cv(eigen_R, cv_R);
        cv::eigen2cv(eigen_T, cv_T);
        fs << "extrinsicRotation" << cv_R << "extrinsicTranslation" << cv_T;
        fs.release();
    }
    if (ESTIMATE_TD)
        ROS_INFO("td %f", estimator.td);
}

void pubOdometry(const Estimator &estimator, const std_msgs::Header &header)
{
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        nav_msgs::Odometry odometry;
        odometry.header          = header;
        odometry.header.frame_id = "vins_world";
        odometry.child_frame_id  = "vins_world";
        Eigen::Quaterniond tmp_Q(estimator.Rs[WINDOW_SIZE]);
        odometry.pose.pose.position.x    = estimator.Ps[WINDOW_SIZE].x();
        odometry.pose.pose.position.y    = estimator.Ps[WINDOW_SIZE].y();
        odometry.pose.pose.position.z    = estimator.Ps[WINDOW_SIZE].z();
        odometry.pose.pose.orientation.x = tmp_Q.x();
        odometry.pose.pose.orientation.y = tmp_Q.y();
        odometry.pose.pose.orientation.z = tmp_Q.z();
        odometry.pose.pose.orientation.w = tmp_Q.w();
        odometry.twist.twist.linear.x    = estimator.Vs[WINDOW_SIZE].x();
        odometry.twist.twist.linear.y    = estimator.Vs[WINDOW_SIZE].y();
        odometry.twist.twist.linear.z    = estimator.Vs[WINDOW_SIZE].z();
        pub_odometry.publish(odometry);
    }
}

// Remaining visualization functions are no-ops (visualization removed)
void pubInitialGuess(const Estimator &, const std_msgs::Header &) {}
void pubKeyPoses   (const Estimator &, const std_msgs::Header &) {}
void pubCameraPose (const Estimator &, const std_msgs::Header &) {}
void pubPointCloud (const Estimator &, const std_msgs::Header &) {}
void pubTF         (const Estimator &, const std_msgs::Header &) {}
void pubKeyframe   (const Estimator &estimator)
{
    // Publish keyframe pose/point for loop detection
    // (loop detection subscriber will handle it)
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR &&
        estimator.marginalization_flag == 0)
    {
        // pubKeyframe_pose and pubKeyframe_point - not implemented in standalone
    }
}
void pubRelocalization(const Estimator &) {}
