// lvi_sam_node.cpp - Single-process, multi-threaded entry point for LVI-SAM.
//
// All pipeline components (lidar odometry, visual feature tracking, visual
// odometry estimation, and loop detection) are instantiated here as threads
// within a single OS process.  Internal messages are exchanged through the
// in-process pub/sub broker provided by ros_compat.h — no middleware
// serialisation or inter-process communication is involved.  Only the
// external sensor streams (IMU, camera, LiDAR) need to be injected via the
// NodeInterface (ros::NodeHandle / ros::Publisher / ros::Subscriber).

#define LVI_SAM_COMBINED_NODE

#include "ros_compat.h"

#include <thread>
#include <string>

// ---------------------------------------------------------------------------
// Lidar odometry startup functions (defined in each lidar .cpp file).
// Each function creates a persistent algorithm instance and registers its
// subscriptions with the in-process broker.
// ---------------------------------------------------------------------------
void startImageProjectionNode();
void startFeatureExtractionNode();
void startIMUPreintegrationNode();
void startMapOptimizationNode(std::thread& loopThread, std::thread& visThread);

// ---------------------------------------------------------------------------
// Visual odometry startup functions.
// ---------------------------------------------------------------------------
// Initialises global state, registers subscriptions/publishers for the visual
// feature tracker.  Config is read from config_file (camera YAML).
void initFeatureTracker(const std::string& config_file);

// Initialises global state, registers subscriptions/publishers for the visual
// estimator, and returns the VIO processing thread.
std::thread initEstimatorNode(const std::string& config_file);

// Initialises global state, registers subscriptions/publishers for the loop
// detector, and returns the loop processing thread.
std::thread initLoopDetection(const std::string& config_file);

// ---------------------------------------------------------------------------
// Combined main
// ---------------------------------------------------------------------------
int main(int argc, char** argv)
{
    // ros::init parses --lidar-config and --camera-config from argv and loads
    // both YAML files into the in-process parameter store.
    ros::init(argc, argv, "lvi_sam");

    // Determine the camera config file path (needed by visual components).
    std::string camera_cfg;
    if (!ros::internal::g_camera_config_file.empty())
        camera_cfg = ros::internal::g_camera_config_file;
    else {
        // Fall back: accept the first positional argument as the camera config.
        for (int i = 1; i < argc; ++i) {
            std::string a = argv[i];
            if (a[0] != '-') { camera_cfg = a; break; }
        }
    }
    if (camera_cfg.empty()) {
        ROS_ERROR("Usage: lvi_sam --lidar-config <lidar.yaml> --camera-config <camera.yaml>");
        return 1;
    }

    ROS_INFO("\033[1;32m====> LVI-SAM Combined Node Starting.\033[0m");

    // ------------------------------------------------------------------
    // 1. Lidar pipeline
    //    Each call creates a persistent object whose constructor registers
    //    all subscriptions with the in-process broker.
    // ------------------------------------------------------------------
    startImageProjectionNode();
    startFeatureExtractionNode();
    startIMUPreintegrationNode();

    std::thread loopClosureThread, visualizeMapThread;
    startMapOptimizationNode(loopClosureThread, visualizeMapThread);

    // ------------------------------------------------------------------
    // 2. Visual pipeline
    //    Each call initialises global state and registers subscriptions.
    //    The process threads returned here run the main estimation loop.
    // ------------------------------------------------------------------
    initFeatureTracker(camera_cfg);
    std::thread estimatorThread  = initEstimatorNode(camera_cfg);
    std::thread loopDetectThread = initLoopDetection(camera_cfg);

    ROS_INFO("\033[1;32m====> LVI-SAM Combined Node Running (single process, multi-threaded).\033[0m");

    // ------------------------------------------------------------------
    // 3. Spin: the in-process broker delivers published messages
    //    synchronously to all registered callbacks.
    // ------------------------------------------------------------------
    ros::spin();

    // ------------------------------------------------------------------
    // 4. Cleanup
    // ------------------------------------------------------------------
    if (estimatorThread.joinable())   estimatorThread.join();
    if (loopDetectThread.joinable())  loopDetectThread.join();
    if (loopClosureThread.joinable()) loopClosureThread.join();
    if (visualizeMapThread.joinable()) visualizeMapThread.join();

    return 0;
}
