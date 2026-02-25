#pragma once

#include "../../ros_compat.h"
#include <opencv2/highgui/highgui.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include <cassert>

typedef pcl::PointXYZI PointType;


extern int ROW;
extern int COL;
extern int FOCAL_LENGTH;
const int NUM_OF_CAM = 1;

extern std::string PROJECT_NAME;
extern std::string IMAGE_TOPIC;
extern std::string IMU_TOPIC;
extern std::string POINT_CLOUD_TOPIC;

extern int USE_LIDAR;
extern int LIDAR_SKIP;

extern std::string FISHEYE_MASK;
extern std::vector<std::string> CAM_NAMES;
extern int MAX_CNT;
extern int MIN_DIST;
extern int WINDOW_SIZE;
extern int FREQ;
extern double F_THRESHOLD;
extern int SHOW_TRACK;
extern int STEREO_TRACK;
extern int EQUALIZE;
extern int FISHEYE;
extern bool PUB_THIS_FRAME;

extern double L_C_TX;
extern double L_C_TY;
extern double L_C_TZ;
extern double L_C_RX;
extern double L_C_RY;
extern double L_C_RZ;


void readParameters(const std::string& config_file);

float pointDistance(PointType p);
float pointDistance(PointType p1, PointType p2);

void publishCloud(ros::Publisher<sensor_msgs::PointCloud2> *thisPub,
                  pcl::PointCloud<PointType>::Ptr thisCloud,
                  ros::Time thisStamp,
                  std::string thisFrame);
