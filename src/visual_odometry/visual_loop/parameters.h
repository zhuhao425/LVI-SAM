#pragma once

#include "../../ros_compat.h"

#include <eigen3/Eigen/Dense>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

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

#include "ThirdParty/DBoW/DBoW2.h"
#include "ThirdParty/DVision/DVision.h"
#include "ThirdParty/DBoW/TemplatedDatabase.h"
#include "ThirdParty/DBoW/TemplatedVocabulary.h"

#include "../visual_feature/camera_models/CameraFactory.h"
#include "../visual_feature/camera_models/CataCamera.h"
#include "../visual_feature/camera_models/PinholeCamera.h"

using namespace std;

extern camodocal::CameraPtr m_camera;

extern Eigen::Vector3d tic;
extern Eigen::Matrix3d qic;

extern string PROJECT_NAME;
extern string IMAGE_TOPIC;

extern int DEBUG_IMAGE;
extern int LOOP_CLOSURE;
extern double MATCH_IMAGE_SCALE;

// Publishers (visualization removed; pub_match_msg is functional for loop closure info)
extern ros::Publisher<std_msgs::Float64MultiArray> pub_match_msg;
extern ros::Publisher<sensor_msgs::Image>          pub_match_img;
extern ros::Publisher<visualization_msgs::MarkerArray> pub_key_pose;


class BriefExtractor
{
public:

    DVision::BRIEF m_brief;

    virtual void operator()(const cv::Mat &im, vector<cv::KeyPoint> &keys, vector<DVision::BRIEF::bitset> &descriptors) const
    {
        m_brief.compute(im, keys, descriptors);
    }

    BriefExtractor(){};

    BriefExtractor(const std::string &pattern_file)
    {
        cv::FileStorage fs(pattern_file.c_str(), cv::FileStorage::READ);
        if (!fs.isOpened()) throw string("Could not open file ") + pattern_file;

        vector<int> x1, y1, x2, y2;
        fs["x1"] >> x1;
        fs["x2"] >> x2;
        fs["y1"] >> y1;
        fs["y2"] >> y2;

        m_brief.importPairs(x1, y1, x2, y2);
    }
};

extern BriefExtractor briefExtractor;
