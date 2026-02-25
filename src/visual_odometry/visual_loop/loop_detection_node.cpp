#include "parameters.h"
#include "keyframe.h"
#include "loop_detection.h"

using namespace Eigen;
using namespace std;

#define SKIP_FIRST_CNT 10

queue<sensor_msgs::ImageConstPtr>           image_buf;
queue<sensor_msgs::PointCloudConstPtr>      point_buf;
queue<nav_msgs::OdometryConstPtr>           pose_buf;

std::mutex m_buf;
std::mutex m_process;

LoopDetector loopDetector;

double SKIP_TIME = 0;
double SKIP_DIST = 0;

camodocal::CameraPtr m_camera;

Eigen::Vector3d tic;
Eigen::Matrix3d qic;

std::string PROJECT_NAME;
std::string IMAGE_TOPIC;

int DEBUG_IMAGE;
int LOOP_CLOSURE;
double MATCH_IMAGE_SCALE;


ros::Publisher<std_msgs::Float64MultiArray>    pub_match_msg;
ros::Publisher<sensor_msgs::Image>             pub_match_img;
ros::Publisher<visualization_msgs::MarkerArray> pub_key_pose;



BriefExtractor briefExtractor;

void new_sequence()
{
    m_buf.lock();
    while (!image_buf.empty()) image_buf.pop();
    while (!point_buf.empty()) point_buf.pop();
    while (!pose_buf.empty())  pose_buf.pop();
    m_buf.unlock();
}

void image_callback(const sensor_msgs::ImageConstPtr &image_msg)
{
    if (!LOOP_CLOSURE)
        return;

    m_buf.lock();
    image_buf.push(image_msg);
    m_buf.unlock();

    static double last_image_time = -1;
    if (last_image_time == -1)
        last_image_time = image_msg->header.stamp.toSec();
    else if (image_msg->header.stamp.toSec() - last_image_time > 1.0 ||
             image_msg->header.stamp.toSec() < last_image_time)
    {
        ROS_WARN("image discontinue! detect a new sequence!");
        new_sequence();
    }
    last_image_time = image_msg->header.stamp.toSec();
}

void point_callback(const sensor_msgs::PointCloudConstPtr &point_msg)
{
    if (!LOOP_CLOSURE)
        return;
    m_buf.lock();
    point_buf.push(point_msg);
    m_buf.unlock();
}

void pose_callback(const nav_msgs::OdometryConstPtr &pose_msg)
{
    if (!LOOP_CLOSURE)
        return;
    m_buf.lock();
    pose_buf.push(pose_msg);
    m_buf.unlock();
}

void extrinsic_callback(const nav_msgs::OdometryConstPtr &pose_msg)
{
    m_process.lock();
    tic = Eigen::Vector3d(pose_msg->pose.pose.position.x,
                          pose_msg->pose.pose.position.y,
                          pose_msg->pose.pose.position.z);
    qic = Eigen::Quaterniond(pose_msg->pose.pose.orientation.w,
                             pose_msg->pose.pose.orientation.x,
                             pose_msg->pose.pose.orientation.y,
                             pose_msg->pose.pose.orientation.z).toRotationMatrix();
    m_process.unlock();
}

void process()
{
    if (!LOOP_CLOSURE)
        return;

    while (ros::ok())
    {
        sensor_msgs::ImageConstPtr      image_msg = nullptr;
        sensor_msgs::PointCloudConstPtr point_msg = nullptr;
        nav_msgs::OdometryConstPtr      pose_msg  = nullptr;

        m_buf.lock();
        if (!image_buf.empty() && !point_buf.empty() && !pose_buf.empty())
        {
            if (image_buf.front()->header.stamp.toSec() > pose_buf.front()->header.stamp.toSec())
            {
                pose_buf.pop();
                printf("throw pose at beginning\n");
            }
            else if (image_buf.front()->header.stamp.toSec() > point_buf.front()->header.stamp.toSec())
            {
                point_buf.pop();
                printf("throw point at beginning\n");
            }
            else if (image_buf.back()->header.stamp.toSec()  >= pose_buf.front()->header.stamp.toSec() &&
                     point_buf.back()->header.stamp.toSec()  >= pose_buf.front()->header.stamp.toSec())
            {
                pose_msg = pose_buf.front();
                pose_buf.pop();
                while (!pose_buf.empty()) pose_buf.pop();
                while (image_buf.front()->header.stamp.toSec() < pose_msg->header.stamp.toSec())
                    image_buf.pop();
                image_msg = image_buf.front();
                image_buf.pop();

                while (point_buf.front()->header.stamp.toSec() < pose_msg->header.stamp.toSec())
                    point_buf.pop();
                point_msg = point_buf.front();
                point_buf.pop();
            }
        }
        m_buf.unlock();

        if (pose_msg != nullptr)
        {
            static int skip_first_cnt = 0;
            if (skip_first_cnt < SKIP_FIRST_CNT)
            {
                skip_first_cnt++;
                continue;
            }

            static double last_skip_time = -1;
            if (pose_msg->header.stamp.toSec() - last_skip_time < SKIP_TIME)
                continue;
            else
                last_skip_time = pose_msg->header.stamp.toSec();

            static Eigen::Vector3d last_t(-1e6, -1e6, -1e6);
            Eigen::Vector3d T(pose_msg->pose.pose.position.x,
                              pose_msg->pose.pose.position.y,
                              pose_msg->pose.pose.position.z);
            Eigen::Matrix3d R = Eigen::Quaterniond(pose_msg->pose.pose.orientation.w,
                                                   pose_msg->pose.pose.orientation.x,
                                                   pose_msg->pose.pose.orientation.y,
                                                   pose_msg->pose.pose.orientation.z).toRotationMatrix();

            if ((T - last_t).norm() > SKIP_DIST)
            {
                cv_bridge::CvImageConstPtr ptr;
                if (image_msg->encoding == "8UC1")
                {
                    sensor_msgs::Image img;
                    img.header       = image_msg->header;
                    img.height       = image_msg->height;
                    img.width        = image_msg->width;
                    img.is_bigendian = image_msg->is_bigendian;
                    img.step         = image_msg->step;
                    img.image        = image_msg->image;
                    img.encoding     = "mono8";
                    auto img_ptr     = std::make_shared<sensor_msgs::Image>(img);
                    ptr = cv_bridge::toCvCopy(img_ptr, sensor_msgs::image_encodings::MONO8);
                }
                else
                {
                    ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
                }
                cv::Mat image = ptr->image;

                vector<cv::Point3f> point_3d;
                vector<cv::Point2f> point_2d_uv;
                vector<cv::Point2f> point_2d_normal;
                vector<double> point_id;

                for (unsigned int i = 0; i < point_msg->points.size(); i++)
                {
                    cv::Point3f p_3d;
                    p_3d.x = point_msg->points[i].x;
                    p_3d.y = point_msg->points[i].y;
                    p_3d.z = point_msg->points[i].z;
                    point_3d.push_back(p_3d);

                    cv::Point2f p_2d_uv, p_2d_normal;
                    double p_id;
                    p_2d_normal.x = point_msg->channels[i].values[0];
                    p_2d_normal.y = point_msg->channels[i].values[1];
                    p_2d_uv.x     = point_msg->channels[i].values[2];
                    p_2d_uv.y     = point_msg->channels[i].values[3];
                    p_id          = point_msg->channels[i].values[4];
                    point_2d_normal.push_back(p_2d_normal);
                    point_2d_uv.push_back(p_2d_uv);
                    point_id.push_back(p_id);
                }

                static int global_frame_index = 0;
                KeyFrame* keyframe = new KeyFrame(pose_msg->header.stamp.toSec(), global_frame_index,
                                                  T, R,
                                                  image,
                                                  point_3d, point_2d_uv, point_2d_normal, point_id);

                m_process.lock();
                loopDetector.addKeyFrame(keyframe, 1);
                m_process.unlock();

                loopDetector.visualizeKeyPoses(pose_msg->header.stamp.toSec());

                global_frame_index++;
                last_t = T;
            }
        }

        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins");
    ros::NodeHandle n;
    ROS_INFO("\033[1;32m----> Visual Loop Detection Started.\033[0m");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn);

    // Config file
    std::string config_file;
    if (!ros::internal::g_camera_config_file.empty())
        config_file = ros::internal::g_camera_config_file;
    else if (argc >= 2)
        config_file = argv[1];
    else {
        ROS_ERROR("Usage: %s <camera_config_file>", argv[0]);
        return 1;
    }

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
        return 1;
    }
    usleep(100);

    // Initialize global params
    fsSettings["project_name"]   >> PROJECT_NAME;
    fsSettings["image_topic"]    >> IMAGE_TOPIC;
    fsSettings["loop_closure"]   >> LOOP_CLOSURE;
    fsSettings["skip_time"]      >> SKIP_TIME;
    fsSettings["skip_dist"]      >> SKIP_DIST;
    fsSettings["debug_image"]    >> DEBUG_IMAGE;
    fsSettings["match_image_scale"] >> MATCH_IMAGE_SCALE;

    if (LOOP_CLOSURE)
    {
        string pkg_path = ros::package::getPath(PROJECT_NAME);

        string vocabulary_file;
        fsSettings["vocabulary_file"] >> vocabulary_file;
        vocabulary_file = pkg_path + vocabulary_file;
        loopDetector.loadVocabulary(vocabulary_file);

        string brief_pattern_file;
        fsSettings["brief_pattern_file"] >> brief_pattern_file;
        brief_pattern_file = pkg_path + brief_pattern_file;
        briefExtractor = BriefExtractor(brief_pattern_file);

        m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(config_file.c_str());
    }

    ros::Subscriber<sensor_msgs::Image>      sub_image     = n.subscribe<sensor_msgs::Image>     (IMAGE_TOPIC, 30, image_callback);
    ros::Subscriber<nav_msgs::Odometry>      sub_pose      = n.subscribe<nav_msgs::Odometry>     (PROJECT_NAME + "/vins/odometry/keyframe_pose",  3, pose_callback);
    ros::Subscriber<sensor_msgs::PointCloud> sub_point     = n.subscribe<sensor_msgs::PointCloud>(PROJECT_NAME + "/vins/odometry/keyframe_point", 3, point_callback);
    ros::Subscriber<nav_msgs::Odometry>      sub_extrinsic = n.subscribe<nav_msgs::Odometry>     (PROJECT_NAME + "/vins/odometry/extrinsic",      3, extrinsic_callback);

    pub_match_img = n.advertise<sensor_msgs::Image>             (PROJECT_NAME + "/vins/loop/match_image", 3);
    pub_match_msg = n.advertise<std_msgs::Float64MultiArray>    (PROJECT_NAME + "/vins/loop/match_frame", 3);
    pub_key_pose  = n.advertise<visualization_msgs::MarkerArray>(PROJECT_NAME + "/vins/loop/keyframe_pose", 3);

    if (!LOOP_CLOSURE)
    {
        // subscribers already created but won't be triggered without LOOP_CLOSURE
    }

    std::thread measurement_process;
    measurement_process = std::thread(process);

    ros::spin();

    return 0;
}
