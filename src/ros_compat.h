#pragma once
// ros_compat.h - Standalone middleware adapter for LVI-SAM.
//
// This file is the *concrete* implementation of the middleware interface
// described in lvi_sam/node_interface.h.  It provides all ROS-compatible
// message types, publisher/subscriber machinery, tf utilities, and logging
// macros so that LVI-SAM compiles and runs without any ROS installation.
//
// To use a different middleware (ROS 1, ROS 2, LCM, ZeroMQ, …):
//   1. Create your own adapter header following the contract documented in
//      lvi_sam/node_interface.h.
//   2. Provide concrete types under the same namespaces used by the
//      algorithmic code (ros::, sensor_msgs::, nav_msgs::, tf::, …).
//   3. Include your adapter instead of this file.
//
// The algorithmic classes are pure libraries; they contain no I/O of their
// own.  All pub/sub wiring lives in the thin *_node.cpp wrappers.

#include <functional>
#include <string>
#include <vector>
#include <map>
#include <deque>
#include <queue>
#include <mutex>
#include <memory>
#include <thread>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cassert>
#include <cmath>
#include <iostream>
#include <sstream>
#include <fstream>
#include <csignal>
#include <array>
#include <unistd.h>

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>

// ============================================================
// Logging macros
// ============================================================
#define ROS_INFO(fmt, ...)    printf("[INFO]  " fmt "\n", ##__VA_ARGS__)
#define ROS_WARN(fmt, ...)    printf("[WARN]  " fmt "\n", ##__VA_ARGS__)
#define ROS_ERROR(fmt, ...)   fprintf(stderr, "[ERROR] " fmt "\n", ##__VA_ARGS__)
#define ROS_DEBUG(fmt, ...)   do {} while(0)
#define ROS_INFO_STREAM(x)    do { std::cout << "[INFO]  " << x << std::endl; } while(0)
#define ROS_WARN_STREAM(x)    do { std::cout << "[WARN]  " << x << std::endl; } while(0)
#define ROS_ERROR_STREAM(x)   do { std::cerr << "[ERROR] " << x << std::endl; } while(0)
#define ROS_DEBUG_STREAM(x)   do {} while(0)
#define ROS_ASSERT(cond)      assert(cond)
#define ROS_BREAK()           abort()
#define ROSCONSOLE_DEFAULT_NAME ""

// ============================================================
// ros namespace - Time, Duration, Rate, logging level stubs
// ============================================================
namespace ros {

namespace console { namespace levels {
enum Level { Debug, Info, Warn, Error, Fatal };
} // levels
inline void set_logger_level(const char*, levels::Level) {}
} // console

struct Duration {
    double sec;
    Duration() : sec(0.0) {}
    explicit Duration(double s) : sec(s) {}
    double toSec() const { return sec; }
};

struct Time {
    double sec;
    Time() : sec(0.0) {}
    explicit Time(double s) : sec(s) {}
    double toSec() const { return sec; }
    uint64_t toNSec() const { return static_cast<uint64_t>(sec * 1e9); }
    bool isZero() const { return sec == 0.0; }

    static Time now() {
        auto t = std::chrono::system_clock::now().time_since_epoch();
        return Time(std::chrono::duration<double>(t).count());
    }
    // fromSec: call on a default-constructed Time() to get a new Time
    Time fromSec(double s) const { return Time(s); }
    static Time from_sec(double s) { return Time(s); }
    Time operator+(const Duration& d) const { return Time(sec + d.sec); }
    Time operator-(const Duration& d) const { return Time(sec - d.sec); }
    bool operator<(const Time& o)  const { return sec < o.sec; }
    bool operator>(const Time& o)  const { return sec > o.sec; }
    bool operator<=(const Time& o) const { return sec <= o.sec; }
    bool operator>=(const Time& o) const { return sec >= o.sec; }
    bool operator==(const Time& o) const { return sec == o.sec; }
};

class Rate {
    double period_s_;
    std::chrono::steady_clock::time_point last_;
public:
    explicit Rate(double hz) : period_s_(1.0/hz), last_(std::chrono::steady_clock::now()) {}
    void sleep() {
        auto target = last_ + std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(period_s_));
        std::this_thread::sleep_until(target);
        last_ = std::chrono::steady_clock::now();
    }
};

} // namespace ros

// ============================================================
// std_msgs
// ============================================================
namespace std_msgs {

struct Header {
    uint32_t seq = 0;
    ros::Time stamp;
    std::string frame_id;
};

struct Bool {
    using ConstPtr = std::shared_ptr<const Bool>;
    bool data = false;
};

struct Float32 { float data = 0.0f; };

struct Float64MultiArray {
    using ConstPtr = std::shared_ptr<const Float64MultiArray>;
    std::vector<double> data;
};

struct ColorRGBA { float r=0, g=0, b=0, a=1; };

} // namespace std_msgs

// ============================================================
// geometry_msgs
// ============================================================
namespace geometry_msgs {

struct Vector3 { double x=0, y=0, z=0; };
struct Point   { double x=0, y=0, z=0; };
struct Point32 { float  x=0, y=0, z=0; };
struct Quaternion { double x=0, y=0, z=0, w=1; };

struct Pose {
    Point position;
    Quaternion orientation;
};

struct PoseWithCovariance {
    Pose pose;
    std::array<double, 36> covariance = {};
};

struct Twist {
    Vector3 linear;
    Vector3 angular;
};

struct TwistWithCovariance {
    Twist twist;
    std::array<double, 36> covariance = {};
};

struct PoseStamped {
    std_msgs::Header header;
    Pose pose;
};

struct PointStamped {
    std_msgs::Header header;
    Point point;
};

} // namespace geometry_msgs

// ============================================================
// sensor_msgs
// ============================================================
namespace sensor_msgs {

struct PointField {
    static const uint8_t INT8    = 1;
    static const uint8_t UINT8   = 2;
    static const uint8_t INT16   = 3;
    static const uint8_t UINT16  = 4;
    static const uint8_t INT32   = 5;
    static const uint8_t UINT32  = 6;
    static const uint8_t FLOAT32 = 7;
    static const uint8_t FLOAT64 = 8;

    std::string name;
    uint32_t offset   = 0;
    uint8_t  datatype = 0;
    uint32_t count    = 1;
};

struct PointCloud2 {
    using ConstPtr = std::shared_ptr<const PointCloud2>;

    std_msgs::Header header;
    uint32_t height      = 0;
    uint32_t width       = 0;
    std::vector<PointField> fields;
    bool     is_bigendian = false;
    uint32_t point_step  = 0;
    uint32_t row_step    = 0;
    std::vector<uint8_t> data;
    bool     is_dense    = false;
};
using PointCloud2ConstPtr = std::shared_ptr<const PointCloud2>;

struct ChannelFloat32 {
    std::string name;
    std::vector<float> values;
};

struct PointCloud {
    using ConstPtr = std::shared_ptr<const PointCloud>;
    using Ptr      = std::shared_ptr<PointCloud>;

    std_msgs::Header header;
    std::vector<geometry_msgs::Point32> points;
    std::vector<ChannelFloat32> channels;
};
using PointCloudConstPtr = std::shared_ptr<const PointCloud>;

namespace image_encodings {
    const std::string MONO8   = "mono8";
    const std::string MONO16  = "mono16";
    const std::string RGB8    = "rgb8";
    const std::string BGR8    = "bgr8";
    const std::string RGBA8   = "rgba8";
    const std::string BGRA8   = "bgra8";
    const std::string TYPE_8UC1 = "8UC1";
} // image_encodings

struct Image {
    using Ptr      = std::shared_ptr<Image>;
    using ConstPtr = std::shared_ptr<const Image>;

    std_msgs::Header header;
    uint32_t height = 0;
    uint32_t width  = 0;
    std::string encoding;
    bool    is_bigendian = false;
    uint32_t step = 0;
    cv::Mat  image; // store directly as cv::Mat
};
using ImageConstPtr = std::shared_ptr<const Image>;
using ImagePtr      = std::shared_ptr<Image>;

struct Imu {
    using ConstPtr = std::shared_ptr<const Imu>;

    std_msgs::Header header;
    geometry_msgs::Quaternion orientation;
    std::array<double,9> orientation_covariance = {};
    geometry_msgs::Vector3 angular_velocity;
    std::array<double,9> angular_velocity_covariance = {};
    geometry_msgs::Vector3 linear_acceleration;
    std::array<double,9> linear_acceleration_covariance = {};
};
using ImuConstPtr = std::shared_ptr<const Imu>;

struct NavSatFix {
    using ConstPtr = std::shared_ptr<const NavSatFix>;
    std_msgs::Header header;
    double latitude=0, longitude=0, altitude=0;
    std::array<double,9> position_covariance = {};
    uint8_t position_covariance_type = 0;
};
using NavSatFixConstPtr = std::shared_ptr<const NavSatFix>;

} // namespace sensor_msgs

// ============================================================
// nav_msgs
// ============================================================
namespace nav_msgs {

struct Odometry {
    using ConstPtr = std::shared_ptr<const Odometry>;

    std_msgs::Header header;
    std::string child_frame_id;
    geometry_msgs::PoseWithCovariance  pose;
    geometry_msgs::TwistWithCovariance twist;
};
using OdometryConstPtr = std::shared_ptr<const Odometry>;

using OdometryConstPtr = std::shared_ptr<const Odometry>;

struct Path {
    std_msgs::Header header;
    std::vector<geometry_msgs::PoseStamped> poses;
};

} // namespace nav_msgs

// ============================================================
// visualization_msgs (stub - visualization is removed)
// ============================================================
namespace visualization_msgs {
struct Marker      { std_msgs::Header header; };
struct MarkerArray { std::vector<Marker> markers; };
} // namespace visualization_msgs

// ============================================================
// cv_bridge replacement
// ============================================================
namespace cv_bridge {

struct CvImage {
    std_msgs::Header header;
    std::string encoding;
    cv::Mat image;

    sensor_msgs::Image::Ptr toImageMsg() const {
        auto msg = std::make_shared<sensor_msgs::Image>();
        msg->header   = header;
        msg->encoding = encoding;
        msg->image    = image;
        msg->height   = static_cast<uint32_t>(image.rows);
        msg->width    = static_cast<uint32_t>(image.cols);
        return msg;
    }
};
using CvImagePtr      = std::shared_ptr<CvImage>;
using CvImageConstPtr = std::shared_ptr<const CvImage>;

inline CvImagePtr toCvCopy(const sensor_msgs::Image::ConstPtr& src,
                            const std::string& encoding = "")
{
    auto result = std::make_shared<CvImage>();
    result->header   = src->header;
    result->encoding = src->encoding;
    result->image    = src->image.clone();

    if (!encoding.empty() && encoding != result->encoding) {
        if (encoding == sensor_msgs::image_encodings::MONO8) {
            if (result->image.channels() == 3)
                cv::cvtColor(result->image, result->image, cv::COLOR_BGR2GRAY);
            else if (result->image.channels() == 4)
                cv::cvtColor(result->image, result->image, cv::COLOR_BGRA2GRAY);
            result->encoding = encoding;
        } else if (encoding == sensor_msgs::image_encodings::RGB8) {
            if (result->image.channels() == 1)
                cv::cvtColor(result->image, result->image, cv::COLOR_GRAY2RGB);
            result->encoding = encoding;
        }
    }
    return result;
}

inline CvImagePtr cvtColor(const CvImageConstPtr& src, const std::string& encoding)
{
    auto result = std::make_shared<CvImage>();
    result->header   = src->header;
    result->encoding = encoding;
    if (encoding == sensor_msgs::image_encodings::RGB8 &&
        (src->encoding == sensor_msgs::image_encodings::MONO8 ||
         src->encoding == sensor_msgs::image_encodings::TYPE_8UC1)) {
        cv::cvtColor(src->image, result->image, cv::COLOR_GRAY2RGB);
    } else {
        result->image = src->image.clone();
    }
    return result;
}

} // namespace cv_bridge

// ============================================================
// PCL conversions (replacement for pcl_conversions)
// ============================================================
namespace pcl {

template<typename T>
void fromROSMsg(const sensor_msgs::PointCloud2& cloud_in, pcl::PointCloud<T>& cloud_out)
{
    pcl::PCLPointCloud2 pc2;
    pc2.header.stamp    = static_cast<uint64_t>(cloud_in.header.stamp.toSec() * 1e6);
    pc2.header.frame_id = cloud_in.header.frame_id;
    pc2.header.seq      = cloud_in.header.seq;
    pc2.height          = cloud_in.height;
    pc2.width           = cloud_in.width;
    pc2.is_bigendian    = cloud_in.is_bigendian;
    pc2.point_step      = cloud_in.point_step;
    pc2.row_step        = cloud_in.row_step;
    pc2.data            = cloud_in.data;
    pc2.is_dense        = cloud_in.is_dense;
    pc2.fields.clear();
    for (const auto& f : cloud_in.fields) {
        pcl::PCLPointField pf;
        pf.name     = f.name;
        pf.offset   = f.offset;
        pf.datatype = f.datatype;
        pf.count    = f.count;
        pc2.fields.push_back(pf);
    }
    pcl::fromPCLPointCloud2(pc2, cloud_out);
}

template<typename T>
void toROSMsg(const pcl::PointCloud<T>& cloud_in, sensor_msgs::PointCloud2& cloud_out)
{
    pcl::PCLPointCloud2 pc2;
    pcl::toPCLPointCloud2(cloud_in, pc2);
    cloud_out.header.stamp    = ros::Time(static_cast<double>(pc2.header.stamp) / 1e6);
    cloud_out.header.frame_id = pc2.header.frame_id;
    cloud_out.header.seq      = pc2.header.seq;
    cloud_out.height          = pc2.height;
    cloud_out.width           = pc2.width;
    cloud_out.is_bigendian    = pc2.is_bigendian;
    cloud_out.point_step      = pc2.point_step;
    cloud_out.row_step        = pc2.row_step;
    cloud_out.data            = pc2.data;
    cloud_out.is_dense        = pc2.is_dense;
    cloud_out.fields.clear();
    for (const auto& pf : pc2.fields) {
        sensor_msgs::PointField f;
        f.name     = pf.name;
        f.offset   = pf.offset;
        f.datatype = pf.datatype;
        f.count    = pf.count;
        cloud_out.fields.push_back(f);
    }
}

} // namespace pcl (extensions, safe since pcl_conversions is not included)

// ============================================================
// tf replacement (Eigen-based)
// ============================================================
namespace tf {

struct Quaternion {
    double x, y, z, w;
    Quaternion() : x(0), y(0), z(0), w(1) {}
    Quaternion(double x_, double y_, double z_, double w_) : x(x_), y(y_), z(z_), w(w_) {}

    Eigen::Quaterniond toEigen() const { return Eigen::Quaterniond(w, x, y, z); }
    static Quaternion fromEigen(const Eigen::Quaterniond& q) {
        return Quaternion(q.x(), q.y(), q.z(), q.w());
    }
    Quaternion operator*(const Quaternion& o) const {
        Eigen::Quaterniond r = toEigen() * o.toEigen();
        return Quaternion(r.x(), r.y(), r.z(), r.w());
    }
    double getW() const { return w; }
    double getX() const { return x; }
    double getY() const { return y; }
    double getZ() const { return z; }
    void setW(double v) { w = v; }
    void setX(double v) { x = v; }
    void setY(double v) { y = v; }
    void setZ(double v) { z = v; }

    void setRPY(double roll, double pitch, double yaw) {
        Eigen::Quaterniond q =
            Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX());
        x = q.x(); y = q.y(); z = q.z(); w = q.w();
    }

    Quaternion slerp(const Quaternion& other, double t) const {
        Eigen::Quaterniond r = toEigen().slerp(t, other.toEigen());
        return Quaternion(r.x(), r.y(), r.z(), r.w());
    }
};

struct Vector3 {
    double x, y, z;
    Vector3() : x(0), y(0), z(0) {}
    Vector3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    double getX() const { return x; }
    double getY() const { return y; }
    double getZ() const { return z; }
};

struct Matrix3x3 {
    Eigen::Matrix3d mat;
    Matrix3x3() { mat.setIdentity(); }
    explicit Matrix3x3(const Quaternion& q) {
        mat = q.toEigen().normalized().toRotationMatrix();
    }
    void getRPY(double& roll, double& pitch, double& yaw) const {
        Eigen::Vector3d rpy = mat.eulerAngles(0, 1, 2);
        roll = rpy(0); pitch = rpy(1); yaw = rpy(2);
    }
    Quaternion getRotation() const {
        Eigen::Quaterniond q(mat);
        return Quaternion(q.x(), q.y(), q.z(), q.w());
    }
};

inline Quaternion createQuaternionFromRPY(double roll, double pitch, double yaw) {
    Eigen::Quaterniond q =
        Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX());
    return Quaternion(q.x(), q.y(), q.z(), q.w());
}

inline geometry_msgs::Quaternion createQuaternionMsgFromRollPitchYaw(double roll, double pitch, double yaw) {
    Quaternion q = createQuaternionFromRPY(roll, pitch, yaw);
    geometry_msgs::Quaternion msg;
    msg.x = q.x; msg.y = q.y; msg.z = q.z; msg.w = q.w;
    return msg;
}

struct Transform {
    Vector3    origin_;
    Quaternion rotation_;

    Transform() {}
    Transform(const Quaternion& q, const Vector3& v) : origin_(v), rotation_(q) {}

    const Vector3&    getOrigin()   const { return origin_; }
    const Quaternion& getRotation() const { return rotation_; }
    void setOrigin(const Vector3& v)    { origin_   = v; }
    void setRotation(const Quaternion& q) { rotation_ = q; }

    Transform inverse() const {
        Eigen::Quaterniond q_inv = rotation_.toEigen().inverse();
        Eigen::Vector3d    t_inv = -(q_inv * Eigen::Vector3d(origin_.x, origin_.y, origin_.z));
        return Transform(Quaternion::fromEigen(q_inv), Vector3(t_inv.x(), t_inv.y(), t_inv.z()));
    }
    Transform operator*(const Transform& o) const {
        Eigen::Quaterniond q1 = rotation_.toEigen();
        Eigen::Vector3d    t1(origin_.x, origin_.y, origin_.z);
        Eigen::Quaterniond q2 = o.rotation_.toEigen();
        Eigen::Vector3d    t2(o.origin_.x, o.origin_.y, o.origin_.z);
        Eigen::Quaterniond qr = q1 * q2;
        Eigen::Vector3d    tr = t1 + q1 * t2;
        return Transform(Quaternion::fromEigen(qr), Vector3(tr.x(), tr.y(), tr.z()));
    }
};

struct StampedTransform : public Transform {
    ros::Time   stamp;
    std::string frame_id;
    std::string child_frame_id;

    StampedTransform() {}
    StampedTransform(const Transform& t, const ros::Time& s,
                     const std::string& f, const std::string& c)
        : Transform(t), stamp(s), frame_id(f), child_frame_id(c) {}
};

class TransformException : public std::exception {
    std::string msg_;
public:
    explicit TransformException(const std::string& m) : msg_(m) {}
    const char* what() const noexcept override { return msg_.c_str(); }
};

class TransformBroadcaster {
public:
    void sendTransform(const StampedTransform&) { /* no-op */ }
};

class TransformListener {
public:
    void waitForTransform(const std::string&, const std::string&,
                          const ros::Time&, const ros::Duration&) {
        throw TransformException("Transform not available in standalone mode");
    }
    void lookupTransform(const std::string&, const std::string&,
                         const ros::Time&, StampedTransform&) {
        throw TransformException("Transform not available in standalone mode");
    }
};

inline void quaternionMsgToTF(const geometry_msgs::Quaternion& msg, Quaternion& q) {
    q.x = msg.x; q.y = msg.y; q.z = msg.z; q.w = msg.w;
}
inline void quaternionTFToMsg(const Quaternion& q, geometry_msgs::Quaternion& msg) {
    msg.x = q.x; msg.y = q.y; msg.z = q.z; msg.w = q.w;
}

inline void poseMsgToTF(const geometry_msgs::Pose& pose, Transform& t) {
    t.setOrigin(Vector3(pose.position.x, pose.position.y, pose.position.z));
    t.setRotation(Quaternion(pose.orientation.x, pose.orientation.y,
                             pose.orientation.z, pose.orientation.w));
}

} // namespace tf

// ============================================================
// ros pub/sub machinery
// ============================================================
namespace ros {

namespace internal {

// ---------------------------------------------------------------------------
// Process-wide shared state accessed via function-local statics.
//
// Using function-local statics (rather than file-scope statics) ensures that
// all translation units in the same process share exactly ONE instance of
// each variable.  File-scope `static` variables would give each translation
// unit its own private copy, breaking the in-process pub/sub when the
// combined single-process executable is built from multiple .cpp files.
// ---------------------------------------------------------------------------

inline std::string& lidarConfigFile() {
    static std::string v;
    return v;
}
inline std::string& cameraConfigFile() {
    static std::string v;
    return v;
}
// Convenience aliases kept for source compatibility.
// Code that uses ros::internal::g_lidar_config_file / g_camera_config_file
// should use these references.
static std::string& g_lidar_config_file  = lidarConfigFile();
static std::string& g_camera_config_file = cameraConfigFile();

inline std::atomic<bool>& okFlag() {
    static std::atomic<bool> v{true};
    return v;
}

using AnyMsg  = std::shared_ptr<void>;
using Callback = std::function<void(const AnyMsg&)>;

struct TopicData {
    std::vector<Callback> callbacks;
    std::mutex mtx;
};

inline std::map<std::string, std::unique_ptr<TopicData>>& topicsMap() {
    static std::map<std::string, std::unique_ptr<TopicData>> v;
    return v;
}
inline std::mutex& topicsMutex() {
    static std::mutex v;
    return v;
}

inline TopicData& getTopic(const std::string& topic) {
    std::lock_guard<std::mutex> lk(topicsMutex());
    auto& m = topicsMap();
    auto it = m.find(topic);
    if (it == m.end()) {
        m[topic] = std::unique_ptr<TopicData>(new TopicData());
        it = m.find(topic);
    }
    return *it->second;
}

// Per-process YAML config loaders
inline cv::FileStorage& lidarFs() {
    static cv::FileStorage v;
    return v;
}
inline cv::FileStorage& cameraFs() {
    static cv::FileStorage v;
    return v;
}

inline bool loadLidarConfig(const std::string& path) {
    g_lidar_config_file = path;
    lidarFs().open(path, cv::FileStorage::READ);
    return lidarFs().isOpened();
}
inline bool loadCameraConfig(const std::string& path) {
    g_camera_config_file = path;
    cameraFs().open(path, cv::FileStorage::READ);
    return cameraFs().isOpened();
}

// Read a (possibly nested) key from a FileStorage.
// key format: "parent/child" or "key" (leading '/' stripped)
template<typename T>
inline bool fsRead(cv::FileStorage& fs, const std::string& raw_key, T& val) {
    if (!fs.isOpened()) return false;
    std::string key = raw_key;
    if (!key.empty() && key[0] == '/') key = key.substr(1);
    size_t slash = key.find('/');
    cv::FileNode node;
    if (slash != std::string::npos) {
        node = fs[key.substr(0, slash)][key.substr(slash + 1)];
    } else {
        node = fs[key];
    }
    if (node.empty()) return false;
    node >> val;
    return true;
}

// Returns the package root directory inferred from the config file path.
inline std::string packageRoot() {
    std::string f = g_camera_config_file.empty() ? g_lidar_config_file : g_camera_config_file;
    if (f.empty()) return ".";
    size_t s1 = f.find_last_of("/\\");
    if (s1 == std::string::npos) return ".";
    std::string dir = f.substr(0, s1); // config dir
    size_t s2 = dir.find_last_of("/\\");
    if (s2 == std::string::npos) return dir;
    return dir.substr(0, s2);
}

} // namespace internal

inline bool ok() { return internal::okFlag().load(); }
inline void shutdown() { internal::okFlag().store(false); }

struct TransportHints {
    TransportHints& tcpNoDelay() { return *this; }
};

inline void init(int argc, char** argv, const std::string& /*node_name*/)
{
    // Parse --lidar-config <file> and --camera-config <file> or positional arg
    std::string lidar_cfg, camera_cfg;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--lidar-config" && i+1 < argc)  { lidar_cfg  = argv[++i]; }
        else if (a == "--camera-config" && i+1 < argc) { camera_cfg = argv[++i]; }
        else if (a[0] != '-') {
            // Guess based on content
            if (lidar_cfg.empty())  lidar_cfg  = a;
            else if (camera_cfg.empty()) camera_cfg = a;
        }
    }
    if (!lidar_cfg.empty())  internal::loadLidarConfig(lidar_cfg);
    if (!camera_cfg.empty()) internal::loadCameraConfig(camera_cfg);

    signal(SIGINT, [](int) { ros::shutdown(); });
}

inline void spin() {
    while (ok())
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
}
inline void spinOnce() { /* callbacks are synchronous */ }

class MultiThreadedSpinner {
    int n_;
public:
    explicit MultiThreadedSpinner(int n = 0) : n_(n) {}
    void spin() { ros::spin(); }
};

// ------------------------------------------------------------
// Publisher
// ------------------------------------------------------------
template<typename T>
class Publisher {
    std::string topic_;
public:
    Publisher() {}
    explicit Publisher(const std::string& t) : topic_(t) {}

    void publish(const T& msg) const {
        if (topic_.empty()) return;
        auto ptr = std::make_shared<T>(msg);
        auto& td = internal::getTopic(topic_);
        std::lock_guard<std::mutex> lk(td.mtx);
        for (auto& cb : td.callbacks) cb(ptr);
    }
    // For visualization pubs: always return 0 so callers skip expensive publish
    int getNumSubscribers() const {
        if (topic_.empty()) return 0;
        std::lock_guard<std::mutex> lk(internal::topicsMutex());
        auto& m = internal::topicsMap();
        auto it = m.find(topic_);
        if (it == m.end()) return 0;
        std::lock_guard<std::mutex> lk2(it->second->mtx);
        return static_cast<int>(it->second->callbacks.size());
    }
    void shutdown() {}
    bool isValid() const { return !topic_.empty(); }
};

// ------------------------------------------------------------
// Subscriber
// ------------------------------------------------------------
template<typename T>
class Subscriber {
    std::string topic_;
public:
    Subscriber() {}
    explicit Subscriber(const std::string& t) : topic_(t) {}
    void shutdown() {}
};

// ------------------------------------------------------------
// NodeHandle
// ------------------------------------------------------------
class NodeHandle {
    std::string ns_;
public:
    NodeHandle() {}
    explicit NodeHandle(const std::string& ns) : ns_(ns) {}

    // param<T>(key, val, default)
    template<typename T>
    bool param(const std::string& key, T& val, const T& def) const {
        if (internal::fsRead(internal::lidarFs(),  key, val)) return true;
        if (internal::fsRead(internal::cameraFs(), key, val)) return true;
        val = def;
        return false;
    }
    template<typename T>
    bool getParam(const std::string& key, T& val) const {
        T dummy{};
        return param(key, val, dummy);
    }

    // advertise
    template<typename T>
    Publisher<T> advertise(const std::string& topic, int /*queue_size*/) {
        return Publisher<T>(topic);
    }

    // subscribe (method pointer)
    template<typename MsgT, typename ClassT>
    Subscriber<MsgT> subscribe(const std::string& topic, int /*q*/,
                               void (ClassT::*cb)(const std::shared_ptr<const MsgT>&),
                               ClassT* obj,
                               const TransportHints& = TransportHints())
    {
        auto& td = internal::getTopic(topic);
        std::lock_guard<std::mutex> lk(td.mtx);
        td.callbacks.push_back([cb, obj](const internal::AnyMsg& m) {
            (obj->*cb)(std::static_pointer_cast<const MsgT>(m));
        });
        return Subscriber<MsgT>(topic);
    }

    // subscribe (free function)
    template<typename MsgT>
    Subscriber<MsgT> subscribe(const std::string& topic, int /*q*/,
                               void (*cb)(const std::shared_ptr<const MsgT>&),
                               const TransportHints& = TransportHints())
    {
        auto& td = internal::getTopic(topic);
        std::lock_guard<std::mutex> lk(td.mtx);
        td.callbacks.push_back([cb](const internal::AnyMsg& m) {
            cb(std::static_pointer_cast<const MsgT>(m));
        });
        return Subscriber<MsgT>(topic);
    }

    // subscribe (std::function)
    template<typename MsgT>
    Subscriber<MsgT> subscribe(const std::string& topic, int /*q*/,
                               std::function<void(const std::shared_ptr<const MsgT>&)> cb,
                               const TransportHints& = TransportHints())
    {
        auto& td = internal::getTopic(topic);
        std::lock_guard<std::mutex> lk(td.mtx);
        td.callbacks.push_back([cb](const internal::AnyMsg& m) {
            cb(std::static_pointer_cast<const MsgT>(m));
        });
        return Subscriber<MsgT>(topic);
    }
};

// ros::package stub
namespace package {
inline std::string getPath(const std::string& /*pkg*/) {
    return internal::packageRoot();
}
} // package

} // namespace ros
