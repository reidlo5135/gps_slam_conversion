#ifndef UTILS__HXX
#define UTILS__HXX

#include <rcutils/logging_macros.h>

static constexpr const double &INTERSECTION_START_POINT_LON = 128.858009083;
static constexpr const double &INTERSECTION_START_POINT_LAT = 35.157430158;
static constexpr const double &INTERSECTION_END_POINT_LON = 128.858870603;
static constexpr const double &INTERSECTION_END_POINT_LAT = 35.158056682;

static constexpr const int &SLAM_MAP_WIDTH = 2080;
static constexpr const int &SLAM_MAP_HEIGHT = 1200;

static constexpr const double &LB_POINT_LON = 128.8575686;
static constexpr const double &LB_POINT_LAT = 35.157430158;
static constexpr const double &RT_POINT_LON = 128.858870603;
static constexpr const double &RT_POINT_LAT = 35.158463143;

static constexpr const char *RCL_NODE_NAME = "gps_slam_converter";
static constexpr const int &RCL_STOP_FLAG = 0;
static constexpr const int &RCL_DEFAULT_INT = 0;
static constexpr const double &RCL_DEFAULT_DOUBLE = 0.0;
static constexpr const char *RCL_ROBOT_POSE_TOPIC = "/robot_pose";
static constexpr const char *RCL_UBLOX_FIX_TOPIC = "/ublox/fix";
static constexpr const char *RCL_GPS_TO_SLAM_TOPIC = "/gps_to_slam";
static constexpr const char *RCL_SLAM_TO_GPS_TOPIC = "/slam_to_gps";
static constexpr const char *RCL_CONVERTER_SERVICE_SERVER_NAME = "/gps_slam_converter/conversion";
static constexpr const char *RCL_CONVERTER_SERVICE_CONVERSION_TARGET_GPS = "GPS";
static constexpr const char *RCL_CONVERTER_SERVICE_CONVERSION_TARGET_SLAM = "SLAM";


#define RCLCPP_LINE_INFO() \
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "LINE : [%d]", __LINE__)

#define RCLCPP_LINE_ERROR() \
    RCUTILS_LOG_ERROR_NAMED(RCL_NODE_NAME, "LINE : [%d]", __LINE__)

#define RCLCPP_LINE_WARN() \
    RCUTILS_LOG_WARN_NAMED(RCL_NODE_NAME, "LINE : [%d]", __LINE__)

#endif