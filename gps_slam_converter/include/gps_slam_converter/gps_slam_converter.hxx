#ifndef GPS_SLAM_CONVERTER__HXX
#define GPS_SLAM_CONVERTER__HXX

#include <stdio.h>
#include <math.h>
#include <cmath>
#include <signal.h>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <gps_slam_conversion_msgs/srv/conversion.hpp>

#include "gps_slam_converter/utils.hxx"

using std::placeholders::_1;
using std::placeholders::_2;

namespace gps_slam_conversion
{
    namespace node
    {
        class GpsSLAMConverter final : public rclcpp::Node
        {
        private:
            rclcpp::Node::SharedPtr rcl_node_;

            geometry_msgs::msg::Pose::SharedPtr slam_pose_;
            sensor_msgs::msg::NavSatFix::SharedPtr gps_;

            rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr slam_pose_subscription_;
            rclcpp::CallbackGroup::SharedPtr slam_pose_subscription_cb_group_;

            rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
            rclcpp::CallbackGroup::SharedPtr gps_subscription_cb_group_;

            rclcpp::CallbackGroup::SharedPtr converted_slam_publisher_cb_group_;
            rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr converted_slam_publisher_;

            rclcpp::CallbackGroup::SharedPtr converted_gps_publisher_cb_group_;
            rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr converted_gps_publisher_;

            rclcpp::Service<gps_slam_conversion_msgs::srv::Conversion>::SharedPtr converter_service_;

            void slam_pose_subscription_cb(geometry_msgs::msg::Pose::SharedPtr slam_pose_cb_data);
            void gps_subscription_cb(sensor_msgs::msg::NavSatFix::SharedPtr gps_cb_data);
            void converter_service_cb(const gps_slam_conversion_msgs::srv::Conversion::Request::SharedPtr request, gps_slam_conversion_msgs::srv::Conversion::Response::SharedPtr response);

        public:
            explicit GpsSLAMConverter();
            virtual ~GpsSLAMConverter();
            static void signal_handler(int signal_input);
        };
    }
}

#endif