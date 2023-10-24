#include "gps_slam_converter/gps_slam_converter.hxx"

gps_slam_conversion::node::GpsSLAMConverter::GpsSLAMConverter()
    : Node(RCL_NODE_NAME)
{
    this->rcl_node_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});
    if (this->rcl_node_ != nullptr)
    {
        RCLCPP_INFO(this->rcl_node_->get_logger(), "%s node created", RCL_NODE_NAME);
        RCLCPP_LINE_INFO();
    }
    else
    {
        RCUTILS_LOG_ERROR_NAMED(RCL_NODE_NAME, "failed to create %s node", RCL_NODE_NAME);
        RCLCPP_LINE_ERROR();
        exit(0);
    }

    this->slam_pose_subscription_cb_group_ = this->rcl_node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions slam_pose_subscription_opts;
    slam_pose_subscription_opts.callback_group = this->slam_pose_subscription_cb_group_;
    this->slam_pose_subscription_ = this->rcl_node_->create_subscription<geometry_msgs::msg::Pose>(
        RCL_ROBOT_POSE_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(10)),
        std::bind(&gps_slam_conversion::node::GpsSLAMConverter::slam_pose_subscription_cb, this, _1),
        slam_pose_subscription_opts);

    this->gps_subscription_cb_group_ = this->rcl_node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions gps_subscription_opts;
    gps_subscription_opts.callback_group = this->gps_subscription_cb_group_;
    this->gps_subscription_ = this->rcl_node_->create_subscription<sensor_msgs::msg::NavSatFix>(
        RCL_UBLOX_FIX_TOPIC,
        rclcpp::SensorDataQoS(),
        std::bind(&gps_slam_conversion::node::GpsSLAMConverter::gps_subscription_cb, this, _1),
        gps_subscription_opts);

    this->converted_slam_publisher_cb_group_ = this->rcl_node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions converted_slam_publisher_opts;
    converted_slam_publisher_opts.callback_group = this->converted_slam_publisher_cb_group_;
    this->converted_slam_publisher_ = this->rcl_node_->create_publisher<geometry_msgs::msg::Pose>(
        RCL_GPS_TO_SLAM_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(10)),
        converted_slam_publisher_opts);

    this->converted_gps_publisher_cb_group_ = this->rcl_node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions converted_gps_publihser_opts;
    converted_gps_publihser_opts.callback_group = this->converted_gps_publisher_cb_group_;
    this->converted_gps_publisher_ = this->rcl_node_->create_publisher<sensor_msgs::msg::NavSatFix>(
        RCL_SLAM_TO_GPS_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(10)),
        converted_gps_publihser_opts);

    this->converter_service_ = this->rcl_node_->create_service<gps_slam_conversion_msgs::srv::Conversion>(
        RCL_CONVERTER_SERVICE_SERVER_NAME,
        std::bind(&gps_slam_conversion::node::GpsSLAMConverter::converter_service_cb, this, _1, _2));
}

gps_slam_conversion::node::GpsSLAMConverter::~GpsSLAMConverter()
{
}

void gps_slam_conversion::node::GpsSLAMConverter::signal_handler(int signal_input)
{
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "===== {%s} has been terminated with SIG [%d] =====", RCL_NODE_NAME, signal_input);
    signal(signal_input, SIG_IGN);
    exit(0);
}

void gps_slam_conversion::node::GpsSLAMConverter::slam_pose_subscription_cb(geometry_msgs::msg::Pose::SharedPtr slam_pose_cb_data)
{
    slam_pose_ = slam_pose_cb_data;

    bool is_slam_pose_null = (slam_pose_cb_data == nullptr);

    if (is_slam_pose_null)
    {
        RCLCPP_ERROR(this->rcl_node_->get_logger(), "/robot_pose callback is null");
        RCLCPP_LINE_ERROR();
    }
    else
    {
        
    }
}

void gps_slam_conversion::node::GpsSLAMConverter::gps_subscription_cb(sensor_msgs::msg::NavSatFix::SharedPtr gps_cb_data)
{
    gps_ = gps_cb_data;

    bool is_gps_null = (gps_ == nullptr);

    if (is_gps_null)
    {
        RCLCPP_ERROR(this->rcl_node_->get_logger(), "/ublox/fix callback is null");
        RCLCPP_LINE_ERROR();
    }
    else
    {
        
    }
}

void gps_slam_conversion::node::GpsSLAMConverter::converter_service_cb(const gps_slam_conversion_msgs::srv::Conversion::Request::SharedPtr request, gps_slam_conversion_msgs::srv::Conversion::Response::SharedPtr response)
{
    const std::string &request_conversion_target_data = request->conversion_target.data;
    std::vector<sensor_msgs::msg::NavSatFix> gps_request_list = request->gps_request_list;
    std::vector<geometry_msgs::msg::Pose> slam_pose_request_list = request->slam_pose_request_list;

    if (request_conversion_target_data == "")
    {
        RCLCPP_ERROR(this->rcl_node_->get_logger(), "[%s] conversion_target is empty", RCL_CONVERTER_SERVICE_SERVER_NAME);
        RCLCPP_LINE_ERROR();
        return;
    }
    else if (request_conversion_target_data == RCL_CONVERTER_SERVICE_CONVERSION_TARGET_SLAM)
    {
        
    }
    else if (request_conversion_target_data == RCL_CONVERTER_SERVICE_CONVERSION_TARGET_GPS)
    {
        
    }
    else
    {
        RCLCPP_ERROR(this->rcl_node_->get_logger(), "%s request is invalid", RCL_CONVERTER_SERVICE_SERVER_NAME);
        RCLCPP_LINE_ERROR();
    }
}