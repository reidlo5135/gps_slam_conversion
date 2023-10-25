#include "gps_slam_converter/gps_slam_converter.hxx"

gps_slam_conversion::node::GpsSLAMConverter::GpsSLAMConverter()
    : Node(RCL_NODE_NAME)
{
    this->position_converter_ = std::make_shared<gps_slam_conversion::position::PositionConverter>();
    this->lon_lat_LB_point_ = std::make_shared<gps_slam_conversion::position::Point>();
    this->lon_lat_RT_point_ = std::make_shared<gps_slam_conversion::position::Point>();
    this->initial_position();

    this->node_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});

    if (this->node_ != nullptr)
    {
        RCLCPP_INFO(this->node_->get_logger(), "%s node created", RCL_NODE_NAME);
        RCLCPP_LINE_INFO();
    }
    else
    {
        RCUTILS_LOG_ERROR_NAMED(RCL_NODE_NAME, "failed to create %s node", RCL_NODE_NAME);
        RCLCPP_LINE_ERROR();
        exit(0);
    }

    this->slam_pose_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions slam_pose_subscription_opts;
    slam_pose_subscription_opts.callback_group = this->slam_pose_subscription_cb_group_;
    this->slam_pose_subscription_ = this->node_->create_subscription<geometry_msgs::msg::Pose>(
        RCL_ROBOT_POSE_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(10)),
        std::bind(&gps_slam_conversion::node::GpsSLAMConverter::slam_pose_subscription_cb, this, _1),
        slam_pose_subscription_opts);

    this->gps_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions gps_subscription_opts;
    gps_subscription_opts.callback_group = this->gps_subscription_cb_group_;
    this->gps_subscription_ = this->node_->create_subscription<sensor_msgs::msg::NavSatFix>(
        RCL_UBLOX_FIX_TOPIC,
        rclcpp::SensorDataQoS(),
        std::bind(&gps_slam_conversion::node::GpsSLAMConverter::gps_subscription_cb, this, _1),
        gps_subscription_opts);

    this->converted_slam_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions converted_slam_publisher_opts;
    converted_slam_publisher_opts.callback_group = this->converted_slam_publisher_cb_group_;
    this->converted_slam_publisher_ = this->node_->create_publisher<geometry_msgs::msg::Pose>(
        RCL_GPS_TO_SLAM_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(10)),
        converted_slam_publisher_opts);

    this->converted_gps_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions converted_gps_publihser_opts;
    converted_gps_publihser_opts.callback_group = this->converted_gps_publisher_cb_group_;
    this->converted_gps_publisher_ = this->node_->create_publisher<sensor_msgs::msg::NavSatFix>(
        RCL_SLAM_TO_GPS_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(10)),
        converted_gps_publihser_opts);

    this->converter_service_ = this->node_->create_service<gps_slam_conversion_msgs::srv::Conversion>(
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
    exit(RCL_STOP_FLAG);
}

void gps_slam_conversion::node::GpsSLAMConverter::initial_position()
{
    std::shared_ptr<gps_slam_conversion::position::Point> intersection_start_point = std::make_shared<gps_slam_conversion::position::Point>();
    intersection_start_point->set__x(INTERSECTION_START_POINT_LON);
    intersection_start_point->set__y(INTERSECTION_START_POINT_LAT);

    std::shared_ptr<gps_slam_conversion::position::Point> intersection_end_point = std::make_shared<gps_slam_conversion::position::Point>();
    intersection_end_point->set__x(INTERSECTION_END_POINT_LON);
    intersection_end_point->set__y(INTERSECTION_END_POINT_LAT);

    this->position_converter_->init_area(SLAM_MAP_WIDTH, SLAM_MAP_HEIGHT, *intersection_start_point, *intersection_end_point);

    this->lon_lat_LB_point_->set__x(LB_POINT_LON);
    this->lon_lat_LB_point_->set__y(LB_POINT_LAT);

    this->lon_lat_RT_point_->set__x(RT_POINT_LON);
    this->lon_lat_RT_point_->set__y(RT_POINT_LAT);
}

std_msgs::msg::Header gps_slam_conversion::node::GpsSLAMConverter::build_header(const char *frame_id)
{
    std::chrono::_V2::system_clock::time_point current_time = std::chrono::system_clock::now();
    std::chrono::_V2::system_clock::duration current_time_duration = current_time.time_since_epoch();

    const int32_t &current_time_sec = std::chrono::duration_cast<std::chrono::seconds>(current_time_duration).count();
    const int32_t &current_time_nanosec = current_time_sec % 1000000000;

    builtin_interfaces::msg::Time::UniquePtr stamp = std::make_unique<builtin_interfaces::msg::Time>();
    stamp->set__sec(current_time_sec);
    stamp->set__nanosec(current_time_nanosec);

    std_msgs::msg::Header::UniquePtr header = std::make_unique<std_msgs::msg::Header>();
    header->set__frame_id(frame_id);

    const builtin_interfaces::msg::Time &&stamp_moved = std::move(*stamp);
    header->set__stamp(stamp_moved);

    const std_msgs::msg::Header &&header_moved = std::move(*header);

    return header_moved;
}

sensor_msgs::msg::NavSatFix gps_slam_conversion::node::GpsSLAMConverter::build_nav_sat_fix(gps_slam_conversion::position::Point converted_gps_point)
{
    const double &lon = converted_gps_point.get__x();
    const double &lat = converted_gps_point.get__y();

    RCLCPP_INFO(this->node_->get_logger(), "converted_gps_point\n\tlat : %f\n\tlon : %f", lat, lon);
    RCLCPP_LINE_INFO();

    const char *header_frame_id = "slam_to_gps";
    std_msgs::msg::Header header = this->build_header(header_frame_id);

    sensor_msgs::msg::NavSatStatus::UniquePtr nav_sat_status = std::make_unique<sensor_msgs::msg::NavSatStatus>();
    nav_sat_status->set__status(static_cast<int8_t>(RCL_DEFAULT_INT));
    nav_sat_status->set__service(static_cast<uint16_t>(RCL_DEFAULT_INT));

    const sensor_msgs::msg::NavSatStatus &&nav_sat_status_moved = std::move(*nav_sat_status);

    sensor_msgs::msg::NavSatFix::UniquePtr nav_sat_fix = std::make_unique<sensor_msgs::msg::NavSatFix>();
    nav_sat_fix->set__header(header);
    nav_sat_fix->set__status(nav_sat_status_moved);
    nav_sat_fix->set__latitude(lat);
    nav_sat_fix->set__longitude(lon);
    nav_sat_fix->set__altitude(RCL_DEFAULT_DOUBLE);

    std::array<double, 9UL> position_covariance;
    for (int i = 0; i < position_covariance.size(); i++)
    {
        position_covariance[i] = RCL_DEFAULT_DOUBLE;
    }

    nav_sat_fix->set__position_covariance(position_covariance);
    nav_sat_fix->set__position_covariance_type(static_cast<uint8_t>(RCL_DEFAULT_INT));

    const sensor_msgs::msg::NavSatFix &&nav_sat_fix_moved = std::move(*nav_sat_fix);

    return nav_sat_fix_moved;
}

geometry_msgs::msg::Pose gps_slam_conversion::node::GpsSLAMConverter::build_pose(gps_slam_conversion::position::Point converted_slam_point)
{
    const double &slam_x = converted_slam_point.get__x();
    const double &slam_y = converted_slam_point.get__y();

    RCLCPP_INFO(this->node_->get_logger(), "converted_slam_point\n\tx : %f\n\ty : %f", slam_x, slam_y);
    RCLCPP_LINE_INFO();

    geometry_msgs::msg::Point::UniquePtr point = std::make_unique<geometry_msgs::msg::Point>();
    point->set__x(slam_x);
    point->set__y(slam_y);
    point->set__z(RCL_DEFAULT_DOUBLE);

    const geometry_msgs::msg::Point &&point_moved = std::move(*point);

    geometry_msgs::msg::Quaternion::UniquePtr quaternion = std::make_unique<geometry_msgs::msg::Quaternion>();
    quaternion->set__x(RCL_DEFAULT_DOUBLE);
    quaternion->set__y(RCL_DEFAULT_DOUBLE);
    quaternion->set__z(RCL_DEFAULT_DOUBLE);
    quaternion->set__w(RCL_DEFAULT_DOUBLE);

    const geometry_msgs::msg::Quaternion &&quaternion_moved = std::move(*quaternion);

    geometry_msgs::msg::Pose::UniquePtr pose = std::make_unique<geometry_msgs::msg::Pose>();
    pose->set__position(point_moved);
    pose->set__orientation(quaternion_moved);

    const geometry_msgs::msg::Pose &&pose_moved = std::move(*pose);

    return pose_moved;
}

void gps_slam_conversion::node::GpsSLAMConverter::slam_pose_subscription_cb(geometry_msgs::msg::Pose::SharedPtr slam_pose_cb_data)
{
    const double &pose_x = slam_pose_cb_data->position.x;
    const double &pose_y = slam_pose_cb_data->position.y;

    gps_slam_conversion::position::Point converted_gps_point = this->position_converter_->convert_slam_to_gps(
        pose_x, pose_y,
        *lon_lat_LB_point_, *lon_lat_RT_point_);

    sensor_msgs::msg::NavSatFix built_nav_sat_fix = this->build_nav_sat_fix(converted_gps_point);
    this->converted_gps_publisher_->publish(built_nav_sat_fix);
}

void gps_slam_conversion::node::GpsSLAMConverter::gps_subscription_cb(sensor_msgs::msg::NavSatFix::SharedPtr gps_cb_data)
{
    const double &current_longitude = gps_cb_data->longitude;
    const double &current_latitude = gps_cb_data->latitude;

    gps_slam_conversion::position::Point converted_slam_point = this->position_converter_->convert_gps_to_slam(
        current_longitude, current_latitude,
        *lon_lat_LB_point_, *lon_lat_RT_point_);

    geometry_msgs::msg::Pose built_pose = this->build_pose(converted_slam_point);
    this->converted_slam_publisher_->publish(built_pose);
}

void gps_slam_conversion::node::GpsSLAMConverter::converter_service_cb(const gps_slam_conversion_msgs::srv::Conversion::Request::SharedPtr request, gps_slam_conversion_msgs::srv::Conversion::Response::SharedPtr response)
{
    const std::string &request_conversion_target_data = request->conversion_target.data;
    RCLCPP_INFO(this->node_->get_logger(), "[%s] conversion_target : %s", RCL_CONVERTER_SERVICE_SERVER_NAME, request_conversion_target_data.c_str());

    std::vector<sensor_msgs::msg::NavSatFix> gps_request_list = request->gps_request_list;
    const size_t gps_request_list_size = gps_request_list.size();

    std::vector<geometry_msgs::msg::Pose> slam_pose_request_list = request->slam_pose_request_list;
    const size_t slam_pose_request_list_size = slam_pose_request_list.size();

    if (request_conversion_target_data == "")
    {
        RCLCPP_ERROR(this->node_->get_logger(), "[%s] conversion_target is empty", RCL_CONVERTER_SERVICE_SERVER_NAME);
        RCLCPP_LINE_ERROR();
        return;
    }
    else if (request_conversion_target_data == RCL_CONVERTER_SERVICE_CONVERSION_TARGET_SLAM)
    {
        bool is_gps_request_list_empty = gps_request_list.empty();
        bool is_slam_request_list_not_empty = !slam_pose_request_list.empty();

        if (is_gps_request_list_empty)
        {
            RCLCPP_ERROR(this->node_->get_logger(), "[%s] GPS conversion gps_request_list is empty", RCL_CONVERTER_SERVICE_SERVER_NAME);
            RCLCPP_LINE_ERROR();
            return;
        }

        if (is_slam_request_list_not_empty)
        {
            RCLCPP_ERROR(this->node_->get_logger(), "[%s] GPS conversion slam_pose_request_list is not empty. cleared", RCL_CONVERTER_SERVICE_SERVER_NAME);
            RCLCPP_LINE_ERROR();
            slam_pose_request_list.clear();
        }

        std::vector<geometry_msgs::msg::Pose> slam_pose_response_list;

        for (const sensor_msgs::msg::NavSatFix &gps : gps_request_list)
        {
            const double &lon = gps.longitude;
            const double &lat = gps.latitude;

            gps_slam_conversion::position::Point converted_slam_point = this->position_converter_->convert_gps_to_slam(
                lon, lat,
                *lon_lat_LB_point_, *lon_lat_RT_point_);

            geometry_msgs::msg::Pose built_pose = this->build_pose(converted_slam_point);

            slam_pose_response_list.push_back(built_pose);
        }

        const size_t slam_pose_response_list_size = slam_pose_response_list.size();

        bool is_converting_finished = (gps_request_list_size == slam_pose_response_list_size);

        if (is_converting_finished)
        {
            std::for_each(slam_pose_response_list.begin(), slam_pose_response_list.end(), [this](const geometry_msgs::msg::Pose &pose) {
                const double &response_pose_x = pose.position.x;
                const double &response_pose_y = pose.position.y;

                RCLCPP_INFO(
                    this->node_->get_logger(),
                    "[%s] slam_pose_reponse_list\n\tx : %f\n\ty :%f",
                    RCL_CONVERTER_SERVICE_SERVER_NAME,
                    response_pose_x, response_pose_y);
                RCLCPP_LINE_INFO();
            });

            response->set__slam_pose_response_list(slam_pose_response_list);
        }
        else
        {
            return;
        }
    }
    else if (request_conversion_target_data == RCL_CONVERTER_SERVICE_CONVERSION_TARGET_GPS)
    {
        bool is_slam_request_list_empty = slam_pose_request_list.empty();
        bool is_gps_request_list_not_empty = !gps_request_list.empty();
        
        if (is_slam_request_list_empty)
        {
            RCLCPP_ERROR(this->node_->get_logger(), "[%s] SLAM conversion slam_pose_request_list is empty", RCL_CONVERTER_SERVICE_SERVER_NAME);
            RCLCPP_LINE_ERROR();
            return;
        }
        
        if (is_gps_request_list_not_empty)
        {
            RCLCPP_ERROR(this->node_->get_logger(), "[%s] SLAM conversion gps_request_list is not empty. cleared", RCL_CONVERTER_SERVICE_SERVER_NAME);
            RCLCPP_LINE_ERROR();
            gps_request_list.clear();
        }

        std::vector<sensor_msgs::msg::NavSatFix> gps_response_list;

        for (const geometry_msgs::msg::Pose &pose : slam_pose_request_list)
        {
            const double &x = pose.position.x;
            const double &y = pose.position.y;

            gps_slam_conversion::position::Point converted_gps_point = this->position_converter_->convert_slam_to_gps(
                x, y,
                *lon_lat_LB_point_, *lon_lat_RT_point_);

            sensor_msgs::msg::NavSatFix built_nav_sat_fix = this->build_nav_sat_fix(converted_gps_point);

            gps_response_list.push_back(built_nav_sat_fix);
        }

        const size_t gps_response_list_size = gps_response_list.size();

        bool is_converting_finished = (slam_pose_request_list_size == gps_response_list_size);

        if (is_converting_finished)
        {
            std::for_each(gps_response_list.begin(), gps_response_list.end(), [this](const sensor_msgs::msg::NavSatFix &nav_sat_fix) {
                const double &response_pose_lon = nav_sat_fix.longitude;
                const double &response_pose_lat = nav_sat_fix.latitude;

                RCLCPP_INFO(
                    this->node_->get_logger(),
                    "[%s] gps_reponse_list\n\tx : %f\n\ty :%f",
                    RCL_CONVERTER_SERVICE_SERVER_NAME,
                    response_pose_lon, response_pose_lat);
                RCLCPP_LINE_INFO();
            });
            
            response->set__gps_response_list(gps_response_list);
        }
        else
        {
            return;
        }
    }
    else
    {
        RCLCPP_ERROR(this->node_->get_logger(), "%s request is invalid", RCL_CONVERTER_SERVICE_SERVER_NAME);
        RCLCPP_LINE_ERROR();
    }
}