#include "gps_slam_converter/gps_slam_converter.hxx"

void position_test()
{
    // slam area : 1040,600

    // 좌측 교점:128.8575686,35.15783502
    // 상단 교점:128.858427988,35.158463143
    // 우측 교점:128.858870603,35.158056682
    // 하단 교점:128.858009083,35.157430158

    // 전제 영역
    // LB : 128.8575686,35.157430158
    // RT : 128.858870603,35.158463143

    // LB : 128.8575686001505289,35.1574301543503580
    // RT : 128.8588706034498728,35.1584631433985280

    std::shared_ptr<gps_slam_conversion::position::Point> intersection_start_point = std::make_shared<gps_slam_conversion::position::Point>();
    intersection_start_point->set__x(128.858009083);
    intersection_start_point->set__y(35.157430158);

    std::shared_ptr<gps_slam_conversion::position::Point> intersection_end_point = std::make_shared<gps_slam_conversion::position::Point>();
    intersection_end_point->set__x(128.858870603);
    intersection_end_point->set__y(35.158056682);

    std::shared_ptr<gps_slam_conversion::position::PositionConverter> position_converter = std::make_shared<gps_slam_conversion::position::PositionConverter>();
    position_converter->init_area(2080, 1200, *intersection_start_point, *intersection_end_point);

    std::shared_ptr<gps_slam_conversion::position::Point> lon_lat_LB_point = std::make_shared<gps_slam_conversion::position::Point>();
    lon_lat_LB_point->set__x(128.8575686);
    lon_lat_LB_point->set__y(35.157430158);

    std::shared_ptr<gps_slam_conversion::position::Point> lon_lat_RT_point = std::make_shared<gps_slam_conversion::position::Point>();
    lon_lat_RT_point->set__x(128.858870603);
    lon_lat_RT_point->set__y(35.158463143);

    gps_slam_conversion::position::Point slam_point1 = position_converter->convert_gps_to_slam(128.8575686, 35.15783502, *lon_lat_LB_point, *lon_lat_RT_point);
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "slam_point1\n\tx : %f\n\ty : %f", slam_point1.get__x(), slam_point1.get__y());
    RCLCPP_LINE_INFO();

    gps_slam_conversion::position::Point slam_point2 = position_converter->convert_gps_to_slam(128.858427988, 35.158463143, *lon_lat_LB_point, *lon_lat_RT_point);
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "slam_point2\n\tx : %f\n\ty : %f", slam_point2.get__x(), slam_point2.get__y());
    RCLCPP_LINE_INFO();

    gps_slam_conversion::position::Point slam_point3 = position_converter->convert_gps_to_slam(128.858870603, 35.158056682, *lon_lat_LB_point, *lon_lat_RT_point);
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "slam_point3\n\tx : %f\n\ty : %f", slam_point3.get__x(), slam_point3.get__y());
    RCLCPP_LINE_INFO();

    gps_slam_conversion::position::Point slam_point4 = position_converter->convert_gps_to_slam(128.858009083, 35.157430158, *lon_lat_LB_point, *lon_lat_RT_point);
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "slam_point4\n\tx : %f\n\ty : %f", slam_point4.get__x(), slam_point4.get__y());
    RCLCPP_LINE_INFO();

    gps_slam_conversion::position::Point slam_point5 = position_converter->convert_gps_to_slam(128.85821960150002, 35.1579457443728, *lon_lat_LB_point, *lon_lat_RT_point);
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "slam_point5\n\tx : %f\n\ty : %f", slam_point5.get__x(), slam_point5.get__y());
    RCLCPP_LINE_INFO();

    gps_slam_conversion::position::Point gps_point1 = position_converter->convert_slam_to_gps(0, 0, *lon_lat_LB_point, *lon_lat_RT_point);
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "gps_point1\n\tlat : %f\n\tlon : %f", gps_point1.get__y(), gps_point1.get__x());
    RCLCPP_LINE_INFO();

    gps_slam_conversion::position::Point gps_point2 = position_converter->convert_slam_to_gps(2080, 0, *lon_lat_LB_point, *lon_lat_RT_point);
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "gps_point2\n\tlat : %f\n\tlon : %f", gps_point2.get__y(), gps_point2.get__x());
    RCLCPP_LINE_INFO();

    gps_slam_conversion::position::Point gps_point3 = position_converter->convert_slam_to_gps(2080, 1200, *lon_lat_LB_point, *lon_lat_RT_point);
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "gps_point3\n\tlat : %f\n\tlon : %f", gps_point3.get__y(), gps_point3.get__x());
    RCLCPP_LINE_INFO();

    gps_slam_conversion::position::Point gps_point4 = position_converter->convert_slam_to_gps(0, 1200, *lon_lat_LB_point, *lon_lat_RT_point);
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "gps_point4\n\tlat : %f\n\tlon : %f", gps_point4.get__y(), gps_point4.get__x());
    RCLCPP_LINE_INFO();

    gps_slam_conversion::position::Point gps_point5 = position_converter->convert_slam_to_gps(1040, 600, *lon_lat_LB_point, *lon_lat_RT_point);
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "gps_point5\n\tlat : %f\n\tlon : %f", gps_point5.get__y(), gps_point5.get__x());
    RCLCPP_LINE_INFO();
}

int main(int argc, const char *const *argv)
{
    // position_test();
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr rcl_node_ptr = std::make_shared<gps_slam_conversion::node::GpsSLAMConverter>();
    rclcpp::spin(rcl_node_ptr);
    rclcpp::shutdown();

    return 0;
}