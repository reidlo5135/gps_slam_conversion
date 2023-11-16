#include "gps_slam_converter/gps_slam_converter.hxx"
#include <iostream>
#include <iomanip>

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

    std::shared_ptr<gps_slam_conversion::position::PositionConverter> position_converter = std::make_shared<gps_slam_conversion::position::PositionConverter>();

    std::shared_ptr<gps_slam_conversion::position::Point> map_point = std::make_shared<gps_slam_conversion::position::Point>();
    map_point->set__x(128.8586);
    map_point->set__y(35.15797);

    std::shared_ptr<gps_slam_conversion::position::Point> lon_lat_LT = std::make_shared<gps_slam_conversion::position::Point>();
    lon_lat_LT->set__x(128.858009083);
    lon_lat_LT->set__y(35.157430158);

    std::shared_ptr<gps_slam_conversion::position::Point> lon_lat_RT = std::make_shared<gps_slam_conversion::position::Point>();
    lon_lat_RT->set__x(128.858870603);
    lon_lat_RT->set__y(35.158056682);

    const std::vector<gps_slam_conversion::position::Point> &virtual_arr = position_converter->convert_slam_to_virtual_map_area(
        260, 65,
        302, 117, 0.05,
        *map_point,
        *lon_lat_LT,
        *lon_lat_RT);

    for (const gps_slam_conversion::position::Point &point : virtual_arr)
    {
        std::cout << std::fixed << std::setprecision(13) << point.get__x() << std::endl;
        std::cout << std::fixed << std::setprecision(13) << point.get__y() << std::endl;

        // RCUTILS_LOG_INFO_NAMED(
        //     RCL_NODE_NAME,
        //     "virtual point arr\n\tx : %f\n\ty : %f",
        //     point.get__x(),
        //     point.get__y());
        // RCLCPP_LINE_INFO();
    }

    std::shared_ptr<gps_slam_conversion::position::Point> intersection_start_point = std::make_shared<gps_slam_conversion::position::Point>();
    intersection_start_point->set__x(128.858009083);
    intersection_start_point->set__y(35.157430158);

    std::shared_ptr<gps_slam_conversion::position::Point> intersection_end_point = std::make_shared<gps_slam_conversion::position::Point>();
    intersection_end_point->set__x(128.858870603);
    intersection_end_point->set__y(35.158056682);

    position_converter->init_area(302, 117, *intersection_start_point, *intersection_end_point);

    std::shared_ptr<gps_slam_conversion::position::Point> lon_lat_LB_point = std::make_shared<gps_slam_conversion::position::Point>();
    lon_lat_LB_point->set__x(128.8584746703773);
    lon_lat_LB_point->set__y(35.15787023134168);

    std::shared_ptr<gps_slam_conversion::position::Point> lon_lat_RT_point = std::make_shared<gps_slam_conversion::position::Point>();
    lon_lat_RT_point->set__x(128.85864084895368);
    lon_lat_RT_point->set__y(35.15800009142947);

    // 원점
    gps_slam_conversion::position::Point gps_point1 = position_converter->convert_slam_to_gps(0, 0, *lon_lat_LB_point, *lon_lat_RT_point);
    double differ_y_1 = 35.15787023134168 - gps_point1.get__y();
    double differ_x_1 = 128.8584746703773 - gps_point1.get__x();
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "gps_point1\n\tlat : %f\n\tlon : %f", gps_point1.get__y(), gps_point1.get__x());
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "differ_1\n\tlat : %f\n\tlon : %f", differ_y_1, differ_x_1);
    RCLCPP_LINE_INFO();

    gps_slam_conversion::position::Point slam_point1 = position_converter->convert_gps_to_slam(gps_point1.get__x(), gps_point1.get__y(), *lon_lat_LB_point, *lon_lat_RT_point);
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "slam_point1\n\tx : %f\n\ty : %f", slam_point1.get__x(), slam_point1.get__y());
    RCLCPP_LINE_INFO();

    // 우하단
    gps_slam_conversion::position::Point gps_point2 = position_converter->convert_slam_to_gps(302, 0, *lon_lat_LB_point, *lon_lat_RT_point);
    double differ_y_2 = 35.15800009142947 - gps_point2.get__y();
    double differ_x_2 = 128.85864084895368 - gps_point2.get__x();
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "gps_point2\n\tlat : %f\n\tlon : %f", gps_point2.get__x(), gps_point2.get__y());
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "differ_2\n\tlat : %f\n\tlon : %f", differ_y_2, differ_x_2);
    RCLCPP_LINE_INFO();

    gps_slam_conversion::position::Point slam_point2 = position_converter->convert_gps_to_slam(gps_point2.get__x(), gps_point2.get__y(), *lon_lat_LB_point, *lon_lat_RT_point);
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "slam_point2\n\tx : %f\n\ty : %f", slam_point2.get__x(), slam_point2.get__y());
    RCLCPP_LINE_INFO();

    // 우상단
    gps_slam_conversion::position::Point gps_point3 = position_converter->convert_slam_to_gps(302, 117, *lon_lat_LB_point, *lon_lat_RT_point);
    double differ_y_3 = 35.15800009142947 - gps_point3.get__y();
    double differ_x_3 = 128.85864084895368 - gps_point3.get__x();
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "gps_point3\n\tlat : %f\n\tlon : %f", gps_point3.get__x(), gps_point3.get__y());
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "differ_3\n\tlat : %f\n\tlon : %f", differ_y_3, differ_x_3);
    RCLCPP_LINE_INFO();

    gps_slam_conversion::position::Point slam_point3 = position_converter->convert_gps_to_slam(gps_point3.get__x(), gps_point3.get__y(), *lon_lat_LB_point, *lon_lat_RT_point);
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "slam_point3\n\tx : %f\n\ty : %f", slam_point3.get__x(), slam_point3.get__y());
    RCLCPP_LINE_INFO();

    // 좌상단
    gps_slam_conversion::position::Point gps_point4 = position_converter->convert_slam_to_gps(0, 117, *lon_lat_LB_point, *lon_lat_RT_point);
    double differ_y_4 = 35.15787023134168 - gps_point4.get__y();
    double differ_x_4 = 128.8584746703773 - gps_point4.get__x();
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "gps_point4\n\tlat : %f\n\tlon : %f", gps_point4.get__y(), gps_point4.get__x());
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "differ_4\n\tlat : %f\n\tlon : %f", differ_y_4, differ_x_4);
    RCLCPP_LINE_INFO();

    gps_slam_conversion::position::Point slam_point4 = position_converter->convert_gps_to_slam(gps_point4.get__x(), gps_point4.get__y(), *lon_lat_LB_point, *lon_lat_RT_point);
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "slam_point4\n\tx : %f\n\ty : %f", slam_point4.get__x(), slam_point4.get__y());
    RCLCPP_LINE_INFO();


    gps_slam_conversion::position::Point gps_point5 = position_converter->convert_slam_to_gps(260, 65, *lon_lat_LB_point, *lon_lat_RT_point);
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "gps_point5\n\tlat : %f\n\tlon : %f", gps_point5.get__y(), gps_point5.get__x());
    RCLCPP_LINE_INFO();

    gps_slam_conversion::position::Point slam_point5 = position_converter->convert_gps_to_slam(gps_point5.get__x(), gps_point5.get__y(), *lon_lat_LB_point, *lon_lat_RT_point);
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "slam_point5\n\tx : %f\n\ty : %f", slam_point5.get__x(), slam_point5.get__y());
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