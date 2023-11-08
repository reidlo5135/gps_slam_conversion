#include "position_converter/position_converter.hxx"

gps_slam_conversion::position::PositionConverter::PositionConverter()
{
}

gps_slam_conversion::position::PositionConverter::~PositionConverter()
{
}

double gps_slam_conversion::position::PositionConverter::get__mapping_max_width() const
{
    return this->mapping_max_width_;
}

void gps_slam_conversion::position::PositionConverter::set__mapping_max_width(double mapping_max_width)
{
    this->mapping_max_width_ = mapping_max_width;
}

double gps_slam_conversion::position::PositionConverter::get__mapping_max_height() const
{
    return this->mapping_max_height_;
}

void gps_slam_conversion::position::PositionConverter::set__mapping_max_height(double mapping_max_height)
{
    this->mapping_max_height_ = mapping_max_height;
}

double gps_slam_conversion::position::PositionConverter::get__slam_rotation_angle() const
{
    return this->slam_rotation_angle_;
}

void gps_slam_conversion::position::PositionConverter::set__slam_rotation_angle(double slam_rotation_angle)
{
    this->slam_rotation_angle_ = slam_rotation_angle;
}

gps_slam_conversion::position::Point gps_slam_conversion::position::PositionConverter::get__area_offset() const
{
    return this->area_offset_;
}

void gps_slam_conversion::position::PositionConverter::set__area_offset(const gps_slam_conversion::position::Point &area_offest)
{
    this->area_offset_ = area_offest;
}

void gps_slam_conversion::position::PositionConverter::init_area(
    int slam_width,
    int slam_height,
    gps_slam_conversion::position::Point intersection_start_point,
    gps_slam_conversion::position::Point intersection_end_point)
{
    // 1) lon2, lat2와 lon1, lat2로 SLAM Map 회전 각도
    const double &slam_rotation_angle = this->get_angle(
        intersection_start_point.get__x(), intersection_start_point.get__y(),
        intersection_end_point.get__x(), intersection_end_point.get__y());
    this->set__slam_rotation_angle(slam_rotation_angle);

    if (slam_rotation_angle < 0)
    {
        RCUTILS_LOG_ERROR_NAMED(RCL_NODE_NAME, "init_area slam_rotation_angle is less than 0. aborting...");
        RCLCPP_LINE_ERROR();
        return;
    }

    // 2) GPS Mapping Map 크기
    const double &mapping_max_width = std::round(sin(slam_rotation_angle) * slam_height + cos(slam_rotation_angle) * slam_width);
    this->set__mapping_max_width(mapping_max_width);

    const double &mapping_max_height = std::round(cos(slam_rotation_angle) * slam_height + sin(slam_rotation_angle) * slam_width);
    this->set__mapping_max_height(mapping_max_height);

    // RCUTILS_LOG_INFO_NAMED(
    //     RCL_NODE_NAME,
    //     "init_area mapping map\n\twidth : %f\n\theight : %f",
    //     mapping_max_width, mapping_max_height);
    // RCLCPP_LINE_INFO();

    // 3) x offset
    std::unique_ptr<gps_slam_conversion::position::Point> area_offset_point = std::make_unique<gps_slam_conversion::position::Point>();
    const double &area_offset_point_x = std::round(sin(slam_rotation_angle) * slam_height);
    area_offset_point->set__x(area_offset_point_x);
    area_offset_point->set__y(0.0);

    // RCUTILS_LOG_INFO_NAMED(
    //     RCL_NODE_NAME,
    //     "init_area area_offset_point\n\tx : %f\n\ty : %f",
    //     area_offset_point->get__x(), area_offset_point->get__y());
    // RCLCPP_LINE_INFO();

    const gps_slam_conversion::position::Point &&point_moved = std::move(*area_offset_point);
    this->set__area_offset(point_moved);
}

gps_slam_conversion::position::Point gps_slam_conversion::position::PositionConverter::convert_gps_to_slam(
    double lon,
    double lat,
    gps_slam_conversion::position::Point lon_lat_LB,
    gps_slam_conversion::position::Point lon_lat_RT)
{
    // 1) GPS to Mapping Map
    const double &mapping_max_width = this->get__mapping_max_width();
    const double &mapping_max_height = this->get__mapping_max_height();

    // RCUTILS_LOG_INFO_NAMED(
    //     RCL_NODE_NAME,
    //     "convert_gps_to_slam mapping map\n\twidth : %f\n\theight : %f",
    //     mapping_max_width, mapping_max_height);
    // RCLCPP_LINE_INFO();

    const double &m_x = ((lon - lon_lat_LB.get__x()) / (lon_lat_RT.get__x() - lon_lat_LB.get__x())) * mapping_max_width;
    const double &m_y = ((lat - lon_lat_LB.get__y()) / (lon_lat_RT.get__y() - lon_lat_LB.get__y())) * mapping_max_height;

    // 2) Mapping Map to SLAM Position
    gps_slam_conversion::position::Point slam_pos = this->convert_slam_pos(static_cast<int>(m_x), static_cast<int>(m_y), GPS);

    return slam_pos;
}

gps_slam_conversion::position::Point gps_slam_conversion::position::PositionConverter::convert_slam_to_gps(
    int x,
    int y,
    gps_slam_conversion::position::Point lon_lat_LB,
    gps_slam_conversion::position::Point lon_lat_RT)
{
    const double &mapping_max_width = this->get__mapping_max_width();
    const double &mapping_max_height = this->get__mapping_max_height();

    // RCUTILS_LOG_INFO_NAMED(
    //     RCL_NODE_NAME,
    //     "convert_slam_to_gps mapping map\n\twidth : %f\n\theight : %f",
    //     mapping_max_width, mapping_max_height);
    // RCLCPP_LINE_INFO();

    // 1) SLAM Position to Mapping Map
    gps_slam_conversion::position::Point slam_pos = this->convert_slam_pos(x, y, SLAM);

    // 2) Mapping Map to GPS
    const double &lon = lon_lat_LB.get__x() + (lon_lat_RT.get__x() - lon_lat_LB.get__x()) * (slam_pos.get__x() / mapping_max_width);
    const double &lat = lon_lat_LB.get__y() + (lon_lat_RT.get__y() - lon_lat_LB.get__y()) * (slam_pos.get__y() / mapping_max_height);

    std::unique_ptr<gps_slam_conversion::position::Point> point = std::make_unique<gps_slam_conversion::position::Point>();
    point->set__x(lon);
    point->set__y(lat);

    // RCUTILS_LOG_INFO_NAMED(
    //     RCL_NODE_NAME,
    //     "convert_slam_to_gps point\n\tx : %f\n\ty : %f",
    //     point->get__x(), point->get__y());
    // RCLCPP_LINE_INFO();

    const gps_slam_conversion::position::Point &&point_moved = std::move(*point);

    return point_moved;
}

gps_slam_conversion::position::Point gps_slam_conversion::position::PositionConverter::convert_slam_pos(int x, int y, gps_slam_conversion::position::WorkType type)
{
    const double &slam_rotation_angle = this->get__slam_rotation_angle();

    if (slam_rotation_angle <= 0)
    {
        RCUTILS_LOG_ERROR_NAMED(RCL_NODE_NAME, "convert_slam_pose slam_rotation_angle is less than 0. aborting...");
        RCLCPP_LINE_ERROR();
    }

    const gps_slam_conversion::position::Point &area_offset = this->get__area_offset();
    std::unique_ptr<gps_slam_conversion::position::Point> point = std::make_unique<gps_slam_conversion::position::Point>();

    if (type == SLAM)
    {
        // 1) x0, y0과 x, y 각도 a
        double a = atan2(y, x);

        // RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "convert_slam_pos SLAM a : %f", a);
        // RCLCPP_LINE_INFO();

        // 2) x0, y0과 x, y 거리 len
        long len = std::round(sqrt(x * x + y * y));

        // RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "convert_slam_pos SLAM len : %ld", len);
        // RCLCPP_LINE_INFO();

        // 3) Mapping Map x
        double m_x = std::round(cos(slam_rotation_angle + a) * len + area_offset.get__x());

        // 4) Mapping Map y
        double m_y = std::round(sin(slam_rotation_angle + a) * len);

        point->set__x(m_x);
        point->set__y(m_y);

        // RCUTILS_LOG_INFO_NAMED(
        //     RCL_NODE_NAME,
        //     "convert_slam_pos SLAM mapping map\n\tm_x : %f\n\tm_y : %f",
        //     point->get__x(), point->get__y());
        // RCLCPP_LINE_INFO();
    }
    else if (type == GPS)
    {
        // 1) x offset, 0 -> x0, y0
        double x_std = x - area_offset.get__x();
        double y_std = y;

        // RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "convert_slam_pos GPS\n\tx_std : %f\n\ty_std : %f", x_std, y_std);
        // RCLCPP_LINE_INFO();

        // 2) x0, y0 과 x', y' 각도 β
        double b = atan2(y_std, x_std);

        // RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "convert_slam_pos GPS b : %f", b);
        // RCLCPP_LINE_INFO();

        // x0, y0 과 x', y' 거리 len
        long len = std::round(sqrt(x_std * x_std + y_std * y_std));
        // RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "convert_slam_pos GPS len : %ld", len);
        // RCLCPP_LINE_INFO();

        double s_x = std::round(cos(b - slam_rotation_angle) * len);
        double s_y = std::round(sin(b - slam_rotation_angle) * len);

        point->set__x(s_x);
        point->set__y(s_y);

        // RCUTILS_LOG_INFO_NAMED(
        //     RCL_NODE_NAME,
        //     "convert_slam_pos GPS mapping map\n\tm_x : %f\n\tm_y : %f",
        //     point->get__x(), point->get__y());
        // RCLCPP_LINE_INFO();
    }
    else
    {
        RCUTILS_LOG_ERROR_NAMED(RCL_NODE_NAME, "convert_slam_pos Unknown WorkType. aborting...");
        RCLCPP_LINE_ERROR();
    }

    const gps_slam_conversion::position::Point &&point_moved = std::move(*point);

    return point_moved;
}

double gps_slam_conversion::position::PositionConverter::get_angle(double lon1, double lat1, double lon2, double lat2)
{
    double y1 = lat1 * M_PI / 180;
    double y2 = lat2 * M_PI / 180;
    double x1 = lon1 * M_PI / 180;
    double x2 = lon2 * M_PI / 180;

    // RCUTILS_LOG_INFO_NAMED(
    //     RCL_NODE_NAME,
    //     "get_angle\n\tx1 : %f\n\ty1 : %f\n\tx2 : %f\n\ty2 : %f",
    //     x1, y1,
    //     x2, y2);
    // RCLCPP_LINE_INFO();

    double y = sin(x2 - x1) * cos(y2);
    double x = (cos(y1) * sin(y2)) - (sin(y1) * cos(y2) * cos(x2 - x1));

    double theta = atan2(y, x);

    // 정북 기준이므로 변환
    double angle = M_PI / 2 - theta;

    // RCUTILS_LOG_INFO_NAMED(
    //     RCL_NODE_NAME,
    //     "get_angle\n\tx : %f\n\ty : %f\n\ttheta : %f\n\tangle : %f",
    //     x, y,
    //     theta, angle);
    // RCLCPP_LINE_INFO();

    return angle;
}

std::vector<gps_slam_conversion::position::Point> gps_slam_conversion::position::PositionConverter::convert_slam_to_virtual_map_area(
    int x, int y, int width, int height,
    double dist_per_pix,
    gps_slam_conversion::position::Point map_point,
    gps_slam_conversion::position::Point lon_lat_lt, gps_slam_conversion::position::Point lon_lat_rt)
{
    const double &map_point_x = map_point.get__x();
    const double &map_point_y = map_point.get__y();
    const double &lon_lat_lt_x = lon_lat_lt.get__x();
    const double &lon_lat_lt_y = lon_lat_lt.get__y();
    const double &lon_lat_rt_x = lon_lat_lt.get__x();
    const double &lon_lat_rt_y = lon_lat_lt.get__y();

    const double &slam_rotation_angle = this->get_angle(
        lon_lat_lt_x, lon_lat_lt_y,
        lon_lat_rt_x, lon_lat_rt_y);

    const double &y_dist_slam = (height - y) * dist_per_pix;
    const double &x_dist_slam = (width - x) * dist_per_pix;
    const double &rt_point_angle = atan2(height - y, width - x);
    const double &dist_slam = sqrt(pow((height - y), 2) + pow((width - x), 2)) * dist_per_pix;

    const double &diagonal_angle = atan2(height, width);
    const double &diagonal_distance = sqrt(pow((height), 2) + pow((width), 2)) * dist_per_pix;
    const double &height_distance = height * dist_per_pix;
    const double &width_distance = width * dist_per_pix;

    const gps_slam_conversion::position::Point &right_top_pos = this->get_moving_lon_lat(
        map_point_x, map_point_y, dist_slam,
        slam_rotation_angle + rt_point_angle);

    const gps_slam_conversion::position::Point &left_top_pos = this->get_moving_lon_lat(
        right_top_pos.get__x(), right_top_pos.get__y(),
        width_distance, slam_rotation_angle + M_PI);
    
    const gps_slam_conversion::position::Point &left_bottom_pos = this->get_moving_lon_lat(
        right_top_pos.get__x(), right_top_pos.get__y(),
        diagonal_distance, slam_rotation_angle + diagonal_angle + M_PI);

    const gps_slam_conversion::position::Point &right_bottom_pos = this->get_moving_lon_lat(
        right_top_pos.get__x(), right_top_pos.get__y(),
        height_distance, slam_rotation_angle + (M_PI + (M_PI / 2)));

    std::unique_ptr<gps_slam_conversion::position::Point> left_point = std::make_unique<gps_slam_conversion::position::Point>();
    left_point->set__x(left_top_pos.get__x());
    left_point->set__y(left_top_pos.get__y());

    gps_slam_conversion::position::Point &&left_point_moved = std::move(*left_point);

    std::unique_ptr<gps_slam_conversion::position::Point> right_point = std::make_unique<gps_slam_conversion::position::Point>();
    right_point->set__x(right_bottom_pos.get__x());
    right_point->set__y(right_bottom_pos.get__y());

    gps_slam_conversion::position::Point &&right_point_moved = std::move(*right_point);

    std::vector<gps_slam_conversion::position::Point> point_vec;
    point_vec.push_back(left_point_moved);
    point_vec.push_back(right_point_moved);

    return point_vec;
}

gps_slam_conversion::position::Point gps_slam_conversion::position::PositionConverter::get_moving_lon_lat(double lon, double lat, double distance, double radian)
{
    double dist_per_lon_degree = 91290.0;
    double dist_per_lat_degree = 110941.0;

    double quadrant_1 = 90 * M_PI / 180;
    double quadrant_2 = 180 * M_PI / 180;
    double quadrant_3 = 270 * M_PI / 180;

    double longitude_move = (sqrt(pow(distance, 2) - pow(sin(radian) * distance, 2))) / dist_per_lon_degree;
    double latitude_move = (sqrt(pow(distance, 2) - pow(cos(radian) * distance, 2))) / dist_per_lat_degree;

    double latitude = 0.0;
    double longitude = 0.0;

    if (quadrant_1 >= radian)
    {
        longitude = lon + longitude_move;
        latitude = lat + latitude_move;
    }
    else if (quadrant_2 >= radian)
    {
        longitude = lon - longitude_move;
        latitude = lat + latitude_move;
    }
    else if (quadrant_3 >= radian)
    {
        longitude = lon + longitude_move;
        latitude = lat - latitude_move;
    }
    else {
        longitude = lon + longitude_move;
        latitude = lat - latitude_move;
    }

    std::unique_ptr<gps_slam_conversion::position::Point> point = std::make_unique<gps_slam_conversion::position::Point>();
    point->set__x(longitude);
    point->set__y(latitude);

    gps_slam_conversion::position::Point &&point_moved = std::move(*point);

    return point_moved;
}