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
    if (slam_rotation_angle_ < 0)
    {
        return;
    }

    const double &slam_rotation_angle = this->get__slam_rotation_angle();

    const double &mapping_max_width = std::round(sin(slam_rotation_angle) * slam_height + cos(slam_rotation_angle) * slam_width);
    this->set__mapping_max_width(mapping_max_width);

    const double &mapping_max_height = std::round(cos(slam_rotation_angle) * slam_height + sin(slam_rotation_angle) * slam_width);
    this->set__mapping_max_height(mapping_max_height);

    std::unique_ptr<gps_slam_conversion::position::Point> point = std::make_unique<gps_slam_conversion::position::Point>();
    const double &point_x = std::round(sin(slam_rotation_angle) * slam_height);
    point->set__x(point_x);
    point->set__y(0.0);
    
    const gps_slam_conversion::position::Point &&point_moved = std::move(*point);
    this->set__area_offset(point_moved);
}

gps_slam_conversion::position::Point gps_slam_conversion::position::PositionConverter::convert_gps_to_slam(
    double lon,
    double lat,
    gps_slam_conversion::position::Point lon_lat_LB,
    gps_slam_conversion::position::Point lon_lat_RT)
{
    const double &mapping_max_width = this->get__mapping_max_width();
    const double &mapping_max_height = this->get__mapping_max_height();

    const double &m_x = ((lon - lon_lat_LB.get__x()) / (lon_lat_RT.get__x() - lon_lat_LB.get__x())) * mapping_max_width;
    const double &m_y = ((lat - lon_lat_LB.get__y()) / (lon_lat_RT.get__y() - lon_lat_LB.get__y())) * mapping_max_height;

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

    gps_slam_conversion::position::Point slam_pos = this->convert_slam_pos(x, y, SLAM);

    const double &lon = lon_lat_LB.get__x() + (lon_lat_RT.get__x() - lon_lat_LB.get__x()) * (slam_pos.get__x() / mapping_max_width);
    const double &lat = lon_lat_LB.get__y() + (lon_lat_RT.get__y() - lon_lat_LB.get__y()) * (slam_pos.get__y() / mapping_max_height);

    std::unique_ptr<gps_slam_conversion::position::Point> point = std::make_unique<gps_slam_conversion::position::Point>();
    point->set__x(lon);
    point->set__y(lat);

    const gps_slam_conversion::position::Point &&point_moved = std::move(*point);

    return point_moved;
}
