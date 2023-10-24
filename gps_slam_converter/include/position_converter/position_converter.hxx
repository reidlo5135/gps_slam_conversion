#ifndef POSITION_CONVERTER__HXX
#define POSITION_CONVERTER__HXX

#include <cmath>
#include <memory>

namespace gps_slam_conversion
{
    namespace position
    {
        enum WorkType
        {
            SLAM,
            GPS
        };

        class Point
        {
        private:
            double x_;
            double y_;

        public:
            inline explicit Point() {}
            inline explicit Point(double x, double y)
            {
                this->x_ = x;
                this->y_ = y;
            }

            inline virtual ~Point() {}

            inline double get__x() const
            {
                return this->x_;
            }

            inline void set__x(double x)
            {
                this->x_ = x;
            }

            inline double get__y() const
            {
                return this->y_;
            }

            inline void set__y(double y)
            {
                this->y_ = y;
            }
        };

        class PositionConverter
        {
        private:
            double mapping_max_width_;
            double mapping_max_height_;
            double slam_rotation_angle_;
            gps_slam_conversion::position::Point area_offset_;

        public:
            explicit PositionConverter();
            virtual ~PositionConverter();

            double gps_slam_conversion::position::PositionConverter::get__mapping_max_width() const;
            void gps_slam_conversion::position::PositionConverter::set__mapping_max_width(double mapping_max_width);
            double gps_slam_conversion::position::PositionConverter::get__mapping_max_height() const;
            void gps_slam_conversion::position::PositionConverter::set__mapping_max_height(double mapping_max_height);
            double gps_slam_conversion::position::PositionConverter::get__slam_rotation_angle() const;
            void gps_slam_conversion::position::PositionConverter::set__slam_rotation_angle(double slam_rotation_angle);
            gps_slam_conversion::position::Point gps_slam_conversion::position::PositionConverter::get__area_offset() const;
            void gps_slam_conversion::position::PositionConverter::set__area_offset(const gps_slam_conversion::position::Point &area_offset);

            void init_area(
                int slam_width,
                int slam_height,
                gps_slam_conversion::position::Point intersection_start_point,
                gps_slam_conversion::position::Point intersection_end_point);

            gps_slam_conversion::position::Point convert_gps_to_slam(
                double lon,
                double lat,
                gps_slam_conversion::position::Point lon_lat_LB,
                gps_slam_conversion::position::Point lon_lat_RT);

            gps_slam_conversion::position::Point convert_slam_to_gps(
                int x,
                int y,
                gps_slam_conversion::position::Point lon_lat_LB,
                gps_slam_conversion::position::Point lon_lat_RT);

            gps_slam_conversion::position::Point convert_slam_pos(int x, int y, WorkType type);
            double get_angle(double lon1, double lat1, double lon2, double lat2);
        };
    }
}

#endif