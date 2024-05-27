import math;
from rclpy.node import Node;
from gps_slam_converter.position.domain import PositionPoint;
from gps_slam_converter.position.domain import Map;
from gps_slam_converter.position.domain import WorkType;


PI: float = math.pi;
POSITION_CONVERTER: str = "PositionConverter";


class PositionConverter():

    def __init__(self, node: Node) -> None:
        self.__node: Node = node;
        self.map: Map = Map();
        self.area_offset: PositionPoint = PositionPoint();
        self.lon_lat_LB: PositionPoint = PositionPoint();
        self.lon_lat_RT: PositionPoint = PositionPoint();

    def initialize(self,
                   x1: int, y1: int,
                   x2: int, y2: int,
                   slam_width: int, slam_height: int,
                   map_point_1: PositionPoint, map_point_2: PositionPoint,
                   start_lon_lat: PositionPoint, end_lon_lat: PositionPoint
                   ) -> None:
        mapping_map_area_list: list = self.convert_slam_to_virtual_map_area(
            x1=x1,
            y1=y1,
            x2=x2,
            y2=y2,
            width=slam_width,
            height=slam_height,
            map_point_1=map_point_1,
            map_point_2=map_point_2,
            start_lon_lat=start_lon_lat,
            end_lon_lat=end_lon_lat
        );
        lon_lat_LB: PositionPoint = PositionPoint(mapping_map_area_list[0].x, mapping_map_area_list[0].y);
        lon_lat_RT: PositionPoint = PositionPoint(mapping_map_area_list[1].x, mapping_map_area_list[1].y);

        self.init_area(
            slam_width=slam_width, slam_height=slam_height,
            start_lon_lat=start_lon_lat, end_lon_lat=end_lon_lat,
            lon_lat_LB=lon_lat_LB, lon_lat_RT=lon_lat_RT);

    def init_area(self,
                  slam_width: int, slam_height: int,
                  start_lon_lat: PositionPoint, end_lon_lat: PositionPoint,
                  lon_lat_LB: PositionPoint, lon_lat_RT: PositionPoint
                  ) -> None:
        """
        1. lon2, lat2와 lon1, lat1로 SLAM 맵 회전 각도
        """
        self.map.slam_rotation_angle = self.__get_angle(lon1=start_lon_lat.x, lat1=start_lon_lat.y, lon2=end_lon_lat.x, lat2=end_lon_lat.y);
        slam_rotation_angle: float = self.map.slam_rotation_angle;

        #self.__node.get_logger().info(f"PC slam_rotation_angle : [{self.map.slam_rotation_angle}], [{slam_rotation_angle}]")

        if (slam_rotation_angle < 0):
            return;

        """
        2. SLAM 맵의 width, height
        """
        """
        3. GPS Mapping 맵 크기
        """
        self.map.mapping_map_width = abs(round(math.sin(slam_rotation_angle) * slam_height + math.cos(slam_rotation_angle) * slam_width));
        self.map.mapping_map_height = abs(round(math.cos(slam_rotation_angle) * slam_height + math.sin(slam_rotation_angle) * slam_width));

        """
        4. x offset
        """
        self.area_offset = PositionPoint(x=round(math.sin(slam_rotation_angle) * slam_height), y=0.0);

        """
        5. 현장 맵을 포함하는 지도의 사각영역 (경위도 직교)
        """
        self.lon_lat_LB = lon_lat_LB;
        self.lon_lat_RT = lon_lat_RT;

        #self.__node.get_logger().info(f"PC lon_lat_LB : [{self.lon_lat_LB}]")
        #self.__node.get_logger().info(f"PC lon_lat_RT : [{self.lon_lat_RT}]")

    def convert_gps_to_slam(self, longitude: float, latitude: float) -> PositionPoint:
        """
        1. GPS to Mapping Map
        """
        m_x: float = ((longitude - self.lon_lat_LB.x) / (self.lon_lat_RT.x - self.lon_lat_LB.x)) * self.map.mapping_map_width;

        """
        2. Mapping Map to SLAM Position
        """
        m_y: float = ((latitude - self.lon_lat_LB.y) / (self.lon_lat_RT.y - self.lon_lat_LB.y)) * self.map.mapping_map_height;

        position_point: PositionPoint = self.__convert_slam_pos(x=int(m_x), y=int(m_y), type=WorkType.GPS);

        return position_point;

    def convert_slam_to_gps(self, x: int, y: int) -> PositionPoint:
        self.__node.get_logger().info(f"{POSITION_CONVERTER} convert_slam_to_gps lon_lat_LB\n\tx : {self.lon_lat_LB.x}\n\ty : {self.lon_lat_LB.y}");
        self.__node.get_logger().info(f"{POSITION_CONVERTER} convert_slam_to_gps lon_lat_RT\n\tx : {self.lon_lat_RT.x}\n\ty : {self.lon_lat_RT.y}");
        self.__node.get_logger().info(f"{POSITION_CONVERTER} convert_slam_to_gps robot_pose\n\tx : {x}\n\ty : {y}");
        self.__node.get_logger().info(f"{POSITION_CONVERTER} convert_slam_to_gps map_info\n\twidth : {self.map.mapping_map_width}\n\theight : {self.map.mapping_map_height}");

        if self.map.mapping_map_width <= 0 or self.map.mapping_map_height <= 0:
            self.__node.get_logger().error(f"{POSITION_CONVERTER} map width and height is lower than zero... aborting");
            return PositionPoint(0, 0);

        """
        1. SLAM Position to Mapping Map
        """
        m_pos: PositionPoint = self.__convert_slam_pos(x=x, y=y, type=WorkType.SLAM);

        if (m_pos == None):
            return PositionPoint(x=0.0, y=0.0);

        """
        2. Mapping Map to GPS
        """
        longitude: float = self.lon_lat_LB.x + (self.lon_lat_RT.x - self.lon_lat_LB.x) * (m_pos.x / self.map.mapping_map_width);
        latitude: float = self.lon_lat_LB.y + (self.lon_lat_RT.y - self.lon_lat_LB.y) * (m_pos.y / self.map.mapping_map_height);

        position_point: PositionPoint = PositionPoint(x=longitude, y=latitude);

        return position_point

    def __convert_slam_pos(self, x: int, y: int, type: WorkType) -> PositionPoint:
        slam_rotation_angle: float = self.map.slam_rotation_angle;
        if (slam_rotation_angle <= 0):
            return None;

        pos: PositionPoint = PositionPoint();

        if (type == WorkType.SLAM):
            """
            1. x0, y0과 x, y 각도 a
            """
            a: float = math.atan2(y, x);

            """
            2. x0, y0과 x, y 거리 len
            """
            len: float = math.sqrt(x * x + y * y);

            """
            3. Mapping Map x
            """
            m_x: float = math.cos(slam_rotation_angle + a) * len + self.area_offset.x;

            """
            4. Mapping Map y
            """
            m_y: float = math.sin(slam_rotation_angle + a) * len;

            pos.x = m_x;
            pos.y = m_y;
        elif (type == WorkType.GPS):
            """
            1. x_offset, 0 -> x0, y0
            """
            x_std: float = x - self.area_offset.x;
            y_std: float = y;

            """
            2. x0,y0 과 x", y" 각도 b
            """
            b: float = math.atan2(y_std, x_std);

            """
            x0, y0과 x", y" 거리 len
            """
            len: float = round(math.sqrt(x_std * x_std + y_std * y_std));

            """
            3. SLAM x, y 좌표
            """
            s_x: float = math.cos(b - slam_rotation_angle) * len;
            s_y: float = math.sin(b - slam_rotation_angle) * len;

            pos.x = s_x;
            pos.y = s_y;
        else:
            return None;

        return pos;

    def convert_slam_to_virtual_map_area(self,
                                         x1: int, y1: int,
                                         x2: int, y2: int,
                                         width: int, height: int,
                                         map_point_1: PositionPoint, map_point_2: PositionPoint,
                                         start_lon_lat: PositionPoint, end_lon_lat: PositionPoint
                                        ) -> list:
        """
        1. 맵의 기울어진 각도 추출
        """
        slam_rotation_angle: float = self.__get_angle(lon1=start_lon_lat.x, lat1=start_lon_lat.y, lon2=end_lon_lat.x, lat2=end_lon_lat.y);

        # SLAM 기준 좌표의 각도 (비율 보정 안됨)
        slam_std_angle: float = self.__get_angle(lon1=map_point_1.x, lat1=map_point_1.y, lon2=map_point_2.x, lat2=map_point_2.y) - slam_rotation_angle;

        """
        2. 두 기준 좌표간 거리
        """
        pt_distance_for_gps: float = self.__get_distance_in_meter(lat1=map_point_1.y, lon1=map_point_1.x, lat2=map_point_2.y, lon2=map_point_2.x);
        x_distance_for_gps: float = pt_distance_for_gps * math.cos(slam_std_angle);
        y_distance_for_gps: float = pt_distance_for_gps * math.sin(slam_std_angle);

        pt_distance_for_slam: float = abs(x2 - x1);
        x_distance_for_slam: float = abs(x2 - x1);

        y_distance_for_slam: float = abs(y2 - y1);

        if (y_distance_for_gps == 0 or y_distance_for_slam == 0):
            y_distance_for_gps = x_distance_for_gps;
            y_distance_for_slam = x_distance_for_slam;

        """
        3. SLAM 픽셀 당 길이
        """
        x_dist_per_pix: float = x_distance_for_gps / x_distance_for_slam;
        #self.__node.get_logger().info(f"PC x_dist_per_pix : [{x_dist_per_pix}]")

        y_dist_per_pix: float = y_distance_for_gps / y_distance_for_slam;
        
        """
        4. SLAM 원점과 우상단의 거리와 각도
        """
        # SLAM 우상단과 우측 기준 좌표와의 각도
        rt_point_angle: float = math.atan2((height - y2) * y_dist_per_pix, (width - x2) * x_dist_per_pix);

        # SLAM 우상단과 우측 기준 좌표와의 거리
        rt_dist_slam: float = math.sqrt(math.pow((height - y2) * y_dist_per_pix, 2) + math.pow((width - x2) * x_dist_per_pix, 2));

        # SLAM 좌하단과 좌측 기준 좌표와의 각도
        lb_point_angle: float = math.atan2(y1 * y_dist_per_pix, x1 * x_dist_per_pix);

        # SLAM 좌하단과 좌측 기준 좌표와의 거리
        lb_dist_slam: float = math.sqrt(math.pow((y1) * y_dist_per_pix, 2) + math.pow((x1) * x_dist_per_pix, 2));

        # 높이
        height_distance: float = height * y_dist_per_pix;


        # SLAM 거리와 각도에 해당하는 현장 맵 우상단 좌표 추출 (현장 맵 원점 기준, 맵 각도 반역)(상단 교점)
        right_top_pos: PositionPoint = self.__get_moving_lon_lat(lon=map_point_2.x, lat=map_point_2.y, distance=rt_dist_slam, radian=(slam_rotation_angle + rt_point_angle));

        # SLAM 거리와 각도의 해당하는 현장맵 좌하단 구하기(하단 교점)
        left_bottom_pos: PositionPoint = self.__get_moving_lon_lat(lon=map_point_1.x, lat=map_point_1.y, distance=lb_dist_slam, radian=(slam_rotation_angle + lb_point_angle + PI));

        # SLAM height로 현장 맵 우하단 구하기(우측 교점)
        right_bottom_pos: PositionPoint = self.__get_moving_lon_lat(lon=right_top_pos.x, lat=right_top_pos.y, distance=height_distance, radian=(slam_rotation_angle + (PI + PI / 2)));

        # SLAM width로 현장 맵 좌상단 구하기(좌측 교점)
        left_top_pos: PositionPoint = self.__get_moving_lon_lat(lon=left_bottom_pos.x, lat=left_bottom_pos.y, distance=height_distance, radian=(slam_rotation_angle + PI / 2));

        # 좌상 lat, 좌하 lon
        left_position_point: PositionPoint = PositionPoint(x=left_top_pos.x, y=left_bottom_pos.y);

        # 우하 lat, 우상 lon
        right_position_point: PositionPoint = PositionPoint(x=right_bottom_pos.x, y=right_top_pos.y);

        position_point_list: list = [];
        position_point_list.append(left_position_point);
        position_point_list.append(right_position_point);

        return position_point_list;

    def __get_moving_lon_lat(self, lon: float, lat: float, distance: float, radian: float) -> PositionPoint:
        # 현재 위도의 1도당 거리 (m)
        dist_per_lat_degree: float = self.__distance_in_meter_by_haversine(lat1=int(lat), lon1=lon, lat2=(int(lat) + 1), lon2=lon);
        dist_per_lon_degree: float = self.__distance_in_meter_by_haversine(lat1=lat, lon1=int(lon), lat2=lat, lon2=(int(lon) + 1));

        # 위도 35의 경도 1도의 길이(m)
        if (dist_per_lat_degree <= 0):
            dist_per_lon_degree = 110941;

        if (dist_per_lon_degree <= 0):
            dist_per_lon_degree = 91290;

        # 4분면 값
        quadrant1: float = 90 * PI / 180;
        quadrant2: float = 180 * PI / 180;
        quadrant3: float = 270 * PI / 180;

        longitude_move: float = (math.sqrt(math.pow(distance, 2) - math.pow(math.sin(radian) * distance, 2))) / dist_per_lon_degree;
        latitude_move: float = (math.sqrt(math.pow(distance, 2) - math.pow(math.cos(radian) * distance, 2))) / dist_per_lat_degree;

        latitude: float = 0.0;
        longitude: float = 0.0;

        if (quadrant1 >= radian):
            longitude = lon + longitude_move;
            latitude = lat + latitude_move;
        elif (quadrant2 >= radian):
            longitude = lon - longitude_move;
            latitude = lat + latitude_move;
        elif (quadrant3 >= radian):
            longitude = lon - longitude_move;
            latitude = lat - latitude_move;
        else:
            longitude = lon + longitude_move;
            latitude = lat - latitude_move;

        position_point: PositionPoint = PositionPoint(x=longitude, y=latitude);

        return position_point;

    def __get_angle(self, lon1: float, lat1: float, lon2: float, lat2: float) -> float:
        y1: float = lat1 * PI / 180;
        #self.__node.get_logger().info(f"PC get_angle y1 : [{y1}]")

        y2: float = lat2 * PI / 180;
        #self.__node.get_logger().info(f"PC get_angle y2 : [{y2}]")

        x1: float = lon1 * PI / 180;
        #self.__node.get_logger().info(f"PC get_angle x1 : [{x1}]")

        x2: float = lon2 * PI / 180;

        y: float = math.sin(x2 - x1) * math.cos(y2);
        x: float = math.cos(y1) * math.sin(y2) - math.sin(y1) * math.cos(y2) * math.cos(x2 - x1);

        theta: float = math.atan2(y, x);
        angle: float = PI / 2 - theta;

        return angle;

    def __get_distance_in_meter(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        theta: float;
        dist: float;
        radius: float = 6371.0;

        theta = lon1 - lon2;
        dist = math.sin(self.__deg2rad(lat1)) * math.sin(self.__deg2rad(lat2)) + math.cos(self.__deg2rad(lat1)) * math.cos(self.__deg2rad(lat2)) * math.cos(self.__deg2rad(theta));

        dist = radius * math.acos(dist);

        return dist * 1000.0;

    def __deg2rad(self, deg: float) -> float:
        radian: float = float(deg * PI / 180.0);

        return radian;

    def __rad2deg(self, rad: float) -> float:
        degree: float = float(rad * 180.0 / PI);

        return degree;

    def __distance_in_meter_by_haversine(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        distance: float;
        radius: float = 6371.0;

        delta_latitude: float = abs(self.__deg2rad(lat1 - lat2));
        delta_longitude: float = abs(self.__deg2rad(lon1 - lon2));

        sin_delta_lat: float = math.sin(delta_latitude / 2);
        sin_delta_lon: float = math.sin(delta_longitude / 2);

        square_root: float = math.sqrt(sin_delta_lat * sin_delta_lat + math.cos(self.__deg2rad(lat1)) * math.cos(self.__deg2rad(lat2)) * sin_delta_lon * sin_delta_lon);

        distance = 2 * radius * math.asin(square_root);

        return distance * 1000.0;


__all__: list[str] = ["PositionConverter"];