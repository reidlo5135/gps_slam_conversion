import math

from rclpy.node import Node

from .domain import Point
from .domain import Map
from .domain import WorkType

PI: float = math.pi

class Converter():
    
    def __init__(self, node: Node) -> None:
        self.__node: Node = node
        self.map: Map = Map()
        self.area_offset: Point = Point()
        self.lon_lat_LB: Point = Point()
        self.lon_lat_RT: Point = Point()
        pass

    
    def init_area(self, slam_width: int, slam_height: int, intersection_start_point: Point, intersection_end_point: Point, longitude_latitude_LB: Point, longitude_latitude_RT: Point) -> None:
        """
        1. lon2, lat2와 lon1, lat1로 SLAM 맵 회전 각도
        """
        self.map.slam_rotation_angle = self.__get_angle(lon1 = intersection_start_point.x, lat1 = intersection_start_point.y, lon2 = intersection_end_point.x, lat2 = intersection_end_point.y)
        slam_rotation_angle: float = self.map.slam_rotation_angle
        
        if (slam_rotation_angle < 0):
            return
        
        """
        2. SLAM 맵의 width, height
        """
        """
        3. GPS Mapping 맵 크기
        """
        self.map.mapping_map_width = round(math.sin(slam_rotation_angle) * slam_height + math.cos(slam_rotation_angle) * slam_width)
        self.map.mapping_map_height = round(math.cos(slam_rotation_angle) * slam_height + math.sin(slam_height) * slam_width)
        
        """
        4. x offset
        """
        self.area_offset = Point(x = round(math.sin(slam_height) * slam_height), y = 0.0)
        
        """
        5. 현장 맵을 포함하는 지도의 사각영역 (경위도 직교)
        """
        self.lon_lat_LB = longitude_latitude_LB
        self.lon_lat_RT = longitude_latitude_RT
    
    
    def convert_gps_to_slam(self, longitude: float, latitude: float) -> Point:
        """
        1. GPS to Mapping Map
        """
        m_x: float = ((longitude - self.lon_lat_LB.x) / (self.lon_lat_RT.x - self.lon_lat_LB.x)) * self.map.mapping_map_width
        
        """
        2. Mapping Map to SLAM Position
        """
        m_y: float = ((latitude - self.lon_lat_LB.y) / (self.lon_lat_RT.y - self.lon_lat_LB.y)) * self.map.mapping_map_height
        
        point: Point = self.__convert_slam_pos(x = int(m_x), y = int(m_y), type = WorkType.GPS)
        
        return point
    
    
    def convert_slam_to_gps(self, x: int, y: int) -> Point:
        """
        1. SLAM Position to Mapping Map
        """
        m_pos: Point = self.__convert_slam_pos(x = x, y = y, type = WorkType.SLAM)
        
        if (m_pos == None):
            return Point(x = 0.0, y = 0.0)

        """
        2. Mapping Map to GPS
        """
        longitude: float = self.lon_lat_LB.x + (self.lon_lat_RT.x - self.lon_lat_LB.x) * (m_pos.x / self.map.mapping_map_width)
        latitude: float = self.lon_lat_LB.y + (self.lon_lat_RT.y - self.lon_lat_LB.y) * (m_pos.y / self.map.mapping_map_height)
        
        point: Point = Point(x = longitude, y = latitude)
        
        return point
    
    
    def __convert_slam_pos(self, x: int, y: int, type: WorkType) -> Point:
        slam_rotation_angle: float = self.map.slam_rotation_angle
        if (slam_rotation_angle <= 0):
            return None
        
        pos: Point = Point()
        
        if (type == WorkType.SLAM):
            """
            1. x0, y0과 x, y 각도 a
            """
            a: float = math.atan2(y, x)
            
            """
            2. x0, y0과 x, y 거리 len
            """
            len: float = round(math.sqrt(x * x + y * y))
            
            """
            3. Mapping Map x
            """
            m_x: float = round(math.cos(slam_rotation_angle + a) * len + self.area_offset.x)
            
            """
            4. Mapping Map y
            """
            m_y: float = round(math.sin(slam_rotation_angle + a) * len)
            
            pos.x = m_x
            pos.y = m_y
        elif (type == WorkType.GPS):
            """
            1. x_offset, 0 -> x0, y0
            """
            x_std: float = x - self.area_offset.x
            y_std: float = y
            
            """
            2. x0,y0 과 x', y' 각도 b
            """
            b: float = math.atan2(y_std, x_std)
            
            """
            x0, y0과 x', y' 거리 len
            """
            len: float = round(math.sqrt(x_std * x_std + y_std * y_std))
            
            """
            3. SLAM x, y 좌표
            """
            s_x: float = round(math.cos(b - slam_rotation_angle * len))
            s_y: float = round(math.sin(b - slam_rotation_angle) * len)
            
            pos.x = s_x
            pos.y = s_y
        else:
            return None
        
        return pos

        
    def __get_angle(self, lon1: float, lat1: float, lon2: float, lat2: float) -> float:
        y1: float = lat1 * PI / 180
        y2: float = lat2 * PI / 180
        x1: float = lon1 * PI / 180
        x2: float = lon2 * PI / 180
        
        y: float = math.sin(x2 - x1) * math.cos(y2)
        x: float = math.cos(y1) * math.sin(y2) - math.sin(y1) * math.cos(y2) * math.cos(x2 - x1)
        
        theta: float = math.atan2(y, x)
        angle: float = PI / 2 - theta
        
        return angle
    
    
    def convert_slam_to_virtual_map_area(self, x: int, y: int, width: int, height: int, dist_per_pix: float, map_point: Point, lon_lat_LT: Point, lon_lat_RT: Point) -> list:
        """
        1. 맵의 기울어진 각도 추출
        """
        slam_rotation_angle: float = self.__get_angle(lon1 = lon_lat_LT.x, lat1 = lon_lat_LT.y, lon2 = lon_lat_RT.x, lat2 = lon_lat_RT.y)
        
        """
        2. SLAM 맵 원점과 우상단의 거리와 각도
        """
        # SLAM 맵 원점 기준 y 축 거리
        y_dist_slam: float = (height - y) * dist_per_pix
        
        # SLAM 맵 원점 기준 x 축 거리
        x_dist_slam: float = (width - x) * dist_per_pix
        
        # SLAM 맵 우상단과 기준 좌표와의 각도
        rt_point_angle: float = math.atan2(height - y, width - x)
        
        # SLAM 맵 우상단과 기준 좌표와의 거리
        dist_slam: float = math.sqrt(math.pow((height - y), 2) + math.pow((width - x), 2)) * dist_per_pix
        
        # SLAM 맵 우상단과 좌하단의 각도
        diagonal_angle: float = math.atan2(height, width)
        
        # SLAM 맵 우상단과 좌하단의 거리
        diagonal_distance: float = math.sqrt(math.pow((height), 2) + math.pow((width), 2)) * dist_per_pix
        
        height_distance: float = height * dist_per_pix
        width_distance: float = width * dist_per_pix
        
        """
        3. SLAM 맵 거리와 각도에 해당하는 현장 맵 우상단 좌표 추출 (현장 맵 원점 기준, 맵 각도 반역)(상단 교점)
        """
        right_top_pos: Point = self.__get_moving_lon_lat(lon = map_point.x, lat = map_point.y, distance = dist_slam, radian = (slam_rotation_angle + rt_point_angle))
        
        """
        4. SLAM 맵 width로 현장 맵 좌상단 구하기 (좌측 교점)
        """
        left_top_pos: Point = self.__get_moving_lon_lat(lon = right_top_pos.x, lat = right_top_pos.y, distance = width_distance, radian = (slam_rotation_angle + PI))
        
        """
        5. SLAM 맵 대각선 길이 rt_dist_slam 현장 맵 좌하단 구하기 (하단 교점)
        """
        left_bottom_pos: Point = self.__get_moving_lon_lat(lon = right_top_pos.x, lat = right_top_pos.y, distance = diagonal_distance, radian = (slam_rotation_angle + diagonal_angle + PI))
        
        """
        6. SLAM 맵 height로 현장맵 우하단 구하기 (우측 교점)
        """
        right_bottom_pos: Point = self.__get_moving_lon_lat(lon = right_top_pos.x, lat = right_top_pos.y, distance = height_distance, radian = (slam_rotation_angle + (PI + PI / 2)))
        
        # 좌상 lat, 좌하 lon
        left_point: Point = Point(x = left_top_pos.x, y = left_bottom_pos.y)
        
        # 우하 lat, 우상 lon
        right_point: Point = Point(x = right_bottom_pos.x, y = right_top_pos.y)
        
        point_list: list = []
        point_list.append(left_point)
        point_list.append(right_point)
        
        self.__node.get_logger().info(f'convert_slam_2_virtual_map_area point_list : {point_list}')
        
        return point_list
        
    
    
    def __get_moving_lon_lat(self, lon: float, lat: float, distance: float, radian: float) -> Point:
        dist_per_lon_degree: int = 91290
        dist_per_lat_degree: int = 110941
        
        quadrant_1: float = 90 * PI / 180
        quadrant_2: float = 180 * PI / 180
        quadrant_3: float = 270 * PI / 180
        
        longitude_move: float = (math.sqrt(math.pow(distance, 2) - math.pow(math.sin(radian) * distance, 2))) / dist_per_lon_degree
        latitude_move: float = (math.sqrt(math.pow(distance, 2) - math.pow(math.cos(radian) * distance, 2))) / dist_per_lat_degree
        
        latitude: float = 0.0
        longitude: float = 0.0
        
        if (quadrant_1 >= radian):
            longitude = lon + longitude_move
            latitude = lat + latitude_move
        elif (quadrant_2 >= radian):
            longitude = lon - longitude_move
            latitude = lat + latitude_move
        elif (quadrant_3 >= radian):
            longitude = lon - longitude_move
            latitude = lat - latitude_move
        else:
            longitude = lon + longitude_move
            latitude = lat - latitude_move
        
        point: Point = Point()
        point.x = longitude
        point.y = latitude
    
        return point