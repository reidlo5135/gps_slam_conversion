from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.service import Service
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import qos_profile_services_default
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from geometry_msgs.msg import Pose
from sensor_msgs.msg import NavSatFix
from gps_slam_conversion_msgs.srv import Conversion

from ..position.converter import Converter
from ..position.domain import Point


NODE_NAME: str = 'gps_slam_converter'

DEFAULT_INT: int = 0
DEFAULT_FLOAT: float = 0.0

PARAM_MAP_WIDTH: str = 'map_width'
PARAM_MAP_HEIGHT: str = 'map_height'
PARAM_MAP_DISTANCE_PER_PIXEL: str = 'map_distance_per_pixel'

PARAM_MAPPING_REFERENCE_POINT_PIXEL_X: str = 'mapping_map_reference_point_pixel_x'
PARAM_MAPPING_REFERENCE_POINT_PIXEL_Y: str = 'mapping_map_reference_point_pixel_y'

PARAM_MAPPING_MAP_POINT_LON: str = 'mapping_map_point_lon'
PARAM_MAPPING_MAP_POINT_LAT: str = 'mapping_map_point_lat'
PARAM_MAPPING_MAP_LT_POINT_LON: str = 'mapping_map_lt_point_lon'
PARAM_MAPPING_MAP_LT_POINT_LAT: str = 'mapping_map_lt_point_lat'
PARAM_MAPPING_MAP_RT_POINT_LON: str = 'mapping_map_rt_point_lon'
PARAM_MAPPING_MAP_RT_POINT_LAT: str = 'mapping_map_rt_point_lat'

PARAM_INTERSECTION_START_POINT_LON: str = 'intersection_start_point_lon'
PARAM_INTERSECTION_START_POINT_LAT: str = 'intersection_start_point_lat'
PARAM_INTERSECTION_END_POINT_LON: str = 'intersection_end_point_lon'
PARAM_INTERSECTION_END_POINT_LAT: str = 'intersection_end_point_lat'

PARAM_LB_POINT_LON: str = 'lb_point_lon'
PARAM_LB_POINT_LAT: str = 'lb_point_lat'
PARAM_RT_POINT_LON: str = 'rt_point_lon'
PARAM_RT_POINT_LAT: str = 'rt_point_lat'


class ConverterNode(Node):
    
    def __init__(self) -> None:
        super().__init__(NODE_NAME)
        self.get_logger().info(f'{NODE_NAME} has been executed...')
        
        self.__converter: Converter = Converter(node = self)
        # self.__position_test()
        
        self.__declare_parameters()
        self.get_logger().info('====================================================================================')
        
        self.__robot_pose_subscription_topic: str = '/robot_pose'
        self.__robot_pose_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__robot_pose_subscription: Subscription = self.create_subscription(
            msg_type = Pose,
            topic = self.__robot_pose_subscription_topic,
            qos_profile = qos_profile_sensor_data,
            callback_group = self.__robot_pose_subscription_cb_group,
            callback = self.__robot_pose_subscription_cb
        )
        
        self.__ublox_fix_subscription_topic: str = '/ublox/fix'
        self.__ublox_fix_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__ublox_fix_subscription: Subscription = self.create_subscription(
            msg_type = NavSatFix,
            topic = self.__ublox_fix_subscription_topic,
            qos_profile = qos_profile_sensor_data,
            callback_group = self.__ublox_fix_subscription_cb_group,
            callback = self.__ublox_fix_subscription_cb
        )
        
        self.__gps_to_slam_publisher_topic: str = '/gps_to_slam'
        self.__gps_to_slam_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__gps_to_slam_publisher: Publisher = self.create_publisher(
            msg_type = Pose,
            topic = self.__gps_to_slam_publisher_topic,
            qos_profile = qos_profile_sensor_data,
            callback_group = self.__gps_to_slam_publisher_cb_group
        )
        
        self.__slam_to_gps_publisher_topic: str = '/slam_to_gps'
        self.__slam_to_gps_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__slam_to_gps_publisher: Publisher = self.create_publisher(
            msg_type = NavSatFix,
            topic = self.__slam_to_gps_publisher_topic,
            qos_profile = qos_profile_sensor_data,
            callback_group = self.__slam_to_gps_publisher_cb_group
        )
        
        self.__gps_slam_conversion_service_name: str = self.get_name() + '/conversion'
        self.__gps_slam_conversion_service_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__gps_slam_conversion_service: Service = self.create_service(
            srv_type = Conversion,
            srv_name = self.__gps_slam_conversion_service_name,
            qos_profile = qos_profile_services_default,
            callback_group = self.__gps_slam_conversion_service_cb_group,
            callback = self.__gps_slam_conversion_service_cb
        )
        
        self.__initialize_mapping_map()

    
    def __declare_parameters(self) -> None:
        parameters_dict: dict = {
            PARAM_MAP_WIDTH: DEFAULT_INT,
            PARAM_MAP_HEIGHT: DEFAULT_INT,
            PARAM_MAP_DISTANCE_PER_PIXEL: DEFAULT_FLOAT,
            PARAM_MAPPING_REFERENCE_POINT_PIXEL_X: DEFAULT_INT,
            PARAM_MAPPING_REFERENCE_POINT_PIXEL_Y: DEFAULT_INT,
            PARAM_MAPPING_MAP_POINT_LON: DEFAULT_FLOAT,
            PARAM_MAPPING_MAP_POINT_LAT: DEFAULT_FLOAT,
            PARAM_MAPPING_MAP_LT_POINT_LON: DEFAULT_FLOAT,
            PARAM_MAPPING_MAP_LT_POINT_LAT: DEFAULT_FLOAT,
            PARAM_MAPPING_MAP_RT_POINT_LON: DEFAULT_FLOAT,
            PARAM_MAPPING_MAP_RT_POINT_LAT: DEFAULT_FLOAT,
            PARAM_INTERSECTION_START_POINT_LON: DEFAULT_FLOAT,
            PARAM_INTERSECTION_START_POINT_LAT: DEFAULT_FLOAT,
            PARAM_INTERSECTION_END_POINT_LON: DEFAULT_FLOAT,
            PARAM_INTERSECTION_END_POINT_LAT: DEFAULT_FLOAT,
            PARAM_LB_POINT_LON: DEFAULT_FLOAT,
            PARAM_LB_POINT_LAT: DEFAULT_FLOAT,
            PARAM_RT_POINT_LON: DEFAULT_FLOAT,
            PARAM_RT_POINT_LAT: DEFAULT_FLOAT
        }
        
        for key, value in parameters_dict.items():
            self.get_logger().info(f'Declaring key : [{key}], value : [{value}]')
            self.declare_parameter(name = key, value = value)
        
    
    def __robot_pose_subscription_cb(self, robot_pose_cb: Pose) -> None:
        pass

        
    def __ublox_fix_subscription_cb(self, ublox_fix_cb: NavSatFix) -> None:
        pass
    
    
    def __gps_slam_conversion_service_cb(self, request: Conversion.Request, response: Conversion.Response) -> Conversion.Response:
        pass
    
    
    def __initialize_mapping_map(self) -> None:
        map_width: int = int(self.get_parameter(PARAM_MAP_WIDTH).value)
        map_height: int = int(self.get_parameter(PARAM_MAP_HEIGHT).value)
        map_distance_per_pixel: float = float(self.get_parameter(PARAM_MAP_DISTANCE_PER_PIXEL).value)
        self.get_logger().info(f'Initialize mapping map Info\n\tmap_width : [{map_width}]\n\tmap_height: [{map_height}]\n\tmap_distance_per_pixel : [{map_distance_per_pixel}]')
        
        mapping_reference_point_pixel_x: int = int(self.get_parameter(PARAM_MAPPING_REFERENCE_POINT_PIXEL_X).value)
        mapping_reference_point_pixel_y: int = int(self.get_parameter(PARAM_MAPPING_REFERENCE_POINT_PIXEL_Y).value)
        self.get_logger().info(f'Initialize mapping map reference_points\n\tmapping_reference_point_pixel_x : [{mapping_reference_point_pixel_x}]\n\tmapping_reference_point_pixel_y : [{mapping_reference_point_pixel_y}]')
        
        mapping_map_point_lon: float = float(self.get_parameter(PARAM_MAPPING_MAP_POINT_LON).value)
        mapping_map_point_lat: float = float(self.get_parameter(PARAM_MAPPING_MAP_POINT_LAT).value)
    
        self.get_logger().info(f'Initialize mapping map map_points\n\tmapping_reference_point_pixel_x : [{mapping_map_point_lon}]\n\tmapping_reference_point_pixel_y : [{mapping_map_point_lat}]')
        
    
    
    def __position_test(self) -> None:
        map_point: Point = Point(x = 128.858427988, y = 35.158463143)
        lon_lat_LT: Point = Point(x = 128.858009083, y = 35.157430158)
        lon_lat_RT: Point = Point(x = 128.858870603, y = 35.158056682)
        
        arr: list = self.__converter.convert_slam_to_virtual_map_area(x = 520, y = 300, width = 1048, height = 602, dist_per_pix = 0.05, map_point = map_point, lon_lat_LT = lon_lat_LT, lon_lat_RT = lon_lat_RT)
        self.get_logger().info(f'position_test arr : {arr}')
        
        intersection_start_point: Point = Point(x = 128.858009083, y = 35.157430158)
        intersection_end_point: Point = Point(x = 128.858870603, y = 35.158056682)
        longitude_latitude_LB: Point = Point(x = arr[0].x, y = arr[0].y)
        longitude_latitude_RT: Point = Point(x = arr[1].x, y = arr[1].y)
        self.__converter.init_area(slam_width = 2080, slam_height = 1200, intersection_start_point = intersection_start_point, intersection_end_point = intersection_end_point, longitude_latitude_LB = longitude_latitude_LB, longitude_latitude_RT = longitude_latitude_RT)
        
        gps_pos_1: Point = self.__converter.convert_slam_to_gps(x = 0, y = 0)
        self.get_logger().info(f'gps_pos_1 : {gps_pos_1}')
        
        slam_pos_1: Point = self.__converter.convert_gps_to_slam(longitude = gps_pos_1.x, latitude = gps_pos_1.y)
        self.get_logger().info(f'slam_pos_1 : {slam_pos_1}')
        
        differ_x_1: float = abs(0 - slam_pos_1.x)
        differ_y_1: float = abs(0 - slam_pos_1.y)
        self.get_logger().info(f'differ_x_1 : {differ_x_1}, differ_y_1 : {differ_y_1}')
        
        gps_pos_2: Point = self.__converter.convert_slam_to_gps(x = 2080, y = 0)
        self.get_logger().info(f'gps_pos_2 : {gps_pos_2}')
        
        slam_pos_2: Point = self.__converter.convert_gps_to_slam(longitude = gps_pos_2.x, latitude = gps_pos_2.y)
        self.get_logger().info(f'slam_pos_2 : {slam_pos_2}')
        
        differ_x_2: float = abs(2080 - slam_pos_2.x)
        differ_y_2: float = abs(0 - slam_pos_2.y)
        self.get_logger().info(f'differ_x_2 : {differ_x_2}, differ_y_2 : {differ_y_2}')
        
        gps_pos_3: Point = self.__converter.convert_slam_to_gps(x = 2080, y = 1200)
        self.get_logger().info(f'gps_pos_3 : {gps_pos_3}')
        
        slam_pos_3: Point = self.__converter.convert_gps_to_slam(longitude = gps_pos_3.x, latitude = gps_pos_3.y)
        self.get_logger().info(f'slam_pos_3 : {slam_pos_3}')
        
        differ_x_3: float = abs(2080 - slam_pos_3.x)
        differ_y_3: float = abs(1200 - slam_pos_3.y)
        self.get_logger().info(f'differ_x_3 : {differ_x_3}, differ_y_3 : {differ_y_3}')
        
        gps_pos_4: Point = self.__converter.convert_slam_to_gps(x = 0, y = 1200)
        self.get_logger().info(f'gps_pos_4 : {gps_pos_4}')
        
        slam_pos_4: Point = self.__converter.convert_gps_to_slam(longitude = gps_pos_4.x, latitude = gps_pos_4.y)
        self.get_logger().info(f'slam_pos_4 : {slam_pos_4}')
        
        differ_x_4: float = abs(0 - slam_pos_4.x)
        differ_y_4: float = abs(1200 - slam_pos_4.y)
        self.get_logger().info(f'differ_x_4 : {differ_x_4}, differ_y_4 : {differ_y_4}')
        
        gps_pos_5: Point = self.__converter.convert_slam_to_gps(x = 1040, y = 600)
        self.get_logger().info(f'gps_pos_5 : {gps_pos_5}')
        
        slam_pos_5: Point = self.__converter.convert_gps_to_slam(longitude = gps_pos_5.x, latitude = gps_pos_5.y)
        self.get_logger().info(f'slam_pos_5 : {slam_pos_5}')
        
        differ_x_5: float = abs(1040 - slam_pos_5.x)
        differ_y_5: float = abs(600 - slam_pos_5.y)
        self.get_logger().info(f'differ_x_5 : {differ_x_5}, differ_y_5 : {differ_y_5}')
        
        
__all__ = ['node_converter_node']