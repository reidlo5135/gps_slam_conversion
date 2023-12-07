import numpy as np

from datetime import datetime
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.service import Service
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import qos_profile_services_default
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from geometry_msgs.msg import Pose
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from gps_slam_conversion_msgs.srv import Conversion
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

from ..position.converter import PositionConverter
from ..position.domain import PositionPoint

from typing import Sequence


NODE_NAME: str = 'gps_slam_converter'

DEFAULT_INT: int = 0
DEFAULT_FLOAT: float = 0.0
CONVERTER_NODE: str = 'ConverterNode'

PARAM_STD_POINT_SLAM_X1: str = 'std_point_slam_x1'
PARAM_STD_POINT_SLAM_Y1: str = 'std_point_slam_y1'
PARAM_STD_POINT_SLAM_X2: str = 'std_point_slam_x2'
PARAM_STD_POINT_SLAM_Y2: str = 'std_point_slam_y2'

PARAM_SLAM_MAP_WIDTH: str = 'slam_map_width'
PARAM_SLAM_MAP_HEIGHT: str = 'slam_map_height'
PARAM_SLAM_MAP_SHIFT: str = 'slam_map_shift'
PARAM_SLAM_MAP_RESOULTION: str = 'slam_map_resolution'

PARAM_STD_POINT_LON1: str = 'std_point_lon1'
PARAM_STD_POINT_LAT1: str = 'std_point_lat1'
PARAM_STD_POINT_LON2: str = 'std_point_lon2'
PARAM_STD_POINT_LAT2: str = 'std_point_lat2'

PARAM_START_POINT_LON: str = 'start_point_lon'
PARAM_START_POINT_LAT: str = 'start_point_lat'

PARAM_END_POINT_LON: str = 'end_point_lon'
PARAM_END_POINT_LAT: str = 'end_point_lat'

CONVERSION_TARGET_GPS: str = 'GPS'
CONVERSION_TARGET_SLAM: str = 'SLAM'

SLAM_MAP_DISTANCE: float = 1.0


class ConverterNode(Node):

    def __init__(self) -> None:
        super().__init__(NODE_NAME)
        self.get_logger().info(f'{NODE_NAME} has been executed...')

        self.__position_converter: PositionConverter = PositionConverter(
            node=self)
        self.__position_test()

        self.__declare_parameters()
        self.get_logger().info(
            '====================================================================================')

        self.__slam_map_resoultion_ratio: float = 0.0
        self.__initialize_mapping_map()

        self.__robot_pose_subscription_topic: str = '/robot_pose'
        self.__robot_pose_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__robot_pose_subscription: Subscription = self.create_subscription(
            msg_type=Pose,
            topic=self.__robot_pose_subscription_topic,
            qos_profile=qos_profile_sensor_data,
            callback_group=self.__robot_pose_subscription_cb_group,
            callback=self.__robot_pose_subscription_cb
        )

        self.__ublox_fix_subscription_topic: str = '/ublox/fix'
        self.__ublox_fix_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__ublox_fix_subscription: Subscription = self.create_subscription(
            msg_type=NavSatFix,
            topic=self.__ublox_fix_subscription_topic,
            qos_profile=qos_profile_sensor_data,
            callback_group=self.__ublox_fix_subscription_cb_group,
            callback=self.__ublox_fix_subscription_cb
        )

        self.__gps_to_slam_publisher_topic: str = '/gps_to_slam'
        self.__gps_to_slam_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__gps_to_slam_publisher: Publisher = self.create_publisher(
            msg_type=Pose,
            topic=self.__gps_to_slam_publisher_topic,
            qos_profile=qos_profile_sensor_data,
            callback_group=self.__gps_to_slam_publisher_cb_group
        )

        self.__slam_to_gps_publisher_topic: str = '/slam_to_gps'
        self.__slam_to_gps_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__slam_to_gps_publisher: Publisher = self.create_publisher(
            msg_type=NavSatFix,
            topic=self.__slam_to_gps_publisher_topic,
            qos_profile=qos_profile_sensor_data,
            callback_group=self.__slam_to_gps_publisher_cb_group
        )

        self.__gps_slam_conversion_service_name: str = self.get_name() + '/conversion'
        self.__gps_slam_conversion_service_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__gps_slam_conversion_service: Service = self.create_service(
            srv_type=Conversion,
            srv_name=self.__gps_slam_conversion_service_name,
            qos_profile=qos_profile_services_default,
            callback_group=self.__gps_slam_conversion_service_cb_group,
            callback=self.__gps_slam_conversion_service_cb
        )

    def __declare_parameters(self) -> None:
        parameters_dict: dict = {
            PARAM_STD_POINT_SLAM_X1: DEFAULT_INT,
            PARAM_STD_POINT_SLAM_Y1: DEFAULT_INT,
            PARAM_STD_POINT_SLAM_X2: DEFAULT_INT,
            PARAM_STD_POINT_SLAM_Y2: DEFAULT_INT,
            PARAM_SLAM_MAP_WIDTH: DEFAULT_INT,
            PARAM_SLAM_MAP_HEIGHT: DEFAULT_INT,
            PARAM_SLAM_MAP_SHIFT: DEFAULT_FLOAT,
            PARAM_SLAM_MAP_RESOULTION: DEFAULT_FLOAT,
            PARAM_STD_POINT_LON1: DEFAULT_FLOAT,
            PARAM_STD_POINT_LAT1: DEFAULT_FLOAT,
            PARAM_STD_POINT_LON2: DEFAULT_FLOAT,
            PARAM_STD_POINT_LAT2: DEFAULT_FLOAT,
            PARAM_START_POINT_LON: DEFAULT_FLOAT,
            PARAM_START_POINT_LAT: DEFAULT_FLOAT,
            PARAM_END_POINT_LON: DEFAULT_FLOAT,
            PARAM_END_POINT_LAT: DEFAULT_FLOAT
        }

        for key, value in parameters_dict.items():
            self.get_logger().info(
                f'{CONVERTER_NODE} Declaring key : [{key}], value : [{value}]')
            self.declare_parameter(name=key, value=value)

    def __initialize_mapping_map(self) -> None:
        map_shift: int = int(self.get_parameter(PARAM_SLAM_MAP_SHIFT).value)

        std_point_slam_x1: int = int(
            self.get_parameter(PARAM_STD_POINT_SLAM_X1).value)
        std_point_slam_y1: int = int(
            self.get_parameter(PARAM_STD_POINT_SLAM_Y1).value)
        self.get_logger().info(
            f'{CONVERTER_NODE} std_points_slam\n\tx1 : [{std_point_slam_x1}]\n\ty1 : [{std_point_slam_y1}]')

        std_point_slam_x2: int = int(
            self.get_parameter(PARAM_STD_POINT_SLAM_X2).value)
        std_point_slam_y2: int = int(
            self.get_parameter(PARAM_STD_POINT_SLAM_Y2).value)
        self.get_logger().info(
            f'{CONVERTER_NODE} std_points_slam\n\tx2 : [{std_point_slam_x2}]\n\ty2 : [{std_point_slam_y2}]')

        slam_map_width: int = int(
            self.get_parameter(PARAM_SLAM_MAP_WIDTH).value)
        slam_map_height: int = int(
            self.get_parameter(PARAM_SLAM_MAP_HEIGHT).value)
        slam_map_resoultion: float = float(
            self.get_parameter(PARAM_SLAM_MAP_RESOULTION).value)
        self.get_logger().info(
            f'{CONVERTER_NODE} slam_map info\n\twidth : [{slam_map_width}]\n\theight : [{slam_map_height}]\n\tresoultion : [{slam_map_resoultion}]')
        
        self.__slam_map_resoultion_ratio = SLAM_MAP_DISTANCE / slam_map_resoultion
        self.get_logger().info(
            f'{CONVERTER_NODE} slam_map info slam_map_resoultion_ratio : [{self.__slam_map_resoultion_ratio}]')

        std_point_lon1: float = float(
            self.get_parameter(PARAM_STD_POINT_LON1).value)
        std_point_lat1: float = float(
            self.get_parameter(PARAM_STD_POINT_LAT1).value)
        self.get_logger().info(
            f'{CONVERTER_NODE} std_point\n\tlon1 : [{std_point_lon1}]\n\tlat1 : [{std_point_lat1}]')

        std_point_lon2: float = float(
            self.get_parameter(PARAM_STD_POINT_LON2).value)
        std_point_lat2: float = float(
            self.get_parameter(PARAM_STD_POINT_LAT2).value)
        self.get_logger().info(
            f'{CONVERTER_NODE} std_point\n\tlon2 : [{std_point_lon2}]\n\tlat2 : [{std_point_lat2}]')

        start_point_lon: float = float(
            self.get_parameter(PARAM_START_POINT_LON).value)
        start_point_lat: float = float(
            self.get_parameter(PARAM_START_POINT_LAT).value)
        self.get_logger().info(
            f'{CONVERTER_NODE} start_point\n\tlon2 : [{start_point_lon}]\n\tlat2 : [{start_point_lat}]')

        end_point_lon: float = float(
            self.get_parameter(PARAM_END_POINT_LON).value)
        end_point_lat: float = float(
            self.get_parameter(PARAM_END_POINT_LAT).value)
        self.get_logger().info(
            f'{CONVERTER_NODE} end_point\n\tlon2 : [{end_point_lon}]\n\tlat2 : [{end_point_lat}]')

        self.__position_converter.initialize(
            x1=(std_point_slam_x1 - map_shift), y1=std_point_slam_y1,
            x2=(std_point_slam_x2 - map_shift), y2=std_point_slam_y2,
            slam_width=(slam_map_width - map_shift), slam_height=slam_map_height,
            map_point_1=PositionPoint(std_point_lon1, std_point_lat1),
            map_point_2=PositionPoint(std_point_lon2, std_point_lat2),
            start_lon_lat=PositionPoint(
                start_point_lon, start_point_lat),
            end_lon_lat=PositionPoint(
                end_point_lon, end_point_lat)
        )

    def __robot_pose_subscription_cb(self, robot_pose_cb: Pose) -> None:
        pose_x: float = robot_pose_cb.position.x
        pose_y: float = robot_pose_cb.position.y

        position_point: PositionPoint = self.__position_converter.convert_slam_to_gps(
            x=int(pose_x), y=int(pose_y)
        )

        if position_point != None:
            nav_sat_fix: NavSatFix = self.__build_nav_sat_fix(position_point=position_point)
            self.__slam_to_gps_publisher.publish(nav_sat_fix)
        else:
            self.get_logger().error(
                f'{CONVERTER_NODE} robot_pose_cb position_point is None... aborting')
            return

    def __ublox_fix_subscription_cb(self, ublox_fix_cb: NavSatFix) -> None:
        lon: float = ublox_fix_cb.longitude
        lat: float = ublox_fix_cb.latitude

        converted_position_point: PositionPoint = self.__position_converter.convert_gps_to_slam(
            longitude=lon, latitude=lat
        )

        if converted_position_point != None:
            position_point: PositionPoint = PositionPoint(x=round(converted_position_point.x * 100) / 100.0, y=round(converted_position_point.y * 100) / 100.0)

            pose: Pose = self.__build_pose(position_point=position_point)
            self.__gps_to_slam_publisher.publish(pose)
        else:
            self.get_logger().error(
                f'{CONVERTER_NODE} ublox_fix_cb position_point is None... aborting')
            return

    def __gps_slam_conversion_service_cb(self, request: Conversion.Request, response: Conversion.Response) -> Conversion.Response:
        conversion_target_data: str = request.conversion_target.data
        self.get_logger().info(f'{CONVERTER_NODE} gps_slam_conversion_service_cb converison_target_data : {conversion_target_data}')
        
        gps_request_list: Sequence = request.gps_request_list
        gps_request_list_len: int = len(gps_request_list)
        self.get_logger().info(f'{CONVERTER_NODE} gps_slam_conversion_service_cb gps_request_list : {gps_request_list}, gps_request_list_len : {gps_request_list_len}')
        
        slam_pose_request_list: Sequence = request.slam_pose_request_list
        slam_pose_request_list_len: int = len(slam_pose_request_list)
        self.get_logger().info(f'{CONVERTER_NODE} gps_slam_conversion_service_cb slam_pose_request_list : {slam_pose_request_list}, slam_pose_request_list_len : {slam_pose_request_list_len}')
        
        if (conversion_target_data == ''):
            self.get_logger().error(f'{CONVERTER_NODE} conversion_target is empty...')
            return None
        elif (conversion_target_data == CONVERSION_TARGET_SLAM):
            is_gps_request_list_empty: bool = not gps_request_list
            is_slam_pose_request_list_empty: bool = not slam_pose_request_list
            
            if (is_gps_request_list_empty):
                self.get_logger().error(f'{CONVERTER_NODE} gps_request_list is empty...')
                return None
            
            if (not is_slam_pose_request_list_empty):
                self.get_logger().error(f'{CONVERTER_NODE} slam_pose_request_list is not empty...')
                slam_pose_request_list = []
                slam_pose_request_list_len = 0
            
            slam_pose_response_list: list = []
            
            for gps_request in gps_request_list:
                lon: float = gps_request.longitude
                lat: float = gps_request.latitude
                self.get_logger().info(f'{CONVERTER_NODE} gps_request_list\n\tlon : [{lon}]\n\tlat : [{lat}]]')
                
                slam_pose_position_point: PositionPoint = self.__position_converter.convert_gps_to_slam(
                    longitude=lon, latitude=lat
                )
                
                pose: Pose = self.__build_pose(position_point=slam_pose_position_point)
                slam_pose_response_list.append(pose)
            
            slam_pose_response_list_len: int = len(slam_pose_response_list)
            
            is_slam_pose_converting_finished: bool = (gps_request_list_len == slam_pose_response_list_len)
            
            if is_slam_pose_converting_finished:
                for slam_pose_response in slam_pose_response_list:
                    slam_pose_x: float = (slam_pose_response.position.x / self.__slam_map_resoultion_ratio)
                    slam_pose_y: float = (slam_pose_response.position.y / self.__slam_map_resoultion_ratio)
                    
                    self.get_logger().info(f'{CONVERTER_NODE} slam_pose_response_list\n\tx : [{slam_pose_x}]\n\ty : [{slam_pose_y}]]')
                
                response.slam_pose_response_list = slam_pose_response_list
                
                return response
        elif (conversion_target_data == CONVERSION_TARGET_GPS):
            is_slam_pose_request_list_empty: bool = not slam_pose_request_list
            is_gps_request_list_empty: bool = not gps_request_list
            
            if (is_slam_pose_request_list_empty):
                self.get_logger().error(f'{CONVERTER_NODE} slam_pose_request_list is empty...')
                return None
                
            if (not is_gps_request_list_empty):
                self.get_logger().error(f'{CONVERTER_NODE} gps_request_list is not empty...')
                gps_request_list = []
                gps_request_list_len = 0
            
            gps_response_list: list = []
            
            for slam_pose_request in slam_pose_request_list:
                pose_x: float = slam_pose_request.position.x
                pose_y: float = slam_pose_request.position.y
                self.get_logger().info(f'{CONVERTER_NODE} slam_pose_request_list\n\tx : [{pose_x}]\n\ty : [{pose_y}]]')
                
                gps_position_point: PositionPoint = self.__position_converter.convert_slam_to_gps(
                    x=pose_x, y=pose_y
                )
                
                nav_sat_fix: NavSatFix = self.__build_nav_sat_fix(position_point=gps_position_point)
                gps_response_list.append(nav_sat_fix)
            
            gps_response_list_len: int = len(gps_response_list)
            
            is_gps_converting_finished: bool = (slam_pose_request_list_len == gps_response_list_len)
            
            if is_gps_converting_finished:
                for gps_response in gps_response_list:
                    lon: float = gps_response.longitude
                    lat: float = gps_response.latitude

                    self.get_logger().info(f'{CONVERTER_NODE} gps_response_list\n\tx : [{lon}]\n\ty : [{lat}]]')
                
                response.gps_response_list = gps_response_list
                
                return response
        else:
            self.get_logger().error(f'{CONVERTER_NODE} unknown conversion target...aborting')
            return None
                

    def __build_header(self, frame_id: str) -> Header:
        header: Header = Header()

        now: datetime = datetime.now()
        stamp: Time = self.get_clock().now().to_msg()
        stamp.sec = int(now.timestamp())
        stamp.nanosec = int(now.microsecond * 1000)

        header.frame_id = frame_id
        header.stamp = stamp

        return header

    def __build_nav_sat_fix(self, position_point: PositionPoint) -> NavSatFix:
        lon: float = position_point.x
        lat: float = position_point.y

        header_frame_id: str = 'slam_to_gps'

        nav_sat_fix: NavSatFix = NavSatFix()
        nav_sat_fix.header = self.__build_header(header_frame_id)

        nav_sat_status: NavSatStatus = NavSatStatus()
        nav_sat_status.status = DEFAULT_INT
        nav_sat_status.service = DEFAULT_INT

        nav_sat_fix.status = nav_sat_status
        nav_sat_fix.longitude = lon
        nav_sat_fix.latitude = lat
        nav_sat_fix.altitude = DEFAULT_FLOAT
        nav_sat_fix.position_covariance = np.zeros(9)
        nav_sat_fix.position_covariance_type = DEFAULT_INT

        return nav_sat_fix

    def __build_pose(self, position_point: PositionPoint) -> Pose:
        slam_x: float = position_point.x
        slam_y: float = position_point.y

        position_point: Point = Point()
        position_point.x = (slam_x / self.__slam_map_resoultion_ratio)
        position_point.y = (slam_y / self.__slam_map_resoultion_ratio)
        position_point.z = DEFAULT_FLOAT

        quaternion: Quaternion = Quaternion()
        quaternion.x = DEFAULT_FLOAT
        quaternion.y = DEFAULT_FLOAT
        quaternion.z = DEFAULT_FLOAT
        quaternion.w = DEFAULT_FLOAT

        pose: Pose = Pose()
        pose.position = position_point
        pose.orientation = quaternion

        return pose

    def __position_test(self) -> None:
        test_pos_1: PositionPoint = PositionPoint(0, 0)
        test_pos_2: PositionPoint = PositionPoint(1238, 765)
        test_pos_3: PositionPoint = PositionPoint(415, 235)
        test_pos_4: PositionPoint = PositionPoint(574, 235)
        test_pos_5: PositionPoint = PositionPoint(1210, 235)
        test_pos_6: PositionPoint = PositionPoint(1210, 541)

        slam_map_width: int = 1238
        slam_map_height: int = 765

        intersection_start_point_lon: float = 128.858009083
        intersection_start_point_lat: float = 35.157430158
        interseciton_end_point_lon: float = 128.858870603
        intersection_end_point_lat: float = 35.1580056682

        std_point_slam_x1: int = 415
        std_point_slam_y1: int = 235
        std_point_slam_x2: int = 1210
        std_point_slam_y2: int = 541

        std_point_lon1: float = 128.8579836
        std_point_lat1: float = 35.1576298
        std_point_lon2: float = 128.858333
        std_point_lat2: float = 35.15818

        shift: int = 0

        self.__position_converter.initialize(
            x1=(std_point_slam_x1 - shift), y1=std_point_slam_y1,
            x2=(std_point_slam_x2 - shift), y2=std_point_slam_y2,
            slam_width=(slam_map_width - shift), slam_height=slam_map_height,
            map_point_1=PositionPoint(std_point_lon1, std_point_lat1),
            map_point_2=PositionPoint(std_point_lon2, std_point_lat2),
            start_lon_lat=PositionPoint(
                intersection_start_point_lon, intersection_start_point_lat),
            end_lon_lat=PositionPoint(
                interseciton_end_point_lon, intersection_end_point_lat)
        )

        self.get_logger().info(
            f'===== {CONVERTER_NODE} position_test [1] =====')
        gps_pos_test_1: PositionPoint = self.__position_converter.convert_slam_to_gps(
            test_pos_1.x, test_pos_1.y)
        slam_pos_test_1: PositionPoint = self.__position_converter.convert_gps_to_slam(
            gps_pos_test_1.x, gps_pos_test_1.y
        )
        self.get_logger().info(
            f'{CONVERTER_NODE} GPS\n\tlon : [{gps_pos_test_1.x}]\n\tlat : [{gps_pos_test_1.y}]')
        self.get_logger().info(
            f'{CONVERTER_NODE} SLAM\n\tx : [{round(slam_pos_test_1.x * 100) / 100.0}]\n\ty : [{round(slam_pos_test_1.y * 100) / 100.0}]')

        self.get_logger().info(
            f'===== {CONVERTER_NODE} position_test [2] =====')
        gps_pos_test_2: PositionPoint = self.__position_converter.convert_slam_to_gps(
            test_pos_2.x, test_pos_2.y)
        slam_pos_test_2: PositionPoint = self.__position_converter.convert_gps_to_slam(
            gps_pos_test_2.x, gps_pos_test_2.y
        )
        self.get_logger().info(
            f'{CONVERTER_NODE} GPS\n\tlon : [{gps_pos_test_2.x}]\n\tlat : [{gps_pos_test_2.y}]')
        self.get_logger().info(
            f'{CONVERTER_NODE} SLAM\n\tx : [{round(slam_pos_test_2.x * 100) / 100.0}]\n\ty : [{round(slam_pos_test_2.y * 100) / 100.0}]')

        self.get_logger().info(
            f'===== {CONVERTER_NODE} position_test [3] =====')
        gps_pos_test_3: PositionPoint = self.__position_converter.convert_slam_to_gps(
            test_pos_3.x, test_pos_3.y)
        slam_pos_test_3: PositionPoint = self.__position_converter.convert_gps_to_slam(
            gps_pos_test_3.x, gps_pos_test_3.y
        )
        
        gps_pos_node_link_3: PositionPoint = PositionPoint(128.857984, 35.15763) 
        gps_pos_differ_3_x: float = abs(gps_pos_node_link_3.x - gps_pos_test_3.x)
        gps_pos_differ_3_y: float = abs(gps_pos_node_link_3.y - gps_pos_test_3.y)
        self.get_logger().info(
            f'{CONVERTER_NODE} GPS\n\tlon : [{gps_pos_test_3.x}]\n\tlat : [{gps_pos_test_3.y}]')
        self.get_logger().info(
            f'{CONVERTER_NODE} GPS Differ\n\tlon : [{gps_pos_differ_3_x}]\n\tlat : [{gps_pos_differ_3_y}]')
        
        slam_pos_node_link_3: PositionPoint = PositionPoint(415, 235)
        slam_pos_differ_3_x: float = abs(slam_pos_node_link_3.x - round(slam_pos_test_3.x * 100) / 100.0)
        slam_pos_differ_3_y: float = abs(slam_pos_node_link_3.y - round(slam_pos_test_3.y * 100) / 100.0)
        self.get_logger().info(
            f'{CONVERTER_NODE} SLAM\n\tx : [{round(slam_pos_test_3.x * 100) / 100.0}]\n\ty : [{round(slam_pos_test_3.y * 100) / 100.0}]')
        self.get_logger().info(
            f'{CONVERTER_NODE} SLAM Differ\n\tx : [{slam_pos_differ_3_x}]\n\ty : [{slam_pos_differ_3_y}]')

        self.get_logger().info(
            f'===== {CONVERTER_NODE} position_test [4] =====')
        gps_pos_test_4: PositionPoint = self.__position_converter.convert_slam_to_gps(
            test_pos_4.x, test_pos_4.y)
        slam_pos_test_4: PositionPoint = self.__position_converter.convert_gps_to_slam(
            gps_pos_test_4.x, gps_pos_test_4.y
        )
        self.get_logger().info(
            f'{CONVERTER_NODE} GPS\n\tlon : [{gps_pos_test_4.x}]\n\tlat : [{gps_pos_test_4.y}]')
        self.get_logger().info(
            f'{CONVERTER_NODE} SLAM\n\tx : [{round(slam_pos_test_4.x * 100) / 100.0}]\n\ty : [{round(slam_pos_test_4.y * 100) / 100.0}]')

        self.get_logger().info(
            f'===== {CONVERTER_NODE} position_test [5] =====')
        gps_pos_test_5: PositionPoint = self.__position_converter.convert_slam_to_gps(
            test_pos_5.x, test_pos_5.y)
        slam_pos_test_5: PositionPoint = self.__position_converter.convert_gps_to_slam(
            gps_pos_test_5.x, gps_pos_test_5.y
        )
        self.get_logger().info(
            f'{CONVERTER_NODE} GPS\n\tlon : [{gps_pos_test_5.x}]\n\tlat : [{gps_pos_test_5.y}]')
        self.get_logger().info(
            f'{CONVERTER_NODE} SLAM\n\tx : [{round(slam_pos_test_5.x * 100) / 100.0}]\n\ty : [{round(slam_pos_test_5.y * 100) / 100.0}]')

        self.get_logger().info(
            f'===== {CONVERTER_NODE} position_test [6] =====')
        gps_pos_test_6: PositionPoint = self.__position_converter.convert_slam_to_gps(
            test_pos_6.x, test_pos_6.y)
        slam_pos_test_6: PositionPoint = self.__position_converter.convert_gps_to_slam(
            gps_pos_test_6.x, gps_pos_test_6.y
        )
        self.get_logger().info(
            f'{CONVERTER_NODE} GPS\n\tlon : [{gps_pos_test_6.x}]\n\tlat : [{gps_pos_test_6.y}]')
        self.get_logger().info(
            f'{CONVERTER_NODE} SLAM\n\tx : [{round(slam_pos_test_6.x * 100) / 100.0}]\n\ty : [{round(slam_pos_test_6.y * 100) / 100.0}]')


__all__ = ['node_converter_node']
