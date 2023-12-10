import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    ld: LaunchDescription = LaunchDescription()

    namespace_name: str = 'gps_slam_conversion'
    package_name: str = 'gps_slam_converter'
    executable_name: str = 'converter_executor'
    package_shared_directory: str = get_package_share_directory(package_name)

    converter_node: Node = Node(
        package=package_name,
        executable=package_name,
        name=package_name,
        output='screen',
        parameters=[
            {
                'std_point_slam_x1': 72,
                'std_point_slam_y1': 140,
                'std_point_slam_x2': 392,
                'std_point_slam_y2': 262,
                'slam_map_width': 579,
                'slam_map_height': 338,
                'slam_map_shift': 0,
                'slam_map_resolution': 0.2,
                'std_point_lon1': 128.8579836,
                'std_point_lat1': 35.1576298,
                'std_point_lon2': 128.858333,
                'std_point_lat2': 35.15818,
                'start_point_lon': 128.858009083,
                'start_point_lat': 35.157430158,
                'end_point_lon': 128.858870603,
                'end_point_lat': 35.158056682
            }
        ]
    )

    ld.add_action(converter_node)

    return ld
