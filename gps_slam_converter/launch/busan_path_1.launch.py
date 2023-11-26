import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description() -> LaunchDescription:
    ld: LaunchDescription = LaunchDescription()

    namespace_name: str = 'gps_slam_conversion'
    package_name: str = 'gps_slam_converter'
    executable_name: str = 'converter'
    package_shared_directory: str = get_package_share_directory(package_name)
    
    converter_node: Node = Node(
        namespace = namespace_name,
        package = package_name,
        executable = executable_name,
        name = executable_name,
        output = 'screen',
        parameters = [
            {
                'virtual_map_x': 1656,
                'virtual_map_y': 521,
                'virtual_map_width': 2377,
                'virtual_map_height': 1258,
                'virtual_map_distance_per_pixel': 0.05,
                'virtual_map_point_lon': 128.858512,
                'virtual_map_point_lat': 35.158016,
                'virtual_lt_point_lon': 128.858009083,
                'virtual_lt_point_lat': 35.157430158,
                'virtual_rt_point_lon': 128.858870603,
                'virtual_rt_point_lat': 35.158056682,
                'slam_map_width': 2377,
                'slam_map_height': 1258,
                'intersection_start_point_lon': 128.858009083,
                'intersection_start_point_lat': 35.157430158,
                'intersection_end_point_lon': 128.858870603,
                'intersection_end_point_lat': 35.158056682,
                'lb_point_lon': 128.857566,
                'lb_point_lat': 35.157345,
                'rt_point_lon': 128.858997,
                'rt_point_lat': 35.158480
            }
        ]
    )
    
    ld.add_action(converter_node)
    
    return ld