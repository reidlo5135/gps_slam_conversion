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

    map_coordinates_parameter: str = os.path.join(
        package_shared_directory,
        'config',
        'map_coordinates.yaml'
    )
    
    converter_node: Node = Node(
        namespace = namespace_name,
        package = package_name,
        executable = executable_name,
        name = executable_name,
        output = 'screen',
        parameters = [
            map_coordinates_parameter
        ]
    )
    
    ld.add_action(converter_node)
    
    return ld