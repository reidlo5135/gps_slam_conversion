import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    ld: LaunchDescription = LaunchDescription()

    package_name: str = 'gps_slam_converter'
    package_shared_directory: str = get_package_share_directory(package_name)
    
    parameter: str = os.path.join(package_shared_directory, 'config', 'wk_valve_1_params.yaml')
    converter_node: Node = Node(
        package=package_name,
        executable=package_name,
        name=package_name,
        output='screen',
        parameters=[parameter]
    )

    ld.add_action(converter_node)

    return ld
