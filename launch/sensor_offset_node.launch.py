import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
        ld = LaunchDescription()

        config = os.path.join(
        get_package_share_directory('slam_demo'),
        'config',
        'params.yaml'
        )

        sensor_offset_node = Node(
            package='slam_demo',
            executable='sensor_offset_node',
            name='sensor_offset_node',
            output='screen',
            parameters=[config],
        )

        ld.add_action(sensor_offset_node)

        return ld