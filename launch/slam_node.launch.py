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

        slam_demo_node = Node(
            package='slam_demo',
            executable='slam_node',
            name='slam_demo_node',
            output='screen',
            parameters=[config],
        )

        ld.add_action(slam_demo_node)

        return ld