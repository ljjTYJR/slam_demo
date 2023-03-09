import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
        ld = LaunchDescription()

        rviz_config = os.path.join(
        get_package_share_directory('slam_demo'),
        'rviz',
        'slam.rviz'
        )

        config = os.path.join(
        get_package_share_directory('slam_demo'),
        'config',
        'params.yaml'
        )

        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[config],
        )

        ld.add_action(rviz_node)

        return ld