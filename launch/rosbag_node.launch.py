import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
        ld = LaunchDescription()

        bag_file_path = os.path.join(
        get_package_share_directory('slam_demo'),
        'bag',
        'robot_lab'
        )

        rosbag_play = ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_file_path, '--topics', '/robot/front_laser/scan', '/robot/throttle/odom', '-r', '3', '--clock'],
            output='screen'
        )

        ld.add_action(rosbag_play)

        return ld