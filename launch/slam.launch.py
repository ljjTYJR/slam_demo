import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

package_name = 'slam_demo'
name_space = 'slam'

def generate_launch_description():
    ld = LaunchDescription()

    slam_demo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                'launch',
                'slam_node.launch.py'
            )
        )
    )

    sensor_offset_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                'launch',
                'sensor_offset_node.launch.py'
            )
        )
    )

    rviz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                'launch',
                'rviz_node.launch.py'
            )
        )
    )

    rosbag_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                'launch',
                'rosbag_node.launch.py'
            )
        )
    )

    # assign the same namespace
    slam_with_namespace = GroupAction(
     actions=[
         PushRosNamespace(name_space),
         sensor_offset_node,
         slam_demo_node,
         rviz_node,
         rosbag_node,
      ]
   )

    ld.add_action(slam_with_namespace)

    return ld