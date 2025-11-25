import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

# Launchfile to spawn Ona on Gazebo Fortress

def generate_launch_description():

    pc_to_scan = Node(
                package='pointcloud_to_laserscan',
                executable='pointcloud_to_laserscan_node',
                name='pointcloud_to_laserscan',
                parameters=[
                    {'min_height': 0.1,
                    'max_height': 15.0,
                    'angle_increment': 0.01,
                    'range_min': 0.1,
                    'target_frame': 'ona2/base_footprint'}
                ],
                remappings=[('cloud_in', 'ona2/sensors/pandar_front/cloud_raw'),
                            ('scan', 'ona2/scan')]
            )
    
    return LaunchDescription([
        pc_to_scan
    ])