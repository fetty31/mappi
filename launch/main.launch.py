import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    pkg_share = os.path.join(get_package_share_directory('mappi'), 'launch')

    # Nav2
    nav2 = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(pkg_share, 'nav2.launch.py'))
                )
    

    # Pipeline
    pipeline = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(pkg_share, 'dummy_pipeline.launch.py'))
                )
    

    return LaunchDescription([
        nav2,
        pipeline,
    ])