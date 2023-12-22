import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
       
        Node(
            package='autorace_core_precall', 
            executable='controller',
            name='controller'),
        
        Node(
            package='check_traffic_light', 
            executable='traffic_light_checker',
            name='tl_checker'),

        Node(
            package='line_following', 
            executable='pid_test',
            name='pid'),

        Node(
            package='sign_detector', 
            executable='sign_detection',
            name='sign_detection'),
            
        Node(
            package='crosswalk', 
            executable='crosswalk',
            name='crosswalk'),
        Node(
            package='avoid_working', 
            executable='avoid_working',
            name='avoidence'),
        Node(
            package='turn', 
            executable='turn',
            name='turn'),
        # Node(
        #     package='parking', 
        #     executable='parking',
        #     name='parking'),
        
    ])
