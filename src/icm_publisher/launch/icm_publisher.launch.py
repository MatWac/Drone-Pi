import os

from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    
    icm_publisher = Node(
        package="icm_publisher",
        executable="icm_publisher",
        parameters=[
            {'frame_id':'base_footprint'}
        ]
    )
    
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter',
        output='screen',
        parameters=['/home/ws/src/icm_publisher/config/ekf.yaml']
    )
    
    return LaunchDescription([
        ekf,
        icm_publisher
            
    ])