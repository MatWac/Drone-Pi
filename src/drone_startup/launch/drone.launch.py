import os
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    # Noeud de publication des données IMU
    icm_publisher = Node(
        package="icm_publisher",
        executable="icm_publisher",
        parameters=[{
            'frame_id': 'base_link',
            'pub_rate': 50,
            'pitch_filter_gain': 0.8
        }]
    )

    pid_pubsub = Node(
        package='pid_pubsub',
        executable='pid_pubsub',
        name='pid_pubsub',
        output='screen'
    )

    # Noeud de l'Extended Kalman Filter (EKF)
    motor_subscriber = Node(
        package='motor_subscriber',
        executable='motorsub',
        name='motor_subscriber',
        output='screen'  # Utiliser le fichier de configuration YAML pour EKF
    )
    
    return LaunchDescription([
        icm_publisher,
        pid_pubsub,
        motor_subscriber
    ])
