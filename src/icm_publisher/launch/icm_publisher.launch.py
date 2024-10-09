import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    
    # Récupérer le chemin du package
    pkg_path = os.path.join(get_package_share_directory('icm_publisher'))

    # Définir le chemin du fichier de configuration EKF
    ekf_config_path = os.path.join(pkg_path, 'config', 'ekf.yaml')

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

    madgwick_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='madgwick_filter_node',
        output='screen'
    )

    # Noeud de l'Extended Kalman Filter (EKF)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path]  # Utiliser le fichier de configuration YAML pour EKF
    )
    
    return LaunchDescription([
        icm_publisher
    ])
