from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # IMU Publisher Node (MMA7660FC)
        Node(
            package='motor_control',
            executable='imu_publisher',
            name='imu_publisher',
            output='screen',
            parameters=[
                {'frame_id': 'imu_link'}
            ]
        ),
        # IMU Filter Madgwick Node
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            parameters=[
                {'use_mag': False},  # No magnetometer
                {'publish_tf': False},
                {'world_frame': 'enu'},
                {'gain': 0.1},
                {'zeta': 0.0}
            ],
            remappings=[
                ('/imu/data_raw', '/imu/data_raw'),
                ('/imu/data', '/imu/data_filtered')
            ]
        ),
        # Robot Localization EKF Node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                {'map_frame': 'map'},
                {'odom_frame': 'odom'},
                {'base_link_frame': 'base_link'},
                {'world_frame': 'odom'},
                {'two_d_mode': True},
                {'publish_tf': True},
                {'publish_acceleration': False},
                {'odom0': '/odom'},
                {'odom0_config': [True, True, False, False, False, True, True, True, False, False, False, True, False, False, False]},
                {'odom0_covariance': [0.1, 0.1, 0.0, 0.0, 0.0, 0.1, 0.1, 0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0]},
                {'odom0_differential': False},
                {'odom0_queue_size': 10},
                {'imu0': '/imu/data_filtered'},
                {'imu0_config': [False, False, False, False, False, True, False, False, False, False, False, False, True, True, True]},
                {'imu0_differential': False},
                {'imu0_queue_size': 10},
                {'imu0_remove_gravitational_acceleration': True},
                {'transform_time_offset': 0.1},
                {'transform_timeout': 0.0}
            ]
        )
    ])
