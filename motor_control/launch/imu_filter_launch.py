from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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
        )
    ])
