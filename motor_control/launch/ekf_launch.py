from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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
                {'odom0_differential': False},
                {'odom0_queue_size': 10},
                {'imu0': '/imu/data_filtered'},
                {'imu0_config': [False, False, False, False, False, True, False, False, False, False, False, False, True, True, True]},
                {'imu0_differential': False},
                {'imu0_queue_size': 10},
                {'imu0_remove_gravitational_acceleration': True},
                {'transform_time_offset': 0.0},
                {'transform_timeout': 0.0}
            ]
        )
    ])
