from setuptools import setup
import os
from glob import glob

package_name = 'motor_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	(os.path.join('share',package_name, 'launch'), ['launch/robot_imu_ekf_launch.py']),
    ],
    install_requires=['setuptools', 'rclpy', 'PyQt5'],
    zip_safe=True,
    maintainer='giganiga',
    maintainer_email='your-email@example.com',
    description='Multi-motor control with GUI using ROS 2 and GPIO',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller = motor_control.three_motors_controller:main',
            'motor_controller1 = motor_control.4_motors:main',
            'motor_controller_rev = motor_control.motor_reversed:main',
            'motor_controller_arrows = motor_control.motors_arrows:main',
            'imu_publisher = motor_control.imu_publisher:main',
            'ros_control = motor_control.ros_control:main',
        ],
    },
)
