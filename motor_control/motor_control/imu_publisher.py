#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import smbus2

# MMA7660FC settings
IMU_ADDR = 0x4c
XOUT = 0x00
YOUT = 0x01
ZOUT = 0x02
MODE = 0x07
SR = 0x08
ACCEL_SCALE = 9.81 / 21.0  # ±1.5g, 21 counts/g to m/s^2

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.timer = self.create_timer(0.03125, self.publish_imu)  # 32 Hz
        self.bus = smbus2.SMBus(1)  # I2C bus 1
        # Initialize MMA7660FC
        try:
            self.bus.write_byte_data(IMU_ADDR, MODE, 0x00)  # Standby mode
            self.bus.write_byte_data(IMU_ADDR, SR, 0x03)    # 32 samples/s
            self.bus.write_byte_data(IMU_ADDR, MODE, 0x01)  # Active mode
            self.get_logger().info('MMA7660FC initialized')
        except Exception as e:
            self.get_logger().error(f'IMU initialization failed: {e}')

    def read_accel(self, reg):
        try:
            val = self.bus.read_byte_data(IMU_ADDR, reg)
            # Check alert bit (bit 6)
            if val & 0x40:
                self.get_logger().warn(f'Alert bit set on register {reg}')
                return 0
            # Convert 6-bit signed value (-32 to +31)
            if val & 0x20:
                return -((0x3F - (val & 0x3F)) + 1)
            return val & 0x3F
        except Exception as e:
            self.get_logger().error(f'I2C read error at register {reg}: {e}')
            return 0

    def publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Read accelerometer (m/s^2)
        accel_x = self.read_accel(XOUT) * ACCEL_SCALE
        accel_y = self.read_accel(YOUT) * ACCEL_SCALE
        accel_z = self.read_accel(ZOUT) * ACCEL_SCALE
        msg.linear_acceleration = Vector3(x=accel_x, y=accel_y, z=accel_z)

        # No gyroscope
        msg.angular_velocity = Vector3(x=0.0, y=0.0, z=0.0)

        # Covariances (based on datasheet noise: ~0.05g RMS)
        msg.linear_acceleration_covariance = [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025]  # (0.05g * 9.81)^2
        msg.angular_velocity_covariance = [-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0]  # No gyro
        msg.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0]  # No orientation

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    imu_publisher = ImuPublisher()
    rclpy.spin(imu_publisher)
    imu_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
