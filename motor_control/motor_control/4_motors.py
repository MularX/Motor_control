import RPi.GPIO as GPIO
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QSlider, QPushButton, QLabel,
    QVBoxLayout, QWidget, QGroupBox
)
from PyQt5.QtCore import Qt, QTimer
from threading import Thread

# ========== GPIO Setup ==========

# Motor pin mappings (PWM, DIR, FG)
MOTOR_PINS = {
    1: {'PWM': 18, 'DIR': 17, 'FG': 22},
    2: {'PWM': 23, 'DIR': 24, 'FG': 25},
    3: {'PWM': 12, 'DIR': 16, 'FG': 20},
    4: {'PWM': 21, 'DIR': 5, 'FG': 6},  # NEW motor pins
}

GPIO.setmode(GPIO.BCM)

pwm_instances = {}

for motor_id, pins in MOTOR_PINS.items():
    GPIO.setup(pins['PWM'], GPIO.OUT)
    GPIO.setup(pins['DIR'], GPIO.OUT)
    GPIO.setup(pins['FG'], GPIO.IN)
    pwm = GPIO.PWM(pins['PWM'], 10000)  # 10 kHz PWM
    pwm.start(0)
    pwm_instances[motor_id] = pwm

# ========== ROS2 Node ==========

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('multi_motor_controller')
        
        self.pwms = pwm_instances
        self.rpm_publishers = {}

        for i in range(1, 5):  # Support 4 motors
            topic = f'motor_{i}_rpm'
            self.rpm_publishers[i] = self.create_publisher(Float32, topic, 10)
            self.create_subscription(Float32, topic, lambda msg, motor=i: self.rpm_callback(msg, motor), 10)

    def rpm_callback(self, msg, motor_id):
        rpm = msg.data
        self.get_logger().info(f'Motor {motor_id} RPM: {rpm}')

    def set_motor_speed(self, motor_id, speed):
        if motor_id in self.pwms:
            self.pwms[motor_id].ChangeDutyCycle(speed)
            self.get_logger().info(f'Motor {motor_id} speed set to: {speed}%')

    def stop_motor(self, motor_id):
        if motor_id in self.pwms:
            self.pwms[motor_id].ChangeDutyCycle(0)
            self.get_logger().info(f'Motor {motor_id} stopped')

# ========== UI ==========

class MotorControlGroup(QGroupBox):
    def __init__(self, motor_id, motor_node: MotorControllerNode):
        super().__init__(f'Motor {motor_id} Control')
        self.motor_id = motor_id
        self.motor_node = motor_node

        layout = QVBoxLayout()

        self.rpm_label = QLabel('RPM: 0')
        layout.addWidget(self.rpm_label)

        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(0, 100)
        self.speed_slider.setValue(0)
        self.speed_slider.valueChanged.connect(self.update_speed)
        layout.addWidget(self.speed_slider)

        self.stop_button = QPushButton('Stop Motor')
        self.stop_button.clicked.connect(self.stop_motor)
        layout.addWidget(self.stop_button)

        self.setLayout(layout)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_rpm)
        self.timer.start(100)

    def update_speed(self):
        speed = self.speed_slider.value()
        self.motor_node.set_motor_speed(self.motor_id, speed)
        self.rpm_label.setText(f'RPM: {speed * 50}')

    def stop_motor(self):
        self.motor_node.stop_motor(self.motor_id)
        self.speed_slider.setValue(0)
        self.rpm_label.setText('RPM: 0')

    def update_rpm(self):
        speed = self.speed_slider.value()
        rpm = speed * 50
        self.rpm_label.setText(f'RPM: {rpm}')

class MotorControlUI(QMainWindow):
    def __init__(self, motor_node: MotorControllerNode):
        super().__init__()

        self.motor_node = motor_node
        self.setWindowTitle('Multi Motor Control')

        main_layout = QVBoxLayout()

        # Create control groups for each motor
        for motor_id in range(1, 5):  # Support 4 motors
            control_group = MotorControlGroup(motor_id, self.motor_node)
            main_layout.addWidget(control_group)

        central_widget = QWidget()
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

# ========== Main Entry Point ==========

def main(args=None):
    rclpy.init(args=args)
    motor_node = MotorControllerNode()

    def ros_spin():
        rclpy.spin(motor_node)

    ros_thread = Thread(target=ros_spin)
    ros_thread.start()

    app = QApplication([])
    ui = MotorControlUI(motor_node)
    ui.show()
    app.exec_()

    ros_thread.join()

    motor_node.destroy_node()
    rclpy.shutdown()

    # Cleanup GPIO
    for pwm in pwm_instances.values():
        pwm.stop()
    GPIO.cleanup()

if __name__ == '__main__':
    main()

