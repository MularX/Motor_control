#!/usr/bin/env python3
#zbiera dane z 4 czujników ultradźwiękowych i publikuje je jako skan
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, LaserScan
import math
import numpy as np

class UltrasonicFusionNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_fusion_node')
        
        # Parametry czujników (dopasowane do Twojej konfiguracji)
        self.frames = ["FL_czujnik", "FR_czujnik", "RR_czujnik", "RL_czujnik"]
        self.topics = ['ultrasonic/FL/range', 'ultrasonic/FR/range', 
                      'ultrasonic/RR/range', 'ultrasonic/RL/range']
        
        self.field_of_view = 0.261799  # ~15 deg
        self.min_range = 0.03
        self.max_range = 3.5
        
        # Pozycje i orientacje czujników względem base_link
        # FL=Front-Left, FR=Front-Right, RR=Rear-Right, RL=Rear-Left
        self.sensor_poses = {
            'FL_czujnik': {'angle': math.pi/4,    'x': 0.15, 'y': 0.15},   # 45° (front-left)
            'FR_czujnik': {'angle': -math.pi/4,   'x': 0.15, 'y': -0.15},  # -45° (front-right) 
            'RR_czujnik': {'angle': -3*math.pi/4, 'x': -0.15, 'y': -0.15}, # -135° (rear-right)
            'RL_czujnik': {'angle': 3*math.pi/4,  'x': -0.15, 'y': 0.15}   # 135° (rear-left)
        }
        
        # Subscribers dla Range messages
        self.range_subscribers = []
        for topic in self.topics:
            sub = self.create_subscription(Range, topic, self.range_callback, 10)
            self.range_subscribers.append(sub)
        
        # Publisher dla fake laser scan
        self.laser_scan_pub = self.create_publisher(LaserScan, 'ultrasonic_scan', 10)
        
        # Przechowuj ostatnie odczyty
        self.last_ranges = {}
        for frame in self.frames:
            self.last_ranges[frame] = float('inf')
        
        # Timer do publikowania LaserScan (10Hz)
        self.timer = self.create_timer(0.1, self.publish_laser_scan)
        
        self.get_logger().info('Ultrasonic fusion node initialized')
        self.get_logger().info(f'Subscribing to topics: {self.topics}')
        self.get_logger().info(f'Publishing to: ultrasonic_scan')
    
    def range_callback(self, msg):
        """Callback dla Range messages"""
        # Określ który czujnik na podstawie frame_id
        if msg.header.frame_id in self.last_ranges:
            # Filtruj nieprawidłowe odczyty
            if msg.range < self.min_range or msg.range > self.max_range:
                self.last_ranges[msg.header.frame_id] = float('inf')
            else:
                self.last_ranges[msg.header.frame_id] = msg.range
    
    def publish_laser_scan(self):
        """Publikuj LaserScan message z połączonych czujników"""
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_link'
        
        # Parametry laser scan
        # Pokrywamy pełny okrąg (360°) z rozdzielczością co 45°
        scan.angle_min = 0.0
        scan.angle_max = 2 * math.pi - math.pi/4  # 315°
        scan.angle_increment = math.pi / 4        # 45° increment (8 points total)
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = self.min_range
        scan.range_max = self.max_range
        
        # Inicjalizuj tablicę ranges (8 punktów dla pełnego okręgu)
        num_readings = 8
        ranges = [float('inf')] * num_readings
        
        # Mapuj czujniki na odpowiednie indeksy w tablicy ranges
        # Index 0 = 0°, Index 1 = 45°, Index 2 = 90°, etc.
        sensor_indices = {
            'FR_czujnik': 7,  # -45° -> index 7 (315°)
            'FL_czujnik': 1,  # 45° -> index 1
            'RL_czujnik': 3,  # 135° -> index 3  
            'RR_czujnik': 5   # -135° (225°) -> index 5
        }
        
        # Przypisz wartości z czujników
        for frame, range_value in self.last_ranges.items():
            if frame in sensor_indices:
                idx = sensor_indices[frame]
                ranges[idx] = range_value
        
        # Interpoluj wartości między czujnikami (opcjonalnie)
        # Dla indices bez czujników (0, 2, 4, 6) możesz:
        # 1. Zostawić inf (brak danych)
        # 2. Interpolować z sąsiednich czujników
        # 3. Użyć max_range
        
        # Opcja 1: Zostawiamy inf dla directions bez czujników
        scan.ranges = ranges
        scan.intensities = []
        
        self.laser_scan_pub.publish(scan)
        
        # Debug info (usuń w produkcji)
        self.get_logger().debug(f'Published LaserScan with ranges: {[f"{r:.2f}" if r != float("inf") else "inf" for r in ranges]}')
    
    def interpolate_missing_directions(self, ranges):
        """Opcjonalna interpolacja dla directions bez czujników"""
        # Dla directions 0°, 90°, 180°, 270° (indices 0, 2, 4, 6)
        # możesz interpolować z sąsiednich czujników
        
        interpolated_ranges = ranges.copy()
        
        # Przykład interpolacji dla direction 0° (przód)
        # Średnia z FL (45°) i FR (315°/−45°)
        if ranges[1] != float('inf') and ranges[7] != float('inf'):
            interpolated_ranges[0] = (ranges[1] + ranges[7]) / 2.0
        
        # Podobnie dla innych directions...
        
        return interpolated_ranges

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicFusionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()