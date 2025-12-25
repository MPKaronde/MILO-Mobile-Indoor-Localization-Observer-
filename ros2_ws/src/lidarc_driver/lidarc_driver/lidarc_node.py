import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

import serial
import math
import time


class LidarcPublisher(Node):

    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        super().__init__('lidarc_publisher')

        # --- ROS Publisher ---
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)

        # --- Serial port ---
        self.serial_port = serial.Serial(port, baudrate)
        self.serial_port.flushInput()
        time.sleep(2)  # allow Arduino to reset

        # --- Scan configuration ---
        self.min_angle_deg = -90.0
        self.max_angle_deg = 90.0
        self.angle_increment_deg = 180.0 / 1024.0  # assuming 1024 steps total
        self.num_readings = int((self.max_angle_deg - self.min_angle_deg) / self.angle_increment_deg) + 1

        # Buffer to store current scan
        self.ranges = [float('inf')] * self.num_readings

        # Timer to check serial periodically
        self.timer = self.create_timer(0.005, self.read_serial)

    # --- Convert ticks to degrees ---
    def ticks_to_degrees(self, ticks):
        if not -512 <= ticks <= 512:
            raise ValueError("Angle ticks out of bounds")
        return (-ticks / 512.0) * 90.0  # invert sign to match Arduino

    # --- Read serial line ---
    def read_serial(self):
        if self.serial_port.in_waiting == 0:
            return

        try:
            line = self.serial_port.readline().decode('utf-8').strip()
            angle_ticks, distance_mm = map(float, line.split(','))
            angle_deg = self.ticks_to_degrees(angle_ticks)
            self.store_reading(angle_deg, distance_mm)
        except Exception as e:
            self.get_logger().warn(f"Bad serial data: {e}")

    # --- Store data and detect sweep end ---
    def store_reading(self, angle_deg, distance_mm):
        index = int((angle_deg - self.min_angle_deg) / self.angle_increment_deg)
        if 0 <= index < self.num_readings:
            self.ranges[index] = distance_mm / 1000.0  # mm → meters

        # Detect end of sweep
        if abs(angle_deg - self.max_angle_deg) < (self.angle_increment_deg / 2):
            self.publish_scan()
            self.reset_scan()

    # --- Publish LaserScan ---
    def publish_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_frame'

        msg.angle_min = math.radians(self.min_angle_deg)
        msg.angle_max = math.radians(self.max_angle_deg)
        msg.angle_increment = math.radians(self.angle_increment_deg)

        msg.range_min = 0.05
        msg.range_max = 5.0
        msg.ranges = self.ranges

        self.scan_pub.publish(msg)
        self.get_logger().info("Published a scan")

    def reset_scan(self):
        self.ranges = [float('inf')] * self.num_readings


def main():
    rclpy.init()
    node = LidarcPublisher(port='/dev/ttyUSB0', baudrate=115200)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down")
    finally:
        node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
