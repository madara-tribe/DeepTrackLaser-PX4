#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from custom_msgs.msg import AbsResult
import serial
import time

class AbsSubscriber(Node):
    def __init__(self):
        super().__init__('abs_subscriber')
        self.subscription = self.create_subscription(
            AbsResult,
            'inference',
            self.listener_callback,
            10
        )
        self.subscription

        try:
            self.declare_parameter('arduino_path', '/dev/ttyACM0')
            arduino_path = self.get_parameter('arduino_path').get_parameter_value().string_value
            self.ser = serial.Serial(arduino_path, 9600, timeout=1)

            # === Initial setup ===
            self.initial_value = 124  # Corresponds to angle = 56 (via 180 - 124)
            self.current_angle = int(180 - self.initial_value)
            self.last_received = None

            # Send initial angle
            self.send_angle(self.current_angle)
            self.get_logger().info(f"Servo initialized at angle: {self.current_angle}")
            time.sleep(2)

        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            self.ser = None

    def send_angle(self, angle):
        if self.ser and self.ser.is_open:
            self.ser.write(f"{angle}\n".encode())
            self.get_logger().info(f"Sent angle: {angle}")

    def map_value_to_angle(self, value):
        return int(180 - float(value))

    def listener_callback(self, msg):
        self.get_logger().info(f"Received: abs_dist={msg.abs_dist:.2f}, angle_deg={msg.angle_deg:.2f}")
        if not self.ser:
            return

        try:
            mapped_angle = self.map_value_to_angle(msg.angle_deg)

            if self.last_received is None:
                self.current_angle = mapped_angle
            else:
                delta = mapped_angle - self.last_received
                self.current_angle += delta

            self.last_received = mapped_angle

            self.send_angle(self.current_angle)

        except Exception as e:
            self.get_logger().error(f"Failed to send angle: {e}")

    def destroy_node(self):
        if self.ser:
            self.ser.close()
            self.get_logger().info("Serial connection closed.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AbsSubscriber()

    try:
        while rclpy.ok():  # Continuous operation until stopped
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

