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

        # === Initialize Serial ===
        try:
            self.declare_parameter('arduino_path', '/dev/ttyACM0')
            arduino_path = self.get_parameter('arduino_path').get_parameter_value().string_value
            self.ser = serial.Serial(arduino_path, 9600, timeout=1)
            time.sleep(5)  # Wait for Arduino to reset

            self.initial_angle = 127
            self.current_angle = self.initial_angle
            self.last_received = None

            self.ser.write(f"{self.current_angle}\n".encode())
            self.get_logger().info(f"Initialized to angle: {self.current_angle}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            self.ser = None

    def listener_callback(self, msg):
        self.get_logger().info(f"Received: abs_dist={msg.abs_dist:.2f}, angle_deg={msg.angle_deg:.2f}")
        if not self.ser:
            return

        try:
            if self.last_received is None:
                # First message: offset from initial
                self.current_angle = self.initial_angle - msg.angle_deg
            else:
                # Following messages: adjust from previous
                delta = self.last_received - msg.angle_deg
                self.current_angle += delta

            self.current_angle = int(max(72, min(127, self.current_angle)))  # Clamp to range
            self.last_received = msg.angle_deg

            self.ser.write(f"{self.current_angle}\n".encode())
            self.get_logger().info(f"Sent angle {self.current_angle} to Arduino.")
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
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

