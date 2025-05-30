#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from custom_msgs.msg import AbsResult
import serial
import time
import numpy as np


def theta(x, z, max_x):
    AO = int(max_x / 2)
    OP = z
    dot = -AO * (x - AO) + OP ** 2
    mag_CA = np.sqrt(AO ** 2 + OP ** 2)
    mag_Cx = np.sqrt((x - AO) ** 2 + OP ** 2)
    cos_theta = dot / (mag_CA * mag_Cx)
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    return np.degrees(np.arccos(cos_theta))


def get_angle_from_x(x):
    # reference table
    points = [
        (0, 57),
        (8, 59),
        (23, 65),
        (40, 71),
        (59, 77),
        (79, 84),
        (98, 89),
        (115, 97),
        (132, 102),
        (148, 108),
        (156, 109)
    ]

    if x < 0 or x > 156:
        return None  # Cambiado: devolver None para errores

    for point_x, point_angle in points:
        if x == point_x:
            return point_angle

    for i in range(len(points) - 1):
        x0, angle0 = points[i]
        x1, angle1 = points[i + 1]
        if x0 <= x <= x1:
            ratio = (x - x0) / (x1 - x0)
            angle = angle0 + ratio * (angle1 - angle0)
            return round(angle, 2)

    return None  # Si no encuentra, devolver None


class AbsSubscriber(Node):
    def __init__(self):
        super().__init__('abs_subscriber')
        self.subscription = self.create_subscription(
            AbsResult,
            'inference',
            self.listener_callback,
            10
        )

        try:
            self.declare_parameter('arduino_path', '/dev/ttyACM1')
            arduino_path = self.get_parameter('arduino_path').get_parameter_value().string_value
            self.ser = serial.Serial(arduino_path, 9600, timeout=1)

            self.initial_value = 56
            self.send_angle(self.initial_value)
            self.get_logger().info(f"Arduino initialized at {arduino_path}, angle set to {self.initial_value}")
            time.sleep(2)  # Opcional: espera para estabilizar Arduino

        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            self.ser = None

    def send_angle(self, angle):
        if self.ser and self.ser.is_open:
            try:
                angle_mapped = int(180 - float(angle))  # Mapeo final: ajuste para tu servo
                self.ser.write(f"{angle_mapped}\n".encode())
                self.get_logger().info(f"Sent angle: {angle_mapped} (Original: {angle})")
            except Exception as e:
                self.get_logger().error(f"Error sending angle: {e}")

    def listener_callback(self, msg):
        self.get_logger().info(
            f"Received: abs_dist={msg.abs_dist:.2f}, x_real_coordinate={msg.x_real_coordinate:.2f}, max_x={msg.max_x:.2f}"
        )

        if not self.ser:
            self.get_logger().error("Serial connection not initialized")
            return

        angle_deg = get_angle_from_x(msg.x_real_coordinate)
        if angle_deg is None:
            self.get_logger().warn(f"x={msg.x_real_coordinate} out of range, angle not sent.")
            return

        self.send_angle(angle_deg)

    def destroy_node(self):
        if self.ser:
            self.ser.close()
            self.get_logger().info("Serial connection closed.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AbsSubscriber()

    try:
        while rclpy.ok():
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
