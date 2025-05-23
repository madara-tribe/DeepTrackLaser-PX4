import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.bridge = CvBridge()
        self.declare_parameter('device_path', '/dev/video4')
        device_path = self.get_parameter('device_path').get_parameter_value().string_value

        self.cap = cv2.VideoCapture(device_path)
        print(device_path)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.pub = self.create_publisher(Image, "/video_stream", qos_profile_sensor_data)

    def run(self):
        while True:
            try:
                ret, frame = self.cap.read()
                if not ret:
                    self.get_logger().warn("Failed to capture frame")
                    return
                self.pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

            except CvBridgeError as e:
                self.get_logger().error(f"CV Bridge Error: {e}")

        self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    node.get_logger().info("Camera publishing started")
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
