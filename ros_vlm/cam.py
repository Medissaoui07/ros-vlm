import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time 
class CameraPublisherNode(Node):
    def __init__(self):
        super().__init__('camera_publisher_node')
        self.publisher = self.create_publisher(Image, 'camera/image', 10)
        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(0) 

        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera ')
            self.timer = None 

        if self.cap is not None and self.cap.isOpened():
            

            time.sleep(1.0)
            for _ in range(10):
                _, _ = self.cap.read()
           # self.get_logger().info("Camera initialized and ready .")
            self.timer = self.create_timer(0.1, self.publish_image)
        else:
            self.get_logger().error("CameraPublisherNode will not publish due to camera initialization failure.")

    def publish_image(self):
        if self.cap is None or not self.cap.isOpened():
            self.get_logger().warn('No camera available to publish from. Skipping frame.')
            return

        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().error('Failed to capture image from camera. `ret` is False. Stream might have ended or camera disconnected.')
            return

        if frame is None or frame.size == 0:
            self.get_logger().error('Captured empty or invalid frame from camera. Skipping publish.')
            return

        

        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(msg)
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')

    def destroy_node(self):
        self.get_logger().info('Destroying CameraPublisherNode. Releasing camera.')
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    camera_publisher_node = CameraPublisherNode()
    rclpy.spin(camera_publisher_node) 
    camera_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()