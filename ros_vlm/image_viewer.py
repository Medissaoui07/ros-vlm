import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

class ImageViewerNode(Node):
    def __init__(self):
        super().__init__('image_viewer_node')
        self.subscription = self.create_subscription(
            Image,
            'camera/image',
            self.image_callback,
            10
        )
        self.text_subscription = self.create_subscription(
            String,
            'vlm_response',
            self.text_callback,
            10
        )
        self.subscription 
        self.response =""
        self.bridge = CvBridge()
        self.get_logger().info('Image Viewer Node initialized. Press "q" in the image window to quit.')
    def text_callback(self, msg): 
        self.response = msg.data

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            if cv_image is None or cv_image.size == 0:
                self.get_logger().warn('Received empty or invalid image from topic. Not displaying.')
                return
            if self.response:
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.5
                color = (0, 255, 0)
                thickness = 1
                x, y = 10, 20
                # Optionally wrap text for long responses
                for i, line in enumerate(self.response.split('\n')):
                    cv2.putText(cv_image, line, (x, y + i*20), font, font_scale, color, thickness, cv2.LINE_AA)
            cv2.imshow("Image Viewer", cv_image)
            key = cv2.waitKey(1) & 0xFF
                 

            if key == ord('q'): 
                self.get_logger().info('Quit key pressed. Shutting down image viewer node.')
                self.destroy_node() 
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
        except Exception as e:
            self.get_logger().error(f'Error displaying image: {e}')

    def destroy_node(self):
        self.get_logger().info('Destroying ImageViewerNode. Destroying OpenCV windows.')
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    image_viewer_node = ImageViewerNode()
    try:
        rclpy.spin(image_viewer_node) 
    except KeyboardInterrupt:
        image_viewer_node.get_logger().info('KeyboardInterrupt received. Shutting down image viewer.')
    finally:
        image_viewer_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()