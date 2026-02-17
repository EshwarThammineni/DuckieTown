import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')

        #camera topic for us was "/duckiescrooge/camera/image"
        
        self.sub = self.create_subscription(
            Image,
            '/duckiescrooge/camera/image',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.get_logger().info("Camera subscriber started.")
    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV image (BGR format)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return
        # Display the image in a window
        cv2.imshow("DuckieDonald Camera", cv_image)
        cv2.waitKey(1) 

def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
