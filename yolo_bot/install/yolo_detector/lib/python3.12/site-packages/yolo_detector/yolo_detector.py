import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.model = YOLO('yolov8n.pt')  # İstersen yolov11.pt kullan
        self.get_logger().info('YOLO model yüklendi ve dinliyor...')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model(frame)[0]
            annotated = results.plot()
            cv2.imshow("YOLOv8 Detection", annotated)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Hata: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()
