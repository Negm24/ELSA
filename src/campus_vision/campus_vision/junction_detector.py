import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2

class JunctionDetector(Node):
    def __init__(self):
        super().__init__('junction_detector')
        self.publisher_ = self.create_publisher(String, 'detected_room', 10)
        self.cap = cv2.VideoCapture(0)  # 1 = second camera (ground-facing)
        self.timer = self.create_timer(0.5, self.detect_callback)
        self.get_logger().info('Junction Detector Started.')

    def detect_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        # Convert to grayscale for easier marker detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Simple color/shape detection for floor markers
        # Each junction marker is a piece of paper with text on the floor
        import easyocr
        if not hasattr(self, 'reader'):
            self.reader = easyocr.Reader(['en'])

        results = self.reader.readtext(gray)
        for (bbox, text, prob) in results:
            cleaned = text.lower().replace(' ', '_')
            if 'junction' in cleaned and prob > 0.5:
                msg = String()
                msg.data = cleaned
                self.publisher_.publish(msg)
                self.get_logger().info(f'Junction detected: {cleaned}')

        cv2.imshow('Junction Camera', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = JunctionDetector()
    rclpy.spin(node)
    node.cap.release()
    cv2.destroyAllWindows()
    rclpy.shutdown()