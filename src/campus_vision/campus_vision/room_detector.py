import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import easyocr

class RoomDetector(Node):
    def __init__(self):
        super().__init__('room_detector')
        self.publisher_ = self.create_publisher(String, 'detected_room', 10)
        self.timer = self.create_timer(2.0, self.detect_callback)  # ← slower timer, less CPU
        self.cap = cv2.VideoCapture(0)

        # Force lower resolution from camera — less data to process
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.reader = easyocr.Reader(['en'], gpu=False)
        self.processing = False  # ← guard flag
        self.get_logger().info('Vision Node Started. Looking for room numbers...')

    def detect_callback(self):
        # Skip this cycle if previous frame still being processed
        if self.processing:
            return
        self.processing = True

        ret, frame = self.cap.read()
        if not ret:
            self.processing = False
            return

        # Resize to half — biggest CPU saver
        frame = cv2.resize(frame, (320, 240))

        # Convert to grayscale — EasyOCR works well on grayscale and it's faster
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        results = self.reader.readtext(gray)
        for (bbox, text, prob) in results:
            if any(char.isdigit() for char in text) and prob > 0.5:
                msg = String()
                msg.data = text
                self.publisher_.publish(msg)
                self.get_logger().info(f'Detected Room: {text} (Confidence: {prob:.2f})')

        self.processing = False
        cv2.imshow('Room Vision', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = RoomDetector()
    rclpy.spin(node)
    node.cap.release()
    cv2.destroyAllWindows()
    rclpy.shutdown()