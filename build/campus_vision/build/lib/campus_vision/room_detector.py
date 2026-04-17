import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import easyocr

class RoomDetector(Node):
    def __init__(self):
        super().__init__('room_detector')
        self.publisher_ = self.create_publisher(String, 'detected_room', 10)
        self.timer = self.create_timer(1.0, self.detect_callback) # Check once per second
        
        # Initialize the Camera and OCR Reader
        self.cap = cv2.VideoCapture(0) # 0 is your laptop webcam
        self.reader = easyocr.Reader(['en']) # 'en' for English
        self.get_logger().info('Vision Node Started. Looking for room numbers...')

    def detect_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        # Use EasyOCR to find text in the frame
        results = self.reader.readtext(frame)

        for (bbox, text, prob) in results:
            # We only care if it looks like a room number (e.g., contains digits)
            if any(char.isdigit() for char in text) and prob > 0.5:
                msg = String()
                msg.data = text
                self.publisher_.publish(msg)
                self.get_logger().info(f'Detected Room: {text} (Confidence: {prob:.2f})')

                # Draw on the screen for us to see (debugging)
                cv2.putText(frame, text, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.imshow('Robot Vision', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = RoomDetector()
    rclpy.spin(node)
    node.cap.release()
    cv2.destroyAllWindows()
    rclpy.shutdown()