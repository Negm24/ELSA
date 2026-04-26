import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
from doctr.models import ocr_predictor
import torch
import re

class RoomDetector(Node):
    def __init__(self):
        super().__init__('room_detector')
        self.publisher_ = self.create_publisher(String, 'detected_room', 10)
        self.timer = self.create_timer(0.2, self.detect_callback)
        
        # IP Camera URL
        ip_camera_url = 'http://admin:admin@192.168.100.78:8081/video' 
        self.cap = cv2.VideoCapture(ip_camera_url)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # Initialize DocTR
        self.model = ocr_predictor(det_arch='db_resnet50', reco_arch='crnn_vgg16_bn', pretrained=True)
        if torch.cuda.is_available():
            self.model.cuda()
            self.get_logger().info('DocTR running on GPU (RTX 3050)')

        self.processing = False
        self.get_logger().info('DEBUG MODE: Checking every word detected...')

    def detect_callback(self):
        if self.processing:
            return
        
        self.processing = True
        ret, frame = self.cap.read()
        if not ret:
            self.processing = False
            return

        # 1. CROP THE TOP (Watermark area)
        # In your screenshot, the watermark takes up a good chunk of the top left.
        # Let's crop the top 20% instead of 10%.
        h, w = frame.shape[:2]
        crop_top = int(h * 0.20) 
        roi_frame = frame[crop_top:h, :]
        roi_h, roi_w = roi_frame.shape[:2]

        rgb_frame = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2RGB)
        result = self.model([rgb_frame])
        output = result.export()

        # Pattern for room numbers (at least 2 digits)
        room_pattern = re.compile(r'\d{2,}')

        # Words to ignore from the "IP Camera Lite" watermark
        blacklist = ['POWERED', 'CAMERA', 'LITE', 'IOS', 'FRONT', 'BACK']

        for page in output['pages']:
            for block in page['blocks']:
                for line in block['lines']:
                    for word in line['words']:
                        text = word['value']
                        prob = word['confidence']

                        # Clean up the text for checking
                        clean_text = text.upper().replace('_', '').replace('-', '')

                        # --- SMART FILTERING ---
                        # Skip if it's in the watermark blacklist
                        if any(b in clean_text for b in blacklist):
                            continue
                        
                        # Only proceed if there's a number and confidence is solid
                        match = room_pattern.search(text)
                        if match and prob > 0.6:
                            room_number = match.group()
                            
                            # Log ONLY the success so your terminal stays clean
                            self.get_logger().info(f'>>> SUCCESS! Extracted Room: {room_number}')
                            
                            msg = String()
                            msg.data = room_number
                            self.publisher_.publish(msg)
                            
                            # Draw feedback
                            geom = word['geometry']
                            p1 = (int(geom[0][0] * roi_w), int(geom[1][0] * roi_h)) # Swapped for correct box
                            p1 = (int(geom[0][0] * roi_w), int(geom[0][1] * roi_h))
                            p2 = (int(geom[1][0] * roi_w), int(geom[1][1] * roi_h))
                            cv2.rectangle(roi_frame, p1, p2, (0, 255, 0), 2)
                            cv2.putText(roi_frame, f"Room {room_number}", (p1[0], p1[1]-5), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        self.processing = False
        cv2.imshow('Room Vision Debug', roi_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = RoomDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()