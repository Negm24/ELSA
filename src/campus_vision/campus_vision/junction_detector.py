import warnings
warnings.filterwarnings("ignore", category=FutureWarning)

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import requests
import numpy as np
from doctr.models import ocr_predictor
import torch
import re

class CampusVision(Node):
    def __init__(self):
        super().__init__('campus_vision')
        self.publisher_ = self.create_publisher(String, 'detected_sign', 10)  # renamed topic

        self.timer = self.create_timer(0.1, self.detect_callback)

        self.ip_camera_url = 'http://172.20.10.10:8080/shot.jpg'
        self.session = requests.Session()

        self.get_logger().info("Initializing Doctr OCR Model...")
        self.model = ocr_predictor(det_arch='db_resnet50', reco_arch='crnn_vgg16_bn', pretrained=True)
        if torch.cuda.is_available():
            self.model.cuda()

        # Matches J1, J2, J12, etc.
        self.junction_pattern = re.compile(r'^J\d+$', re.IGNORECASE)

        # Matches R101, R202, R3, etc.
        self.room_pattern = re.compile(r'^R\d+$', re.IGNORECASE)

        self.blacklist = ['POWERED', 'CAMERA', 'LITE', 'IOS', 'FRONT', 'BACK', 'BAT']
        self.processing = False

    def detect_callback(self):
        if self.processing:
            return

        self.processing = True

        try:
            response = self.session.get(self.ip_camera_url, timeout=1)
            imgnp = np.frombuffer(response.content, dtype=np.uint8)
            frame = cv2.imdecode(imgnp, -1)

            if frame is None:
                self.processing = False
                return

        except Exception as e:
            self.get_logger().warn(f"Phone fetch failed: {e}")
            self.processing = False
            return

        h, w = frame.shape[:2]
        roi_frame = frame[int(h * 0.15):h, :]
        roi_h, roi_w = roi_frame.shape[:2]

        result = self.model([cv2.cvtColor(roi_frame, cv2.COLOR_BGR2RGB)])
        output = result.export()

        for page in output['pages']:
            for block in page['blocks']:
                for line in block['lines']:
                    for word in line['words']:
                        text = word['value'].strip()

                        if word['confidence'] < 0.5:
                            continue
                        if any(b in text.upper() for b in self.blacklist):
                            continue

                        detection_id = self.classify_sign(text)

                        if detection_id:
                            msg = String()
                            msg.data = detection_id
                            self.publisher_.publish(msg)
                            self.get_logger().info(f'EYES SAW: {detection_id}')
                            self.draw_debug(roi_frame, word, detection_id, roi_w, roi_h)

        self.processing = False
        cv2.imshow('ELSA Vision Feed', roi_frame)
        cv2.waitKey(1)

    def classify_sign(self, text):
        """
        Returns the canonical sign ID that brain_node understands, or "" if not a sign.

        Sign formats on the floor:
          Junctions → "J1", "J2", "J3" ...   published as-is: "J1"
          Rooms     → "R101", "R202" ...      published as-is: "R101"

        OCR sometimes adds noise (lowercase, extra chars), so we normalize first.
        """
        # Normalize: strip noise, uppercase
        normalized = re.sub(r'[^A-Za-z0-9]', '', text).upper()

        if self.junction_pattern.match(normalized):
            return normalized          # e.g. "J1", "J3"

        if self.room_pattern.match(normalized):
            return normalized          # e.g. "R101", "R302"

        return ""

    def draw_debug(self, frame, word, label, w, h):
        geom = word['geometry']
        p1 = (int(geom[0][0] * w), int(geom[0][1] * h))
        p2 = (int(geom[1][0] * w), int(geom[1][1] * h))
        cv2.rectangle(frame, p1, p2, (0, 255, 0), 2)
        cv2.putText(frame, label, (p1[0], p1[1] - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)


def main(args=None):
    rclpy.init(args=args)
    node = CampusVision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



"""
source /opt/ros/jazzy/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

source ~/Desktop/ELSA2/ws/install/setup.bash
ros2 run navigation_brain brain_node

source ~/Desktop/ELSA2/ws/install/setup.bash
ros2 run campus_vision junction_detector

source ~/Desktop/ELSA2/ws/install/setup.bash
ros2 run campus_vision room_detector

"""