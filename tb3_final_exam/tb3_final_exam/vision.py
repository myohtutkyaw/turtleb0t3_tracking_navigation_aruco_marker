import cv2, numpy as np
from cv_bridge import CvBridge

class Vision:
    def __init__(self):
        self.bridge = CvBridge()
        self.yellow_hsv = ((18, 80, 80), (35, 255, 255))  # tune

    def yellow_centroid(self, ros_img):
        img = self.bridge.imgmsg_to_cv2(ros_img, desired_encoding='bgr8')
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.yellow_hsv[0], self.yellow_hsv[1])
        M = cv2.moments(mask)
        if M['m00'] < 1e-3: return None, img
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        return (cx, cy), img

    def detect_qr_text(self, ros_img):
        img = self.bridge.imgmsg_to_cv2(ros_img, 'bgr8')
        det = cv2.QRCodeDetector()
        data, _, _ = det.detectAndDecode(img)
        return data if data else None
