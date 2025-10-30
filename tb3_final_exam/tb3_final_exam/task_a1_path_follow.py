#!/usr/bin/env python3
import rclpy, numpy as np, cv2
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class A1Path(Node):
    def __init__(self):
        super().__init__('task_a1_path_follow')
        self.declare_parameter('vision.image_topic', '/camera/image_raw')
        self.declare_parameter('vision.width_px', 640)
        self.declare_parameter('vision.hsv_low', [18,80,80])
        self.declare_parameter('vision.hsv_high', [35,255,255])
        self.declare_parameter('vision.roi_top', 0.55)
        self.declare_parameter('vision.roi_bottom', 1.0)
        self.declare_parameter('vision.centroid_deadband_px', 8)
        self.declare_parameter('vision.ang_kp', 1.3)
        self.declare_parameter('vision.max_lin_vel', 0.17)
        self.declare_parameter('vision.search_ang_vel', 0.35)

        p = self.get_parameters([
            'vision.image_topic','vision.width_px','vision.hsv_low','vision.hsv_high',
            'vision.roi_top','vision.roi_bottom','vision.centroid_deadband_px',
            'vision.ang_kp','vision.max_lin_vel','vision.search_ang_vel'
        ])
        self.topic = p[0].value; self.w = int(p[1].value)
        self.low = np.array(p[2].value, dtype=np.uint8)
        self.high= np.array(p[3].value, dtype=np.uint8)
        self.roi_top = float(p[4].value); self.roi_bot=float(p[5].value)
        self.dead = int(p[6].value); self.kp=float(p[7].value)
        self.vx = float(p[8].value); self.search=float(p[9].value)

        self.cv = CvBridge()
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Image, self.topic, self.on_img, 10)
        self.get_logger().info('A1 Path follow ready.')

    def on_img(self, msg):
        img = self.cv.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h = hsv.shape[0]
        r1, r2 = int(self.roi_top*h), int(self.roi_bot*h)
        roi = hsv[r1:r2, :]
        mask = cv2.inRange(roi, self.low, self.high)

        M = cv2.moments(mask)
        t = Twist()
        if M['m00'] < 1e-3:
            t.linear.x = 0.0; t.angular.z = self.search
            self.cmd.publish(t); return

        cx = int(M['m10']/M['m00'])
        err = cx - (mask.shape[1]//2)
        if abs(err) <= self.dead: err = 0
        t.linear.x = self.vx
        t.angular.z = - self.kp * (err / (self.w/2.0))
        self.cmd.publish(t)

def main():
    rclpy.init(); node = A1Path(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()
