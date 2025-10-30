#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np

class A2QRStopTurn(Node):
    """
    Task A2:
      - Detect a QR code that encodes a STOP text (case-insensitive).
      - Immediately stop and hold for a short duration.
      - Execute a right 90° turn using a timed angular velocity command.
    """
    def __init__(self):
        super().__init__('task_a2_qr_turn')

        # ---- Parameters (can be set via exam_params.yaml or CLI) ----
        self.declare_parameter('vision.image_topic', '/camera/image_raw')
        self.declare_parameter('qr.stop_text', 'STOP')
        self.declare_parameter('qr.stop_hold_ms', 500)       # ms to hold before turn
        self.declare_parameter('qr.debug_log_every', 10)      # log every N frames (reduce spam)
        self.declare_parameter('turn.vel', -0.8)              # rad/s (negative = right)
        self.declare_parameter('turn.duration_s', 1.20)       # seconds (≈90°, tune as needed)

        self.image_topic   = self.get_parameter('vision.image_topic').value
        self.stop_text     = str(self.get_parameter('qr.stop_text').value).strip().upper()
        self.hold_ms       = int(self.get_parameter('qr.stop_hold_ms').value)
        self.debug_every   = int(self.get_parameter('qr.debug_log_every').value)
        self.turn_vel      = float(self.get_parameter('turn.vel').value)
        self.turn_duration = float(self.get_parameter('turn.duration_s').value)

        # ---- Interfaces ----
        self.cv = CvBridge()
        self.sub = self.create_subscription(Image, self.image_topic, self.on_image, 10)
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        # ---- OpenCV QR detector ----
        self.det = cv2.QRCodeDetector()

        # ---- Simple FSM ----
        self.state = 'SEARCH'   # SEARCH -> HOLD -> TURN -> DONE
        self.hold_timer = None
        self.turn_timer = None
        self.turn_start_time = None
        self.frame_count = 0

        self.get_logger().info(
            f'A2 ready. Listening on {self.image_topic}; STOP="{self.stop_text}", '
            f'hold={self.hold_ms} ms, turn vel={self.turn_vel} rad/s for {self.turn_duration}s.'
        )

    # ----------------- Helpers -----------------
    def publish_stop(self):
        self.cmd.publish(Twist())

    def start_hold_then_turn(self):
        # Stop & hold using a one-shot timer
        self.publish_stop()
        self.state = 'HOLD'
        self.get_logger().info('STOP detected → holding...')
        hold_sec = max(self.hold_ms / 1000.0, 0.0)
        # one-shot timer
        self.hold_timer = self.create_timer(hold_sec, self._start_turn_once)

    def _start_turn_once(self):
        # Cancel the one-shot hold timer
        if self.hold_timer is not None:
            self.hold_timer.cancel()
            self.hold_timer = None
        # Start periodic timer to publish angular velocity until duration reached
        self.state = 'TURN'
        self.turn_start_time = self.get_clock().now()
        self.get_logger().info('Holding complete → turning right ~90°...')
        self.turn_timer = self.create_timer(0.02, self._turn_cb)  # 50 Hz

    def _turn_cb(self):
        # publish angular velocity until elapsed >= duration
        now = self.get_clock().now()
        elapsed = (now - self.turn_start_time).nanoseconds * 1e-9
        if elapsed < self.turn_duration:
            t = Twist()
            t.angular.z = self.turn_vel
            self.cmd.publish(t)
        else:
            # stop turning
            self.turn_timer.cancel()
            self.turn_timer = None
            self.publish_stop()
            self.state = 'DONE'
            self.get_logger().info('Turn complete.')

    # --------------- Image callback ---------------
    def on_image(self, msg: Image):
        if self.state in ('HOLD', 'TURN', 'DONE'):
            return

        # Convert to CV image
        img = self.cv.imgmsg_to_cv2(msg, 'bgr8')

        # Improve robustness: grayscale + slight blur + CLAHE for contrast
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(4, 4))
        gray = clahe.apply(gray)

        # Detect & decode
        data, bbox, _ = self.det.detectAndDecode(gray)

        self.frame_count += 1
        if bbox is not None and len(bbox) > 0:
            if self.frame_count % self.debug_every == 0:
                self.get_logger().info(f"QR seen, data='{data}'")
            # (Optional) If you want to publish debug image, add an Image publisher; omitted here.

        # Accept if the decoded text matches (case-insensitive)
        if data and data.strip().upper() == self.stop_text:
            self.get_logger().info('STOP QR detected!')
            self.start_hold_then_turn()

def main():
    rclpy.init()
    node = A2QRStopTurn()
    try:
        rclpy.spin(node)
    finally:
        # Cleanup timers on shutdown
        if node.hold_timer:
            node.hold_timer.cancel()
        if node.turn_timer:
            node.turn_timer.cancel()
        node.destroy_node()
        rclpy.shutdown()
