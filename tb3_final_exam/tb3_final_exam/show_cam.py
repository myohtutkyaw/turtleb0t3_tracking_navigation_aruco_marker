#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2

class ShowCam(Node):
    def __init__(self):
        super().__init__('show_cam')
        self.bridge = CvBridge()

        # Params
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('use_compressed', True)
        self.declare_parameter('window_name', 'camera')

        topic = self.get_parameter('image_topic').get_parameter_value().string_value
        use_comp = self.get_parameter('use_compressed').get_parameter_value().bool_value
        self.win = self.get_parameter('window_name').get_parameter_value().string_value

        # Create window (requires X11 / a GUI session)
        try:
            cv2.namedWindow(self.win, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.win, 960, 540)
        except Exception as e:
            self.get_logger().warn(f'OpenCV window could not be created: {e}')

        # Subscribe
        if use_comp:
            self.sub = self.create_subscription(
                CompressedImage, f'{topic}/compressed', self.on_comp, 10
            )
            self.get_logger().info(f'Subscribing to {topic}/compressed (CompressedImage)')
        else:
            self.sub = self.create_subscription(
                Image, topic, self.on_raw, 10
            )
            self.get_logger().info(f'Subscribing to {topic} (Image)')

        # Periodic UI pump (avoid stuck windows if no frames)
        self.timer = self.create_timer(0.05, self._tick)

    def _tick(self):
        # give OpenCV time to process window events
        try:
            cv2.waitKey(1)
        except Exception:
            pass

    def on_raw(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow(self.win, frame)
        except Exception as e:
            self.get_logger().warn(f'raw convert/show failed: {e}')

    def on_comp(self, msg: CompressedImage):
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg)
            cv2.imshow(self.win, frame)
        except Exception as e:
            self.get_logger().warn(f'compressed convert/show failed: {e}')

def main():
    # Optional: let OpenCV pick X display from environment (useful over ssh -Y)
    if 'DISPLAY' not in os.environ:
        # Without DISPLAY, cv.imshow cannot render. Warn early.
        print('[show_cam] WARNING: No DISPLAY found. Use a GUI session or ssh -Y.')
    rclpy.init()
    node = ShowCam()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
