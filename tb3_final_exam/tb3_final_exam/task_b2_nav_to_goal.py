#!/usr/bin/env python3
import rclpy, math
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from tb3_final_exam.aruco import ArucoHelper
from tb3_final_exam.nav_client import NavClient

class B2Nav(Node):
    def __init__(self):
        super().__init__('task_b2_nav_goal')
        self.declare_parameter('aruco.dictionary', 'DICT_4X4_50')
        self.declare_parameter('aruco.marker_length_m', 0.05)
        self.declare_parameter('goals', {'0':[2.0,0.5,0.0]})
        self.declare_parameter('vision.image_topic', '/camera/image_raw')

        dictn = self.get_parameter('aruco.dictionary').value
        mlen  = float(self.get_parameter('aruco.marker_length_m').value)
        self.goals = {int(k):v for k,v in self.get_parameter('goals').value.items()}
        self.topic = self.get_parameter('vision.image_topic').value

        self.aruco = ArucoHelper(dictn, mlen)
        self.nav = NavClient(self)

        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_cinfo, 10)
        self.create_subscription(Image, self.topic, self.on_img, 10)
        self.sent = False
        self.get_logger().info('B2 Nav-to-goal ready.')

    def on_cinfo(self, msg):
        import numpy as np
        K = np.array(msg.k).reshape(3,3); D = np.array(msg.d)
        self.aruco.set_camera_info(K, D)

    async def go(self, gid):
        x,y,yaw = self.goals[gid]
        status = await self.nav.goto_xyyaw(x,y,yaw)
        self.get_logger().info(f'Nav2 status: {status}')

    def on_img(self, msg):
        if self.sent: return
        det = self.aruco.detect(msg)
        if det and det.get('id') in self.goals:
            self.sent = True
            self.get_logger().info(f'Found goal marker id={det["id"]}')
            self.create_task(self.go(det['id']))

def main():
    rclpy.init(); node = B2Nav(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()
