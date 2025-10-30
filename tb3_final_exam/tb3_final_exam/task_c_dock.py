#!/usr/bin/env python3
import rclpy, math
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from tb3_final_exam.aruco import ArucoHelper
from tb3_final_exam.docking import Docker

class CDock(Node):
    def __init__(self):
        super().__init__('task_c_dock')
        self.declare_parameter('aruco.dictionary', 'DICT_4X4_50')
        self.declare_parameter('aruco.marker_length_m', 0.05)
        self.declare_parameter('vision.image_topic', '/camera/image_raw')
        self.declare_parameter('docking.target_distance_m', 0.10)
        self.declare_parameter('docking.distance_tol_m', 0.02)

        dictn = self.get_parameter('aruco.dictionary').value
        mlen  = float(self.get_parameter('aruco.marker_length_m').value)
        self.topic = self.get_parameter('vision.image_topic').value
        self.dstar = float(self.get_parameter('docking.target_distance_m').value)
        self.tol   = float(self.get_parameter('docking.distance_tol_m').value)

        self.aruco = ArucoHelper(dictn, mlen)
        self.docker = Docker(self)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_cinfo, 10)
        self.create_subscription(Image, self.topic, self.on_img, 10)
        self.get_logger().info('C Dock ready.')

    def on_cinfo(self, msg):
        import numpy as np
        K = np.array(msg.k).reshape(3,3); D = np.array(msg.d)
        self.aruco.set_camera_info(K, D)

    def on_img(self, msg):
        det = self.aruco.detect(msg)
        if not det or det.get('distance') is None:
            self.docker.stop(); return
        dist = det['distance']
        heading = math.atan2(det['tvec'][0], det['tvec'][2])  # x/z
        self.docker.approach(dist, heading)
        if abs(dist - self.dstar) <= self.tol:
            self.docker.stop()
            self.get_logger().info('Docked at target distance.')

def main():
    rclpy.init(); node = CDock(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()
