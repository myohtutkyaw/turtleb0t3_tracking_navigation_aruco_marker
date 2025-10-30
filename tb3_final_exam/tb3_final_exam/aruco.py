import cv2, numpy as np
from cv_bridge import CvBridge

class ArucoHelper:
    def __init__(self, dictionary, marker_len_m):
        self.bridge = CvBridge()
        self.dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
        self.params = cv2.aruco.DetectorParameters()
        self.marker_len = marker_len_m
        # simple fx, fy from camera_info later
        self.camera_matrix = None
        self.dist = None

    def set_camera_info(self, K, D):
        self.camera_matrix = K; self.dist = D

    def detect(self, ros_img):
        img = self.bridge.imgmsg_to_cv2(ros_img, 'bgr8')
        corners, ids, _ = cv2.aruco.detectMarkers(img, self.dict, parameters=self.params)
        if ids is None: return None
        if self.camera_matrix is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_len, self.camera_matrix, self.dist)
            # return closest marker id, distance (m), yaw approx
            i = int(np.argmin(np.linalg.norm(tvecs.reshape(-1,3), axis=1)))
            tid = int(ids[i][0]); t = tvecs[i].reshape(3)
            dist = float(np.linalg.norm(t))
            return {'id': tid, 'distance': dist, 'tvec': t, 'rvec': rvecs[i].reshape(3)}
        return {'id': int(ids[0][0]), 'distance': None}
