#!/usr/bin/env python3
import math
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from nav_msgs.msg import Odometry, OccupancyGrid
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from nav2_msgs.action import NavigateToPose
from cv_bridge import CvBridge
from builtin_interfaces.msg import Time as RosTime


# ---------- Optional helper (ok if missing) ----------
try:
    from tb3_final_exam.aruco import ArucoHelper
except Exception:
    ArucoHelper = None


# ---------- small utils ----------
def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    half = 0.5 * yaw
    q.w = math.cos(half)
    q.z = math.sin(half)
    q.x = 0.0
    q.y = 0.0
    return q

def ros_now(node: Node) -> RosTime:
    return node.get_clock().now().to_msg()


class Phase:
    A   = 'A'        # Yellow line → QR stop → 90° → wait 1s
    B1  = 'B1_SLAM'  # Optional SLAM wait (if not skipped)
    B   = 'B'        # FRONT-ONLY ArUco detect → Nav2(goal by id)
    C   = 'C'        # Dock to 0.10 m from SAME marker
    DONE= 'DONE'


# ============================ PART A ============================
class PartA:
    """Yellow track follow → QR stop → 90° turn (odom) → wait 1s → (B1 or B)."""
    def __init__(self, n:'ExamAll'):
        self.n = n
        self.present_frames = 0
        self.ang_ema = 0.0
        self.last_side = 1
        self.hold_timer = None
        self.turn_timer = None
        self.after_turn_timer = None
        self.turn_start = None
        self.turn_start_yaw = None
        self.turn_target_rad = math.radians(abs(self.n.turn_target_deg))
        self.qr_latched = False
        self.post_turn_wait_s = 1.0

    def on_image(self, msg: Image):
        if self.n.phase != Phase.A:
            return

        # freeze while stopping/turning/settling
        if self.qr_latched or self.hold_timer or self.turn_timer or self.after_turn_timer:
            self.n.stop_cmd()
            return

        img = self.n.cv.imgmsg_to_cv2(msg, 'bgr8')
        self._path_follow(img)

        if self._try_qr_stop(img):
            self.qr_latched = True
            self.n.stop_cmd()
            if self.hold_timer is None:
                # one-shot hold -> begin turn
                self.hold_timer = self.n.create_timer(
                    self.n.hold_ms/1000.0,
                    self._begin_turn_once
                )

    def on_odom(self, _yaw: float):
        # not needed for Part A (odom used inside timer callback)
        return

    def destroy(self):
        for t in (self.hold_timer, self.turn_timer, self.after_turn_timer):
            try:
                if t: t.cancel()
            except Exception:
                pass

    # ---- line following
    def _path_follow(self, img_bgr):
        hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        H, W = hsv.shape[:2]
        r1, r2 = int(self.n.roi_top * H), int(self.n.roi_bot * H)
        roi = hsv[r1:r2, :]

        mask = cv2.inRange(roi, self.n.low, self.n.high)
        if self.n.k_open > 0:
            k1 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.n.k_open, self.n.k_open))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k1)
        if self.n.k_close > 0:
            k2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.n.k_close, self.n.k_close))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k2)

        frac = float((mask > 0).sum()) / float(mask.size)
        M = cv2.moments(mask)
        t = Twist()

        if M['m00'] < 1e-3 or frac < self.n.min_cov:
            # lost -> search rotate toward last side
            t.linear.x = 0.0
            t.angular.z = self.n.search_w * self.last_side
            self.ang_ema = 0.0
            self.n.cmd.publish(t)
            return

        cx = int(M['m10'] / M['m00'])
        err_px = cx - (mask.shape[1] // 2)
        self.last_side = 1 if err_px > 0 else -1

        if abs(err_px) <= self.n.dead_px:
            err_px = 0
        err_norm = err_px / (mask.shape[1] / 2.0)

        ang_cmd = - self.n.kp_ang * err_norm
        self.ang_ema = self.n.alpha * ang_cmd + (1.0 - self.n.alpha) * self.ang_ema
        ang = float(np.clip(self.ang_ema, -self.n.max_w, self.n.max_w))
        lin = self.n.max_v * max(0.2, 1.0 - min(abs(err_norm), 1.0))

        t.linear.x = lin
        t.angular.z = ang
        self.n.cmd.publish(t)

    # ---- robust QR (guard invalid source points)
    def _try_qr_stop(self, img_bgr) -> bool:
        gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        if abs(self.n.qr_resize - 1.0) > 1e-3:
            gray = cv2.resize(gray, None, fx=self.n.qr_resize, fy=self.n.qr_resize, interpolation=cv2.INTER_LINEAR)

        candidates = [("raw", gray)]
        if self.n.qr_use_shrp:
            blur = cv2.GaussianBlur(gray, (0,0), 1.0)
            sharp = cv2.addWeighted(gray, 1.7, blur, -0.7, 0)
            candidates.append(("sharp", sharp))
        if self.n.qr_use_bin:
            binimg = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 31, 5)
            candidates.append(("binary", binimg))

        def accept(txt, boxes, tag):
            if boxes is None or len(boxes) == 0:
                return False
            p = np.array(boxes[0], dtype=np.float32)
            if p.shape != (4,2):
                return False
            # guard: degenerate quad
            if cv2.contourArea(p) <= 0.0:
                return False

            size_px = int(max(
                np.linalg.norm(p[0]-p[1]),
                np.linalg.norm(p[1]-p[2]),
                np.linalg.norm(p[2]-p[3]),
                np.linalg.norm(p[3]-p[0]),
            ))
            ok_text = (txt or "").strip().upper() == self.n.stop_text

            self._cnt = getattr(self, "_cnt", 0) + 1
            if self._cnt % self.n.qr_dbg_n == 0:
                self.n.get_logger().info(f"QR[{tag}]: text='{txt}' size≈{size_px}px (armed={self.n.a_armed})")

            if ok_text:
                self.present_frames = 0
                return True

            if self.n.stop_on_any and size_px >= self.n.min_bbox and self.n.a_armed:
                self.present_frames += 1
                if self.present_frames >= self.n.need_frames:
                    self.n.get_logger().info(f"QR present size≈{size_px}px ×{self.present_frames} → STOP.")
                    self.present_frames = 0
                    return True
            else:
                self.present_frames = 0
            return False

        qrd = cv2.QRCodeDetector()
        for tag, im in candidates:
            # single
            try:
                data, bbox, _ = qrd.detectAndDecode(im)
            except Exception:
                data, bbox = "", None
            if bbox is not None and len(bbox) > 0:
                b = bbox if (isinstance(bbox, np.ndarray) and bbox.ndim == 3) else np.array([bbox], np.float32)
                if accept(data, b, f"{tag}-single"):
                    return True

            # multi
            try:
                ok, texts, boxes, _ = cv2.QRCodeDetector().detectAndDecodeMulti(im)
            except Exception:
                ok, texts, boxes = False, [], None
            if ok and boxes is not None and len(boxes) > 0:
                for i, txt in enumerate(texts or []):
                    bb = np.array([boxes[i]], dtype=np.float32)
                    if accept(txt, bb, f"{tag}-multi"):
                        return True
        return False

    # ---- turning
    @staticmethod
    def _angdiff(a, b):
        return (b - a + math.pi) % (2*math.pi) - math.pi

    def _turned_abs(self):
        if self.n.curr_yaw is None or self.turn_start_yaw is None:
            return 0.0
        return abs(self._angdiff(self.turn_start_yaw, self.n.curr_yaw))

    def _begin_turn_once(self):
        if self.hold_timer is not None:
            try: self.hold_timer.cancel()
            except Exception: pass
            self.hold_timer = None
        if self.turn_timer is not None:
            return

        self.turn_start = self.n.get_clock().now()
        self.turn_start_yaw = self.n.curr_yaw

        if self.n.turn_method == 'odom' and self.n.curr_yaw is not None:
            self.turn_timer = self.n.create_timer(0.02, self._turn_cb_odom)
            self.n.get_logger().info(
                f"A: Turning by odom {self.n.turn_target_deg:.1f}° (timeout {self.n.turn_timeout}s)")
        else:
            self.turn_timer = self.n.create_timer(0.02, self._turn_cb_time)
            self.n.get_logger().warn("A: No /odom → time-based turn fallback.")

    def _finish_turn_and_settle(self):
        self.n.stop_cmd()
        def _after_settle():
            if self.after_turn_timer:
                try: self.after_turn_timer.cancel()
                except Exception: pass
            self.after_turn_timer = None

            if self.n.slam_skip_b1:
                self.n.phase = Phase.B
                self.n.phaseB.begin_scan()
                self.n.get_logger().info('A→B: skipping B1 (slam.skip_b1=true).')
            else:
                self.n.phase = Phase.B1
                self.n.phaseB1.begin()
                self.n.get_logger().info('A→B1: begin SLAM wait.')

        self.after_turn_timer = self.n.create_timer(self.post_turn_wait_s, _after_settle)
        self.n.get_logger().info('A: 90° turn complete → STOP and wait 1.0s...')

    def _turn_cb_odom(self):
        elapsed = (self.n.get_clock().now() - self.turn_start).nanoseconds * 1e-9
        t = Twist(); t.angular.z = self.n.turn_w; self.n.cmd.publish(t)
        if self._turned_abs() >= self.turn_target_rad or elapsed >= self.n.turn_timeout:
            try: self.turn_timer.cancel()
            except Exception: pass
            self.turn_timer = None
            self._finish_turn_and_settle()

    def _turn_cb_time(self):
        elapsed = (self.n.get_clock().now() - self.turn_start).nanoseconds * 1e-9
        if elapsed < self.n.turn_dur:
            t = Twist(); t.angular.z = self.n.turn_w; self.n.cmd.publish(t)
        else:
            try: self.turn_timer.cancel()
            except Exception: pass
            self.turn_timer = None
            self._finish_turn_and_settle()


# ============================ PART B1 ============================
class PartB1Slam:
    """Optional wait for /map (slam_toolbox)."""
    def __init__(self, n:'ExamAll'):
        self.n = n
        self.map_updates = 0
        self.started_at = None
        self.waiting = False
        self.sub_map = self.n.create_subscription(OccupancyGrid, '/map', self._on_map, 10)
        self.cli_change = None
        if self.n.slam_use_lifecycle:
            self.cli_change = self.n.create_client(ChangeState, f'/{self.n.slam_node}/change_state')

    def begin(self):
        self.map_updates = 0
        self.started_at = self.n.get_clock().now()
        self.waiting = True
        self.n.stop_cmd()

        if self.n.slam_use_lifecycle and self.cli_change is not None:
            if not self.cli_change.service_is_ready():
                self.n.get_logger().warn('B1: lifecycle service not ready; proceeding on /map.')
            else:
                try:
                    req = ChangeState.Request(); req.transition.id = Transition.TRANSITION_CONFIGURE
                    self.cli_change.call_async(req)
                    req = ChangeState.Request(); req.transition.id = Transition.TRANSITION_ACTIVATE
                    self.cli_change.call_async(req)
                except Exception as e:
                    self.n.get_logger().warn(f'B1: lifecycle activate failed: {e}')

        self.n.get_logger().info(
            f'B1: Waiting for SLAM (/map). Need {self.n.slam_min_updates} updates or {self.n.slam_max_wait}s.')

    def _on_map(self, msg: OccupancyGrid):
        if self.n.phase != Phase.B1 or not self.waiting:
            return
        if msg.info.width > 0 and msg.info.height > 0:
            self.map_updates += 1
        elapsed = (self.n.get_clock().now() - self.started_at).nanoseconds * 1e-9
        if self.map_updates >= self.n.slam_min_updates or elapsed >= self.n.slam_max_wait:
            self.waiting = False
            self.n.stop_cmd()
            self.n.get_logger().info(f'B1: SLAM ready (updates={self.map_updates}) → B.')
            self.n.phase = Phase.B
            self.n.phaseB.begin_scan()


# ============================ PART B ============================
class PartB:
    """Front-only ArUco detect → Nav2 to goals[id] (no spin)."""
    def __init__(self, n:'ExamAll'):
        self.n = n
        self.armed_id = None
        self.nav_goal_sent = False
        self.nav_timer = None

    def begin_scan(self):
        self.armed_id = None
        self.nav_goal_sent = False
        self.n.stop_cmd()
        self.n.get_logger().info('B: FRONT-ONLY ArUco detect. settle=300ms, timeout controlled by Nav2.')

    def on_image(self, msg: Image):
        if self.n.phase != Phase.B or self.nav_goal_sent:
            return

        det = self.n._detect_aruco(msg)
        if not det or det.get('id') is None:
            # front-only hold
            self.n.stop_cmd()
            return

        ar_id = int(det['id'])
        if ar_id not in self.n.goals:
            self.n.get_logger().warn(f'B: Detected ArUco id={ar_id} but no goals[{ar_id}] in params.')
            self.n.stop_cmd()
            return

        gx, gy, gyaw = self.n.goals[ar_id]
        self.n.get_logger().info(f'B: ArUco id={ar_id} → Goal from YAML: (x={gx:.2f}, y={gy:.2f}, yaw={gyaw:.2f})')
        self.armed_id = ar_id

        if not self.n.send_nav_goal(gx, gy, gyaw):
            self.n.get_logger().warn('B: Nav2 action server unavailable; skipping to C.')
            self.n.phase = Phase.C
            self.n.dock_marker_id = self.armed_id
            return

        self.nav_goal_sent = True
        self.nav_timer = self.n.create_timer(0.25, self._check_nav)
        self.n.get_logger().info(f'B: Sent Nav2 goal for id={ar_id} → ({gx:.2f}, {gy:.2f}, yaw={gyaw:.2f}).')

    def _check_nav(self):
        if self.n.nav_result_future is None:
            return
        if self.n.nav_result_future.done():
            try:
                _res = self.n.nav_result_future.result()
                self.n.get_logger().info('B: Nav2 finished.')
            except Exception as e:
                self.n.get_logger().warn(f'B: Nav2 result error: {e}')
            if self.nav_timer:
                try: self.nav_timer.cancel()
                except Exception: pass
                self.nav_timer = None
            self.n.dock_marker_id = self.armed_id
            self.n.phase = Phase.C
            self.n.phaseC.reset(self.armed_id)
            self.n.get_logger().info('B→C: starting precise 10 cm docking.')


# ----------------------------- Part C (align by ODOM first, then straight → 10 cm) -----------------------------
class PartC:
    """
    Dock to the SAME ArUco id at 0.10 m.
    Step 1: align robot yaw (from /odom) to the goal yaw for that ArUco id (from YAML/goals) — vision NOT required.
    Step 2: if marker not yet visible, go straight forward slowly ("blind") up to a timeout.
    Step 3: once marker is visible, approach to exactly 0.10 m with a small centering steer.
    """
    def __init__(self, node:'ExamAll'):
        self.n = node
        self.align_phase = True
        self.started_at = None
        self.blind_started_at = None

        # approach/steer caps
        self.forward_speed_cap = 0.10
        self.centering_gain = 0.8
        self.centering_cap  = 0.5

    def reset(self, goal_id=None):
        """Call from Part B just before switching to Phase.C."""
        if goal_id is not None:
            self.n.dock_marker_id = int(goal_id)

        # target yaw from YAML goals (fallback 0.0)
        try:
            if self.n.dock_marker_id in self.n.goals:
                _x, _y, gyaw = self.n.goals[self.n.dock_marker_id]
                self.n.dock_target_yaw = float(gyaw)
            else:
                self.n.dock_target_yaw = 0.0
        except Exception:
            self.n.dock_target_yaw = 0.0

        self.align_phase = True
        self.started_at = self.n.get_clock().now()
        self.blind_started_at = None

        self.n.get_logger().info(
            f"C: reset for id={self.n.dock_marker_id}, target_yaw={self.n.dock_target_yaw:.2f} rad"
        )

    @staticmethod
    def _angdiff(a, b):
        return (b - a + math.pi) % (2*math.pi) - math.pi

    def on_image(self, msg: Image):
        if self.n.phase != Phase.C:
            return

        # ---------------- STEP 1: ALIGN BY /ODOM (no vision needed) ----------------
        if self.align_phase:
            if self.n.curr_yaw is None:
                # can't align without odom; proceed to step 2
                self.align_phase = False
            else:
                yaw_target = float(getattr(self.n, 'dock_target_yaw', 0.0))
                yaw_err = self._angdiff(self.n.curr_yaw, yaw_target)
                if abs(yaw_err) > self.n.ang_tol:
                    t = Twist()
                    t.angular.z = float(np.clip(- self.n.d_ang_kp * yaw_err,
                                                -self.n.d_max_ang, self.n.d_max_ang))
                    t.linear.x  = 0.0
                    self.n.cmd.publish(t)
                    return
                else:
                    self.align_phase = False
                    self.n.get_logger().info("C: Odom yaw aligned to goal yaw → forward to acquire marker.")
                    self.n.stop_cmd()
                    # fall-through to step 2

        # Try to detect the marker (used by steps 2 & 3)
        det = self.n._detect_aruco(msg)

        # ---------------- STEP 2: BLIND FORWARD (if marker not yet visible) ----------------
        if (not det) or (det.get('id') is None) or (self.n.dock_marker_id is None) or (det.get('tvec') is None) \
           or int(det['id']) != int(self.n.dock_marker_id):
            # start (or continue) blind forward motion
            if self.blind_started_at is None:
                self.blind_started_at = self.n.get_clock().now()

            elapsed = (self.n.get_clock().now() - self.blind_started_at).nanoseconds * 1e-9
            if elapsed < self.n.c_blind_timeout_s:
                t = Twist()
                t.linear.x = float(np.clip(self.n.c_blind_fwd_vel, -self.forward_speed_cap, self.forward_speed_cap))
                t.angular.z = 0.0    # do NOT spin; you asked to go straight
                self.n.cmd.publish(t)
                return
            else:
                # safety: stop if we couldn't see it in time
                self.n.stop_cmd()
                return

        # ---------------- STEP 3: VISION-BASED FINAL APPROACH (10 cm) ----------------
        tvec = np.array(det['tvec'], dtype=float)
        dist = float(np.linalg.norm(tvec)) if det.get('distance') is None else float(det['distance'])
        cam_heading_err = math.atan2(tvec[0], tvec[2])  # +left/-right

        if abs(dist - self.n.d_target) <= self.n.d_tol:
            self.n.stop_cmd()
            self.n.get_logger().info(
                f"C: Dock complete at {dist:.3f} m (target {self.n.d_target:.3f})"
            )
            self.n.phase = Phase.DONE
            return

        steer = float(np.clip(- self.centering_gain * cam_heading_err,
                              -self.centering_cap, self.centering_cap))

        lin_cmd = (dist - self.n.d_target) * self.n.d_lin_kp
        t = Twist()
        t.linear.x  = float(np.clip(lin_cmd, -self.forward_speed_cap, self.forward_speed_cap))
        t.angular.z = float(np.clip(steer, -self.n.d_max_ang, self.n.d_max_ang))
        self.n.cmd.publish(t)





# ============================ ORCHESTRATOR ============================
class ExamAll(Node):
    """Owns params, pubs/subs, helpers, and orchestrates A → (B1) → B → C."""
    def __init__(self):
        super().__init__('exam_all')

        # ---- Parameters ----
        # vision
        self.declare_parameter('vision.image_topic', 'camera/image_raw')
        self.declare_parameter('vision.roi_top', 0.60)
        self.declare_parameter('vision.roi_bottom', 1.00)
        self.declare_parameter('vision.hsv_low',  [18, 70, 70])
        self.declare_parameter('vision.hsv_high', [40, 255, 255])
        self.declare_parameter('vision.morph_open', 3)
        self.declare_parameter('vision.morph_close', 5)
        self.declare_parameter('vision.min_coverage', 0.005)
        self.declare_parameter('vision.centroid_deadband_px', 8)
        self.declare_parameter('vision.show_debug', False)
        # control
        self.declare_parameter('ctrl.kp_ang', 1.3)
        self.declare_parameter('ctrl.max_ang_vel', 1.0)
        self.declare_parameter('ctrl.max_lin_vel', 0.16)
        self.declare_parameter('ctrl.search_ang_vel', 0.6)
        self.declare_parameter('ctrl.ang_ema_alpha', 0.5)
        # qr
        self.declare_parameter('qr.stop_text', 'STOP')
        self.declare_parameter('qr.min_bbox_px', 90)
        self.declare_parameter('qr.consecutive_required', 3)
        self.declare_parameter('qr.stop_on_any_qr', True)
        self.declare_parameter('qr.stop_hold_ms', 800)
        self.declare_parameter('qr.resize_factor', 1.4)
        self.declare_parameter('qr.debug_every', 15)
        self.declare_parameter('qr.use_binary_pass', True)
        self.declare_parameter('qr.use_sharpen_pass', True)
        # turn
        self.declare_parameter('turn.method', 'odom')
        self.declare_parameter('turn.target_deg', 90.0)
        self.declare_parameter('turn.timeout_s', 3.0)
        self.declare_parameter('turn.vel', -0.7)
        self.declare_parameter('turn.duration_s', 1.30)
        # slam wait (B1)
        self.declare_parameter('slam.skip_b1', True)
        self.declare_parameter('slam.use_lifecycle', False)
        self.declare_parameter('slam.node_name', 'slam_toolbox')
        self.declare_parameter('slam.min_map_updates', 0)
        self.declare_parameter('slam.max_wait_s', 0.0)
        self.declare_parameter('slam.service_wait_s', 0.0)
        # aruco
        self.declare_parameter('aruco.dictionary', 'DICT_5X5_100')
        self.declare_parameter('aruco.marker_length_m', 0.10)
        self.declare_parameter('aruco.settle_ms', 300)
        self.declare_parameter('aruco.scan_ang_vel', 0.25)
        self.declare_parameter('aruco.scan_timeout_s', 8.0)
        self.declare_parameter('aruco.min_perim_px', 40)
        self.declare_parameter('aruco.show_debug', False)
        # docking (C)
        self.declare_parameter('docking.target_distance_m', 0.10)
        self.declare_parameter('docking.distance_tol_m', 0.02)
        self.declare_parameter('docking.lin_kp', 0.6)
        self.declare_parameter('docking.ang_kp', 1.2)
        self.declare_parameter('docking.max_lin', 0.18)
        self.declare_parameter('docking.max_ang', 1.0)
        self.declare_parameter('docking.ang_tol_deg', 6.0)
        self.declare_parameter('docking.blind_fwd_vel', 0.06)   # m/s while going straight to acquire marker
        self.declare_parameter('docking.blind_timeout_s', 3.0)  # max blind forward time

        # Part C helper caps/gains
        self.declare_parameter('docking.center_kp',        0.6)   # small steering while approaching
        self.declare_parameter('docking.center_cap',       0.5)   # |rad/s| cap for steering
        self.declare_parameter('docking.forward_cap',      0.10)  # |m/s| cap for forward/back in C

        # goals.* (read from YAML)
        _ = self.get_parameters_by_prefix('goals')  # just to register prefix

        # ---- Read params ----
        gp = self.get_parameter
        # vision
        self.img_topic  = gp('vision.image_topic').value
        self.roi_top    = float(gp('vision.roi_top').value)
        self.roi_bot    = float(gp('vision.roi_bottom').value)
        self.low        = np.array(gp('vision.hsv_low').value,  np.uint8)
        self.high       = np.array(gp('vision.hsv_high').value, np.uint8)
        self.k_open     = int(gp('vision.morph_open').value)
        self.k_close    = int(gp('vision.morph_close').value)
        self.min_cov    = float(gp('vision.min_coverage').value)
        self.dead_px    = int(gp('vision.centroid_deadband_px').value)
        self.show_debug = bool(gp('vision.show_debug').value)
        # control
        self.kp_ang   = float(gp('ctrl.kp_ang').value)
        self.max_w    = float(gp('ctrl.max_ang_vel').value)
        self.max_v    = float(gp('ctrl.max_lin_vel').value)
        self.search_w = float(gp('ctrl.search_ang_vel').value)
        self.alpha    = float(gp('ctrl.ang_ema_alpha').value)
        # qr
        self.stop_text   = str(gp('qr.stop_text').value).strip().upper()
        self.min_bbox    = int(gp('qr.min_bbox_px').value)
        self.need_frames = int(gp('qr.consecutive_required').value)
        self.stop_on_any = bool(gp('qr.stop_on_any_qr').value)
        self.hold_ms     = int(gp('qr.stop_hold_ms').value)
        self.qr_resize   = float(gp('qr.resize_factor').value)
        self.qr_dbg_n    = int(gp('qr.debug_every').value)
        self.qr_use_bin  = bool(gp('qr.use_binary_pass').value)
        self.qr_use_shrp = bool(gp('qr.use_sharpen_pass').value)
        # turn
        self.turn_method     = gp('turn.method').value
        self.turn_target_deg = float(gp('turn.target_deg').value)
        self.turn_timeout    = float(gp('turn.timeout_s').value)
        self.turn_w          = float(gp('turn.vel').value)
        self.turn_dur        = float(gp('turn.duration_s').value)
        # slam
        self.slam_skip_b1        = bool(gp('slam.skip_b1').value)
        self.slam_use_lifecycle  = bool(gp('slam.use_lifecycle').value)
        self.slam_node           = str(gp('slam.node_name').value)
        self.slam_min_updates    = int(gp('slam.min_map_updates').value)
        self.slam_max_wait       = float(gp('slam.max_wait_s').value)
        self.slam_service_wait   = float(gp('slam.service_wait_s').value)
        # aruco
        self.dict_name   = gp('aruco.dictionary').value
        self.marker_len  = float(gp('aruco.marker_length_m').value)
        self.aruco_settle_ms = int(gp('aruco.settle_ms').value)
        self.aruco_scan_w    = float(gp('aruco.scan_ang_vel').value)
        self.aruco_scan_to   = float(gp('aruco.scan_timeout_s').value)
        self.aruco_min_perim = int(gp('aruco.min_perim_px').value)
        self.aruco_show_dbg  = bool(gp('aruco.show_debug').value)
        # docking
        self.d_target = float(gp('docking.target_distance_m').value)
        self.d_tol    = float(gp('docking.distance_tol_m').value)
        self.d_lin_kp = float(gp('docking.lin_kp').value)
        self.d_ang_kp = float(gp('docking.ang_kp').value)
        self.d_max_lin= float(gp('docking.max_lin').value)
        self.d_max_ang= float(gp('docking.max_ang').value)
        self.ang_tol  = math.radians(float(gp('docking.ang_tol_deg').value))
        self.c_blind_fwd_vel   = float(self.get_parameter('docking.blind_fwd_vel').value)
        self.c_blind_timeout_s = float(self.get_parameter('docking.blind_timeout_s').value)
        # new Part C helper params
        self.c_center_kp  = float(gp('docking.center_kp').value)
        self.c_center_cap = float(gp('docking.center_cap').value)
        self.c_forward_cap= float(gp('docking.forward_cap').value)
        


        # ---- Goals (defaults → overridden by YAML if present) ----
        self.goals = {
            1: [3.0,  -0.18,  0.0],
            2: [3.0,  -0.55,  0.0],
            3: [3.0, -0.90,  0.0],
            4: [3.0, -1.26,  0.0],
            5: [3.0, -1.62,  0.0],
        }
        pref = self.get_parameters_by_prefix('goals')
        if pref:
            # parameters_by_prefix returns a dict { '1': Parameter, '2': Parameter, ... }
            try:
                self.goals = {int(k): list(v.value) for k, v in pref.items()}
                self.get_logger().info(f"Loaded goals from YAML: {self.goals}")
            except Exception as e:
                self.get_logger().warn(f"Failed to parse goals.* from YAML, using defaults. err={e}")

        # ---- IO ----
        self.cv = CvBridge()
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_img  = self.create_subscription(Image, self.img_topic, self._on_image, 10)
        self.sub_cam  = self.create_subscription(CameraInfo, 'camera_info', self._on_cinfo, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self._on_odom, 50)
        
        # ArUco helper (optional)
        self.aruco = None
        if ArucoHelper is not None:
            try:
                self.aruco = ArucoHelper(self.dict_name, self.marker_len)
                self.aruco.configure(dictionary=self.dict_name,
                                     marker_length=self.marker_len,
                                     show_debug=self.aruco_show_dbg,
                                     min_perim_px=self.aruco_min_perim)
            except Exception:
                self.aruco = None

        # Nav2 client
        self.nav_ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_result_future = None

        # phases & shared state
        self.phase = Phase.A
        self.phaseA  = PartA(self)
        self.phaseB1 = PartB1Slam(self)
        self.phaseB  = PartB(self)
        self.phaseC  = PartC(self)

        self.curr_yaw = None
        self._K = None
        self._D = None

        self.a_armed = True
        self.dock_marker_id = None
        self.dock_target_yaw = None

        self.get_logger().info('ExamAll: A (path/QR/90°+1s) → (B1 optional) → B (Aruco→Nav2) → C (dock 10 cm)')

    # ---- Nav2 helper ----
    def send_nav_goal(self, x, y, yaw) -> bool:
        if not self.nav_ac.wait_for_server(timeout_sec=0.3):
            return False
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.stamp = ros_now(self)
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation = yaw_to_quat(float(yaw))

        send_future = self.nav_ac.send_goal_async(goal)

        def _on_goal_resp(fut):
            try:
                goal_handle = fut.result()
            except Exception as e:
                self.get_logger().warn(f'Nav2 send_goal failed: {e}')
                self.nav_result_future = None
                return
            if not goal_handle.accepted:
                self.get_logger().warn('Nav2 goal was rejected.')
                self.nav_result_future = None
                return
            self.nav_result_future = goal_handle.get_result_async()

        send_future.add_done_callback(_on_goal_resp)
        return True

    # ---- topic fan-out ----
    def _on_image(self, msg: Image):
        if self.phase == Phase.A:
            self.phaseA.on_image(msg)
        elif self.phase == Phase.B:
            self.phaseB.on_image(msg)
        elif self.phase == Phase.C:
            self.phaseC.on_image(msg)

    def _on_cinfo(self, msg: CameraInfo):
        self._K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self._D = np.array(msg.d, dtype=np.float64)
        try:
            if self.aruco is not None:
                self.aruco.set_camera_info(self._K, self._D)
        except Exception:
            pass

    def _on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w*q.z + q.x*q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.curr_yaw = math.atan2(siny_cosp, cosy_cosp)
        if self.phase == Phase.A:
            self.phaseA.on_odom(self.curr_yaw)

    # ---- shared helpers ----
    def stop_cmd(self):
        self.cmd.publish(Twist())

    def _detect_aruco(self, image_msg: Image):
        if self.aruco is not None:
            try:
                det = self.aruco.detect(image_msg)
                if det:
                    return det
            except Exception:
                pass
        return self._detect_aruco_cv(self.cv.imgmsg_to_cv2(image_msg, 'bgr8'))

    def _detect_aruco_cv(self, bgr_img):
        if not hasattr(cv2, 'aruco'):
            return None
        gray = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)
        try:
            dconst = getattr(cv2.aruco, str(self.dict_name))
        except Exception:
            dconst = cv2.aruco.DICT_5X5_100
        aruco_dict = cv2.aruco.getPredefinedDictionary(dconst)

        # detector params
        try:
            params = cv2.aruco.DetectorParameters()
        except Exception:
            params = cv2.aruco.DetectorParameters_create()
        if hasattr(params, 'minMarkerPerimeterRate'):
            params.minMarkerPerimeterRate = 0.03
        try:
            if hasattr(params, 'minMarkerPerimeterPixels'):
                params.minMarkerPerimeterPixels = int(max(4, self.aruco_min_perim))
        except Exception:
            pass

        try:
            if hasattr(cv2.aruco, 'ArucoDetector'):
                detector = cv2.aruco.ArucoDetector(aruco_dict, params)
                corners, ids, _ = detector.detectMarkers(gray)
            else:
                corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)
        except Exception:
            return None

        if ids is None or len(ids) == 0:
            return None

        perims = [cv2.arcLength(c.astype(np.float32), True) for c in corners]
        i = int(np.argmax(perims))
        mid = int(ids[i][0])
        c = corners[i]

        tvec = None
        distance = None
        rvecs = tvecs = None
        if self._K is not None and self.marker_len > 0:
            try:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    c, float(self.marker_len), self._K, self._D
                )
                tvec = tvecs[0][0]
                distance = float(np.linalg.norm(tvec))
            except Exception:
                pass

        return {'id': mid, 'tvec': tvec, 'distance': distance}


def main():
    rclpy.init()
    node = ExamAll()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.phaseA.destroy()
        except Exception:
            pass
        node.stop_cmd()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
