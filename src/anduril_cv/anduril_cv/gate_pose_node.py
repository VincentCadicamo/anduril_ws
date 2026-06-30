#!/usr/bin/env python3
"""
gate_pose_node.py — single-stage YOLO-Pose gate detection + PnP pose estimation.

Subscribes: /cam0/image_raw  (sensor_msgs/Image, bgr8 or mono8)
Publishes:
  gate/pose   (geometry_msgs/PoseWithCovarianceStamped, frame_id: cam0)
  gate/debug  (sensor_msgs/Image, bgr8)

Coordinate convention for gate/pose
  The position is expressed in cam0 frame using FLU axes:
    +X  forward  (camera optical Z)
    +Y  left     (−camera optical X)
    +Z  up       (−camera optical Y)
  The orientation is the gate frame relative to cam0 (from PnP rvec).
  Covariance scales with distance^2 (σ_pos ≈ 5 cm at 1 m, 50 cm at 10 m).

Camera intrinsics default to spec §3.8 values (fx=fy=320, cx=320, cy=180,
no distortion) — overridable via ROS parameters.
Gate object points default to the outer gate corners ±1.35 m (spec §3.7,
2700 mm outer square) — overridable via the gate_size parameter.
"""

from __future__ import annotations

import cv2
import numpy as np
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image as SensorImage
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge

import onnxruntime as ort


_BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# Corner colors for debug overlay (BGR): TL, TR, BR, BL
_CORNER_COLORS = [(0, 0, 255), (255, 0, 0), (0, 255, 255), (255, 0, 255)]


def _rvec_to_quat(rvec: np.ndarray) -> tuple[float, float, float, float]:
    """Convert a Rodrigues rotation vector to (x, y, z, w) quaternion."""
    R, _ = cv2.Rodrigues(rvec)
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return float(x), float(y), float(z), float(w)


class GatePoseNode(Node):
    """Detect the nearest gate and publish its 6-DOF pose in camera frame."""

    def __init__(self) -> None:
        super().__init__("gate_pose_node")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("model_path", "")
        self.declare_parameter("image_topic", "/cam0/image_raw")
        self.declare_parameter("detect_conf", 0.5)
        self.declare_parameter("keypoint_conf", 0.5)
        self.declare_parameter("gate_size", 2.7)   # outer gate width (m), spec §3.7
        # Camera intrinsics — spec §3.8 defaults
        self.declare_parameter("fx", 320.0)
        self.declare_parameter("fy", 320.0)
        self.declare_parameter("cx", 320.0)
        self.declare_parameter("cy", 180.0)

        p = self.get_parameter
        model_path   = p("model_path").value
        image_topic  = p("image_topic").value
        self._det_conf = float(p("detect_conf").value)
        self._kpt_conf = float(p("keypoint_conf").value)
        gate_size    = float(p("gate_size").value)
        fx = float(p("fx").value)
        fy = float(p("fy").value)
        cx = float(p("cx").value)
        cy = float(p("cy").value)

        # ── Camera matrix (no distortion, spec §3.8) ──────────────────────────
        self._K    = np.array([[fx, 0.0, cx],
                               [0.0, fy, cy],
                               [0.0, 0.0, 1.0]], dtype=np.float32)
        self._dist = np.zeros((4, 1), dtype=np.float32)

        # ── 3D gate model: outer corners TL→TR→BR→BL in gate plane ───────────
        # RATM/TII keypoint order matches IPPE_SQUARE point order.
        half = gate_size / 2.0
        self._obj_pts = np.array([
            [-half, -half, 0.0],  # 0: TL
            [ half, -half, 0.0],  # 1: TR
            [ half,  half, 0.0],  # 2: BR
            [-half,  half, 0.0],  # 3: BL
        ], dtype=np.float32)

        # ── ONNX Runtime session ──────────────────────────────────────────────
        if not model_path:
            self.get_logger().fatal("model_path parameter must be set")
            raise RuntimeError("model_path not set")
        self.get_logger().info(f"Loading YOLO-Pose ONNX model: {model_path}")
        sess_opts = ort.SessionOptions()
        sess_opts.intra_op_num_threads = 4
        sess_opts.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        sess_opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        self._session = ort.InferenceSession(
            model_path,
            sess_options=sess_opts,
            providers=["CUDAExecutionProvider", "CPUExecutionProvider"],
        )
        self.get_logger().info(
            f"ONNX Runtime active provider: {self._session.get_providers()[0]}"
        )
        inp = self._session.get_inputs()[0]
        self._input_name = inp.name
        _, _, self._model_h, self._model_w = inp.shape  # e.g. 384, 640
        self._canvas = np.zeros((self._model_h, self._model_w, 3), dtype=np.uint8)

        self._bridge = CvBridge()

        # ── Publishers ────────────────────────────────────────────────────────
        self._pose_pub  = self.create_publisher(PoseWithCovarianceStamped, "gate/pose", 10)
        self._debug_pub = self.create_publisher(SensorImage, "gate/debug", 10)

        # ── Background inference thread ───────────────────────────────────────
        self._pending_frame = None   # (bgr, header) or None
        self._frame_lock = threading.Lock()
        self._frame_event = threading.Event()
        self._inference_thread = threading.Thread(target=self._inference_loop, daemon=True)
        self._inference_thread.start()

        # ── Subscriber ────────────────────────────────────────────────────────
        self.create_subscription(SensorImage, image_topic, self._on_image, _BEST_EFFORT_QOS)
        self.get_logger().info(f"Subscribed to {image_topic}")

    # ── Image callback ────────────────────────────────────────────────────────

    def _on_image(self, msg: SensorImage) -> None:
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"Image conversion failed: {exc}")
            return
        with self._frame_lock:
            self._pending_frame = (bgr, msg.header)
        self._frame_event.set()

    # ── Background inference loop ─────────────────────────────────────────────

    def _inference_loop(self) -> None:
        while rclpy.ok():
            self._frame_event.wait()
            self._frame_event.clear()
            with self._frame_lock:
                item = self._pending_frame
                self._pending_frame = None
            if item is None:
                continue
            bgr, header = item

            publish_debug = self._debug_pub.get_subscription_count() > 0
            debug = bgr.copy() if publish_debug else None
            tensor, pad_top, pad_left = self._preprocess(bgr)
            raw = self._session.run(None, {self._input_name: tensor})
            boxes, kpts_xy, kpts_conf = self._decode_detections(
                raw[0], pad_top, pad_left, self._det_conf
            )

            if boxes is None:
                if publish_debug:
                    self._publish_debug(debug, header)
                continue

            best_idx = self._annotate_and_pick(boxes, kpts_xy, kpts_conf, debug)
            if best_idx is not None:
                self._solve_and_publish(
                    kpts_xy[best_idx], kpts_conf[best_idx],
                    boxes[best_idx], header, debug)

            if publish_debug:
                self._publish_debug(debug, header)

    # ── Preprocessing / postprocessing ───────────────────────────────────────

    def _preprocess(self, bgr: np.ndarray) -> tuple[np.ndarray, int, int]:
        h, w = bgr.shape[:2]
        pad_top  = (self._model_h - h) // 2
        pad_left = (self._model_w - w) // 2
        self._canvas[:] = 0
        self._canvas[pad_top:pad_top + h, pad_left:pad_left + w] = bgr
        rgb = self._canvas[:, :, ::-1].astype(np.float32) / 255.0
        tensor = rgb.transpose(2, 0, 1)[np.newaxis]  # [1, 3, H, W]
        return tensor, pad_top, pad_left

    def _decode_detections(self, raw, pad_top, pad_left, conf_thresh):
        dets = raw[0]  # [300, 18]
        mask = dets[:, 4] >= conf_thresh
        dets = dets[mask]
        if len(dets) == 0:
            return None, None, None
        boxes = dets[:, :4].copy()
        boxes[:, 0] -= pad_left
        boxes[:, 1] -= pad_top
        boxes[:, 2] -= pad_left
        boxes[:, 3] -= pad_top
        kpts = dets[:, 6:].reshape(-1, 4, 3)
        kpts_xy = kpts[:, :, :2].copy()
        kpts_xy[:, :, 0] -= pad_left
        kpts_xy[:, :, 1] -= pad_top
        kpts_conf = kpts[:, :, 2]
        return boxes, kpts_xy, kpts_conf

    # ── Gate selection ────────────────────────────────────────────────────────

    def _annotate_and_pick(self, boxes, kpts_xy, kpts_conf, debug):
        """Draw all detections; return index of highest-confidence gate."""
        best_idx, best_score = None, -1.0
        for i in range(len(boxes)):
            if debug is not None:
                x1, y1, x2, y2 = map(int, boxes[i][:4])
                cv2.rectangle(debug, (x1, y1), (x2, y2), (0, 255, 0), 2)
                for k in range(min(4, kpts_xy.shape[1])):
                    px, py = int(kpts_xy[i][k][0]), int(kpts_xy[i][k][1])
                    cv2.circle(debug, (px, py), 4, _CORNER_COLORS[k], -1)

            if kpts_conf.shape[1] >= 4 and np.all(kpts_conf[i, :4] >= self._kpt_conf):
                score = float(kpts_conf[i, :4].mean())
                if score > best_score:
                    best_score = score
                    best_idx = i
        return best_idx

    # ── PnP + publish ─────────────────────────────────────────────────────────

    def _solve_and_publish(self, kpts_xy, kpts_conf, box, header, debug) -> None:
        img_pts = kpts_xy[:4].astype(np.float32)
        obj_pts = self._obj_pts
        confident = kpts_conf[:4] >= self._kpt_conf

        if not np.all(confident):
            # Partial PnP fallback — need ≥ 3 good keypoints
            if confident.sum() < 3:
                return
            img_pts = img_pts[confident]
            obj_pts = obj_pts[confident]
            pnp_flags = cv2.SOLVEPNP_SQPNP
        else:
            # IPPE_SQUARE: optimal closed-form solver for square planar targets
            pnp_flags = cv2.SOLVEPNP_IPPE_SQUARE

        ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, self._K, self._dist, flags=pnp_flags)
        if not ok:
            return

        x_c, y_c, z_c = tvec.flatten()
        qx, qy, qz, qw = _rvec_to_quat(rvec)
        dist_m = float(np.linalg.norm(tvec))

        # σ_pos grows with range (5 cm at 1 m, 50 cm at 10 m)
        pos_var = max(0.0025, 5e-3 * dist_m ** 2)

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = header          # use image timestamp, not wall clock
        pose_msg.header.frame_id = "cam0"
        # Convert from OpenCV camera frame (X right, Y down, Z fwd) to FLU cam0:
        pose_msg.pose.pose.position.x = float(z_c)    # forward
        pose_msg.pose.pose.position.y = float(-x_c)   # left
        pose_msg.pose.pose.position.z = float(-y_c)   # up
        pose_msg.pose.pose.orientation.x = qx
        pose_msg.pose.pose.orientation.y = qy
        pose_msg.pose.pose.orientation.z = qz
        pose_msg.pose.pose.orientation.w = qw
        # 6×6 covariance (row-major); only position diagonal set
        pose_msg.pose.covariance[0]  = pos_var   # x–x
        pose_msg.pose.covariance[7]  = pos_var   # y–y
        pose_msg.pose.covariance[14] = pos_var   # z–z
        pose_msg.pose.covariance[21] = 0.1        # rx–rx (orientation, constant)
        pose_msg.pose.covariance[28] = 0.1        # ry–ry
        pose_msg.pose.covariance[35] = 0.1        # rz–rz
        self._pose_pub.publish(pose_msg)

        if debug is not None:
            cv2.drawFrameAxes(debug, self._K, self._dist, rvec, tvec, 0.5)
            x1, y1 = int(box[0]), int(box[1])
            cv2.putText(
                debug,
                f"d={dist_m:.1f}m ({x_c:.2f},{y_c:.2f},{z_c:.2f})",
                (x1, max(18, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1,
            )

    # ── Debug image ───────────────────────────────────────────────────────────

    def _publish_debug(self, img, header) -> None:
        try:
            msg = self._bridge.cv2_to_imgmsg(img, encoding="bgr8")
            msg.header = header
            self._debug_pub.publish(msg)
        except Exception as exc:
            self.get_logger().error(f"Debug publish failed: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = GatePoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
