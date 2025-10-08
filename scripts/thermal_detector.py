#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import Image                      # incoming image
from sensor_msgs.msg import Image as ImageMsg          # debug mask output
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PointStamped
from rclpy.qos import qos_profile_sensor_data


class ThermalDetector(Node):
    def __init__(self):
        super().__init__('thermal_detector')

        # ---- Launch-friendly params (yours, kept)
        self.declare_parameter('source_topic', '')
        self.declare_parameter('threshold_K', 320.0)
        self.declare_parameter('min_hot_px', 200)
        self.declare_parameter('publish_debug', False)
        self.declare_parameter('alert_topic', '/hotspot_detected')

        # ---- Legacy/compat (yours, kept)
        self.declare_parameter('use_mono8', False)
        self.declare_parameter('kelvin_threshold', 320.0)
        self.declare_parameter('counts_per_kelvin', 0.0)   # <=0 => auto-infer
        self.declare_parameter('mono8_threshold', 180)
        self.declare_parameter('min_hot_pixels', 200)

        # ---- New: "looking at" gate + optional text alerts
        self.declare_parameter('center_radius_frac', 0.25)  # 25% of min(w,h)
        self.declare_parameter('min_center_px', 200)
        self.declare_parameter('alert_text_topic', '')      # '' => disabled; else publish String

        # Map aliases to your legacy names so existing launch still works
        if self.get_parameter('min_hot_px').value is not None:
            self.set_parameters([rclpy.parameter.Parameter(
                'min_hot_pixels', rclpy.parameter.Parameter.Type.INTEGER,
                int(self.get_parameter('min_hot_px').value)
            )])
        if self.get_parameter('threshold_K').value is not None:
            self.set_parameters([rclpy.parameter.Parameter(
                'kelvin_threshold', rclpy.parameter.Parameter.Type.DOUBLE,
                float(self.get_parameter('threshold_K').value)
            )])

        # Choose source topic
        src = self.get_parameter('source_topic').value
        if src:
            topic = src
        else:
            use_mono8 = bool(self.get_parameter('use_mono8').value)
            topic = '/parrot/thermal/image_mono8' if use_mono8 else '/parrot/thermal/image_raw'

        # Publishers (yours, kept)
        alert_topic = self.get_parameter('alert_topic').value or '/hotspot_detected'
        self.pub_detected = self.create_publisher(Bool, alert_topic, 10)
        self.pub_centroid = self.create_publisher(PointStamped, '/hotspot_centroid', 10)

        # Optional: text alert publisher
        alert_text_topic = self.get_parameter('alert_text_topic').value
        self.pub_text = (self.create_publisher(String, alert_text_topic, 10)
                         if alert_text_topic else None)

        # Optional debug mask (yours, kept)
        self.publish_debug = bool(self.get_parameter('publish_debug').value)
        self.pub_mask = self.create_publisher(ImageMsg, '/parrot/thermal/hot_mask', 1) if self.publish_debug else None

        # State
        self._warned_mono8 = False
        self._warned_mono16 = False
        self._prev_detected = None
        self._prev_looking = None  # new

        # Subscribe
        self.sub = self.create_subscription(Image, topic, self.cb, qos_profile_sensor_data)
        self.get_logger().info(f'ThermalDetector listening on {topic} → alerts on {alert_topic}'
                               + (f' (text → {alert_text_topic})' if self.pub_text else ''))

    # ---- Heuristic: infer counts-per-Kelvin for L16 streams
    @staticmethod
    def _infer_counts_per_kelvin(img16: np.ndarray) -> int:
        """
        Many Gazebo thermal streams encode ~Kelvin, or Kelvin*10, *100.
        Use image median to guess a reasonable scale.
        """
        med = float(np.median(img16))
        # Ambient ~ 280K. Choose scale so 280K maps near the median.
        if med > 8000:          # ~ 280K * 30
            return 100
        if med > 800:           # ~ 280K * 3
            return 10
        return 1

    def cb(self, msg: Image):
        try:
            use_mono8 = bool(self.get_parameter('use_mono8').value)
            min_hot = int(self.get_parameter('min_hot_pixels').value)

            # ---- Decode image
            if use_mono8:
                if msg.encoding not in ('mono8', '8UC1') and not self._warned_mono8:
                    self.get_logger().warn(f'Unexpected encoding: {msg.encoding} (expected mono8/8UC1)')
                    self._warned_mono8 = True
                img8 = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
                thr8 = int(self.get_parameter('mono8_threshold').value)
                mask = img8 >= thr8
            else:
                if msg.encoding in ('mono16', '16UC1', 'L16'):
                    img16 = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
                    k_thr = float(self.get_parameter('kelvin_threshold').value)
                    cppk  = float(self.get_parameter('counts_per_kelvin').value)  # <=0 => auto
                    if cppk <= 0.0:
                        cppk = self._infer_counts_per_kelvin(img16)
                    thr_counts = int(k_thr * cppk + 0.5)
                    mask = img16 >= thr_counts
                elif msg.encoding in ('mono8', '8UC1', 'L8'):
                    img8 = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
                    thr8 = int(self.get_parameter('mono8_threshold').value)
                    mask = img8 >= thr8
                    if not self._warned_mono16:
                        self.get_logger().warn('Source is mono8; using mono8_threshold path.')
                        self._warned_mono16 = True
                else:
                    self.get_logger().warn(f'Unsupported encoding: {msg.encoding}')
                    return

            # ---- Detection + centroid (yours, kept)
            hot_pixels = int(mask.sum())
            detected = hot_pixels >= min_hot

            # Centroid if detected
            cx = cy = None
            if detected:
                ys, xs = np.nonzero(mask)
                # Guard: if all True (edge case), np.nonzero returns full grid correctly
                if xs.size > 0:
                    cx, cy = float(xs.mean()), float(ys.mean())
                    pt = PointStamped()
                    pt.header.stamp = self.get_clock().now().to_msg()
                    pt.header.frame_id = msg.header.frame_id or 'thermal_cam'
                    pt.point.x, pt.point.y, pt.point.z = cx, cy, 0.0
                    self.pub_centroid.publish(pt)

            # ---- "Looking at" gate (new)
            looking = False
            if detected and cx is not None and cy is not None:
                h, w = msg.height, msg.width
                r = float(self.get_parameter('center_radius_frac').value) * min(w, h)
                min_center_px = int(self.get_parameter('min_center_px').value)
                dx = cx - (w * 0.5)
                dy = cy - (h * 0.5)
                centre_ok = (dx*dx + dy*dy) <= (r*r)
                # Largest blob area approx by hot_pixels if one dominant blob; keep simple
                looking = centre_ok and (hot_pixels >= min_center_px)

            # ---- Publish + log (keep your Bool; add text on "looking")
            self.pub_detected.publish(Bool(data=detected))

            if looking != self._prev_looking:
                if looking:
                    msg_txt = f'HOTSPOT IN VIEW: pixels={hot_pixels}, centroid=({cx:.1f},{cy:.1f})'
                    self.get_logger().info(msg_txt)
                    if self.pub_text:
                        self.pub_text.publish(String(data=msg_txt))
                else:
                    self.get_logger().info('Hotspot not centred / not in view')
                    if self.pub_text:
                        self.pub_text.publish(String(data=''))
                self._prev_looking = looking

            # Keep your original “detected” edge logging too (less verbose)
            if detected != self._prev_detected:
                self.get_logger().info('Detected hot area' if detected else 'No thermal target in FOV')
                self._prev_detected = detected

            # ---- Optional debug mask (yours, kept)
            if self.publish_debug and self.pub_mask:
                out = ImageMsg()
                out.header = msg.header
                out.height = msg.height
                out.width  = msg.width
                out.encoding = 'mono8'
                out.is_bigendian = 0
                out.step = msg.width
                out.data = (mask.astype(np.uint8) * 255).tobytes()
                self.pub_mask.publish(out)

        except Exception as e:
            self.get_logger().error(f'Processing error: {e}')


def main():
    rclpy.init()
    node = ThermalDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
