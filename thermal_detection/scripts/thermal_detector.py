#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped
import numpy as np

class ThermalDetector(Node):
    def __init__(self):
        super().__init__('thermal_detector')

        # Parameters (tunable)
        self.declare_parameter('use_mono8', False)        # True -> use /parrot/thermal/image_mono8
        self.declare_parameter('kelvin_threshold', 320.0)  # Only for mono16 (L16)
        self.declare_parameter('counts_per_kelvin', 100.0) # L16 scale guess; adjust to calibrate
        self.declare_parameter('mono8_threshold', 180)     # Only for mono8 pathway
        self.declare_parameter('min_hot_pixels', 200)      # noise filter

        use_mono8 = self.get_parameter('use_mono8').value
        topic = '/parrot/thermal/image_mono8' if use_mono8 else '/parrot/thermal/image_raw'
        self.get_logger().info(f'Subscribing to {topic}')

        self.sub = self.create_subscription(Image, topic, self.cb, 10)
        self.pub_detected = self.create_publisher(Bool, '/hotspot_detected', 10)
        self.pub_centroid = self.create_publisher(PointStamped, '/hotspot_centroid', 10)

    def cb(self, msg: Image):
        try:
            use_mono8 = self.get_parameter('use_mono8').value

            if use_mono8:
                if msg.encoding not in ('mono8', '8UC1'):
                    self.get_logger().warn_once(f'Unexpected encoding: {msg.encoding} (expected mono8)')
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
                thr = int(self.get_parameter('mono8_threshold').value)
                mask = img >= thr
            else:
                if msg.encoding not in ('mono16', '16UC1'):
                    self.get_logger().warn_once(f'Unexpected encoding: {msg.encoding} (expected mono16)')
                img = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
                k_thr = float(self.get_parameter('kelvin_threshold').value)
                cppk = float(self.get_parameter('counts_per_kelvin').value)
                thr_counts = int(k_thr * cppk)
                mask = img >= thr_counts

            hot_pixels = int(mask.sum())
            min_hot = int(self.get_parameter('min_hot_pixels').value)
            detected = hot_pixels >= min_hot

            self.pub_detected.publish(Bool(data=detected))

            if detected:
                ys, xs = np.nonzero(mask)
                cx, cy = float(xs.mean()), float(ys.mean())
                pt = PointStamped()
                pt.header.stamp = self.get_clock().now().to_msg()
                pt.header.frame_id = msg.header.frame_id or 'thermal_cam'
                pt.point.x, pt.point.y, pt.point.z = cx, cy, 0.0
                self.pub_centroid.publish(pt)

                if use_mono8:
                    self.get_logger().info(f'HOTSPOT: pixels={hot_pixels}, mono8_thr={thr}, centroid=({cx:.1f},{cy:.1f})')
                else:
                    cppk = float(self.get_parameter('counts_per_kelvin').value)
                    ambient_K = (int(np.median(img)) / cppk) if cppk > 0 else 0.0
                    self.get_logger().info(
                        f'HOTSPOT: pixels={hot_pixels}, K_thr={k_thr}, centroid=({cx:.1f},{cy:.1f}), ambient~{ambient_K:.1f}K'
                    )

        except Exception as e:
            self.get_logger().error(f'Processing error: {e}')

def main():
    rclpy.init()
    node = ThermalDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

