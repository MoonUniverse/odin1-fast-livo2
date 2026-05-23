#!/usr/bin/env python3
"""
Real-time ROS2 topic frequency monitor for odin + fast-livo system.
Displays per-topic publish rates with color-coded health indicators.
"""

import time
import sys
from collections import defaultdict
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


# (topic, type, expected_hz, category)
TOPICS_CONFIG = [
    # === Odin: critical input for SLAM ===
    ("/odin1/cloud_raw",       "sensor_msgs/PointCloud2",   100,  "LiDAR"),
    ("/odin1/imu",             "sensor_msgs/Imu",           400,  "IMU"),
    ("/odin1/image/undistorted","sensor_msgs/Image",          0,  "Camera"),
    # === Odin: secondary output ===
    ("/odin1/odometry",        "nav_msgs/Odometry",           0,  "Odin Out"),
    ("/odin1/odometry_highfreq","nav_msgs/Odometry",          0,  "Odin Out"),
    ("/odin1/wiwc",            "nav_msgs/Odometry",           0,  "Odin Out"),
    ("/odin1/cloud_slam",      "sensor_msgs/PointCloud2",     0,  "Odin Out"),
    ("/odin1/cloud_render",    "sensor_msgs/PointCloud2",     0,  "Odin Out"),
    # === Fast-LIVO: odometry & mapping ===
    ("/aft_mapped_to_init",    "nav_msgs/Odometry",           0,  "LIVO"),
    ("/cloud_registered",      "sensor_msgs/PointCloud2",     0,  "LIVO"),
    ("/Laser_map",             "sensor_msgs/PointCloud2",     0,  "LIVO"),
]


def import_msg_type(type_str):
    parts = type_str.split("/")
    pkg, name = parts[0], parts[2] if len(parts) > 2 else parts[1]
    mod = __import__(f"{pkg}.msg", fromlist=[name])
    return getattr(mod, name)


class TopicRateMonitor(Node):
    def __init__(self):
        super().__init__("topic_rate_monitor")

        self.declare_parameter("expected_topics", [t[0] for t in TOPICS_CONFIG])
        self.declare_parameter("report_interval", 2.0)

        self.report_interval = self.get_parameter("report_interval").value
        self.topic_stats: Dict[str, Dict] = {}
        self.topic_config_map: Dict[str, Tuple[int, str]] = {}

        # Build subscriber per topic
        for topic, msg_type_str, expected_hz, category in TOPICS_CONFIG:
            self.topic_config_map[topic] = (expected_hz, category)
            self.topic_stats[topic] = {
                "count": 0, "last_ts": None,
                "hz": 0.0, "category": category,
                "expected": expected_hz,
            }
            try:
                MsgType = import_msg_type(msg_type_str)
            except Exception:
                self.get_logger().warn(f"Cannot import {msg_type_str}, using generic sub")
                MsgType = None

            qos = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            # Generic subscriber: automatically deserialize any message
            if MsgType is not None:
                self.create_subscription(
                    MsgType, topic,
                    lambda msg, t=topic: self._cb(t),
                    qos,
                )
            else:
                from rclpy.generic_subscription import GenericSubscription
                # fallback: raw bytes counting
                self.create_generic_subscription(
                    topic, lambda msg: None, qos,
                )

        self._timer = self.create_timer(self.report_interval, self._report)
        self._start_time = time.time()

    def _cb(self, topic: str):
        now = time.time()
        stats = self.topic_stats.get(topic)
        if stats is None:
            return
        stats["count"] += 1
        if stats["last_ts"] is None:
            stats["last_ts"] = now

    def _compute_hz(self, stats: Dict) -> float:
        if stats["last_ts"] is None:
            return 0.0
        elapsed = time.time() - stats["last_ts"]
        if elapsed <= 0:
            return 0.0
        return stats["count"] / elapsed

    def _health(self, stats: Dict) -> str:
        expected = stats["expected"]
        hz = stats["hz"]
        if expected == 0:
            return "  "  # no target
        if hz <= 0:
            return "\033[31m!!\033[0m"  # red: dead
        ratio = hz / expected
        if ratio >= 0.8:
            return "\033[32mOK\033[0m"  # green
        elif ratio >= 0.5:
            return "\033[33m~~\033[0m"  # yellow
        else:
            return "\033[31m!!\033[0m"  # red

    def _report(self):
        # Compute HZ before printing
        for topic, stats in self.topic_stats.items():
            stats["hz"] = self._compute_hz(stats)

        # Clear screen every 10 reports
        elapsed = time.time() - self._start_time
        sys.stdout.write("\033[2J\033[H")  # clear + home
        print(f"=== Topic Rate Monitor | Uptime {elapsed:.0f}s | "
              f"Interval {self.report_interval}s ===\n")
        print(f"{'Topic':<35s} {'Hz':>9s} {'Health':>8s}  Category")
        print("-" * 75)

        for topic, stats in self.topic_stats.items():
            health = self._health(stats)
            print(f"{topic:<35s} {stats['hz']:8.1f}  {health:>15s}  "
                  f"{stats['category']:>10s}")

        # Summary
        total_msgs = sum(s["count"] for s in self.topic_stats.values())
        print(f"\nTotal messages received: {total_msgs}")

        # Reset counters for next window
        for stats in self.topic_stats.values():
            stats["count"] = 0
            stats["last_ts"] = time.time()


def main():
    rclpy.init()
    node = TopicRateMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
