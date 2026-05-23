#!/usr/bin/env python3
"""
Precision topic frequency tester — subscribes and records per-message
timestamps to compute real publish rates with jitter analysis.
"""

import time
import sys
import argparse
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


MSG_TYPE_MAP = {
    "sensor_msgs/Imu": "sensor_msgs.msg.Imu",
    "sensor_msgs/Image": "sensor_msgs.msg.Image",
    "sensor_msgs/PointCloud2": "sensor_msgs.msg.PointCloud2",
    "nav_msgs/Odometry": "nav_msgs.msg.Odometry",
    "sensor_msgs/CompressedImage": "sensor_msgs.msg.CompressedImage",
}


def import_msg(typename: str):
    # Handle both "sensor_msgs/msg/Imu" and "sensor_msgs.msg.Imu"
    if "/" in typename:
        parts = typename.split("/")
        pkg, name = parts[0], parts[-1]
    else:
        parts = typename.split(".")
        pkg, name = parts[0], parts[-1]
    mod = __import__(f"{pkg}.msg", fromlist=[name])
    return getattr(mod, name)


class TopicTester(Node):
    def __init__(self, topic: str, msg_type_str: str, duration: float):
        super().__init__("topic_tester_" + topic.replace("/", "_").lstrip("_"))
        self._topic = topic
        self._duration = duration
        self._arrival_times: list[float] = []
        self._header_stamps: list[float] = []
        self._first_arrival = None
        self._start_time = time.time()

        MsgType = import_msg(msg_type_str)
        qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._sub = self.create_subscription(
            MsgType, topic, self._cb, qos,
        )

        self._timer = self.create_timer(duration, self._finish)

    def _cb(self, msg):
        now = time.time()
        if self._first_arrival is None:
            self._first_arrival = now
        self._arrival_times.append(now)

        # Try to extract header stamp
        hdr_stamp = None
        if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
            hdr_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._header_stamps.append(hdr_stamp)

    def _finish(self):
        if len(self._arrival_times) < 2:
            print(f"[{self._topic}] No messages received in {self._duration}s")
            raise SystemExit(1)

        intervals = [
            self._arrival_times[i] - self._arrival_times[i - 1]
            for i in range(1, len(self._arrival_times))
        ]

        total_elapsed = self._arrival_times[-1] - self._arrival_times[0]
        avg_hz = (len(self._arrival_times) - 1) / total_elapsed if total_elapsed > 0 else 0

        intervals_sorted = sorted(intervals)
        p50 = intervals_sorted[len(intervals_sorted) // 2]
        p99 = intervals_sorted[int(len(intervals_sorted) * 0.99)]
        p1 = intervals_sorted[int(len(intervals_sorted) * 0.01)] if len(intervals_sorted) > 1 else intervals_sorted[0]

        min_iv = intervals_sorted[0]
        max_iv = intervals_sorted[-1]
        avg_iv = sum(intervals) / len(intervals)
        jitter = (sum((iv - avg_iv) ** 2 for iv in intervals) / len(intervals)) ** 0.5

        # Per-second breakdown
        per_sec = {}
        base = self._arrival_times[0]
        for t in self._arrival_times:
            sec = int(t - base)
            per_sec[sec] = per_sec.get(sec, 0) + 1

        print(f"\n{'='*65}")
        print(f"  Topic: {self._topic}")
        print(f"  Duration: {total_elapsed:.2f}s | Messages: {len(self._arrival_times)}")
        print(f"{'='*65}")
        print(f"  Average rate:     {avg_hz:>10.3f} Hz")
        print(f"  P50 interval:     {p50*1000:>10.3f} ms  ({1/p50:.1f} Hz)")
        print(f"  P1  interval:     {p1*1000:>10.3f} ms")
        print(f"  P99 interval:     {p99*1000:>10.3f} ms")
        print(f"  Min  interval:    {min_iv*1000:>10.3f} ms")
        print(f"  Max  interval:    {max_iv*1000:>10.3f} ms")
        print(f"  Mean interval:    {avg_iv*1000:>10.3f} ms")
        print(f"  Jitter (std):     {jitter*1000:>10.3f} ms")
        print(f"  First arrival:    {self._arrival_times[0] - self._start_time:>10.3f}s after start")

        # Header vs arrival
        valid_headers = [(h, a) for h, a in zip(self._header_stamps, self._arrival_times)
                         if h is not None and h > 0]
        if valid_headers:
            hdr_delays = [a - h for h, a in valid_headers]
            print(f"  Header latency:   {min(hdr_delays)*1000:.3f} ~ {max(hdr_delays)*1000:.3f} ms")
            if len(valid_headers) > 3:
                hdr_intervals = [
                    valid_headers[i][0] - valid_headers[i-1][0]
                    for i in range(1, len(valid_headers))
                ]
                hdr_hz = (len(valid_headers) - 1) / (valid_headers[-1][0] - valid_headers[0][0])
                print(f"  Header stamp Hz:  {hdr_hz:>10.3f} Hz")

        print(f"\n  Per-second breakdown:")
        for sec in sorted(per_sec.keys()):
            bar = "#" * min(per_sec[sec], 60)
            print(f"    s{sec:>3d}: {per_sec[sec]:>5d} msgs  {bar}")

        print(f"{'='*65}\n")
        raise SystemExit(0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("topic", help="Topic name")
    parser.add_argument("--type", "-t", default="sensor_msgs/Imu",
                        help="Message type (e.g. sensor_msgs/Imu)")
    parser.add_argument("--duration", "-d", type=float, default=10.0,
                        help="Test duration in seconds")
    args = parser.parse_args()

    if args.type in MSG_TYPE_MAP:
        msg_type = MSG_TYPE_MAP[args.type]
    else:
        msg_type = args.type

    rclpy.init()
    node = TopicTester(args.topic, msg_type, args.duration)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
