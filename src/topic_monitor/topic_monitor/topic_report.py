#!/usr/bin/env python3
"""One-shot ROS2 topic diagnostics report for Odin + FAST-LIVO2 runs."""

import json
import math
import os
import struct
import time
from datetime import datetime
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


TOPICS_CONFIG = [
    ("/odin1/imu", "sensor_msgs/Imu", 400.0, "Odin input"),
    ("/odin1/cloud_raw", "sensor_msgs/PointCloud2", 10.0, "Odin input"),
    ("/odin1/image/undistorted", "sensor_msgs/Image", 10.0, "Odin input"),
    ("/aft_mapped_to_init", "nav_msgs/Odometry", 0.0, "FAST-LIVO2"),
    ("/cloud_registered", "sensor_msgs/PointCloud2", 0.0, "FAST-LIVO2"),
    ("/Laser_map", "sensor_msgs/PointCloud2", 0.0, "FAST-LIVO2"),
]


def import_msg(typename: str):
    if "/" in typename:
        parts = typename.split("/")
        pkg, name = parts[0], parts[-1]
    else:
        parts = typename.split(".")
        pkg, name = parts[0], parts[-1]
    mod = __import__(f"{pkg}.msg", fromlist=[name])
    return getattr(mod, name)


def percentile(values: List[float], pct: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(values)
    idx = int(round((len(ordered) - 1) * pct / 100.0))
    return ordered[max(0, min(idx, len(ordered) - 1))]


def intervals(values: List[float]) -> List[float]:
    return [values[i] - values[i - 1] for i in range(1, len(values))]


def stamp_to_sec(msg) -> Optional[float]:
    if isinstance(msg, (bytes, bytearray, memoryview)):
        return stamp_to_sec_from_cdr(msg)
    if not hasattr(msg, "header") or not hasattr(msg.header, "stamp"):
        return None
    stamp = msg.header.stamp
    value = stamp.sec + stamp.nanosec * 1e-9
    return value if value > 0.0 else None


def stamp_to_sec_from_cdr(msg) -> Optional[float]:
    data = memoryview(msg)
    if len(data) < 12:
        return None
    # ROS2 CDR payload starts with a 4-byte encapsulation header. All monitored
    # messages have std_msgs/Header as their first field, so stamp follows it.
    little_endian = data[1] == 1
    fmt = "<iI" if little_endian else ">iI"
    try:
        sec, nanosec = struct.unpack_from(fmt, data, 4)
    except struct.error:
        return None
    value = sec + nanosec * 1e-9
    return value if value > 0.0 else None


def nearest_abs_delta_ms(source: List[float], target: List[float]) -> Optional[Dict[str, float]]:
    if not source or not target:
        return None
    target_sorted = sorted(target)
    deltas = []
    j = 0
    for value in sorted(source):
        while j + 1 < len(target_sorted) and abs(target_sorted[j + 1] - value) <= abs(target_sorted[j] - value):
            j += 1
        deltas.append(abs(target_sorted[j] - value) * 1000.0)
    return {
        "p50_ms": percentile(deltas, 50.0),
        "p95_ms": percentile(deltas, 95.0),
        "p99_ms": percentile(deltas, 99.0),
        "max_ms": max(deltas),
    }


class TopicReport(Node):
    def __init__(self):
        super().__init__("topic_report")
        self.declare_parameter("output_dir", "/tmp/fast_livo_topic_reports")
        self.declare_parameter("report_name", "")
        self.declare_parameter("duration_s", 0.0)
        self.declare_parameter("warn_ratio", 0.8)

        self.output_dir = self.get_parameter("output_dir").value
        self.report_name = self.get_parameter("report_name").value
        self.duration_s = float(self.get_parameter("duration_s").value)
        self.warn_ratio = float(self.get_parameter("warn_ratio").value)
        self.done = False
        self.start_wall = time.time()
        self.start_iso = datetime.now().isoformat(timespec="seconds")
        self.stats: Dict[str, Dict] = {}

        qos = QoSProfile(
            depth=200,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        for topic, typename, expected_hz, category in TOPICS_CONFIG:
            self.stats[topic] = {
                "type": typename,
                "category": category,
                "expected_hz": expected_hz,
                "arrival_times": [],
                "header_stamps": [],
            }
            msg_type = import_msg(typename)
            self.create_subscription(msg_type, topic, lambda msg, t=topic: self._cb(t, msg), qos, raw=True)

        if self.duration_s > 0.0:
            self.create_timer(self.duration_s, self._finish)

    def _cb(self, topic: str, msg):
        stat = self.stats[topic]
        stat["arrival_times"].append(time.time())
        header_stamp = stamp_to_sec(msg)
        if header_stamp is not None:
            stat["header_stamps"].append(header_stamp)

    def _finish(self):
        self.done = True

    def build_report(self) -> Dict:
        end_wall = time.time()
        topics = {}
        for topic, stat in self.stats.items():
            arrivals = stat["arrival_times"]
            headers = stat["header_stamps"]
            arrival_intervals = intervals(arrivals)
            header_intervals = intervals(headers)
            elapsed = arrivals[-1] - arrivals[0] if len(arrivals) >= 2 else 0.0
            header_elapsed = headers[-1] - headers[0] if len(headers) >= 2 else 0.0
            hz = (len(arrivals) - 1) / elapsed if elapsed > 0.0 else 0.0
            header_hz = (len(headers) - 1) / header_elapsed if header_elapsed > 0.0 else 0.0
            expected_hz = stat["expected_hz"]
            status = "ok"
            if expected_hz > 0.0:
                if hz <= 0.0:
                    status = "missing"
                elif hz < expected_hz * self.warn_ratio:
                    status = "low_rate"

            topics[topic] = {
                "type": stat["type"],
                "category": stat["category"],
                "expected_hz": expected_hz,
                "status": status,
                "count": len(arrivals),
                "hz": hz,
                "header_hz": header_hz,
                "arrival_interval_ms": self._interval_summary(arrival_intervals),
                "header_interval_ms": self._interval_summary(header_intervals),
                "first_arrival_offset_s": arrivals[0] - self.start_wall if arrivals else None,
                "last_arrival_offset_s": arrivals[-1] - self.start_wall if arrivals else None,
            }

        stamps = {topic: stat["header_stamps"] for topic, stat in self.stats.items()}
        sync = {
            "cloud_to_nearest_imu": nearest_abs_delta_ms(stamps["/odin1/cloud_raw"], stamps["/odin1/imu"]),
            "image_to_nearest_imu": nearest_abs_delta_ms(stamps["/odin1/image/undistorted"], stamps["/odin1/imu"]),
            "image_to_nearest_cloud": nearest_abs_delta_ms(stamps["/odin1/image/undistorted"], stamps["/odin1/cloud_raw"]),
        }
        return {
            "started_at": self.start_iso,
            "ended_at": datetime.now().isoformat(timespec="seconds"),
            "duration_s": end_wall - self.start_wall,
            "topics": topics,
            "sync": sync,
        }

    @staticmethod
    def _interval_summary(values_s: List[float]) -> Dict[str, Optional[float]]:
        if not values_s:
            return {"p50": None, "p95": None, "p99": None, "max": None}
        values_ms = [value * 1000.0 for value in values_s]
        return {
            "p50": percentile(values_ms, 50.0),
            "p95": percentile(values_ms, 95.0),
            "p99": percentile(values_ms, 99.0),
            "max": max(values_ms),
        }

    def write_report(self) -> Tuple[str, str]:
        report = self.build_report()
        os.makedirs(self.output_dir, exist_ok=True)
        base = self.report_name.strip() or "fast_livo_topic_report_" + datetime.now().strftime("%Y%m%d_%H%M%S")
        json_path = os.path.join(self.output_dir, base + ".json")
        md_path = os.path.join(self.output_dir, base + ".md")
        with open(json_path, "w", encoding="utf-8") as handle:
            json.dump(report, handle, indent=2)
        with open(md_path, "w", encoding="utf-8") as handle:
            handle.write(self._markdown(report))
        return json_path, md_path

    @staticmethod
    def _markdown(report: Dict) -> str:
        lines = [
            "# FAST-LIVO2 Topic Diagnostics",
            "",
            f"- Started: `{report['started_at']}`",
            f"- Ended: `{report['ended_at']}`",
            f"- Duration: `{report['duration_s']:.1f}s`",
            "",
            "## Topics",
            "",
            "| Topic | Count | Hz | Header Hz | Status | Arrival p95 ms | Header p95 ms |",
            "|---|---:|---:|---:|---|---:|---:|",
        ]
        for topic, stat in report["topics"].items():
            arrival_p95 = stat["arrival_interval_ms"]["p95"]
            header_p95 = stat["header_interval_ms"]["p95"]
            lines.append(
                f"| `{topic}` | {stat['count']} | {stat['hz']:.3f} | {stat['header_hz']:.3f} | "
                f"{stat['status']} | {format_optional(arrival_p95)} | {format_optional(header_p95)} |"
            )
        lines.extend(["", "## Synchronization", ""])
        for name, sync in report["sync"].items():
            if sync is None:
                lines.append(f"- `{name}`: no data")
            else:
                lines.append(
                    f"- `{name}`: p95 `{sync['p95_ms']:.3f} ms`, p99 `{sync['p99_ms']:.3f} ms`, max `{sync['max_ms']:.3f} ms`"
                )
        lines.append("")
        return "\n".join(lines)


def format_optional(value: Optional[float]) -> str:
    if value is None or (isinstance(value, float) and math.isnan(value)):
        return ""
    return f"{value:.3f}"


def main():
    rclpy.init()
    node = TopicReport()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        json_path, md_path = node.write_report()
        print(f"Topic report written to {json_path} and {md_path}", flush=True)
        os._exit(0)


if __name__ == "__main__":
    main()
