#!/usr/bin/python3
"""
Record raw IMU data from a ROS2 topic to a text file for Allan variance analysis.

Place the sensor on a stable, vibration-free surface before recording.
A 1-2 hour recording is recommended to resolve bias random walk parameters.
"""

import argparse
import os
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuRecorder(Node):
    def __init__(self, topic: str, output_path: str, duration: float):
        super().__init__("imu_static_recorder")
        self._output_path = output_path
        self._duration = duration
        self._start_time = None
        self._first_stamp = None
        self._count = 0
        self._last_report = 0
        self._file = None
        self._done = False

        self._sub = self.create_subscription(
            Imu, topic, self._callback, 1000
        )
        self._timer = self.create_timer(0.5, self._timer_cb)

        self.get_logger().info(
            f"Recording from '{topic}' -> '{output_path}' for {duration}s"
        )
        self.get_logger().info(
            "Waiting for IMU messages... (sensor must be publishing)"
        )

    def _callback(self, msg: Imu):
        if self._done:
            return

        if self._start_time is None:
            self._start_time = time.time()
            self._first_stamp = (
                msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            )
            self._file = open(self._output_path, "w")
            self._last_report = self._start_time
            self.get_logger().info(
                f"First IMU message received. Recording for {self._duration}s..."
            )

        t = (
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        ) - self._first_stamp

        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        self._file.write(
            f"{t:.9f} {gx:.9f} {gy:.9f} {gz:.9f} {ax:.9f} {ay:.9f} {az:.9f}\n"
        )
        self._count += 1

        now = time.time()
        if now - self._last_report >= 60:
            elapsed = now - self._start_time
            remaining = self._duration - elapsed
            self.get_logger().info(
                f"Recorded {self._count} samples in {elapsed:.0f}s "
                f"({self._count / elapsed:.1f} Hz)  "
                f"remaining: {remaining:.0f}s"
            )
            self._last_report = now
            self._file.flush()
            os.fsync(self._file.fileno())

    def _timer_cb(self):
        if self._done:
            return

        if self._start_time is None:
            # No data received yet — just wait
            return

        elapsed = time.time() - self._start_time
        if elapsed >= self._duration:
            self.get_logger().info(
                f"Duration reached. Total: {self._count} samples "
                f"in {elapsed:.0f}s."
            )
            self._done = True
            if self._file:
                self._file.flush()
                os.fsync(self._file.fileno())
                self._file.close()
                self._file = None
            self.get_logger().info(
                f"Data saved to {os.path.abspath(self._output_path)}"
            )
            raise SystemExit(0)


def main():
    parser = argparse.ArgumentParser(
        description="Record static IMU data for Allan variance calibration"
    )
    parser.add_argument(
        "--topic", default="/odin1/imu",
        help="IMU topic (default: /odin1/imu)"
    )
    parser.add_argument(
        "--duration", type=float, default=3600,
        help="Recording duration in seconds (default: 3600 = 1 hour)"
    )
    parser.add_argument(
        "--output", default="imu_static.txt",
        help="Output file path (default: imu_static.txt)"
    )
    args = parser.parse_args()

    rclpy.init(args=sys.argv)
    node = ImuRecorder(args.topic, args.output, args.duration)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node._file:
            node._file.flush()
            os.fsync(node._file.fileno())
            node._file.close()
            node._file = None
        node.get_logger().info(
            f"Interrupted. {node._count} samples saved to "
            f"{os.path.abspath(args.output)}"
        )
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
