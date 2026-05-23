#!/usr/bin/env python3
import argparse
import shutil
import struct
from pathlib import Path

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image, Imu
from livox_ros_driver2.msg import CustomMsg, CustomPoint


OP_CHUNK = 0x05
OP_CONNECTION = 0x07
OP_MSG_DATA = 0x02


def read_ros1_header(stream):
    raw = stream.read(4)
    if len(raw) < 4:
        return None, None
    header_len = struct.unpack_from("<I", raw)[0]
    header_data = stream.read(header_len)
    fields = {}
    offset = 0
    while offset < len(header_data):
        field_len = struct.unpack_from("<I", header_data, offset)[0]
        offset += 4
        field = header_data[offset:offset + field_len]
        offset += field_len
        key, value = field.split(b"=", 1)
        fields[key.decode()] = value
    data_len = struct.unpack_from("<I", stream.read(4))[0]
    return fields, data_len


class Cursor:
    def __init__(self, data):
        self.data = data
        self.offset = 0

    def u8(self):
        value = self.data[self.offset]
        self.offset += 1
        return value

    def u32(self):
        value = struct.unpack_from("<I", self.data, self.offset)[0]
        self.offset += 4
        return value

    def u64(self):
        value = struct.unpack_from("<Q", self.data, self.offset)[0]
        self.offset += 8
        return value

    def f32(self):
        value = struct.unpack_from("<f", self.data, self.offset)[0]
        self.offset += 4
        return value

    def f64(self):
        value = struct.unpack_from("<d", self.data, self.offset)[0]
        self.offset += 8
        return value

    def string(self):
        size = self.u32()
        value = self.data[self.offset:self.offset + size].decode(errors="replace")
        self.offset += size
        return value

    def bytes(self):
        size = self.u32()
        value = self.data[self.offset:self.offset + size]
        self.offset += size
        return value


def read_ros1_stamp(value):
    sec, nsec = struct.unpack_from("<II", value)
    return sec, nsec


def read_std_header(cursor):
    cursor.u32()
    sec = cursor.u32()
    nsec = cursor.u32()
    frame_id = cursor.string()
    return sec, nsec, frame_id


def set_header(msg, sec, nsec, frame_id):
    msg.header.stamp.sec = int(sec)
    msg.header.stamp.nanosec = int(nsec)
    msg.header.frame_id = frame_id


def parse_imu(data):
    c = Cursor(data)
    sec, nsec, frame_id = read_std_header(c)
    msg = Imu()
    set_header(msg, sec, nsec, frame_id)
    msg.orientation.x = c.f64()
    msg.orientation.y = c.f64()
    msg.orientation.z = c.f64()
    msg.orientation.w = c.f64()
    msg.orientation_covariance = [c.f64() for _ in range(9)]
    msg.angular_velocity.x = c.f64()
    msg.angular_velocity.y = c.f64()
    msg.angular_velocity.z = c.f64()
    msg.angular_velocity_covariance = [c.f64() for _ in range(9)]
    msg.linear_acceleration.x = c.f64()
    msg.linear_acceleration.y = c.f64()
    msg.linear_acceleration.z = c.f64()
    msg.linear_acceleration_covariance = [c.f64() for _ in range(9)]
    return msg


def parse_livox(data):
    c = Cursor(data)
    sec, nsec, frame_id = read_std_header(c)
    msg = CustomMsg()
    set_header(msg, sec, nsec, frame_id)
    msg.timebase = c.u64()
    msg.point_num = c.u32()
    msg.lidar_id = c.u8()
    msg.rsvd = [c.u8(), c.u8(), c.u8()]
    point_count = c.u32()
    msg.points = []
    for _ in range(point_count):
        point = CustomPoint()
        point.offset_time = c.u32()
        point.x = c.f32()
        point.y = c.f32()
        point.z = c.f32()
        point.reflectivity = c.u8()
        point.tag = c.u8()
        point.line = c.u8()
        msg.points.append(point)
    return msg


def parse_compressed_image_as_raw(data, raw_image_encoding):
    c = Cursor(data)
    sec, nsec, frame_id = read_std_header(c)
    c.string()
    encoded = np.frombuffer(c.bytes(), dtype=np.uint8)
    imread_flag = cv2.IMREAD_GRAYSCALE if raw_image_encoding == "mono8" else cv2.IMREAD_COLOR
    decoded = cv2.imdecode(encoded, imread_flag)
    if decoded is None:
        raise RuntimeError("failed to decode compressed image")
    msg = Image()
    set_header(msg, sec, nsec, frame_id)
    msg.height = decoded.shape[0]
    msg.width = decoded.shape[1]
    msg.encoding = raw_image_encoding
    msg.is_bigendian = 0
    msg.step = int(decoded.shape[1] if raw_image_encoding == "mono8" else decoded.shape[1] * decoded.shape[2])
    msg.data = decoded.tobytes()
    return msg


def parse_raw_image(data):
    c = Cursor(data)
    sec, nsec, frame_id = read_std_header(c)
    msg = Image()
    set_header(msg, sec, nsec, frame_id)
    msg.height = c.u32()
    msg.width = c.u32()
    msg.encoding = c.string()
    msg.is_bigendian = c.u8()
    msg.step = c.u32()
    msg.data = c.bytes()
    return msg


def iter_ros1_messages(path):
    connections = read_connections(path)
    with Path(path).open("rb") as bag:
        magic = bag.readline()
        if magic.strip() != b"#ROSBAG V2.0":
            raise RuntimeError("not a ROS1 bag v2 file")
        while True:
            header, data_len = read_ros1_header(bag)
            if header is None:
                break
            op = header.get("op", b"\x00")[0]
            data = bag.read(data_len)
            if op == OP_CHUNK:
                if header.get("compression", b"none") != b"none":
                    raise RuntimeError("compressed ROS1 chunks are not supported by this converter")
                chunk = Cursor(data)
                while chunk.offset < len(data):
                    rec_header, rec_data_len = read_ros1_header_from_cursor(chunk)
                    rec_data = chunk.data[chunk.offset:chunk.offset + rec_data_len]
                    chunk.offset += rec_data_len
                    if rec_header.get("op", b"\x00")[0] != OP_MSG_DATA:
                        continue
                    conn = struct.unpack_from("<I", rec_header["conn"])[0]
                    sec, nsec = read_ros1_stamp(rec_header["time"])
                    yield connections[conn], sec * 1_000_000_000 + nsec, rec_data


def read_connections(path):
    connections = {}
    with Path(path).open("rb") as bag:
        magic = bag.readline()
        if magic.strip() != b"#ROSBAG V2.0":
            raise RuntimeError("not a ROS1 bag v2 file")
        while True:
            header, data_len = read_ros1_header(bag)
            if header is None:
                break
            op = header.get("op", b"\x00")[0]
            if op == OP_CONNECTION:
                conn = struct.unpack_from("<I", header["conn"])[0]
                connections[conn] = header["topic"].decode()
            bag.seek(data_len, 1)
    return connections


def read_ros1_header_from_cursor(cursor):
    header_len = cursor.u32()
    header_data = cursor.data[cursor.offset:cursor.offset + header_len]
    cursor.offset += header_len
    fields = {}
    offset = 0
    while offset < len(header_data):
        field_len = struct.unpack_from("<I", header_data, offset)[0]
        offset += 4
        field = header_data[offset:offset + field_len]
        offset += field_len
        key, value = field.split(b"=", 1)
        fields[key.decode()] = value
    data_len = cursor.u32()
    return fields, data_len


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input")
    parser.add_argument("output")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument(
        "--raw-image-encoding",
        choices=("bgr8", "mono8"),
        default="bgr8",
        help="Encoding used when converting compressed camera frames to sensor_msgs/Image.",
    )
    args = parser.parse_args()

    output = Path(args.output)
    if output.exists():
        if not args.overwrite:
            raise RuntimeError(f"{output} exists; pass --overwrite")
        shutil.rmtree(output)

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(output), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    writer.create_topic(rosbag2_py.TopicMetadata("/livox/imu", "sensor_msgs/msg/Imu", "cdr"))
    writer.create_topic(rosbag2_py.TopicMetadata("/livox/lidar", "livox_ros_driver2/msg/CustomMsg", "cdr"))
    writer.create_topic(rosbag2_py.TopicMetadata("/left_camera/image", "sensor_msgs/msg/Image", "cdr"))

    first_time = None
    counts = {"/livox/imu": 0, "/livox/lidar": 0, "/left_camera/image": 0}
    for topic, t_ns, data in iter_ros1_messages(args.input):
        if first_time is None:
            first_time = t_ns
        if args.duration > 0 and (t_ns - first_time) > int(args.duration * 1e9):
            break
        if topic == "/livox/imu":
            out_topic, msg = topic, parse_imu(data)
        elif topic == "/livox/lidar":
            out_topic, msg = topic, parse_livox(data)
        elif topic == "/left_camera/image":
            out_topic, msg = topic, parse_raw_image(data)
        elif topic == "/left_camera/image/compressed":
            out_topic, msg = "/left_camera/image", parse_compressed_image_as_raw(data, args.raw_image_encoding)
        else:
            continue
        writer.write(out_topic, serialize_message(msg), t_ns)
        counts[out_topic] += 1

    print(counts)


if __name__ == "__main__":
    main()
