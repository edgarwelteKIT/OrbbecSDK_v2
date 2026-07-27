#!/usr/bin/env python3
"""ROS2 bridge client for ob_femto_proxy ZeroMQ stream."""

import argparse
import json
import time
import zlib

import numpy as np
import zmq

import rclpy
from builtin_interfaces.msg import Time as RosTime
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header


def decode_payload(header, payload):
    if header["encoding"] == "zlib":
        payload = zlib.decompress(payload)

    frame_type = header["type"]
    if frame_type == "depth":
        return np.frombuffer(payload, dtype=np.uint16).reshape((header["height"], header["width"]))
    if frame_type == "color":
        return np.frombuffer(payload, dtype=np.uint8).reshape((header["height"], header["width"], header["channels"]))
    if frame_type == "pointcloud":
        return np.frombuffer(payload, dtype=np.float32).reshape((-1, 3))
    return payload


def to_ros_time(ts_us: int) -> RosTime:
    t = RosTime()
    t.sec = int(ts_us // 1_000_000)
    t.nanosec = int((ts_us % 1_000_000) * 1000)
    return t


class ProxyRos2Bridge(Node):
    def __init__(self, args):
        super().__init__("ob_femto_proxy_ros2_bridge")
        self.args = args
        self.depth_pub = self.create_publisher(Image, args.depth_topic, 10)
        self.color_pub = self.create_publisher(Image, args.color_topic, 10)
        self.pc_pub = self.create_publisher(PointCloud2, args.pointcloud_topic, 10)
        self.counters = {"depth": 0, "color": 0, "pointcloud": 0}
        self.last_report = time.time()

    def make_header(self, frame_id: str, ts_us: int) -> Header:
        h = Header()
        h.frame_id = frame_id
        h.stamp = to_ros_time(ts_us)
        return h

    def publish_depth(self, header, depth):
        msg = Image()
        msg.header = self.make_header(self.args.depth_frame_id, int(header.get("system_ts_us", 0)))
        msg.height = int(header["height"])
        msg.width = int(header["width"])
        msg.encoding = "16UC1"
        msg.is_bigendian = 0
        msg.step = msg.width * 2
        msg.data = depth.tobytes()
        self.depth_pub.publish(msg)

    def publish_color(self, header, color):
        msg = Image()
        msg.header = self.make_header(self.args.color_frame_id, int(header.get("system_ts_us", 0)))
        msg.height = int(header["height"])
        msg.width = int(header["width"])
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = color.tobytes()
        self.color_pub.publish(msg)

    def publish_pointcloud(self, header, points):
        msg = PointCloud2()
        msg.header = self.make_header(self.args.pointcloud_frame_id, int(header.get("system_ts_us", 0)))
        msg.height = 1
        msg.width = int(points.shape[0])
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = False
        msg.data = np.asarray(points, dtype=np.float32).tobytes()
        self.pc_pub.publish(msg)

    def maybe_report(self, topic, header, point_count):
        now_us = int(time.time() * 1e6)
        latency_ms = (now_us - int(header.get("system_ts_us", now_us))) / 1000.0
        if time.time() - self.last_report >= 1.0:
            self.get_logger().info(
                f"topic={topic} seq={header.get('seq')} latency_ms={latency_ms:.2f} "
                f"counters={self.counters} points={point_count}"
            )
            self.last_report = time.time()


def main():
    parser = argparse.ArgumentParser(description="Subscribe to ob_femto_proxy and republish as ROS2 topics")
    parser.add_argument("--endpoint", default="tcp://127.0.0.1:5555", help="Proxy PUB endpoint")
    parser.add_argument("--topics", default="depth,color,pointcloud", help="Comma separated ZMQ topics")
    parser.add_argument("--depth-topic", default="/orbbec/depth/image_raw", help="ROS2 depth image topic")
    parser.add_argument("--color-topic", default="/orbbec/color/image_raw", help="ROS2 color image topic")
    parser.add_argument("--pointcloud-topic", default="/orbbec/pointcloud", help="ROS2 point cloud topic")
    parser.add_argument("--depth-frame-id", default="orbbec_depth_optical_frame", help="Depth frame_id")
    parser.add_argument("--color-frame-id", default="orbbec_color_optical_frame", help="Color frame_id")
    parser.add_argument("--pointcloud-frame-id", default="orbbec_depth_optical_frame", help="Point cloud frame_id")
    args = parser.parse_args()

    topics = [t.strip() for t in args.topics.split(",") if t.strip()]

    context = zmq.Context()
    sock = context.socket(zmq.SUB)
    sock.setsockopt(zmq.RCVHWM, 1)
    sock.connect(args.endpoint)
    for topic in topics:
        sock.setsockopt_string(zmq.SUBSCRIBE, topic)

    rclpy.init()
    node = ProxyRos2Bridge(args)

    try:
        while rclpy.ok():
            topic_b, header_b, payload = sock.recv_multipart()
            topic = topic_b.decode("utf-8")
            header = json.loads(header_b.decode("utf-8"))
            decoded = decode_payload(header, payload)
            node.counters[topic] = node.counters.get(topic, 0) + 1

            point_count = 0
            if topic == "depth":
                node.publish_depth(header, decoded)
            elif topic == "color":
                node.publish_color(header, decoded)
            elif topic == "pointcloud":
                point_count = decoded.shape[0]
                node.publish_pointcloud(header, decoded)

            node.maybe_report(topic, header, point_count)
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        sock.close(0)
        context.term()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
