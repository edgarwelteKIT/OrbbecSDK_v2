#!/usr/bin/env python3
"""Example ZeroMQ client for ob_femto_proxy."""

import argparse
import json
import time
import zlib

import numpy as np
import zmq


def decode_payload(header, payload):
    if header["encoding"] == "zlib":
        payload = zlib.decompress(payload)

    frame_type = header["type"]
    if frame_type == "depth":
        arr = np.frombuffer(payload, dtype=np.uint16).reshape((header["height"], header["width"]))
        return arr
    if frame_type == "color":
        arr = np.frombuffer(payload, dtype=np.uint8).reshape((header["height"], header["width"], header["channels"]))
        return arr
    if frame_type == "pointcloud":
        arr = np.frombuffer(payload, dtype=np.float32).reshape((-1, 3))
        return arr
    return payload


def main():
    parser = argparse.ArgumentParser(description="Subscribe to ob_femto_proxy stream")
    parser.add_argument("--endpoint", default="tcp://127.0.0.1:5555", help="Proxy PUB endpoint")
    parser.add_argument("--topics", default="depth,color,pointcloud", help="Comma separated topics")
    parser.add_argument("--show", action="store_true", help="Show depth and color windows if OpenCV is installed")
    args = parser.parse_args()

    topics = [t.strip() for t in args.topics.split(",") if t.strip()]

    context = zmq.Context()
    sock = context.socket(zmq.SUB)
    sock.setsockopt(zmq.RCVHWM, 1)
    sock.connect(args.endpoint)

    for topic in topics:
        sock.setsockopt_string(zmq.SUBSCRIBE, topic)

    cv2 = None
    if args.show:
        try:
            import cv2  # type: ignore
        except Exception:
            print("OpenCV not installed, disabling --show")

    counters = {"depth": 0, "color": 0, "pointcloud": 0}
    last_report = time.time()

    while True:
        topic_b, header_b, payload = sock.recv_multipart()
        topic = topic_b.decode("utf-8")
        header = json.loads(header_b.decode("utf-8"))
        decoded = decode_payload(header, payload)
        counters[topic] = counters.get(topic, 0) + 1

        now_us = int(time.time() * 1e6)
        latency_ms = (now_us - int(header.get("system_ts_us", now_us))) / 1000.0

        if topic == "depth" and cv2 is not None:
            depth = decoded
            depth_vis = np.clip(depth / 8, 0, 255).astype(np.uint8)
            cv2.imshow("depth", depth_vis)
            cv2.waitKey(1)
        elif topic == "color" and cv2 is not None:
            color = decoded[:, :, ::-1]  # RGB -> BGR
            cv2.imshow("color", color)
            cv2.waitKey(1)

        if time.time() - last_report >= 1.0:
            if topic == "pointcloud":
                point_count = decoded.shape[0]
            else:
                point_count = 0
            print(
                f"topic={topic:10s} seq={header.get('seq')} latency_ms={latency_ms:7.2f} "
                f"size={header.get('payload_bytes')}/{header.get('raw_bytes')} "
                f"counters={counters} points={point_count}"
            )
            last_report = time.time()


if __name__ == "__main__":
    main()
