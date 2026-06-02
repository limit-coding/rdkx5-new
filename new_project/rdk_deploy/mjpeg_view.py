#!/usr/bin/env python3
"""
轻量 MJPEG 预览服务器
用法: python3 mjpeg_view.py --port 8080 --fps 20
浏览器打开: http://<RDK_IP>:8080
停止: Ctrl+C
"""
import argparse
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

_frame = None
_lock = threading.Lock()
_quality = 70
_delay = 0.08


class _Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            body = (
                b"<html><head><title>RDK Camera</title></head>"
                b"<body style='margin:0;background:#111'>"
                b"<img src='/stream' style='width:100vw;height:100vh;object-fit:contain'>"
                b"</body></html>"
            )
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=f")
            self.end_headers()
            try:
                while True:
                    with _lock:
                        frame = _frame
                    if frame is not None:
                        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, _quality])
                        if ok:
                            data = buf.tobytes()
                            self.wfile.write(
                                b"--f\r\nContent-Type: image/jpeg\r\n"
                                b"Content-Length: " + str(len(data)).encode() + b"\r\n\r\n"
                                + data + b"\r\n"
                            )
                    time.sleep(_delay)
            except Exception:
                pass

    def log_message(self, *_):
        pass


def get_lan_ips():
    ips = []
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ip = info[4][0]
            if ip not in ips and not ip.startswith("127."):
                ips.append(ip)
    except Exception:
        pass
    return ips


def main():
    global _frame
    global _quality
    global _delay

    parser = argparse.ArgumentParser(description="Serve ROS camera topic as MJPEG over HTTP.")
    parser.add_argument("--image-topic", default="/image")
    parser.add_argument("--raw-topic", default="/image_raw")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--fps", type=float, default=20.0)
    parser.add_argument("--quality", type=int, default=75)
    args = parser.parse_args()

    _quality = max(10, min(95, int(args.quality)))
    _delay = 1.0 / max(1.0, float(args.fps))

    rclpy.init()
    node = Node("mjpeg_view")

    # 尝试 CompressedImage（JPEG）
    def on_compressed(msg):
        global _frame
        arr = np.frombuffer(bytes(msg.data), np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is not None:
            with _lock:
                _frame = img

    # 尝试 raw Image（NV12 或 BGR8）
    def on_raw(msg):
        global _frame
        try:
            arr = np.frombuffer(bytes(msg.data), np.uint8)
            if msg.encoding in ("nv12", "NV12"):
                img = cv2.cvtColor(
                    arr.reshape(msg.height * 3 // 2, msg.width),
                    cv2.COLOR_YUV2BGR_NV12,
                )
            elif msg.encoding in ("bgr8", "rgb8"):
                img = arr.reshape(msg.height, msg.width, 3)
                if msg.encoding == "rgb8":
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            else:
                return
            with _lock:
                _frame = img
        except Exception:
            pass

    node.create_subscription(CompressedImage, args.image_topic, on_compressed, 1)
    node.create_subscription(Image, args.raw_topic, on_raw, 1)

    threading.Thread(
        target=lambda: HTTPServer((args.host, args.port), _Handler).serve_forever(),
        daemon=True,
    ).start()
    print(f"订阅: CompressedImage {args.image_topic}, raw Image {args.raw_topic}")
    print(f"MJPEG: host={args.host} port={args.port} fps={args.fps:g} quality={_quality}")
    for ip in get_lan_ips():
        print(f"浏览器打开: http://{ip}:{args.port}")
    print("Ctrl+C 停止")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
