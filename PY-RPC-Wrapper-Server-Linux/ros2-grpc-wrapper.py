import signal
import threading
import os
from concurrent import futures
import cv2
import numpy as np
import json as _json
from http.server import BaseHTTPRequestHandler, HTTPServer
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from cv_bridge import CvBridge

import grpc
import sys
_PROTOS = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'generated_protos')
sys.path.insert(1, _PROTOS)
import rpc_demo_pb2
import rpc_demo_pb2_grpc

HTTP_PORT = 7043


class RPCDemoImpl(rpc_demo_pb2_grpc.RPCDemoServicer):
    def __init__(self, node):
        self.node = node
        self.lock = threading.Lock()
        self.data = [0.0, 0.0, 0.0]
        self.img_compressed = None
        self.shape = None
        self.br = CvBridge()
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0

        self.node.create_subscription(Odometry, '/odom', self.update_odom, 10)
        self.node.create_subscription(Image, '/image_result', self.update_image, 10)
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        print("Initialized gRPC Server")

    def update_odom(self, msg):
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        with self.lock:
            self.data[0] = msg.pose.pose.position.x
            self.data[1] = msg.pose.pose.position.y
            self.data[2] = yaw

    def update_image(self, msg):
        img_original = self.br.imgmsg_to_cv2(msg)
        self.shape = img_original.shape
        compressed = np.array(
            cv2.imencode('.jpg', img_original, [cv2.IMWRITE_JPEG_QUALITY, 75])[1]
        ).tobytes()
        with self.lock:
            self.img_compressed = compressed

    def publish_cmd_vel(self):
        twist = Twist()
        with self.lock:
            twist.linear.x = self.cmd_linear
            twist.angular.z = self.cmd_angular
        self.cmd_vel_pub.publish(twist)

    def GetMultCoords(self, request, context):
        results = rpc_demo_pb2.MultCoords()
        with self.lock:
            results.values.extend(self.data)
        return results

    def GetImageResult(self, request, context):
        results = rpc_demo_pb2.ImageResult()
        with self.lock:
            if self.img_compressed is not None:
                results.b64img = self.img_compressed
                results.width = self.shape[1]
                results.height = self.shape[0]
        return results


class WrapperHTTPHandler(BaseHTTPRequestHandler):
    service = None

    def do_GET(self):
        if self.path == '/image':
            with self.service.lock:
                frame = self.service.img_compressed
            if frame is None:
                self.send_response(503)
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                return
            self.send_response(200)
            self.send_header('Content-Type', 'image/jpeg')
            self.send_header('Content-Length', str(len(frame)))
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(frame)

        elif self.path == '/odom':
            with self.service.lock:
                d = list(self.service.data)
            body = _json.dumps({
                "x": d[0],
                "y": d[1],
                "theta": d[2] if len(d) > 2 else 0.0
            }).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(body)

        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        if self.path == '/cmd_vel':
            length = int(self.headers.get('Content-Length', 0))
            body = _json.loads(self.rfile.read(length) or b'{}')
            with self.service.lock:
                self.service.cmd_linear = float(body.get('linear', 0))
                self.service.cmd_angular = float(body.get('angular', 0))
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(b'{"ok":true}')
        else:
            self.send_response(404)
            self.end_headers()

    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()

    def log_message(self, format, *args):
        pass


terminate = threading.Event()


def terminate_server(signum, frame):
    print(f"Got signal {signum}")
    rclpy.shutdown()
    terminate.set()


def main(args=None):
    print("----ROS-gRPC-Wrapper----")
    signal.signal(signal.SIGINT, terminate_server)

    print("Starting ROS Node")
    rclpy.init(args=args)
    node = rclpy.create_node('object_position_wrapper')

    print("Starting gRPC Server")
    service = RPCDemoImpl(node)
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    rpc_demo_pb2_grpc.add_RPCDemoServicer_to_server(service, server)
    server.add_insecure_port("[::]:7042")
    server.start()
    print("gRPC Server listening on [::]:7042")

    WrapperHTTPHandler.service = service
    http_server = HTTPServer(('0.0.0.0', HTTP_PORT), WrapperHTTPHandler)
    http_thread = threading.Thread(target=http_server.serve_forever, daemon=True)
    http_thread.start()
    print(f"HTTP Server listening on port {HTTP_PORT}")

    def publish_loop():
        while not terminate.is_set():
            service.publish_cmd_vel()
            time.sleep(0.05)  # 20 Hz

    threading.Thread(target=publish_loop, daemon=True).start()

    print("Running ROS Node")
    executor = futures.ThreadPoolExecutor()
    executor.submit(lambda: rclpy.spin(node))

    terminate.wait()
    print("Stopping...")
    server.stop(1).wait()
    http_server.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    print("Exited")


if __name__ == "__main__":
    main()
