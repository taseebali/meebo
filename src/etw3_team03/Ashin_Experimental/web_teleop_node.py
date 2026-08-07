#!/usr/bin/env python3
import os
import sys
import time
import json
import threading
import cv2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn

# Global state
latest_twist = {"linear": 0.0, "angular": 0.0}
last_cmd_time = time.time()
camera = None
camera_lock = threading.Lock()

# Initialize Camera with minimal buffer
def get_camera_frame():
    global camera
    with camera_lock:
        if camera is None or not camera.isOpened():
            camera = cv2.VideoCapture(0)
            camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        # Flush stale buffer
        for _ in range(2):
            camera.grab()
        ret, frame = camera.retrieve()
        
        if not ret or frame is None:
            return None
        
        # Encode as JPEG
        ret, jpeg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 65])
        return jpeg.tobytes() if ret else None


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


class WebTeleopHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        return  # Suppress HTTP logging clutter

    def do_GET(self):
        global last_cmd_time
        path = self.path.split('?')[0]
        
        if path == '/' or path == '/index.html':
            self.serve_file('static/index.html', 'text/html')
        elif path == '/style.css':
            self.serve_file('static/style.css', 'text/css')
        elif path == '/app.js':
            self.serve_file('static/app.js', 'application/javascript')
        elif path == '/video_feed':
            self.serve_mjpeg()
        else:
            self.send_error(404, "File Not Found")

    def do_POST(self):
        global latest_twist, last_cmd_time
        if self.path == '/api/cmd_vel':
            length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(length).decode('utf-8')
            try:
                data = json.loads(body)
                latest_twist["linear"] = float(data.get("linear", 0.0))
                latest_twist["angular"] = float(data.get("angular", 0.0))
                last_cmd_time = time.time()
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(b'{"status":"ok"}')
            except Exception as e:
                self.send_error(400, str(e))
        elif self.path == '/api/estop':
            latest_twist["linear"] = 0.0
            latest_twist["angular"] = 0.0
            last_cmd_time = 0.0  # Trigger immediate stop
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(b'{"status":"estop_triggered"}')
        else:
            self.send_error(404)

    def serve_file(self, rel_path, content_type):
        base_dir = os.path.dirname(os.path.abspath(__file__))
        full_path = os.path.join(base_dir, rel_path)
        if os.path.exists(full_path):
            self.send_response(200)
            self.send_header('Content-Type', content_type)
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            with open(full_path, 'rb') as f:
                self.wfile.write(f.read())
        else:
            self.send_error(404)

    def serve_mjpeg(self):
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        
        while True:
            try:
                frame_bytes = get_camera_frame()
                if frame_bytes:
                    self.wfile.write(b'--frame
')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', str(len(frame_bytes)))
                    self.end_headers()
                    self.wfile.write(frame_bytes)
                    self.wfile.write(b'
')
                time.sleep(0.033)  # ~30 FPS
            except (ConnectionResetError, BrokenPipeError):
                break
            except Exception as e:
                time.sleep(0.1)


class WebTeleopROSNode(Node):
    def __init__(self):
        super().__init__('web_teleop_node')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.publish_cmd)  # 20 Hz
        self.get_logger().info("Web Teleop ROS 2 Node initialized. Publishing to /cmd_vel")

    def publish_cmd(self):
        global latest_twist, last_cmd_time
        msg = Twist()
        # Safety watchdog: zero velocity if no command in 0.5 sec
        if time.time() - last_cmd_time < 0.5:
            msg.linear.x = float(latest_twist["linear"])
            msg.angular.z = float(latest_twist["angular"])
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        self.publisher.publish(msg)


def run_http_server():
    server_address = ('', 8080)
    httpd = ThreadedHTTPServer(server_address, WebTeleopHandler)
    print("🚀 Web Teleop Dashboard running at http://0.0.0.0:8080")
    httpd.serve_forever()


def main():
    rclpy.init()
    node = WebTeleopROSNode()
    
    server_thread = threading.Thread(target=run_http_server, daemon=True)
    server_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
