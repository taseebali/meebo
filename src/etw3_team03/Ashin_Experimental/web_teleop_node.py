#!/usr/bin/env python3
import os
import sys
import time
import json
import signal
import threading
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn

# Try importing Ordinary_Car directly for zero-latency direct motor control
try:
    from freenove_driver.motor import Ordinary_Car
    car_driver = Ordinary_Car()
    print("✅ Ordinary_Car motor driver attached directly!")
except Exception as e:
    car_driver = None
    print(f"⚠️ Motor driver note: {e} (Will use ROS 2 /cmd_vel topic)")

LINEAR_SCALE = 2500.0   # Duty scale for motor speed
ANGULAR_SCALE = 2500.0  # Duty scale for steering

latest_twist = {"linear": 0.0, "angular": 0.0}
last_cmd_time = time.time()
is_shutting_down = False

# Global JPEG bytes buffer from ROS 2 camera topic
latest_jpeg_bytes = None
jpeg_lock = threading.Lock()
httpd = None


def cleanup_all():
    global is_shutting_down, car_driver
    is_shutting_down = True
    print("\n🧹 Executing graceful shutdown sequence...")
    
    # Stop motors
    if car_driver:
        try:
            car_driver.set_motor_model(0, 0, 0, 0)
            car_driver.close()
            print("🛑 Motors stopped safely.")
        except Exception:
            pass
        car_driver = None


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


class WebTeleopHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        return  # Suppress HTTP console logs

    def set_cors_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')

    def do_OPTIONS(self):
        self.send_response(200, "ok")
        self.set_cors_headers()
        self.end_headers()

    def do_GET(self):
        if is_shutting_down:
            self.send_error(503, "Server Shutting Down")
            return
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
        if is_shutting_down:
            self.send_error(503, "Server Shutting Down")
            return

        if self.path == '/api/cmd_vel':
            length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(length).decode('utf-8')
            try:
                data = json.loads(body)
                latest_twist["linear"] = float(data.get("linear", 0.0))
                latest_twist["angular"] = float(data.get("angular", 0.0))
                last_cmd_time = time.time()
                
                self.send_response(200)
                self.set_cors_headers()
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(b'{"status":"ok"}')
            except Exception as e:
                self.send_error(400, str(e))
        elif self.path == '/api/estop':
            latest_twist["linear"] = 0.0
            latest_twist["angular"] = 0.0
            last_cmd_time = 0.0
            if car_driver:
                car_driver.set_motor_model(0, 0, 0, 0)
            self.send_response(200)
            self.set_cors_headers()
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(b'{"status":"estop_triggered"}')
        elif self.path == '/api/shutdown':
            self.send_response(200)
            self.set_cors_headers()
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(b'{"status":"shutting_down"}')
            threading.Thread(target=self.trigger_remote_shutdown, daemon=True).start()
        else:
            self.send_error(404)

    def trigger_remote_shutdown(self):
        time.sleep(0.5)
        cleanup_all()
        os._exit(0)

    def serve_file(self, rel_path, content_type):
        base_dir = os.path.dirname(os.path.abspath(__file__))
        full_path = os.path.join(base_dir, rel_path)
        if os.path.exists(full_path):
            self.send_response(200)
            self.set_cors_headers()
            self.send_header('Content-Type', content_type)
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            with open(full_path, 'rb') as f:
                self.wfile.write(f.read())
        else:
            self.send_error(404)

    def serve_mjpeg(self):
        self.send_response(200)
        self.set_cors_headers()
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        
        while not is_shutting_down:
            try:
                frame_bytes = None
                with jpeg_lock:
                    frame_bytes = latest_jpeg_bytes

                if frame_bytes:
                    self.wfile.write(b'--frame\r\n')
                    self.wfile.write(b'Content-Type: image/jpeg\r\n')
                    self.wfile.write(f'Content-Length: {len(frame_bytes)}\r\n'.encode())
                    self.wfile.write(b'\r\n')
                    self.wfile.write(frame_bytes)
                    self.wfile.write(b'\r\n')
                    self.wfile.flush()
                time.sleep(0.033)
            except (ConnectionResetError, BrokenPipeError):
                break
            except Exception:
                time.sleep(0.1)


class WebTeleopROSNode(Node):
    def __init__(self):
        super().__init__('web_teleop_node')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.04, self.publish_and_drive)  # 25 Hz
        
        # Subscribe to ROS 2 camera topic (/camera/image_raw/compressed)
        self.sub_cam = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.on_camera_image,
            10
        )
        self.get_logger().info("Web Teleop Node active at http://0.0.0.0:8080")
        self.get_logger().info("Subscribed to ROS 2 topic: /camera/image_raw/compressed")

    def on_camera_image(self, msg: CompressedImage):
        global latest_jpeg_bytes
        # Store compressed JPEG bytes directly for ultra-low latency streaming
        with jpeg_lock:
            latest_jpeg_bytes = bytes(msg.data)

    def publish_and_drive(self):
        global latest_twist, last_cmd_time, car_driver
        if is_shutting_down:
            return

        msg = Twist()
        # Watchdog: stop if no command within 0.5s
        if time.time() - last_cmd_time < 0.5:
            lin = float(latest_twist["linear"])
            ang = float(latest_twist["angular"])
        else:
            lin = 0.0
            ang = 0.0

        msg.linear.x = lin
        msg.angular.z = ang
        self.publisher.publish(msg)

        # Direct motor actuation
        if car_driver:
            left = LINEAR_SCALE * lin - ANGULAR_SCALE * ang
            right = LINEAR_SCALE * lin + ANGULAR_SCALE * ang
            left = max(-4095, min(4095, int(left)))
            right = max(-4095, min(4095, int(right)))
            car_driver.set_motor_model(left, left, right, right)


def run_http_server():
    global httpd
    server_address = ('', 8080)
    httpd = ThreadedHTTPServer(server_address, WebTeleopHandler)
    print("🚀 Web Teleop Server online at http://0.0.0.0:8080")
    try:
        httpd.serve_forever()
    except Exception:
        pass


def handle_signal(sig, frame):
    cleanup_all()
    sys.exit(0)


def main():
    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    rclpy.init()
    node = WebTeleopROSNode()
    
    server_thread = threading.Thread(target=run_http_server, daemon=True)
    server_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cleanup_all()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
