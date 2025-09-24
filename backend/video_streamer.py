# --- IMPORTS ---
from pathlib import Path
import time
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import os
import sys
import hailo
import numpy as np
import cv2
import socket
import json
import threading
from flask import Flask, Response, render_template_string
from hailo_apps.hailo_app_python.core.common.buffer_utils import get_caps_from_pad, get_numpy_from_buffer
from hailo_apps.hailo_app_python.core.gstreamer.gstreamer_app import app_callback_class
from hailo_apps.hailo_app_python.apps.detection.detection_pipeline import GStreamerDetectionApp

# --- CONFIGURATION ---
# UDP Communication Config
UDP_IP = "127.0.0.2"
UDP_PORT = 5005
CONTROL_SERVER_PORT = 5006

# Video Streaming Config
SERVER_PORT = 5001

# --- NEW: CAMERA CENTER OFFSET ---
# Adjust this value to change the vertical position of the "true center".
# 0.5 = geometric center of the frame.
# 0.75 = 75% of the way down the frame (for a forward-mounted camera).
VERTICAL_CENTER_RATIO = 0.10
# --- END NEW ---

# --- FLASK SETUP (unchanged) ---
flask_app = Flask(__name__)
app_user_data = None

def generate_frames():
    """Generator function for the video streaming."""
    while True:
        if app_user_data is None: time.sleep(0.1); continue
        frame = app_user_data.get_output_frame()
        if frame is None: time.sleep(0.01); continue
        (flag, encoded_image) = cv2.imencode(".jpg", frame)
        if not flag: continue
        yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' +
               bytearray(encoded_image) + b'\r\n')
        GLib.usleep(50000)

@flask_app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@flask_app.route('/')
def index():
    return render_template_string(
        '<html><body><h1>Hailo AI Detection Stream</h1>'
        '<img src="{{ url_for(\'video_feed\') }}">'
        '</body></html>'
    )

def control_command_listener():
    """Listens for TCP commands ('pause'/'resume') to control the UDP stream."""
    global app_user_data
    if app_user_data is None: return
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(('0.0.0.0', CONTROL_SERVER_PORT))
        s.listen()
        print(f"Control command listener started on port {CONTROL_SERVER_PORT}.")
        while True:
            conn, addr = s.accept()
            with conn:
                data = conn.recv(1024).decode('utf-8')
                if data == 'pause':
                    app_user_data.set_udp_sending(False)
                    print("UDP stream PAUSED.")
                elif data == 'resume':
                    app_user_data.set_udp_sending(True)
                    print("UDP stream RESUMED.")

# --- USER CALLBACK CLASS (unchanged) ---
class user_app_callback_class(app_callback_class):
    def __init__(self):
        super().__init__()
        self.frame_lock = threading.Lock()
        self.output_frame = None
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print(f"UDP socket created to send data to {UDP_IP}:{UDP_PORT}")
        self.control_lock = threading.Lock()
        self.is_sending_udp = True
    def set_output_frame(self, frame):
        with self.frame_lock: self.output_frame = frame.copy()
    def get_output_frame(self):
        with self.frame_lock: return self.output_frame
    def set_udp_sending(self, should_send: bool):
        with self.control_lock: self.is_sending_udp = should_send
    def can_send_udp(self):
        with self.control_lock: return self.is_sending_udp
    def close_socket(self):
        self.sock.close()

# --- GSTREAMER APP CALLBACK FUNCTION ---
def app_callback(pad, info, user_data):
    """This function is called for every frame processed by the GStreamer pipeline."""
    buffer = info.get_buffer()
    if buffer is None: return Gst.PadProbeReturn.OK

    user_data.increment()
    format, width, height = get_caps_from_pad(pad)

    frame = None
    if user_data.use_frame and all(v is not None for v in [format, width, height]):
        frame = get_numpy_from_buffer(buffer, format, width, height)

    # --- DETECTION PROCESSING (unchanged) ---
    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)
    valid_detections = []
    for det in detections:
        if det.get_label() == "target":
            valid_detections.append({"detection": det, "class_id": 0})
        elif det.get_label() == "target_drop":
            valid_detections.append({"detection": det, "class_id": 1})
            
    data_packet = {}
    if valid_detections:
        largest_detection_info = max(valid_detections, key=lambda d: d["detection"].get_bbox().width() * d["detection"].get_bbox().height())
        largest_target = largest_detection_info["detection"]
        bbox = largest_target.get_bbox()
        x_min_pix = int(bbox.xmin() * width)
        y_min_pix = int(bbox.ymin() * height)
        x_max_pix = int((bbox.xmin() + bbox.width()) * width)
        y_max_pix = int((bbox.ymin() + bbox.height()) * height)
        x_center = (x_min_pix + x_max_pix) / 2
        y_center = (y_min_pix + y_max_pix) / 2
        
        data_packet = {
            "x_center": float(x_center),
            "y_center": float(y_center),
            "area": float((x_max_pix - x_min_pix) * (y_max_pix - y_min_pix)),
            "frame_width": int(width),
            "frame_height": int(height),
            "state": "TRACKING",
            "class_id": largest_detection_info["class_id"]
        }
        if frame is not None:
            cv2.rectangle(frame, (x_min_pix, y_min_pix), (x_max_pix, y_max_pix), (0, 255, 0), 2)
            label = f"{largest_target.get_label()} ({largest_target.get_confidence():.2f})"
            cv2.putText(frame, label, (x_min_pix, y_min_pix - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    else:
        data_packet = {"state": "SEARCHING"}
    
    if user_data.can_send_udp():
        user_data.sock.sendto(json.dumps(data_packet).encode(), (UDP_IP, UDP_PORT))

    if user_data.use_frame and frame is not None:
        bgr_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # --- NEW: DRAW CENTER MARKER ON FRAME ---
        if width and height:
            center_x = int(width / 2)
            # Use the offset to calculate the true center Y position
            target_y = int(height * VERTICAL_CENTER_RATIO)
            
            # Draw a red crosshair
            cv2.line(bgr_frame, (center_x - 15, target_y), (center_x + 15, target_y), (0, 0, 255), 2)
            cv2.line(bgr_frame, (center_x, target_y - 15), (center_x, target_y + 15), (0, 0, 255), 2)
        # --- END NEW ---

        user_data.set_output_frame(bgr_frame)

    return Gst.PadProbeReturn.OK

# --- MAIN EXECUTION BLOCK (unchanged) ---
if __name__ == "__main__":
    user_data = user_app_callback_class()
    app_user_data = user_data
    print("Starting control command listener thread...")
    control_thread = threading.Thread(target=control_command_listener, daemon=True)
    control_thread.start()
    print(f"Starting video streaming server on http://0.0.0.0:{SERVER_PORT}")
    flask_thread = threading.Thread(target=lambda: flask_app.run(host='0.0.0.0', port=SERVER_PORT, debug=False), daemon=True)
    flask_thread.start()
    fake_argv = [
       "video_streamer.py",
       "--hef-path", "/home/kingphoenix/Web-GCS-Raspi5/models/kpDetectv2.0-yolov11n.hef",
       "--labels-json", "/home/kingphoenix/Web-GCS-Raspi5/backend/target.json",
       "--arch" , "hailo8",
       "--input", "/dev/video0",
       "--use-frame",
    ]
    sys.argv = fake_argv
    app = GStreamerDetectionApp(app_callback, user_data)
    try:
        app.run()
    finally:
        user_data.close_socket()
