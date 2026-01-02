import os
import time
import cv2
import socket
import threading
from datetime import datetime
from flask import Flask, Response, redirect, send_from_directory
from pathlib import Path
import argparse

from flare_detector import FlareDetector
from gate_detector import GateDetector


LOG_DIR         = Path.home() / "robot-log"
VIDEO_SAVE_DIR  = LOG_DIR / "video"


class Camera:
    def __init__(self, width, height, fps):
        self.width = width
        self.height = height
        self.fps = fps
        self.use_picamera = False
        self.cap = None

        try:
            from picamera2 import Picamera2
            self.cam = Picamera2()
            self.cam.configure(
                self.cam.create_video_configuration(
                    main={"format": "RGB888", "size": (self.width, self.height)}
                )
            )
            self.cam.start()
            self.use_picamera = True
            print("[INFO] Picamera2 available, used as main camera")
        except Exception as e:
            print("[WARN] Picamera2 unavailable, using OpenCV:", e)
            self.cap = cv2.VideoCapture(0)
            self.cap.set(3, self.width)
            self.cap.set(4, self.height)

    def read(self):
        if self.use_picamera:
            return True, self.cam.capture_array()
        return self.cap.read()


class Logger:
    def __init__(self, camera, max_vid_duration):
        os.makedirs(VIDEO_SAVE_DIR, exist_ok=True)
        self.camera = camera
        self.max_vid_duration = max_vid_duration
        self.video_writer = None
        self.last_rotate = 0

        self.frame_count = 0
        self.fps_estimate = camera.fps
        self.start_time = None

        self.new_video_log()

    def new_video_log(self):
        if self.video_writer:
            self.video_writer.release()

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = VIDEO_SAVE_DIR / f"{timestamp}.mp4"
        self.video_writer = cv2.VideoWriter(
            path,
            cv2.VideoWriter_fourcc(*"mp4v"),
            self.fps_estimate,
            (self.camera.width, self.camera.height)
        )

        self.last_rotate = time.time()
        self.start_time = self.last_rotate
        self.frame_count = 0
        print(f"[LOG] Recording {path}")

    def log_frame(self, frame):
        self.video_writer.write(frame)
        self.frame_count += 1

        elapsed = time.time() - self.start_time

        if elapsed >= 1.0:
            self.fps_estimate = self.frame_count / elapsed

        if elapsed >= self.max_vid_duration:
            self.new_video_log()


class Streamer:
    def __init__(self, target_port):
        self.app = Flask(__name__)
        self.target_port = target_port
        self.frame = None
        self.running = False

        @self.app.route("/")
        def root():
            return redirect("/live")

        @self.app.route("/live")
        def live_video():
            return Response(
                self.generate_image_response(),
                mimetype="multipart/x-mixed-replace; boundary=frame"
            )

        @self.app.route("/recording")
        def recorded_video():
            files = sorted(f.name for f in VIDEO_SAVE_DIR.glob("*"))
            return "<h1>Recorded video log:</h1>" + "<br>".join(
                f'<a href="/recording/{name}">{name}</a>' for name in files
            )

        @self.app.route("/recording/<filename>")
        def download_log(filename):
            return send_from_directory(VIDEO_SAVE_DIR, filename, as_attachment=True)

    def generate_image_response(self):
        while True:
            if self.frame is None:
                time.sleep(0.01)
                continue

            ok, buf = cv2.imencode(".jpg", self.frame)
            if ok:
                yield (b"--frame\r\n" + b"Content-Type: image/jpeg\r\n\r\n" + buf.tobytes() + b"\r\n")
            else:
                print("false")

    def start(self):
        if not self.running:
            threading.Thread(
                target=self.app.run,
                kwargs={"host": "0.0.0.0", "port": self.target_port, "threaded": True},
                daemon=True,
            ).start()
            self.running = True
            print("[INFO] Wifi connected, debug streaming enabled")


class CameraService:
    def __init__(self, do_log, do_stream):
        self.camera = Camera(width=640, height=480, fps=60)
        self.logger = (Logger(self.camera, max_vid_duration=30) if do_log else None)
        self.streamer = (Streamer(target_port=1234) if do_stream else None)

    def is_online(self):
        try:
            socket.create_connection(("8.8.8.8", 53), timeout=2)
            return True
        except OSError:
            return False

    def run(self):
        print("[INFO] Camera service running")
        if self.streamer and self.is_online():
                self.streamer.start()
        while True:
            ret, frame = self.camera.read()
            if not ret:
                print("here")
                continue

            # ================= CV PROCESSING =================
            # processing

            self.streamer.frame = frame

            if self.logger:
                self.logger.log_frame(frame)



if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Camera service options")
    parser.add_argument("--log", action="store_true", help="Enable video logging")
    parser.add_argument("--stream", action="store_true", help="Enable stream")

    args = parser.parse_args()
    camera_service = CameraService(args.log, args.stream)

    try:
        camera_service.run()

    except KeyboardInterrupt:
        print("[INFO] Interrupted, shutting down...")
        if args.log and camera_service.logger.video_writer:
            camera_service.logger.video_writer.release()

    except Exception as e:
        print(f"[ERROR] Unknown error: {e}")
