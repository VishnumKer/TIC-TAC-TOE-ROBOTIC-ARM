"""
MechArm 270 — Unified Robot Controller & Assembly Vision Cockpit
================================================================

Combined single-process Flask + SocketIO server running on port 5000.
Integrates:
  1. Wi-Fi Robot TCP Command Buffer Engine (RobotManager)
  2. Stepper Rail HTTP preset controller (RailManager)
  3. Continuous Webcam AprilTag + HSV Color contour detector (vision thread)
  4. Instant process memory part validation (zero inter-server network latency)
  5. Consolidated Operator Dashboard (dashboard.html)
"""

import sys
import os
import time
import threading
import logging
import traceback
import queue
import json
# requests removed — serial-only build
import io
import cv2
import numpy as np
from flask import Flask, render_template, jsonify, request, send_file, Response
from flask_socketio import SocketIO, emit

# Adjust path to import config
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from pymycobot import MechArm270
import config

# Enable DEBUG logging so vision pipeline logs are visible in the console
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S"
)

# ─────────────────────────────────────────────────────────────────────────────
# Global thread-safe Vision State Variables
# ─────────────────────────────────────────────────────────────────────────────
latest_frame = None
latest_annotated_frame = None
latest_raw_frame_encoded = None   # pre-encoded JPEG bytes for raw stream (no re-encode in stream path)
frame_lock = threading.Lock()

stable_detected_parts = {
    "Chassis 1": False,
    "Chassis 2": False,
    "WheelBase 1a": False,
    "WheelBase 2a": False,
    "WheelBase 1b": False,
    "WheelBase 2b": False,
    "Wheels": False,
    "Wheel Slot 1": False,
    "Wheel Slot 2": False,
    "Wheel Slot 3": False,
    "Wheel Slot 4": False,
    "Wheel Slot 5": False,
    "Wheel Slot 6": False,
    "Wheel Slot 7": False,
    "Wheel Slot 8": False,
    "Body 1": False,
    "Body 2": False,
}
detection_lock = threading.Lock()

mock_detections = {
    "Chassis 1": False,
    "Chassis 2": False,
    "WheelBase 1a": False,
    "WheelBase 2a": False,
    "WheelBase 1b": False,
    "WheelBase 2b": False,
    "Wheels": False,
    "Wheel Slot 1": False,
    "Wheel Slot 2": False,
    "Wheel Slot 3": False,
    "Wheel Slot 4": False,
    "Wheel Slot 5": False,
    "Wheel Slot 6": False,
    "Wheel Slot 7": False,
    "Wheel Slot 8": False,
    "Body 1": False,
    "Body 2": False,
}
mock_lock = threading.Lock()

_vision_frame_count = [0]  # mutable list so the vision thread can mutate without global keyword

latest_tag_coords = {}
latest_tag_coords_lock = threading.Lock()

latest_raw_tags = []
latest_raw_tags_lock = threading.Lock()

last_detection_snapshot = None
last_detection_snapshot_lock = threading.Lock()

# Path where the last detection JPEG is persisted across server restarts
SNAPSHOT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static", "last_snapshot.jpg")

# Millisecond timestamp of the last saved snapshot (0 = none saved yet).
# Used by /home_snapshot_ts so the browser can detect changes without downloading the full JPEG.
_snapshot_timestamp = 0

def _save_snapshot(jpeg_bytes: bytes):
    """Persist the latest detection frame to disk so it survives server restarts."""
    global _snapshot_timestamp
    try:
        os.makedirs(os.path.dirname(SNAPSHOT_PATH), exist_ok=True)
        with open(SNAPSHOT_PATH, "wb") as f:
            f.write(jpeg_bytes)
        _snapshot_timestamp = int(time.time() * 1000)
    except Exception as e:
        logging.warning("Could not save snapshot to disk: %s", e)

# Load the persisted snapshot at startup so the first page load has something to display
if os.path.exists(SNAPSHOT_PATH):
    try:
        with open(SNAPSHOT_PATH, "rb") as _f:
            last_detection_snapshot = _f.read()
        _snapshot_timestamp = int(os.path.getmtime(SNAPSHOT_PATH) * 1000)
        logging.info("Loaded last detection snapshot from disk (%d bytes).", len(last_detection_snapshot))
    except Exception as _e:
        logging.warning("Could not load persisted snapshot: %s", _e)

_frame_ready = threading.Event()  # signals generate_stream() that a new frame is available
_vision_detection_active = False  # True only while PlaybackEngine is doing a vision check
_detection_target_part = None     # The specific part name PlaybackEngine is currently scanning for
debug_overlays_enabled = False    # True if user forces annotated CV overlays on cockpit page
_confirmed_parts: set = set()     # parts confirmed+picked this sequence; cleared on each run start
_confirmed_parts_lock = threading.Lock()

# Frame counter reset requests — PlaybackEngine writes here, vision_thread processes next frame
_frame_seen_reset: dict = {}
_frame_seen_reset_lock = threading.Lock()

# Per-part ambient black ratio captured on the first active frame (arm settled at scan pose).
# Detection requires ratio - baseline >= BLACK_PIXEL_RATIO so ambient darkness is excluded.
_detection_baseline: dict = {}
_detection_baseline_lock = threading.Lock()

def _vision_reset_part(part: str):
    """Request vision_thread to zero frame counters and clear baseline for this part."""
    with _frame_seen_reset_lock:
        _frame_seen_reset[part] = True
    with _detection_baseline_lock:
        _detection_baseline.pop(part, None)

_camera_settings = {}
_camera_settings_lock = threading.Lock()
_camera_settings_dirty = threading.Event()

_CAP_PROP_MAP = {
    "brightness":    cv2.CAP_PROP_BRIGHTNESS,
    "contrast":      cv2.CAP_PROP_CONTRAST,
    "saturation":    cv2.CAP_PROP_SATURATION,
    "gain":          cv2.CAP_PROP_GAIN,
    "exposure":      cv2.CAP_PROP_EXPOSURE,
    "sharpness":     cv2.CAP_PROP_SHARPNESS,
    "auto_exposure": cv2.CAP_PROP_AUTO_EXPOSURE,
}

def _apply_camera_settings(cap):
    if cap is None or not cap.isOpened():
        logging.debug("📷 Skipping hardware camera settings application (camera is not open).")
        return
    with _camera_settings_lock:
        settings = dict(_camera_settings)
    
    # 1. Force Manual or Auto Exposure mode so exposure values are accepted by the driver
    auto_exp = settings.get("auto_exposure")
    if auto_exp is not None:
        try:
            # 3.0 represents standard UVC Aperture Priority Auto Exposure, 1.0 represents Manual Exposure mode
            mode_val = 3.0 if (auto_exp == 1.0 or auto_exp is True) else 1.0
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, mode_val)
            logging.info("📷 Forced Auto Exposure mode to: %s", "AUTO (3.0)" if mode_val == 3.0 else "MANUAL (1.0)")
        except Exception as e:
            logging.warning("⚠️ Failed to set Auto Exposure hardware mode: %s", e)

    # 2. Apply other settings sequentially
    for key, prop in _CAP_PROP_MAP.items():
        if key == "auto_exposure":
            continue
        val = settings.get(key)
        if val is not None:
            try:
                cap.set(prop, float(val))
                logging.debug("📷 Set camera property %s to %s successfully", key, val)
            except Exception as e:
                logging.warning("⚠️ Failed to set camera property %s: %s", key, e)


# ─────────────────────────────────────────────────────────────────────────────
# Black colour detection helpers
# ─────────────────────────────────────────────────────────────────────────────

def _get_roi(part_name: str):
    """Return [x0, y0, x1, y1] for a part slot from ROI_DEFINITIONS or ZONE_DEFINITIONS fallback."""
    roi = config.ROI_DEFINITIONS.get(part_name)
    if roi:
        return roi
    for zone_parts in config.ZONE_DEFINITIONS.values():
        if part_name in zone_parts:
            return zone_parts[part_name]
    return None

def _detect_black_in_roi(frame_hsv, roi):
    """Return (present: bool, ratio: float) — fraction of black pixels in the ROI."""
    x0, y0, x1, y1 = roi
    if x0 >= x1 or y0 >= y1:
        return False, 0.0
    region = frame_hsv[y0:y1, x0:x1]
    mask = cv2.inRange(region,
                       np.array(config.BLACK_HSV_LOWER, dtype=np.uint8),
                       np.array(config.BLACK_HSV_UPPER, dtype=np.uint8))
    area = (x1 - x0) * (y1 - y0)
    if area == 0:
        return False, 0.0
    ratio = cv2.countNonZero(mask) / area
    return ratio >= config.BLACK_PIXEL_RATIO, ratio


# ─────────────────────────────────────────────────────────────────────────────
# RobotManager (TCP Socket Connection Control)
# ─────────────────────────────────────────────────────────────────────────────
class RobotManager:
    JOINT_LIMITS = [
        (-170, 170),   # J1
        (-180, 180),   # J2
        (-180, 180),   # J3
        (-175, 175),   # J4
        (-170, 170),   # J5
        (-180, 180),   # J6
    ]
    COORD_LIMITS = {
        "x": (-400, 400),
        "y": (-400, 400),
        "z": (-400, 400),
        "rx": (-180, 180),
        "ry": (-180, 180),
        "rz": (-180, 180),
    }

    # Serial ports to scan for the MechArm USB cable (in priority order)
    # /dev/ttyACM0 is reserved for the rail ESP32; arm is typically ACM1
    ARM_SERIAL_PORTS = ["/dev/ttyACM1", "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM2"]
    ARM_SERIAL_BAUD  = 115200

    def __init__(self):
        self.serial_port = None   # set from config or auto-detected
        self.mc = None
        self._transport = "none"  # "serial" | "none"
        self.lock = threading.RLock()
        self.cmd_queue = queue.Queue()
        self._angles  = [0.0]*6
        self._coords  = [150.0,0.0,100.0,0.0,0.0,0.0]
        self._is_moving = False
        self._powered   = False
        self._connected = False
        self._last_error = ""
        self._move_active = False  # True while a move command is in progress
        self._vacuum     = False
        self._hw_errors  = 0
        self._hw_next_err = []
        self.coord_calibration = [0.0]*6
        self.angle_calibration = [0.0]*6
        self.config_path = os.path.join(os.path.dirname(__file__), "arm_config.json")
        self.load_config()
        self.completion_callbacks = []
        self._worker = threading.Thread(target=self._run_worker, daemon=True)
        self._worker.start()

    def load_config(self):
        try:
            if os.path.exists(self.config_path):
                with open(self.config_path) as f:
                    d = json.load(f)
                    self.serial_port = d.get("serial_port", self.serial_port)
                    self.coord_calibration = d.get("coord_calibration", [0.0]*6)
                    self.angle_calibration = d.get("angle_calibration", [0.0]*6)
        except Exception as e:
            logging.error("Arm config load: %s", e)

    def save_config(self, serial_port=None):
        if serial_port is not None: self.serial_port = serial_port or None
        try:
            with open(self.config_path, "w") as f:
                json.dump({
                    "serial_port": self.serial_port,
                    "coord_calibration": self.coord_calibration,
                    "angle_calibration": self.angle_calibration
                }, f, indent=4)
        except Exception as e:
            logging.error("Arm config save: %s", e)

    def _run_worker(self):
        logging.info("Worker started.")
        last_poll = 0
        active_cmd = None
        move_start_time = 0
        last_connect_attempt = 0
        
        while True:
            try:
                now = time.time()
                if not self._connected:
                    if now - last_connect_attempt > 15.0:
                        last_connect_attempt = now
                        self._attempt_connect()
                
                if active_cmd:
                    elapsed = time.time() - move_start_time
                    # Guard: give the arm at least 0.8s to start moving.
                    # On serial, firmware returns -1 transiently before
                    # motors start — must NOT treat as done.
                    if elapsed > 0.8:
                        try:
                            moving = self.mc.is_moving()
                            # 0 = confirmed stopped → move complete
                            if moving == 0:
                                self._move_active = False
                                self._finish_command(active_cmd)
                                active_cmd = None
                            elif elapsed > 25.0:
                                logging.warning("[ARM] Move timeout 25s — forcing done.")
                                self._move_active = False
                                self._finish_command(active_cmd)
                                active_cmd = None
                        except Exception as e:
                            logging.warning("[ARM] is_moving() error: %s", e)
                            self._move_active = False
                            active_cmd = None
                        # Throttle: only check is_moving every 0.3s
                        time.sleep(0.3)

                if not active_cmd:
                    try:
                        cmd = self.cmd_queue.get(timeout=0.1)
                        # Drop commands (except reconnect) when disconnected
                        if not self._connected and cmd.get("action") != "reconnect":
                            logging.warning("[ARM] Dropping %s — not connected.", cmd.get("action"))
                            self._finish_command(cmd)
                            self.cmd_queue.task_done()
                        else:
                            success = self._execute_command(cmd)
                            if success and cmd["action"] in ["move_coords", "move_angles"]:
                                active_cmd = cmd
                                move_start_time = time.time()
                                self._move_active = True
                            else:
                                self._finish_command(cmd)
                            self.cmd_queue.task_done()
                            last_poll = 0
                    except queue.Empty:
                        pass

                # Only poll status when NO move is active — prevents serial flooding
                if not self._move_active:
                    now2 = time.time()
                    if now2 - last_poll > 1.0:
                        self._poll_status()
                        last_poll = now2
                
                time.sleep(0.02)
            except Exception as e:
                logging.error("Worker error: %s\n%s", e, traceback.format_exc())
                self._connected = False
                time.sleep(3.0)

    def _finish_command(self, cmd):
        status_data = {"action": cmd.get("action"), "cmd_id": cmd.get("cmd_id")}
        socketio.emit("command_complete", status_data)
        for cb in self.completion_callbacks:
            try: cb(cmd.get("cmd_id"))
            except Exception as e: logging.error("Callback error: %s", e)
        with self.lock:
            self._is_moving = False

    def _attempt_connect(self):
        """Try USB serial ports. Stays disconnected if unavailable — no simulation."""
        if self.mc:
            try: self.mc.close()
            except: pass
            self.mc = None
        self._transport = "none"

        is_win = sys.platform.startswith("win")
        try:
            import serial.tools.list_ports
            detected_ports = [p.device for p in serial.tools.list_ports.comports()]
        except Exception:
            detected_ports = []
            
        arm_ports = detected_ports if is_win else self.ARM_SERIAL_PORTS
        ports_to_try = ([self.serial_port] if self.serial_port else []) + arm_ports
        
        # Remove duplicates while preserving order
        seen = set()
        ports_to_try = [x for x in ports_to_try if not (x in seen or seen.add(x))]

        for port in ports_to_try:
            mc = None
            try:
                logging.info("[ARM] Trying serial %s @ %d …", port, self.ARM_SERIAL_BAUD)
                mc = MechArm270(port, self.ARM_SERIAL_BAUD)
                time.sleep(2.0)
                a = mc.get_angles()
                if a and isinstance(a, list) and len(a) >= 6 and a[0] != -1:
                    self.mc = mc
                    self.serial_port = port
                    self._transport = "serial"
                    with self.lock:
                        self._connected = True
                        self._powered   = True
                        self._angles    = [round(v, 2) for v in a[:6]]
                    logging.info("✅ Arm serial connected on %s (MechArm270).", port)
                    return
                else:
                    raise Exception("get_angles() returned invalid data: %s" % a)
            except Exception as e:
                logging.debug("[ARM] Serial %s failed: %s", port, e)
                if mc:
                    try: mc.close()
                    except: pass

        # Stay disconnected — will retry in 15s
        with self.lock:
            self._connected = False
            self._last_error = "No serial port found — will retry"
        logging.warning("⚠️  Arm not found. Will retry in 15s.")

    def _execute_command(self, cmd):
        action = cmd.get("action"); params = cmd.get("params", {})
        try:
            if action == "move_coords":
                coords = [round(float(v), 2) for v in params["coords"]]
                coords = [coords[i]+self.coord_calibration[i] for i in range(6)]
                speed  = int(params["speed"]); mode = int(params.get("mode",0))

                logging.info("⚙️  [MOVE] Coords: %s | Speed: %s | Mode: %s", coords, speed, mode)

                if not self._powered:
                    self.mc.power_on(); time.sleep(0.5)
                    a = self.mc.get_angles()
                    if a and isinstance(a, list) and len(a) >= 6 and a[0] != -1:
                        self.mc.set_fresh_mode(1); time.sleep(0.1)
                        self.mc.send_angles([round(v,2) for v in a[:6]], 10)
                        time.sleep(0.5)
                    self.mc.set_fresh_mode(0)
                    with self.lock: self._powered = True

                valid, msg = self._validate_coords(coords)
                if not valid:
                    with self.lock: self._last_error = msg
                    return

                with self.lock: self._is_moving = True
                self.mc.send_coords(coords, speed, mode)
                return True

            elif action == "move_angles":
                angles = [round(float(v),2) for v in params["angles"]]
                angles = [angles[i]+self.angle_calibration[i] for i in range(6)]
                speed  = int(params["speed"])

                logging.info("⚙️  [MOVE] Angles: %s | Speed: %s", angles, speed)

                if not self._powered:
                    self.mc.power_on(); time.sleep(0.5)
                    a = self.mc.get_angles()
                    if a and isinstance(a, list) and len(a) >= 6 and a[0] != -1:
                        self.mc.set_fresh_mode(1); time.sleep(0.1)
                        self.mc.send_angles([round(v,2) for v in a[:6]], 10)
                        time.sleep(0.5)
                    self.mc.set_fresh_mode(0)
                    with self.lock: self._powered = True

                valid, msg = self._validate_angles(angles)
                if not valid:
                    with self.lock: self._last_error = msg
                    return

                with self.lock: self._is_moving = True
                self.mc.send_angles(angles, speed)
                return True

            elif action == "power_on":
                if self.mc:
                    self.mc.power_on(); time.sleep(0.5)
                    self.mc.focus_all_servos(); time.sleep(0.5)
                    a = self.mc.get_angles()
                    if a and isinstance(a, list) and len(a) >= 6 and a[0] != -1:
                        actual = [round(v, 2) for v in a[:6]]
                        self.mc.set_fresh_mode(1); time.sleep(0.1)
                        self.mc.send_angles(actual, 10); time.sleep(0.5)
                        self.mc.set_fresh_mode(0)
                with self.lock: self._powered = True

            elif action == "power_off":
                self.mc.release_all_servos()
                with self.lock: self._powered = False
                return True
            
            elif action == "stop":
                if self.mc:
                    self.mc.stop()
                logging.info("🛑 Emergency Stop signal sent to hardware.")
                with self.lock: self._is_moving = False

            elif action == "vacuum":
                vac_state = params.get("state", "off")
                vac_delay = int(params.get("delay", 0))
                if self.mc:
                    if vac_state == "on":
                        # Turn pump ON (5->0) and close valve (2->1)
                        self.mc.set_basic_output(5, 0)
                        self.mc.set_basic_output(2, 0)
                        time.sleep(1.5)
                        # Turn pump OFF (5->1) to hold vacuum sealed
                        self.mc.set_basic_output(5, 1)
                    else:
                        # Turn pump OFF (5->1)
                        self.mc.set_basic_output(5, 1)
                        # Open valve to vent (2->0)
                        self.mc.set_basic_output(2, 1)
                        time.sleep(0.3)
                
                with self.lock:
                    self._vacuum = (vac_state == "on")
                
                if vac_delay > 0:
                    time.sleep(vac_delay / 1000.0)

            elif action == "joint_power":
                joint_id = int(params.get("joint_id"))
                state = params.get("state")
                if self.mc:
                    if state == "on": self.mc.focus_servo(joint_id)
                    else: self.mc.release_servo(joint_id)
                    time.sleep(0.05)

            elif action == "reconnect":
                self._attempt_connect()

        except Exception as e:
            logging.error("Cmd exec failed: %s", e)
            self._connected = False

    def _poll_status(self):
        if not self.mc or not self._connected: return
        now = time.time()
        try:
            moving = self.mc.is_moving()
            with self.lock: 
                self._is_moving = (moving == 1)
                is_moving_now = self._is_moving

            if is_moving_now:
                return 

            a = self.mc.get_angles()
            if a and isinstance(a,list) and len(a)>=6 and a[0]!=-1:
                with self.lock:
                    self._angles = [round(v,2) for v in a[:6]]
                    if not self._powered:
                        self._powered = True
            
            time.sleep(0.05)

            if not hasattr(self, '_last_slow_poll'): self._last_slow_poll = 0
            if now - self._last_slow_poll > 1.0:
                c = self.mc.get_coords()
                if c and isinstance(c,list) and len(c)>=6 and c[0]!=-1:
                    with self.lock: self._coords = [round(v,2) for v in c[:6]]
                
                time.sleep(0.05)
                
                try:
                    err_code = self.mc.get_error_information()
                    next_err = self.mc.read_next_error()
                    with self.lock:
                        self._hw_errors = err_code
                        self._hw_next_err = next_err if isinstance(next_err, list) else []
                except:
                    pass
                
                self._last_slow_poll = now
        except Exception as e:
            with self.lock: self._connected = False

    def send_coords(self, coords, speed=config.ARM_SPEED, mode=0, cmd_id=None):
        self.cmd_queue.put({"action":"move_coords","params":{"coords":coords,"speed":speed,"mode":mode},"cmd_id":cmd_id})
        return {"success":True}

    def send_angles(self, angles, speed=config.ARM_SPEED, cmd_id=None):
        self.cmd_queue.put({"action":"move_angles","params":{"angles":angles,"speed":speed},"cmd_id":cmd_id})
        return {"success":True}

    def lock_servos(self):
        self.cmd_queue.put({"action":"power_on"}); return {"success":True}

    def release_servos(self):
        self.cmd_queue.put({"action":"power_off"}); return {"success":True}

    def set_vacuum(self, state, delay=0, cmd_id=None):
        self.cmd_queue.put({"action":"vacuum","params":{"state":state,"delay":delay},"cmd_id":cmd_id})
        return {"success":True}

    def set_joint_power(self, joint_id, state, cmd_id=None):
        self.cmd_queue.put({"action":"joint_power","params":{"joint_id":joint_id,"state":state},"cmd_id":cmd_id})
        return {"success":True}

    def reconnect(self, serial_port=None):
        self.save_config(serial_port=serial_port)
        self.cmd_queue.put({"action": "reconnect"})
        return {"success": True}

    def emergency_stop(self):
        while not self.cmd_queue.empty():
            try: self.cmd_queue.get_nowait()
            except: pass
        self.cmd_queue.put({"action": "stop"})
        return {"success": True}

    def _validate_coords(self, coords):
        for i,n in enumerate(["x","y","z","rx","ry","rz"]):
            lo,hi = self.COORD_LIMITS[n]
            if not (lo <= coords[i] <= hi):
                return False, f"{n}={coords[i]} out of [{lo},{hi}]"
        return True, "OK"

    def _validate_angles(self, angles):
        for i in range(6):
            lo,hi = self.JOINT_LIMITS[i]
            if not (lo <= angles[i] <= hi):
                return False, f"J{i+1}={angles[i]} out of [{lo},{hi}]"
        return True, "OK"

    @property
    def state(self):
        with self.lock:
            return {
                "angles": list(self._angles),
                "coords": list(self._coords),
                "is_moving": self._is_moving,
                "connection_state": "connected" if self._connected else "disconnected",
                "mode": "hardware",
                "transport": self._transport,   # "serial" | "none"
                "powered": self._powered,
                "vacuum": self._vacuum,
                "error": self._last_error,
                "serial_port": self.serial_port,
                "hw_errors": self._hw_errors,
                "hw_next_err": self._hw_next_err,
                "calibration": {
                    "coords": list(self.coord_calibration),
                    "angles": list(self.angle_calibration)
                },
                "timestamp": time.time(),
            }


# ─────────────────────────────────────────────────────────────────────────────
# RailManager  —  USB Serial primary, WiFi HTTP fallback
# ─────────────────────────────────────────────────────────────────────────────
class RailManager:
    """
    Communicates with the ESP32 rail controller via USB Serial only.

    ESP32 broadcasts "STATUS:{...json...}" every 500 ms; Python reads
    it in a background reader thread so poll() never blocks.
    """

    # Ports to scan in order when auto-detecting the ESP32
    SERIAL_PORTS = ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0", "/dev/ttyUSB1"]
    SERIAL_BAUD  = 115200

    def __init__(self):
        self.state = {
            "running": False, "homed": False, "point": "NONE",
            "absCm": 0.0, "rpm": 150, "tempC": -99.0, "doorOpen": True,
            "connected": False,
        }
        self.lock        = threading.Lock()
        self._last_poll  = 0

        # Serial state
        self._serial           = None
        self._serial_lock      = threading.Lock()
        self._serial_connected = False

        # Start serial connection attempt in background (non-blocking)
        threading.Thread(target=self._connect_serial, daemon=True).start()



    # ── Serial transport ─────────────────────────────────────────────────────
    def _connect_serial(self):
        """Try each candidate port. On success, start the reader thread."""
        try:
            import serial as _serial_mod
            import serial.tools.list_ports
        except ImportError:
            logging.warning("⚠️  pyserial not installed — install with: pip install pyserial")
            return

        is_win = sys.platform.startswith("win")
        ports_to_try = [p.device for p in serial.tools.list_ports.comports()] if is_win else self.SERIAL_PORTS

        for port in ports_to_try:
            try:
                logging.info("[RAIL] Trying serial %s @ %d …", port, self.SERIAL_BAUD)
                ser = _serial_mod.Serial(port, self.SERIAL_BAUD, timeout=1)
                ser.dtr = False          # prevent ESP32 reset on connect
                time.sleep(0.5)          # let USB enumerate
                ser.reset_input_buffer()
                
                # Verify that this is the rail by waiting for a STATUS message
                verified = False
                start_time = time.time()
                while time.time() - start_time < 15.0:
                    raw = ser.readline()
                    if raw:
                        line = raw.decode("utf-8", errors="ignore").strip()
                        if line.startswith("STATUS:"):
                            verified = True
                            break
                            
                if verified:
                    with self._serial_lock:
                        self._serial           = ser
                        self._serial_connected = True
                    logging.info("✅ Rail serial connected on %s at %d baud", port, self.SERIAL_BAUD)
                    threading.Thread(target=self._serial_reader, daemon=True).start()
                    return
                else:
                    logging.info("[RAIL] Port %s is not the Rail (no STATUS: stream). Closing.", port)
                    ser.close()
            except Exception as e:
                logging.debug("Serial port %s unavailable: %s", port, e)

        logging.warning("⚠️  No serial port found for rail — rail unavailable.")

    def _serial_reader(self):
        """Background thread: reads STATUS: lines from ESP32 and updates state."""
        while True:
            try:
                with self._serial_lock:
                    ser = self._serial
                if ser is None or not ser.is_open:
                    time.sleep(1)
                    continue
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="ignore").strip()
                if not line.startswith("STATUS:"):
                    continue
                data = json.loads(line[7:])
                vmax = data.get("vmax", 150000)
                rpm  = int(vmax * (420 / 500000))
                with self.lock:
                    self.state = {
                        "running":   data.get("running",  False),
                        "homed":     data.get("homed",    False),
                        "homing":    data.get("homing",   False),
                        "point":     data.get("point",    "NONE"),
                        "absCm":     data.get("absCm",    0.0),
                        "rpm":       rpm,
                        "vmax":      vmax,
                        "tempC":     data.get("tempC",    -99.0),
                        "doorOpen":  data.get("doorOpen", True),
                        "connected": True,
                    }
            except Exception as e:
                logging.warning("Rail serial reader error: %s — reconnecting...", e)
                with self._serial_lock:
                    self._serial_connected = False
                    try:
                        if self._serial: self._serial.close()
                    except: pass
                    self._serial = None
                time.sleep(3)
                threading.Thread(target=self._connect_serial, daemon=True).start()
                return

    def _send_serial(self, cmd: str) -> bool:
        """Write a newline-terminated command to the ESP32 via serial."""
        with self._serial_lock:
            ser       = self._serial
            connected = self._serial_connected
        if ser and connected:
            try:
                ser.write((cmd + "\n").encode())
                return True
            except Exception as e:
                logging.warning("Rail serial send failed: %s", e)
                self._serial_connected = False
        return False

    # ── Public API ────────────────────────────────────────────────────────────
    def move_to_preset(self, name):
        """Send MOVE command via serial."""
        if not name or name == "NONE":
            return True
        sent = self._send_serial(f"MOVE:{name.upper()}")
        if not sent:
            logging.warning("Rail move: serial unavailable, command dropped.")
        return sent

    def set_speed(self, rpm):
        """Send SPEED command via serial."""
        vmax = int(rpm * (500000 / 420))
        vmax = max(50000, min(500000, vmax))
        sent = self._send_serial(f"SPEED:{vmax}")
        if not sent:
            logging.warning("Rail speed: serial unavailable, command dropped.")
        return sent

    def stop(self):
        """Send STOP command via serial."""
        sent = self._send_serial("STOP")
        if not sent:
            logging.warning("Rail stop: serial unavailable, command dropped.")
        return sent

    def poll(self):
        """State is updated continuously by the background serial reader — nothing to do here."""
        pass  # serial-only build

    @property
    def current_state(self):
        with self.lock:
            return dict(self.state)


# ─────────────────────────────────────────────────────────────────────────────
# RailIndicatorManager  —  Serial primary, WiFi HTTP fallback
# ─────────────────────────────────────────────────────────────────────────────
class RailIndicatorManager:
    """
    Controls the 12V relay indicators on the ESP32 rail board.
    Sends INDICATOR:<mode> over serial; falls back to HTTP if serial unavailable.

    Modes: alert | green | off | play | auto
    """

    def __init__(self, rail: "RailManager"):
        self._rail      = rail
        self._last_mode = ""

    def _send(self, mode: str):
        if mode == self._last_mode:
            return  # deduplicate
        self._last_mode = mode
        cmd = f"INDICATOR:{mode}"
        if self._rail._send_serial(cmd):
            logging.info("🚦 Indicator → %s (serial)", mode.upper())
        else:
            logging.warning("⚠️  Indicator serial unavailable, command dropped (%s).", mode)

    def alert(self):       self._send("alert")
    def green(self):       self._send("green")
    def off(self):         self._send("off")
    def play_started(self): self._send("play")
    def reset(self):       self._send("auto")



# ─────────────────────────────────────────────────────────────────────────────
# WaypointStore
# ─────────────────────────────────────────────────────────────────────────────
class WaypointStore:
    def __init__(self, filepath=None):
        self.filepath = filepath or os.path.join(os.path.dirname(__file__), "waypoints_wifi.json")
        self.waypoints = []
        self.lock = threading.Lock()
        self.load()
        
        # Preload fallback
        if not self.waypoints:
            fallback_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../recordings/car_1_rough.json"))
            if os.path.exists(fallback_path):
                logging.info("Pre-loading default waypoints from: %s", fallback_path)
                try:
                    with open(fallback_path) as f:
                        self.waypoints = json.load(f)
                    self._save()
                except Exception as e:
                    logging.error("Failed to load fallback: %s", e)

    def add(self, wp: dict) -> int:
        with self.lock:
            wp["name"] = wp.get("name") or f"Point {len(self.waypoints)+1}"
            wp.setdefault("vacuum", "none")
            wp.setdefault("vacuum_delay_ms", 300)
            wp.setdefault("delay_ms", 200)
            wp.setdefault("rail_preset", "NONE")
            wp["timestamp"] = time.time()
            self.waypoints.append(wp)
            idx = len(self.waypoints) - 1
        self._save()
        return idx

    def update(self, index: int, field_index: int, value: float) -> bool:
        with self.lock:
            if 0 <= index < len(self.waypoints):
                wp = self.waypoints[index]
                val = round(float(value), 4)
                wp["data"][field_index] = val
                if wp.get("type") == "both" and "angles" in wp:
                    wp["angles"][field_index] = val
                self._save_locked()
                return True
        return False

    def update_field(self, index: int, field: str, value) -> bool:
        with self.lock:
            if 0 <= index < len(self.waypoints):
                if field in ("speed", "vacuum_delay_ms", "delay_ms"):
                    self.waypoints[index][field] = int(value)
                elif field == "rail_preset":
                    self.waypoints[index][field] = str(value).upper()
                else:
                    self.waypoints[index][field] = value
                self._save_locked()
                return True
        return False

    def update_name(self, index: int, name: str) -> bool:
        return self.update_field(index, "name", name)

    def update_speed(self, index: int, speed: int) -> bool:
        return self.update_field(index, "speed", int(speed))

    def update_all_data(self, index: int, data: list) -> bool:
        with self.lock:
            if 0 <= index < len(self.waypoints) and len(data) == 6:
                wp = self.waypoints[index]
                vals = [round(float(v), 4) for v in data]
                wp["data"] = vals
                if wp.get("type") == "both" and "angles" in wp:
                    wp["angles"] = list(vals)
                self._save_locked()
                return True
        return False

    def duplicate(self, index: int) -> int:
        with self.lock:
            if 0 <= index < len(self.waypoints):
                import copy
                wp = copy.deepcopy(self.waypoints[index])
                wp["name"] = wp.get("name", "Point") + " (copy)"
                wp["timestamp"] = time.time()
                self.waypoints.insert(index + 1, wp)
                self._save_locked()
                return index + 1
        return -1

    def delete(self, index: int) -> bool:
        with self.lock:
            if 0 <= index < len(self.waypoints):
                self.waypoints.pop(index)
                self._save_locked()
                return True
        return False

    def clear(self):
        with self.lock:
            self.waypoints = []
            self._save_locked()

    def reorder(self, from_idx: int, to_idx: int):
        with self.lock:
            n = len(self.waypoints)
            if 0 <= from_idx < n and 0 <= to_idx < n:
                self.waypoints.insert(to_idx, self.waypoints.pop(from_idx))
                self._save_locked()

    def get_all(self):
        with self.lock:
            return list(self.waypoints)

    def _save(self):
        with self.lock:
            self._save_locked()

    def _save_locked(self):
        try:
            with open(self.filepath, "w") as f:
                json.dump(self.waypoints, f, indent=2)
        except Exception as e:
            logging.error("Waypoint save: %s", e)

    def load(self):
        try:
            if os.path.exists(self.filepath):
                with open(self.filepath) as f:
                    self.waypoints = json.load(f)
                logging.info("Loaded %d waypoints.", len(self.waypoints))
        except Exception as e:
            logging.error("Waypoint load: %s", e)
            self.waypoints = []


# ─────────────────────────────────────────────────────────────────────────────
# PlaybackEngine (Zero Inter-Server Network Latency - Pure Memory Validation)
# ─────────────────────────────────────────────────────────────────────────────
def interruptible_sleep(duration_sec, stop_event, step=0.05):
    end_time = time.time() + duration_sec
    while time.time() < end_time:
        if stop_event.is_set():
            break
        time.sleep(min(step, end_time - time.time()))

class PlaybackEngine:
    def __init__(self, robot: RobotManager, rail: RailManager, store: WaypointStore,
                 indicator: RailIndicatorManager = None):
        self.robot     = robot
        self.rail      = rail
        self.store     = store
        self.indicator = indicator  # May be None if ESP32 is unreachable at startup
        self._state = "IDLE"
        self._current_index = 0
        self._pause_event = threading.Event()
        self._pause_event.set()
        self._stop_event  = threading.Event()
        self._thread = None
        self._cmd_done_event  = threading.Event()
        self._pending_cmd_id  = None
        self._lock = threading.Lock()
        self._error_msg = ""

    @property
    def state(self):
        return self._state

    def on_command_complete(self, cmd_id):
        if cmd_id == self._pending_cmd_id:
            self._cmd_done_event.set()

    def play(self, start_index=0, speed=config.ARM_SPEED, delay_ms=200, loop=False, async_rail=False, dynamic_mode=False, skip_vision=False):
        # 1. Stop any existing thread first without holding the playback lock during join
        if self._thread and self._thread.is_alive():
            logging.info("Stopping active playback thread before starting new one...")
            self.stop()
            self._thread.join(timeout=2.0)

        with self._lock:
            if self._state == "PLAYING":
                return {"success":False,"error":"Already playing"}
            self._stop_event.clear()
            self._pause_event.set()
            self._state = "PLAYING"
            self._error_msg = ""
            self._thread = threading.Thread(
                target=self._run,
                args=(start_index, speed, delay_ms, loop, async_rail, dynamic_mode, skip_vision),
                daemon=True
            )
            self._thread.start()

        # Green LED ON immediately — play sequence started
        if self.indicator:
            threading.Thread(target=self.indicator.play_started, daemon=True).start()

        return {"success":True}

    def pause(self):
        with self._lock:
            if self._state != "PLAYING":
                return {"success":False,"error":"Not playing"}
            self._pause_event.clear()
        return {"success":True}

    def resume(self):
        with self._lock:
            self._pause_event.set()
        return {"success":True}

    def stop(self):
        with self._lock:
            self._stop_event.set()
            self._pause_event.set()
            self._cmd_done_event.set()  # unblocks any wait in _run()
            self._state = "IDLE"
            self._error_msg = ""
            self.robot.emergency_stop()
        # Join thread BEFORE emitting IDLE — thread is dead and cannot
        # emit a PLAYING event that would race and override our IDLE.
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        # Reset indicators to OFF when operator stops the sequence
        if self.indicator:
            self.indicator.off()
        socketio.emit("status", {
            "state": "IDLE",
            "step": 1,
            "total": len(self.store.get_all()),
            "message": "Playback stopped."
        })
        return {"success": True}

    def restart(self):
        # Stop any active thread and wait for it to join
        if self._thread and self._thread.is_alive():
            self.stop()
            self._thread.join(timeout=2.0)
            
        with self._lock:
            self._current_index = 0
            self._state = "IDLE"
            self._error_msg = ""
            socketio.emit("status", {
                "state": "IDLE",
                "step": 1,
                "total": len(self.store.get_all()),
                "message": "Sequence restarted. Ready to play."
            })
        return {"success":True}

    def _emit_playing(self, step: int, total: int, message: str):
        """Emit PLAYING status only if stop has NOT been requested — prevents race condition."""
        if not self._stop_event.is_set():
            socketio.emit("status", {
                "state": "PLAYING",
                "step": step,
                "total": total,
                "message": message,
            })

    def _run(self, start_index, speed, delay_ms, loop, async_rail, dynamic_mode, skip_vision=False):
        global _vision_detection_active, _detection_target_part, last_detection_snapshot, _confirmed_parts
        if self.rail:
            self.rail.set_speed(config.RAIL_SPEED_RPM)
        self.transitions = {"P1": 0, "P2": 0, "P3": 0, "P4": 0}
        with _confirmed_parts_lock:
            _confirmed_parts.clear()
        _arm_holding = False  # True when vacuum is on (component held by arm)
        waypoints = self.store.get_all()
        total = len(waypoints)
        if total == 0:
            with self._lock:
                if threading.current_thread() == self._thread:
                    self._state = "IDLE"
            socketio.emit("status", {"state": "IDLE", "step": 1, "total": 0, "message": "Sequence empty."})
            return

        if self.robot._connected:
            with self.robot.lock:
                powered = self.robot._powered
            if not powered:
                logging.info("⚡ Auto-locking servos for playback...")
                self.robot.lock_servos()
                time.sleep(2.0)

        run_start = start_index
        current_rail = None

        while True:
            for i in range(run_start, total):
                if self._stop_event.is_set():
                    break

                self._pause_event.wait()
                if self._stop_event.is_set():
                    break

                self._current_index = i
                wp = waypoints[i]
                wp_speed = min(wp.get("speed", speed), 100)
                cmd_id = f"pb_{time.time()}_{i}"

                # ─────────────────────────────────────────────────────────────
                # 0. Vision check marker — data-driven, no arm/rail movement
                # ─────────────────────────────────────────────────────────────
                if wp.get("type") == "vision_check":
                    station = wp.get("rail_preset", "P4")
                    if not skip_vision:
                        part = wp.get("check_part")
                        if part:
                            msg = f"Vision check marker: verifying {part} at {station}..."
                            logging.info("👁️  %s", msg)
                            self._emit_playing(i + 1, total, msg)
                            socketio.emit("vision_log", {"level": "info", "message": msg})

                            scan_pose = config.SCAN_POSES.get(station, [0.0] * 6)
                            scan_id = f"scan_{time.time()}_{station}_vc"
                            self._cmd_done_event.clear()
                            self._pending_cmd_id = scan_id
                            self.robot.send_angles(scan_pose, speed=100, cmd_id=scan_id)
                            self._cmd_done_event.wait(timeout=8)

                            interruptible_sleep(1.5, self._stop_event)
                            if self._stop_event.is_set():
                                return

                            with detection_lock:
                                stable_detected_parts[part] = False
                            _vision_reset_part(part)

                            try:
                                _detection_target_part = part
                                _vision_detection_active = True
                                socketio.emit("detection_started", {"part": part, "station": station})

                                found = False
                                wait_start = time.time()
                                _timeout = config.VISION_WAIT_TIMEOUT
                                _ind_alert_sent = False

                                while not self._stop_event.is_set():
                                    with detection_lock:
                                        found = stable_detected_parts.get(part, False)
                                    if found:
                                        break

                                    elapsed = int(time.time() - wait_start)
                                    wait_msg = f"⏳ Waiting for {part} at {station}... ({elapsed}s)"
                                    logging.info(wait_msg)
                                    socketio.emit("status", {
                                        "state": "WAITING", "step": i + 1, "total": total,
                                        "message": wait_msg, "part": part, "station": station,
                                    })

                                    ind_threshold = config.INDICATOR_ALERT_TIMEOUT
                                    if (self.indicator and ind_threshold > 0
                                            and elapsed >= ind_threshold and not _ind_alert_sent):
                                        socketio.emit("vision_log", {"level": "warn", "message":
                                            f"🔔 No {part} at {station} after {elapsed}s — "
                                            f"Red LED ON + Buzzer active. Place part to continue."})
                                        threading.Thread(target=self.indicator.alert, daemon=True).start()
                                        _ind_alert_sent = True

                                    if _timeout > 0 and elapsed >= _timeout:
                                        self.robot.emergency_stop()
                                        if self.indicator and not _ind_alert_sent:
                                            threading.Thread(target=self.indicator.alert, daemon=True).start()
                                        with self._lock:
                                            self._state = "ERROR"
                                            self._error_msg = (
                                                f"❌ {part} not placed at {station} within {_timeout}s. "
                                                f"Place part and restart."
                                            )
                                        socketio.emit("error", {"part": part, "position": station,
                                                                "message": self._error_msg})
                                        socketio.emit("vision_log", {"level": "error",
                                                                      "message": f"✗ {part} not found at {station}"})
                                        return

                                    interruptible_sleep(1.0, self._stop_event)

                                if self._stop_event.is_set() or not found:
                                    return

                                if self.indicator:
                                    threading.Thread(target=self.indicator.green, daemon=True).start()

                                socketio.emit("part_verified", {
                                    "part": part, "rail": station,
                                    "snapshot_ts": int(time.time() * 1000),
                                })
                                with _confirmed_parts_lock:
                                    _confirmed_parts.add(part)
                                success_msg = f"{part} confirmed at {station} — resuming"
                                logging.info("✅ %s", success_msg)
                                self._emit_playing(i + 1, total, success_msg)

                                _vision_detection_active = False
                                socketio.emit("detection_stopped", {"part": part, "station": station, "found": True})

                            finally:
                                if _vision_detection_active:
                                    socketio.emit("detection_stopped",
                                                  {"part": part, "station": station, "found": False})
                                _vision_detection_active = False
                                _detection_target_part = None
                    # Always increment so subsequent rail-change visits map to the correct slot
                    if station in self.transitions:
                        self.transitions[station] += 1
                    continue  # skip arm move / rail move for vision_check waypoints

                new_rail = wp.get("rail_preset", "NONE")

                # ─────────────────────────────────────────────────────────────
                # 1. Arm moves FIRST to its waypoint position
                # ─────────────────────────────────────────────────────────────
                self._emit_playing(i + 1, total, f"Moving to: {wp.get('name', 'Point')}")
                socketio.emit("vision_log", {"level": "move", "message": f"▶ Moving to: {wp.get('name', 'Point')}"})
                socketio.emit("playback_progress", {
                    "state": "playing",
                    "index": i,
                    "total": total,
                    "name": wp.get("name", "Point")
                })

                self._cmd_done_event.clear()
                self._pending_cmd_id = cmd_id

                if wp["type"] == "coords":
                    self.robot.send_coords(wp["data"], wp_speed, int(wp.get("moveMode",0)), cmd_id=cmd_id)
                elif wp["type"] == "both":
                    self.robot.send_angles(wp["angles"], wp_speed, cmd_id=cmd_id)
                else:
                    self.robot.send_angles(wp["data"], wp_speed, cmd_id=cmd_id)

                if dynamic_mode:
                    threshold = 3.0
                    timeout = time.time() + 15
                    while time.time() < timeout:
                        if self._stop_event.is_set(): break
                        self._pause_event.wait()
                        state = self.robot.state
                        actual = state["coords"] if wp["type"] == "coords" else state["angles"]
                        target = wp["data"] if wp["type"] != "both" else (wp["coords"] if wp["type"]=="coords" else wp["angles"])
                        try:
                            dist = sum((actual[j] - target[j])**2 for j in range(6))**0.5
                            if dist < threshold: break
                        except: break
                        time.sleep(0.05)
                else:
                    self._cmd_done_event.wait(timeout=20)

                if not self._stop_event.is_set():
                    vac = wp.get("vacuum", "none")
                    if vac in ("on", "off"):
                        vac_id = f"vac_{time.time()}_{i}"
                        self._cmd_done_event.clear()
                        self._pending_cmd_id = vac_id
                        self.robot.set_vacuum(vac, wp.get("vacuum_delay_ms", 300), cmd_id=vac_id)
                        self._cmd_done_event.wait(timeout=5)
                        if vac == "on":
                            _arm_holding = True
                        elif vac == "off":
                            _arm_holding = False
                        _vac_action = "Picking" if vac == "on" else "Releasing"
                        socketio.emit("vision_log", {"level": "pick", "message": f"⊙ {_vac_action}: {wp.get('name', 'Point')}"})

                # ─────────────────────────────────────────────────────────────
                # 2. Rail moves AFTER arm completes — then vision check if new station
                # ─────────────────────────────────────────────────────────────
                if new_rail and new_rail != "NONE" and new_rail != current_rail:
                    logging.info("🛤️ Moving Rail to %s (arm already at position)...", new_rail)
                    self.rail.move_to_preset(new_rail)

                    # Wait for rail to complete movement
                    time.sleep(0.1)
                    timeout = time.time() + 15
                    while time.time() < timeout:
                        self.rail.poll()
                        st = self.rail.current_state
                        if not st.get("connected", False):
                            break  # Skip waiting if hardware is disconnected
                        if not st.get("running", False) and st.get("point") == new_rail:
                            break
                        if self._stop_event.is_set(): break
                        time.sleep(0.05)

                    if self._stop_event.is_set():
                        break

                    if not skip_vision and not _arm_holding and new_rail in config.POSITION_PART_MAP:
                        # Increment visit count for the rail preset
                        self.transitions[new_rail] = self.transitions.get(new_rail, 0) + 1
                        count = self.transitions[new_rail]

                        # Determine part to check dynamically
                        part = None
                        if new_rail == "P1":
                            part = "Chassis 1" if count == 1 else "Chassis 2"
                        elif new_rail == "P2":
                            if count == 1: part = "WheelBase 1a"
                            elif count == 2: part = "WheelBase 2a"
                            elif count == 3: part = "WheelBase 1b"
                            else: part = "WheelBase 2b"
                        elif new_rail == "P4":
                            part = f"Wheel Slot {count}" if 1 <= count <= 8 else "Wheels"
                        elif new_rail == "P5":
                            part = "Body 1" if count == 1 else "Body 2"

                        if part:
                            msg = f"Rail arrived at {new_rail} (visit #{count}). Verifying {part}..."
                            logging.info("👁️  %s", msg)
                            self._emit_playing(i + 1, total, msg)

                            # Move arm to scan pose and wait for arrival
                            scan_pose = config.SCAN_POSES.get(new_rail, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                            scan_id = f"scan_{time.time()}_{new_rail}"
                            self._cmd_done_event.clear()
                            self._pending_cmd_id = scan_id
                            self.robot.send_angles(scan_pose, speed=100, cmd_id=scan_id)

                            # Wait up to 8 seconds for arm to reach scan pose
                            self._cmd_done_event.wait(timeout=8)

                            # Camera focus stabilization
                            interruptible_sleep(1.5, self._stop_event)
                            if self._stop_event.is_set():
                                return

                            # Clear stale state from prior visits at this station
                            with detection_lock:
                                stable_detected_parts[part] = False
                            _vision_reset_part(part)

                            # Activate detection — try/finally guarantees cleanup on all exit paths
                            try:
                                _detection_target_part = part  # tells vision_thread exactly which part to scan
                                _vision_detection_active = True
                                socketio.emit("detection_started", {"part": part, "station": new_rail})

                                found = False
                                wait_start = time.time()
                                _timeout = config.VISION_WAIT_TIMEOUT
                                _ind_alert_sent = False  # Track if alert already triggered

                                while not self._stop_event.is_set():
                                    with detection_lock:
                                        found = stable_detected_parts.get(part, False)
                                    if found:
                                        break

                                    elapsed = int(time.time() - wait_start)
                                    wait_msg = f"⏳ Waiting for {part} at {new_rail}... ({elapsed}s)"
                                    logging.info(wait_msg)
                                    socketio.emit("status", {
                                        "state": "WAITING",
                                        "step": i + 1,
                                        "total": total,
                                        "message": wait_msg,
                                        "part": part,
                                        "station": new_rail,
                                    })

                                    # ── Trigger indicator alert once threshold is crossed ──
                                    ind_threshold = config.INDICATOR_ALERT_TIMEOUT
                                    if (self.indicator and ind_threshold > 0
                                            and elapsed >= ind_threshold
                                            and not _ind_alert_sent):
                                        logging.warning(
                                            "🔴 No part detected for %ds — activating Red+Buzzer", elapsed
                                        )
                                        socketio.emit("vision_log", {
                                            "level": "warn",
                                            "message": (
                                                f"🔔 No {part} at {new_rail} after {elapsed}s — "
                                                f"Red LED ON + Buzzer active. Place part to continue."
                                            )
                                        })
                                        threading.Thread(
                                            target=self.indicator.alert, daemon=True
                                        ).start()
                                        _ind_alert_sent = True

                                    if _timeout > 0 and elapsed >= _timeout:
                                        logging.warning("⏱ Timeout: %s not found at %s after %ds", part, new_rail, _timeout)
                                        self.robot.emergency_stop()
                                        # Ensure alert is active on timeout
                                        if self.indicator and not _ind_alert_sent:
                                            threading.Thread(
                                                target=self.indicator.alert, daemon=True
                                            ).start()
                                        with self._lock:
                                            self._state = "ERROR"
                                            self._error_msg = (
                                                f"❌ {part} not placed at {new_rail} within {_timeout}s. "
                                                f"Place part and restart."
                                            )
                                        socketio.emit("error", {
                                            "part": part,
                                            "position": new_rail,
                                            "message": self._error_msg,
                                        })
                                        socketio.emit("vision_log", {"level": "error", "message": f"✗ {part} not found at {new_rail}"})
                                        return  # finally sets _vision_detection_active = False

                                    interruptible_sleep(1.0, self._stop_event)

                                if self._stop_event.is_set() or not found:
                                    return  # finally sets _vision_detection_active = False

                                # Part confirmed — light up green and clear the alert
                                if self.indicator:
                                    threading.Thread(
                                        target=self.indicator.green, daemon=True
                                    ).start()

                                # Snapshot is already saved by vision_thread at the exact moment
                                # stable_detected_parts[part] transitioned to True.

                                socketio.emit("part_verified", {
                                    "part": part,
                                    "rail": new_rail,
                                    "count": count,
                                    "snapshot_ts": int(time.time() * 1000),
                                })
                                with _confirmed_parts_lock:
                                    _confirmed_parts.add(part)

                                success_msg = f"{part} confirmed at {new_rail} — resuming"
                                logging.info("✅ %s", success_msg)
                                self._emit_playing(i + 1, total, success_msg)

                                _vision_detection_active = False
                                socketio.emit("detection_stopped", {"part": part, "station": new_rail, "found": True})

                                # Return arm to waypoint position after scan (rail stays in place)
                                return_id = f"return_{time.time()}_{i}"
                                self._cmd_done_event.clear()
                                self._pending_cmd_id = return_id
                                if wp["type"] == "coords":
                                    self.robot.send_coords(wp["data"], wp_speed, int(wp.get("moveMode", 0)), cmd_id=return_id)
                                else:
                                    self.robot.send_angles(wp["data"], wp_speed, cmd_id=return_id)
                                self._cmd_done_event.wait(timeout=20)

                            finally:
                                # Only emit detection_stopped here if the success path didn't already
                                # clear the flag — avoids a spurious found=False after a found=True.
                                if _vision_detection_active:
                                    socketio.emit("detection_stopped", {"part": part, "station": new_rail, "found": False})
                                _vision_detection_active = False  # always revert on any exit
                                _detection_target_part = None    # clear target so vision_thread returns to station-filter mode

                    current_rail = new_rail

                if not self._stop_event.is_set() and not dynamic_mode:
                    wp_delay = wp.get("delay_ms", delay_ms)
                    if wp_delay > 0:
                        interruptible_sleep(wp_delay / 1000.0, self._stop_event)

            if self._stop_event.is_set() or not loop:
                break
            run_start = start_index

        with self._lock:
            is_active = (threading.current_thread() == self._thread)
            
        if is_active:
            if self._stop_event.is_set():
                with self._lock:
                    self._state = "IDLE"
                socketio.emit("status", {"state": "IDLE", "step": 1, "total": total, "message": "Playback stopped."})
                socketio.emit("playback_progress", {"state": "idle", "index": -1, "total": total, "name": ""})
            else:
                with self._lock:
                    self._state = "COMPLETE"
                socketio.emit("complete", {"message": "Both cars assembled successfully"})
                socketio.emit("status", {
                    "state": "COMPLETE", 
                    "step": total, 
                    "total": total, 
                    "message": "Both cars assembled successfully"
                })
                socketio.emit("playback_complete", {})
                socketio.emit("playback_progress", {"state": "idle", "index": -1, "total": total, "name": ""})
                # Sequence finished — turn all indicators off
                if self.indicator:
                    threading.Thread(target=self.indicator.off, daemon=True).start()
# ─────────────────────────────────────────────────────────────────────────────
# Flask + SocketIO Setup
# ─────────────────────────────────────────────────────────────────────────────
app      = Flask(__name__, template_folder="templates")
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")
robot    = RobotManager()  
rail     = RailManager()
store    = WaypointStore(filepath=os.path.join(os.path.dirname(__file__), "waypoints_wifi.json"))
indicator = RailIndicatorManager(rail)   # ESP32 relay indicator controller
playback = PlaybackEngine(robot, rail, store, indicator)

robot.completion_callbacks.append(playback.on_command_complete)

def _push_waypoints():
    socketio.emit("waypoints_updated", {"waypoints": store.get_all()})


@socketio.on("connect")
def on_client_connect():
    """Push current state snapshot to every newly-connected dashboard client."""
    wps = store.get_all()
    total = len(wps)
    idx = playback._current_index

    # 1. Progress bar
    emit("playback_progress", {
        "state": playback.state.lower(),
        "index": idx if playback.state != "IDLE" else 0,
        "total": total,
        "name": wps[idx].get("name", "") if 0 <= idx < total else ""
    })

    # 2. Status text + state
    emit("status", {
        "state": playback.state,
        "step": idx + 1,
        "total": total,
        "message": playback._error_msg if playback.state == "ERROR" else (
            "Ready — waiting for playback." if playback.state == "IDLE"
            else f"Running step {idx + 1} of {total}"
        )
    })

    # 3. Full arm + rail state so the connection badge updates immediately on page load
    with detection_lock:
        detected = [p for p, v in stable_detected_parts.items() if v]
    emit("robot_status", {
        **robot.state,
        "rail": rail.current_state,
        "vision_detected": detected
    })

    # 4. Send snapshot timestamp so the browser can initialise its baseline.
    # NOTE: do NOT emit 'snapshot_updated' here — that event triggers window.location.reload()
    # which would cause an infinite reload loop on every page load.
    emit("snapshot_ts", {"ts": _snapshot_timestamp})

    # 5. Welcome log line
    emit("vision_log", {
        "level": "info",
        "message": f"[Dashboard] Connected — {total} waypoints loaded, state: {playback.state}"
    })


# ── Vision Camera Background Processing Thread ──────────────────────────────
def get_annotated_mock_frame(w=1280, h=720):
    frame = np.zeros((h, w, 3), dtype=np.uint8)
    for x in range(0, w, 80):
        cv2.line(frame, (x, 0), (x, h), (25, 27, 33), 1)
    for y in range(0, h, 80):
        cv2.line(frame, (0, y), (w, y), (25, 27, 33), 1)
        
    cv2.rectangle(frame, (20, 20), (w-20, h-20), (0, 210, 255), 2)
    cv2.putText(frame, "INTEGRATED COCKPIT CAMERA FEED - SIMULATED", (40, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 210, 255), 2)

    pulse = int(127 + 127 * np.sin(time.time() * 3.5))
    cv2.circle(frame, (w - 50, 45), 8, (0, pulse, 0), -1)
    cv2.putText(frame, "LIVE", (w - 100, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, pulse, 0), 2)

    cv2.putText(frame, f"TELEMETRY TIME: {time.strftime('%Y-%m-%d %H:%M:%S')}", (40, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (120, 130, 145), 1)

    y_pos = 160
    with mock_lock:
        for part, active in mock_detections.items():
            if active:
                if part in ["Chassis", "WheelBase"]:
                    tag_id = 0 if part == "Chassis" else 1
                    cx, cy = 300 + tag_id * 350, 360
                    cv2.rectangle(frame, (cx-80, cy-80), (cx+80, cy+80), (0, 255, 136), 3)
                    cv2.rectangle(frame, (cx-45, cy-45), (cx+45, cy+45), (10, 10, 15), -1)
                    cv2.putText(frame, f"TAG16H5 (ID {tag_id})", (cx-70, cy-95),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 136), 2)
                    cv2.putText(frame, f"STABLE: {part}", (cx-70, cy+110),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 136), 2)
                    cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)
                else:
                    is_body = "Body" in part
                    cx, cy = (300 if "1" in part else 640) if is_body else 700, 530
                    color_bgr = (0, 255, 0) if is_body else (255, 0, 0)
                    color_label = f"BLACK PART ({part})" if is_body else "BLUE STICKER (Wheels)"
                    cv2.circle(frame, (cx, cy), 45, color_bgr, -1)
                    cv2.rectangle(frame, (cx-65, cy-65), (cx+65, cy+65), color_bgr, 2)
                    cv2.putText(frame, color_label, (cx-75, cy-80),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, color_bgr, 2)
                    cv2.putText(frame, f"STABLE: {part}", (cx-75, cy+95),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, color_bgr, 2)

            status_color = (0, 255, 136) if active else (110, 120, 135)
            status_text = "DETECTED" if active else "MISSING"
            cv2.putText(frame, f"{part}: {status_text}", (40, y_pos),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, status_color, 2)
            y_pos += 30
            
    return frame

def vision_thread():
    global latest_frame, latest_annotated_frame, latest_raw_frame_encoded, stable_detected_parts, last_detection_snapshot
    try:
        camera_idx = getattr(config, "CAMERA_INDEX", 0)
        backend = getattr(config, "CAMERA_BACKEND", None)
        backend_val = getattr(cv2, backend) if backend and hasattr(cv2, backend) else None

        if backend_val is not None:
            logging.info("📹 Opening VideoCapture(%d) with %s backend...", camera_idx, backend)
            cap = cv2.VideoCapture(camera_idx, backend_val)
        else:
            logging.info("📹 Opening VideoCapture(%d)...", camera_idx)
            cap = cv2.VideoCapture(camera_idx)
    except Exception as e:
        logging.error("❌ Exception during VideoCapture initialization: %s", e)
        cap = cv2.VideoCapture()  # fallback to an empty/unopened capture object

    if cap.isOpened():
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)                                        # drop stale frame queue → lower latency
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')) # MJPEG: USB bandwidth limit → 640x480 is stable on this Jetson USB controller
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)

        # Load saved settings from disk once the camera feed is successfully opened
        _cs_path = os.path.join(os.path.dirname(__file__), "camera_settings.json")
        if os.path.exists(_cs_path):
            try:
                with open(_cs_path) as _f:
                    with _camera_settings_lock:
                        _camera_settings.update(json.load(_f))
                logging.info("📷 Camera settings loaded from camera_settings.json")
            except Exception as _e:
                logging.warning("⚠️ Could not load camera_settings.json: %s", _e)

        _apply_camera_settings(cap)
    else:
        logging.warning("⚠️ Physical camera could not be opened. Automatically falling back to Simulated Mock Camera Stream.")

    
    _parts = list(stable_detected_parts.keys())
    frame_seen    = {p: 0 for p in _parts}
    frame_missing = {p: 0 for p in _parts}
    _prev_stable  = {p: False for p in _parts}  # track transitions for live log events

    logging.info("Unified Vision thread initialized.")
    
    while True:
        try:
            loop_start = time.time()

            if not cap.isOpened():
                now = time.time()
                if not hasattr(vision_thread, '_last_reopen_attempt'):
                    vision_thread._last_reopen_attempt = 0
                if now - vision_thread._last_reopen_attempt > 5.0:
                    vision_thread._last_reopen_attempt = now
                    logging.info("📹 Retrying to open VideoCapture(%d) with %s backend...", camera_idx, backend)
                    try:
                        if backend_val is not None:
                            cap = cv2.VideoCapture(camera_idx, backend_val)
                        else:
                            cap = cv2.VideoCapture(camera_idx)
                        
                        if cap.isOpened():
                            logging.info("✅ Physical camera opened successfully on retry!")
                            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
                            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                            cap.set(cv2.CAP_PROP_FPS, 30)

                            # Load saved settings from disk once the camera feed is successfully opened
                            _cs_path = os.path.join(os.path.dirname(__file__), "camera_settings.json")
                            if os.path.exists(_cs_path):
                                try:
                                    with open(_cs_path) as _f:
                                        with _camera_settings_lock:
                                            _camera_settings.update(json.load(_f))
                                    logging.info("📷 Camera settings loaded from camera_settings.json")
                                except Exception as _e:
                                    logging.warning("⚠️ Could not load camera_settings.json: %s", _e)

                            _apply_camera_settings(cap)
                    except Exception as e:
                        logging.error("❌ Exception during VideoCapture retry: %s", e)

            if _camera_settings_dirty.is_set():
                _camera_settings_dirty.clear()
                _apply_camera_settings(cap)

            ret, frame = False, None
            if cap.isOpened():
                ret, frame = cap.read()

            if not ret or frame is None:
                # Fallback mock frame
                frame = get_annotated_mock_frame(1280, 720)
                with mock_lock:
                    current_mocks = dict(mock_detections)
                with detection_lock:
                    for part in stable_detected_parts:
                        stable_detected_parts[part] = current_mocks[part]

                # Populate simulated coordinates for active mocks
                mock_coords = {}
                mock_raw = []
                for part, active in current_mocks.items():
                    if active:
                        if "Chassis" in part:
                            tag_id = 0 if "1" in part else 1
                            cx, cy = 300 + tag_id * 350, 360
                            mock_coords[part] = [cx, cy]
                            mock_raw.append({"station": "P1", "layer": "AprilTag", "tag_id": tag_id, "cx": cx, "cy": cy})
                        elif "WheelBase" in part:
                            tag_id = 2 if "1a" in part else (3 if "2a" in part else (4 if "1b" in part else 5))
                            cx, cy = 200 + tag_id * 100, 240
                            mock_coords[part] = [cx, cy]
                            mock_raw.append({"station": "P2", "layer": "AprilTag", "tag_id": tag_id, "cx": cx, "cy": cy})
                        elif "Body" in part:
                            cx, cy = (300 if "1" in part else 640), 530
                            mock_coords[part] = [cx, cy]
                            mock_raw.append({"station": "P5", "layer": "BlackROI", "tag_id": part, "cx": cx, "cy": cy})
                        elif part == "Wheels":
                            cx, cy = 700, 530
                            mock_coords[part] = [cx, cy]
                            mock_raw.append({"station": "P4", "layer": "HSV", "tag_id": "Wheels", "cx": cx, "cy": cy})
                with latest_tag_coords_lock:
                    latest_tag_coords.clear()
                    latest_tag_coords.update(mock_coords)
                with latest_raw_tags_lock:
                    latest_raw_tags.clear()
                    latest_raw_tags.extend(mock_raw)

                _, _mock_buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                _mock_encoded = _mock_buf.tobytes()
                with frame_lock:
                    latest_frame = frame.copy()
                    latest_raw_frame_encoded = _mock_encoded
                    latest_annotated_frame = _mock_encoded
                _frame_ready.set()

                time.sleep(max(0.01, 0.033 - (time.time() - loop_start)))
                continue

            # Downscale to 640×480 for stream — 4× fewer pixels = lower latency, smaller JPEG
            # CV detection still uses full-res frame below
            _stream_raw = cv2.resize(frame, (640, 480))
            _, _raw_buf = cv2.imencode('.jpg', _stream_raw, [cv2.IMWRITE_JPEG_QUALITY, 85])
            _raw_encoded = _raw_buf.tobytes()
            with frame_lock:
                latest_raw_frame_encoded = _raw_encoded
                latest_frame = frame

            # Process any stale-state reset requests from PlaybackEngine
            with _frame_seen_reset_lock:
                for _rp in list(_frame_seen_reset.keys()):
                    frame_seen[_rp]    = 0
                    frame_missing[_rp] = 0
                    del _frame_seen_reset[_rp]

            is_active_this_frame = _vision_detection_active
            annotated_frame = frame.copy()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(frame, (5, 5), 0)
            hsv     = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            detected_this_frame = {
                "Chassis 1": False, "Chassis 2": False,
                "WheelBase 1a": False, "WheelBase 2a": False, "WheelBase 1b": False, "WheelBase 2b": False,
                "Wheels": False,
                "Wheel Slot 1": False, "Wheel Slot 2": False, "Wheel Slot 3": False, "Wheel Slot 4": False,
                "Wheel Slot 5": False, "Wheel Slot 6": False, "Wheel Slot 7": False, "Wheel Slot 8": False,
                "Body 1": False, "Body 2": False,
            }
            raw_detections = dict(detected_this_frame)  # pre-stable, for overlay
            # draw_instructions: list of (part_name, roi, ratio)
            draw_instructions = []

            # ── Black colour detection per ROI ────────────────────────────
            current_station = rail.current_state.get("point")  # "P1","P2","P4","P5" or None
            active_parts = config.STATION_PARTS.get(current_station, [])  # empty = no detection

            # Exclude parts already confirmed+picked this sequence run
            with _confirmed_parts_lock:
                _done = set(_confirmed_parts)
            active_parts = [p for p in active_parts if p not in _done]

            # During an active vision scan the PlaybackEngine sets _detection_target_part to the
            # exact part it is waiting for. Add it to active_parts so detection still runs even
            # when the rail state reports None (disconnection, polling lag, etc.).
            forced_part = _detection_target_part  # read snapshot — written by PlaybackEngine thread
            if forced_part and forced_part not in active_parts and forced_part not in _done:
                active_parts = list(active_parts) + [forced_part]

            current_coords = {}
            raw_tags_list = []
            # Always build draw_instructions for every active-station part so ROI boxes
            # are visible as a live overlay regardless of whether a sequence is running.
            for part_name in list(detected_this_frame.keys()):
                if part_name not in active_parts:
                    continue
                roi = _get_roi(part_name)
                if roi is None:
                    continue
                present, ratio = _detect_black_in_roi(hsv, roi)
                # Only update detection state + coords during an active vision check —
                # prevents false positives from ambient black objects during transit.
                if is_active_this_frame:
                    detected_this_frame[part_name] = present
                    raw_detections[part_name] = present
                    if present:
                        x0, y0, x1, y1 = roi
                        current_coords[part_name] = [(x0 + x1) // 2, (y0 + y1) // 2]
                        raw_tags_list.append({
                            "station": current_station,
                            "layer": "BlackROI",
                            "tag_id": part_name,
                            "cx": current_coords[part_name][0],
                            "cy": current_coords[part_name][1],
                            "ratio": round(ratio, 3),
                        })
                        logging.debug("[BlackROI] %s present ratio=%.3f roi=%s", part_name, ratio, roi)
                draw_instructions.append((part_name, roi, ratio))

            with latest_tag_coords_lock:
                latest_tag_coords.clear()
                latest_tag_coords.update(current_coords)

            with latest_raw_tags_lock:
                latest_raw_tags.clear()
                latest_raw_tags.extend(raw_tags_list)

            # ── Periodic summary — console log + Live Feed push every 90 frames (~3s) ─
            _vision_frame_count[0] += 1
            if _vision_frame_count[0] % 90 == 0:
                raw_found  = [p for p, v in raw_detections.items()     if v]
                stable_now = [p for p, v in stable_detected_parts.items() if v]
                logging.info("[Vision] frame=%d | raw_parts=%s | stable=%s | coords=%s",
                             _vision_frame_count[0],
                             raw_found  if raw_found  else "none",
                             stable_now if stable_now else "none",
                             current_coords)
                socketio.emit("robot_status", {"vision_detected": stable_now})

            # Update stable detections + emit live log on state transitions.
            # ONLY accumulate when _vision_detection_active — prevents false positives
            # from arm movement or other black objects while not at a checking position.
            transitions = []  # collect outside lock to avoid emitting inside it
            if is_active_this_frame:
                with detection_lock:
                    for part in stable_detected_parts.keys():
                        with mock_lock:
                            mocked = mock_detections[part]

                        if detected_this_frame[part] or mocked:
                            frame_missing[part] = 0
                            frame_seen[part] += 1
                            if frame_seen[part] >= config.CONFIRM_FRAMES:
                                was = stable_detected_parts[part]
                                stable_detected_parts[part] = True
                                if not was:  # MISSING → STABLE transition
                                    transitions.append(("stable", part))
                        else:
                            frame_seen[part] = 0
                            frame_missing[part] += 1
                            if frame_missing[part] >= config.DISAPPEAR_FRAMES:
                                was = stable_detected_parts[part]
                                stable_detected_parts[part] = False
                                if was:  # STABLE → MISSING transition
                                    transitions.append(("lost", part))

            for kind, part in transitions:
                if kind == "stable":
                    logging.info("[Vision] %s → STABLE", part)
                    socketio.emit("vision_log", {"level": "success", "message": f"✓ {part} detected (stable)"})
                else:
                    logging.warning("[Vision] %s → MISSING", part)
                    socketio.emit("vision_log", {"level": "warning", "message": f"⚠ {part} lost"})


            # ── Draw ROI boxes ONLY during active detection scan ──────────────
            # Boxes appear when PlaybackEngine triggers a vision check and
            # disappear automatically once detection is confirmed or stopped.
            if is_active_this_frame:
                for part_name, roi, ratio in draw_instructions:
                    x0, y0, x1, y1 = roi
                    with detection_lock:
                        is_stable = stable_detected_parts.get(part_name, False)
                    raw_seen = raw_detections.get(part_name, False)

                    if is_stable:
                        color = (0, 255, 0)
                        if "WheelBase" in part_name: color = (255, 165, 0)
                        elif "Wheel" in part_name:   color = (255, 80,  0)
                        thickness = 2
                    elif raw_seen:
                        color = (0, 220, 255)  # yellow — accumulating frames
                        thickness = 1
                    else:
                        color = (80, 90, 100)  # grey — absent
                        thickness = 1

                    cv2.rectangle(annotated_frame, (x0, y0), (x1, y1), color, thickness)
                    cx_b, cy_b = (x0 + x1) // 2, (y0 + y1) // 2
                    cv2.circle(annotated_frame, (cx_b, cy_b), 3, color, -1)
                    _chk = "\u2713" if is_stable else "?"
                    lbl = f"{part_name} {_chk} {ratio*100:.0f}%"
                    cv2.rectangle(annotated_frame, (x0, y0 - 14), (x0 + 160, y0), color, -1)
                    cv2.putText(annotated_frame, lbl, (x0 + 2, y0 - 3),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.32, (0, 0, 0), 1)

            # Status panel — only show active parts for current station
            panel_parts = active_parts if active_parts else []
            station_label = current_station or "TRANSIT"
            panel_h = 20 + max(len(panel_parts), 1) * 16 + 6
            cv2.rectangle(annotated_frame, (5, 5), (220, panel_h), (10, 11, 14), -1)
            cv2.rectangle(annotated_frame, (5, 5), (220, panel_h), (50, 60, 75), 1)
            cv2.putText(annotated_frame, f"STN: {station_label}", (10, 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)

            y_pos = 32
            if not panel_parts:
                cv2.putText(annotated_frame, "No active station", (10, y_pos),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.30, (120, 130, 145), 1)
            else:
                with detection_lock:
                    snap = dict(stable_detected_parts)
                for part in panel_parts:
                    stable_p = snap.get(part, False)
                    raw_p = raw_detections.get(part, False)
                    if stable_p:
                        lbl_color = (0, 255, 136)
                        status_txt = "OK"
                    elif raw_p:
                        lbl_color = (0, 220, 255)
                        status_txt = "..."
                    else:
                        lbl_color = (120, 130, 145)
                        status_txt = "--"
                    cv2.putText(annotated_frame, f"{part}: {status_txt}", (10, y_pos),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.30, lbl_color, 1)
                    y_pos += 16

            # ── Bottom debug bar ─────────────────────────────────────────────
            h_frame = annotated_frame.shape[0]
            det_info = f"Stn:{station_label} | thr={config.BLACK_PIXEL_RATIO} | f#{_vision_frame_count[0]}"
            cv2.rectangle(annotated_frame, (0, h_frame - 16), (annotated_frame.shape[1], h_frame), (20, 20, 20), -1)
            cv2.putText(annotated_frame, det_info, (6, h_frame - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.30, (160, 160, 160), 1)


            # Downscale annotated frame to 640×480 for stream
            _stream_annotated = cv2.resize(annotated_frame, (640, 480))
            _, buffer = cv2.imencode('.jpg', _stream_annotated, [cv2.IMWRITE_JPEG_QUALITY, 80])
            _encoded = buffer.tobytes()
            with frame_lock:
                latest_annotated_frame = _encoded
            _frame_ready.set()  # wake generate_stream() immediately

            # Update last-detection snapshot only during an active vision check
            # (arm is at a checking station) and only when a part transitions to stable.
            # This prevents random environmental detections from overwriting the snapshot.
            if is_active_this_frame and any(k == "stable" for k, _ in transitions):
                with last_detection_snapshot_lock:
                    last_detection_snapshot = latest_annotated_frame
                _save_snapshot(latest_annotated_frame)
                socketio.emit("snapshot_updated", {"ts": int(time.time() * 1000)})

            time.sleep(max(0.005, 0.033 - (time.time() - loop_start)))
        except Exception as e:
            logging.error("❌ Exception in continuous vision loop: %s\n%s", e, traceback.format_exc())
            time.sleep(0.1)

threading.Thread(target=vision_thread, daemon=True).start()


# ── Consolidated HTTP Endpoints ──────────────────────────────────────────────

@app.route("/")
def home_showcase():
    """Renders the Mouser Roadshow showcase dashboard (main landing page)."""
    return render_template("home_dashboard.html")


@app.route("/camera_view")
def index():
    """Renders the Camera View / Operator Cockpit."""
    return render_template("dashboard.html")


@app.route("/editor")
def waypoint_editor():
    """Renders the full-featured Waypoint Recording & Editing Dashboard."""
    return render_template("waypoint_dashboard_wifi.html",
                           rail_rpm=config.RAIL_SPEED_RPM,
                           arm_ip=robot.serial_port or "")


@app.route("/home_snapshot")
def home_snapshot():
    """Returns the last verified-detection frame as JPEG. 204 if no detection has occurred yet."""
    with last_detection_snapshot_lock:
        frame = last_detection_snapshot
    if frame is None:
        resp = Response(status=204)
    else:
        resp = Response(frame, mimetype="image/jpeg")
    resp.headers['Cache-Control'] = 'no-store, no-cache, must-revalidate'
    resp.headers['Pragma'] = 'no-cache'
    return resp


@app.route("/home_snapshot_ts")
def home_snapshot_ts():
    """Returns the millisecond timestamp of the last saved snapshot.
    Lightweight endpoint — browser polls this to detect new detections without
    downloading the full JPEG on every tick.
    """
    return jsonify({"ts": _snapshot_timestamp})


@app.route("/api/camera_settings", methods=["GET"])
def get_camera_settings():
    logging.info("📷 GET /api/camera_settings called")
    with _camera_settings_lock:
        return jsonify(dict(_camera_settings))


@app.route("/api/camera_settings", methods=["POST"])
def set_camera_settings():
    data = request.get_json(force=True) or {}
    logging.info("📷 POST /api/camera_settings called with data: %s", data)
    with _camera_settings_lock:
        _camera_settings.update(data)
    _camera_settings_dirty.set()
    _cs_path = os.path.join(os.path.dirname(__file__), "camera_settings.json")
    with _camera_settings_lock:
        with open(_cs_path, "w") as f:
            json.dump(dict(_camera_settings), f, indent=2)
    # Broadcast status to cockpit's Live Status Feed
    socketio.emit("vision_log", {"level": "success", "message": "📷 Camera settings updated & saved successfully."})
    return jsonify({"success": True})


@app.route("/api/camera_settings/reset", methods=["POST"])
def reset_camera_settings():
    logging.info("📷 POST /api/camera_settings/reset called")
    _DEFAULTS = {
        "brightness":    0,
        "contrast":      0,
        "saturation":    32,
        "gain":          0,
        "sharpness":     0,
        "auto_exposure": 1,   # 1 = auto ON; sets CAP_PROP_AUTO_EXPOSURE to 3.0 (Aperture Priority)
        # "exposure" intentionally omitted — let auto-exposure control it
    }
    with _camera_settings_lock:
        _camera_settings.clear()
        _camera_settings.update(_DEFAULTS)
    _camera_settings_dirty.set()
    _cs_path = os.path.join(os.path.dirname(__file__), "camera_settings.json")
    try:
        with open(_cs_path, "w") as _f:
            json.dump(_DEFAULTS, _f, indent=2)
    except Exception as _e:
        logging.warning("⚠️ Could not write reset camera_settings.json: %s", _e)
    # Broadcast reset to cockpit's Live Status Feed
    socketio.emit("vision_log", {"level": "info", "message": "📷 Camera settings reset to defaults."})
    socketio.emit("camera_settings_updated", _DEFAULTS)
    return jsonify({"success": True, "settings": _DEFAULTS})




@app.route("/play", methods=["POST"])
def dashboard_play():
    if playback.state == "PLAYING":
        return jsonify({"success": False, "error": "Already playing"})
    
    # Safely stop and reset the index
    playback.restart()
    
    # Read optional play parameters passed from client body (fallback to defaults if absent)
    data = request.get_json(force=True) if request.data else {}
    speed = int(data.get("speed", 40))
    delay_ms = int(data.get("delay_ms", 200))
    loop = bool(data.get("loop", False))
    async_rail = bool(data.get("async_rail", False))
    dynamic_mode = bool(data.get("dynamic_mode", False))
    
    res = playback.play(
        start_index=0, 
        speed=speed, 
        delay_ms=delay_ms, 
        loop=loop, 
        async_rail=async_rail, 
        dynamic_mode=dynamic_mode
    )
    return jsonify(res)


@app.route("/stop", methods=["POST"])
def dashboard_stop():
    res = playback.stop()
    return jsonify(res)


@app.route("/restart", methods=["POST"])
def dashboard_restart():
    res = playback.restart()
    return jsonify(res)


@app.route("/state")
def dashboard_state():
    wps = store.get_all()
    step_num = playback._current_index + 1 if playback.state != "IDLE" else 1
    return jsonify({
        "state": playback.state,
        "step": step_num,
        "total": len(wps),
        "message": playback._error_msg if playback.state == "ERROR" else ""
    })


# ── Vision Endpoints (Served Locally from Same Process) ──────────────────────

@app.route("/check")
def check_part():
    part = request.args.get("part")
    if not part:
        return jsonify({"error": "Missing part query parameter"}), 400
    method = "BlackROI"
    with detection_lock:
        found = stable_detected_parts.get(part, False)
    return jsonify({"found": found, "part": part, "method": method})


@app.route("/status")
def vision_status():
    detected_list = []
    with detection_lock:
        for part, found in stable_detected_parts.items():
            if found: detected_list.append(part)
    return jsonify({"detected": detected_list, "timestamp": time.time()})


def generate_stream():
    while True:
        _frame_ready.wait(timeout=0.1)
        _frame_ready.clear()
        if _vision_detection_active or debug_overlays_enabled:
            with frame_lock:
                frame_bytes = latest_annotated_frame
        else:
            # Raw mode: use pre-encoded bytes from vision thread — zero encoding cost here
            with frame_lock:
                frame_bytes = latest_raw_frame_encoded
        if frame_bytes is None:
            continue
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')


@app.route("/stream")
def video_stream():
    resp = Response(generate_stream(), mimetype='multipart/x-mixed-replace; boundary=frame')
    resp.headers['Cache-Control']     = 'no-cache, no-store, must-revalidate'
    resp.headers['X-Accel-Buffering'] = 'no'
    return resp


@app.route("/frame")
def single_frame():
    """Return the latest raw camera frame as a single JPEG (640×480, no CV overlays)."""
    with frame_lock:
        data = latest_raw_frame_encoded
    if data is None:
        return Response(status=503)
    return Response(data, mimetype='image/jpeg',
                    headers={'Cache-Control': 'no-cache, no-store, must-revalidate'})


@app.route("/mock", methods=["POST"])
def configure_mock():
    data = request.get_json(force=True) if request.data else {}
    part = data.get("part")
    state = bool(data.get("state", False))
    if part not in mock_detections:
        return jsonify({"success": False, "error": f"Invalid part: {part}"}), 400
    with mock_lock:
        mock_detections[part] = state
    logging.info("🔧 Mock toggle: %s set to %s", part, state)
    return jsonify({"success": True, "part": part, "mock_state": state})


@app.route("/api/debug_overlays", methods=["GET"])
def get_debug_overlays():
    global debug_overlays_enabled
    return jsonify({"enabled": debug_overlays_enabled})


@app.route("/api/debug_overlays", methods=["POST"])
def toggle_debug_overlays():
    global debug_overlays_enabled
    data = request.get_json(force=True) if request.data else {}
    debug_overlays_enabled = bool(data.get("enabled", False))
    logging.info("🔧 Debug CV overlays set to %s", debug_overlays_enabled)
    # Broadcast to Live Status Feed
    msg = "📷 Live CV Debug Overlays ENABLED." if debug_overlays_enabled else "📷 Live CV Debug Overlays DISABLED."
    socketio.emit("vision_log", {"level": "info", "message": msg})
    return jsonify({"success": True, "enabled": debug_overlays_enabled})


# ── Robot & Rail Config APIs (Same as original) ──────────────────────────────

@app.route("/api/reconnect", methods=["POST"])
def api_reconnect():
    data = request.get_json(force=True) if request.data else {}
    port_val = data.get("ip") or data.get("serial_port")
    return jsonify(robot.reconnect(serial_port=port_val))

@app.route("/api/emergency_stop", methods=["POST"])
def api_emergency_stop():
    return jsonify(robot.emergency_stop())

@app.route("/api/power_on", methods=["POST"])
def api_power_on():
    return jsonify(robot.lock_servos())

@app.route("/api/power_off", methods=["POST"])
def api_power_off():
    return jsonify(robot.release_servos())

@app.route("/api/joint_power", methods=["POST"])
def api_joint_power():
    data = request.get_json(force=True) if request.data else {}
    return jsonify(robot.set_joint_power(data.get("joint_id"), data.get("state")))

@app.route("/api/vacuum/on", methods=["POST"])
def api_vacuum_on():
    data = request.get_json(force=True) if request.data else {}
    delay = int(data.get("delay", 0))
    return jsonify(robot.set_vacuum("on", delay))

@app.route("/api/vacuum/off", methods=["POST"])
def api_vacuum_off():
    data = request.get_json(force=True) if request.data else {}
    delay = int(data.get("delay", 0))
    return jsonify(robot.set_vacuum("off", delay))

@app.route("/api/rail/config", methods=["POST"])
def api_rail_config():
    data = request.get_json(force=True) if request.data else {}
    ip = data.get("ip")
    if ip:
        rail.save_config(ip)
        return jsonify({"success":True})
    return jsonify({"success":False})

@app.route("/api/rail/move", methods=["POST"])
def api_rail_move():
    data = request.get_json(force=True) if request.data else {}
    preset = data.get("preset", "HOME")
    return jsonify({"success": rail.move_to_preset(preset)})

@app.route("/api/rail/stop", methods=["POST"])
def api_rail_stop():
    return jsonify({"success": rail.stop()})

@app.route("/api/rail/speed", methods=["POST"])
def api_rail_speed():
    data = request.get_json(force=True) if request.data else {}
    rpm = data.get("rpm", 150)
    return jsonify({"success": rail.set_speed(rpm)})

@app.route("/api/get_calibration")
def api_get_calibration():
    return jsonify({"success":True,"calibration":{"coords":robot.coord_calibration,"angles":robot.angle_calibration}})

@app.route("/api/set_calibration", methods=["POST"])
def api_set_calibration():
    data = request.get_json(force=True) if request.data else {}
    cal_type = data.get("type","coords"); offsets = data.get("offsets",[])
    try:
        if cal_type == "coords" and len(offsets)==6:
            with robot.lock: robot.coord_calibration = [float(v) for v in offsets]
            robot.save_config()
            return jsonify({"success":True})
        elif cal_type == "angles" and len(offsets)==6:
            with robot.lock: robot.angle_calibration = [float(v) for v in offsets]
            robot.save_config()
            return jsonify({"success":True})
        return jsonify({"success":False,"error":"Invalid data"})
    except Exception as e:
        return jsonify({"success":False,"error":str(e)})

@app.route("/api/calibrate_from_point", methods=["POST"])
def api_calibrate_from_point():
    data = request.get_json(force=True) if request.data else {}
    idx = int(data.get("index", 0))
    wps = store.get_all()
    if not (0 <= idx < len(wps)):
        return jsonify({"success": False, "error": "Invalid waypoint index"})
    wp = wps[idx]
    current_state = robot.state
    try:
        if wp["type"] == "coords":
            target = wp["data"]
            actual = current_state["coords"]
            with robot.lock:
                new_offsets = [round(target[i] - (actual[i] - robot.coord_calibration[i]), 2) for i in range(6)]
                robot.coord_calibration = new_offsets
        else:
            target = wp["data"] if wp["type"] == "angles" else wp.get("angles", wp["data"])
            actual = current_state["angles"]
            with robot.lock:
                new_offsets = [round(target[i] - (actual[i] - robot.angle_calibration[i]), 2) for i in range(6)]
                robot.angle_calibration = new_offsets
        robot.save_config()
        return jsonify({
            "success": True, 
            "offsets": robot.coord_calibration if wp["type"]=="coords" else robot.angle_calibration,
            "type": wp["type"]
        })
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})

@app.route("/api/set_scan_pose", methods=["POST"])
def api_set_scan_pose():
    data = request.get_json(force=True) if request.data else {}
    position = data.get("position")
    if position not in ["P1", "P2", "P4", "P5"]:
        return jsonify({"success": False, "error": "Invalid position"})
    
    current_angles = list(robot.state["angles"])
    config.SCAN_POSES[position] = current_angles
    
    try:
        poses_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scan_poses.json")
        with open(poses_path, "w") as f:
            json.dump(config.SCAN_POSES, f, indent=4)
        logging.info("📸 Saved scan pose for %s: %s", position, current_angles)
        return jsonify({"success": True, "angles": current_angles})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})


# ── Waypoint APIs ────────────────────────────────────────────────────────────

@app.route("/api/waypoints")
def api_waypoints_get():
    return jsonify({"success":True,"waypoints":store.get_all()})

@app.route("/api/waypoints/add", methods=["POST"])
def api_waypoints_add():
    data = request.get_json(force=True) if request.data else {}
    mode = data.get("type","coords")
    s    = robot.state
    wp = {
        "type":     mode,
        "moveMode": int(data.get("moveMode", 0)),
        "speed":    int(data.get("speed", 40)),
        "name":     data.get("name",""),
        "rail_preset": data.get("rail_preset", "NONE"),
    }
    if mode == "both":
        wp["coords"] = list(s["coords"])
        wp["angles"] = list(s["angles"])
        wp["data"]   = list(s["angles"])
    else:
        wp["data"]   = list(s["coords"]) if mode=="coords" else list(s["angles"])
    idx = store.add(wp)
    _push_waypoints()
    return jsonify({"success":True,"index":idx})

@app.route("/api/waypoints/insert", methods=["POST"])
def api_waypoints_insert():
    """Insert a fully-formed waypoint dict at a specific index."""
    data = request.get_json(force=True) if request.data else {}
    insert_at = int(data.get("index", -1))
    wp = data.get("waypoint", {})
    if not wp:
        return jsonify({"success": False, "error": "No waypoint data"})
    wp.setdefault("timestamp", time.time())
    with store.lock:
        if insert_at < 0 or insert_at >= len(store.waypoints):
            store.waypoints.append(wp)
            result_idx = len(store.waypoints) - 1
        else:
            store.waypoints.insert(insert_at, wp)
            result_idx = insert_at
        store._save_locked()
    _push_waypoints()
    return jsonify({"success": True, "index": result_idx})

@app.route("/api/waypoints/update", methods=["POST"])
def api_waypoints_update():
    data = request.get_json(force=True) if request.data else {}
    idx   = int(data.get("index",0))
    field = data.get("field","")
    value = data.get("value")
    if field == "data":
        ok = store.update(idx, int(data.get("field_index",0)), value)
    elif field == "all_data":
        ok = store.update_all_data(idx, value)
    elif field in ("name", "speed", "vacuum", "vacuum_delay_ms", "delay_ms", "rail_preset", "is_reference", "check_part"):
        ok = store.update_field(idx, field, value)
    else:
        ok = False
    _push_waypoints()
    return jsonify({"success":ok})

@app.route("/api/waypoints/delete", methods=["POST"])
def api_waypoints_delete():
    data = request.get_json(force=True) if request.data else {}
    ok = store.delete(int(data.get("index",0)))
    _push_waypoints()
    return jsonify({"success":ok})

@app.route("/api/waypoints/clear", methods=["POST"])
def api_waypoints_clear():
    store.clear()
    _push_waypoints()
    return jsonify({"success":True})

@app.route("/api/waypoints/reorder", methods=["POST"])
def api_waypoints_reorder():
    data = request.get_json(force=True) if request.data else {}
    store.reorder(int(data.get("from",0)), int(data.get("to",0)))
    _push_waypoints()
    return jsonify({"success":True})

@app.route("/api/waypoints/duplicate", methods=["POST"])
def api_waypoints_duplicate():
    data = request.get_json(force=True) if request.data else {}
    new_idx = store.duplicate(int(data.get("index",0)))
    _push_waypoints()
    return jsonify({"success": new_idx >= 0, "index": new_idx})

@app.route("/api/waypoints/batch_update", methods=["POST"])
def api_waypoints_batch():
    data = request.get_json(force=True) if request.data else {}
    speed = data.get("speed")
    delay = data.get("delay_ms")
    with store.lock:
        for wp in store.waypoints:
            if speed is not None: wp["speed"] = int(speed)
            if delay is not None: wp["delay_ms"] = int(delay)
        store._save_locked()
    _push_waypoints()
    return jsonify({"success": True})

@app.route("/api/waypoints/export")
def api_waypoints_export():
    content = json.dumps(store.get_all(), indent=2)
    buf = io.BytesIO(content.encode())
    buf.seek(0)
    return send_file(buf, mimetype="application/json",
                     as_attachment=True, download_name="waypoints.json")

@app.route("/api/waypoints/import", methods=["POST"])
def api_waypoints_import():
    try:
        f = request.files.get("file")
        if not f: return jsonify({"success":False,"error":"No file"})
        data = json.load(f)
        if not isinstance(data, list): return jsonify({"success":False,"error":"Expected JSON array"})
        store.clear()
        for wp in data: store.add(wp)
        _push_waypoints()
        return jsonify({"success":True,"count":len(data)})
    except Exception as e:
        return jsonify({"success":False,"error":str(e)})


@app.route("/api/playback/play", methods=["POST"])
def api_playback_play():
    data = request.get_json(force=True) if request.data else {}
    start_index = int(data.get("start_index", 0))
    speed = int(data.get("speed", 40))
    delay_ms = int(data.get("delay_ms", 200))
    loop = bool(data.get("loop", False))
    async_rail = bool(data.get("async_rail", False))
    dynamic_mode = bool(data.get("dynamic_mode", False))
    skip_vision = bool(data.get("skip_vision", False))
    res = playback.play(start_index, speed, delay_ms, loop, async_rail, dynamic_mode, skip_vision)
    return jsonify(res)

@app.route("/api/playback/pause", methods=["POST"])
def api_playback_pause():
    return jsonify(playback.pause())

@app.route("/api/playback/resume", methods=["POST"])
def api_playback_resume():
    return jsonify(playback.resume())

@app.route("/api/playback/stop", methods=["POST"])
def api_playback_stop():
    return jsonify(playback.stop())

@app.route("/api/playback/step_forward", methods=["POST"])
def api_playback_step_forward():
    return jsonify({"success": False, "error": "Not implemented"})

@app.route("/api/playback/step_backward", methods=["POST"])
def api_playback_step_backward():
    return jsonify({"success": False, "error": "Not implemented"})


# ── SocketIO Communications ──────────────────────────────────────────────────

@socketio.on("send_coords")
def on_send_coords(data):
    try:
        coords = [float(data.get(k,0)) for k in ["x","y","z","rx","ry","rz"]]
        robot.send_coords(coords, int(data.get("speed",40) or 40),
                          int(data.get("mode",0) or 0), cmd_id=data.get("cmd_id"))
    except Exception as e:
        logging.error("send_coords: %s", e)

@socketio.on("send_angles")
def on_send_angles(data):
    try:
        angles = [float(data.get(f"j{i+1}",0)) for i in range(6)]
        robot.send_angles(angles, int(data.get("speed",40) or 40), cmd_id=data.get("cmd_id"))
    except Exception as e:
        logging.error("send_angles: %s", e)

@socketio.on("go_home")
def on_go_home():
    robot.send_angles([0,0,0,0,0,0], 20)

@socketio.on("command_complete")
def on_command_complete(data):
    playback.on_command_complete(data.get("cmd_id"))


# ── Vision Zone & Calibration APIs (Integrated from editor logic) ────────────

@app.route("/api/go_to_scan_pose", methods=["POST"])
def api_go_to_scan_pose():
    data = request.get_json(force=True) if request.data else {}
    position = (data.get("position") or "").upper()
    if position not in config.SCAN_POSES:
        return jsonify({"success": False, "error": f"Unknown position: {position}"}), 400
    angles = config.SCAN_POSES.get(position, [])
    if not angles or all(a == 0.0 for a in angles):
        return jsonify({"success": False, "error": f"No scan pose recorded for {position} yet. Record it first."}), 400
    robot.send_angles(angles, speed=100)
    return jsonify({"success": True, "position": position, "angles": angles})


@app.route("/api/zones", methods=["GET"])
def api_get_zones():
    return jsonify(config.ZONE_DEFINITIONS)


@app.route("/api/update_zone", methods=["POST"])
def api_update_zone():
    data = request.get_json(force=True) if request.data else {}
    station = data.get("station")
    part    = data.get("part")
    bbox    = data.get("bbox")
    if not (station and part and isinstance(bbox, list) and len(bbox) == 4):
        return jsonify({"success": False, "error": "Require station, part, bbox [x0,y0,x1,y1]"}), 400
    if station not in config.ZONE_DEFINITIONS:
        config.ZONE_DEFINITIONS[station] = {}
    config.ZONE_DEFINITIONS[station][part] = [int(v) for v in bbox]
    zones_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "zone_boundaries.json")
    try:
        with open(zones_path, "w") as f:
            json.dump(config.ZONE_DEFINITIONS, f, indent=2)
        return jsonify({"success": True})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 500


@app.route("/api/map_coordinate", methods=["POST"])
def api_map_coordinate():
    data = request.get_json(force=True) if request.data else {}
    part = data.get("part")
    cx = data.get("cx")
    cy = data.get("cy")
    
    if not part or cx is None or cy is None:
        return jsonify({"success": False, "error": "Missing part, cx, or cy"}), 400
        
    part_to_station = {
        "Chassis 1": "P1",
        "Chassis 2": "P1",
        "WheelBase 1a": "P2",
        "WheelBase 1b": "P2",
        "WheelBase 2a": "P2",
        "WheelBase 2b": "P2",
        "Wheel Slot 1": "P4",
        "Wheel Slot 2": "P4",
        "Wheel Slot 3": "P4",
        "Wheel Slot 4": "P4",
        "Wheel Slot 5": "P4",
        "Wheel Slot 6": "P4",
        "Wheel Slot 7": "P4",
        "Wheel Slot 8": "P4",
        "Body 1": "P5",
        "Body 2": "P5",
    }
    
    station = part_to_station.get(part)
    if not station:
        return jsonify({"success": False, "error": f"Unknown target part: {part}"}), 400
        
    # Generate standard +/- 80px bounding box tolerance
    tolerance = 80
    x0 = max(0, int(cx) - tolerance)
    y0 = max(0, int(cy) - tolerance)
    x1 = min(1280, int(cx) + tolerance)
    y1 = min(720, int(cy) + tolerance)
    
    if station not in config.ZONE_DEFINITIONS:
        config.ZONE_DEFINITIONS[station] = {}
        
    config.ZONE_DEFINITIONS[station][part] = [x0, y0, x1, y1]
    
    zones_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "zone_boundaries.json")
    try:
        with open(zones_path, "w") as f:
            json.dump(config.ZONE_DEFINITIONS, f, indent=2)
        logging.info("🎯 Mapped %s to center [%d, %d] -> bbox [%d, %d, %d, %d]", part, cx, cy, x0, y0, x1, y1)
        return jsonify({"success": True, "station": station, "part": part, "bbox": [x0, y0, x1, y1]})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 500


@app.route("/api/set_roi", methods=["POST"])
def api_set_roi():
    data = request.get_json(force=True) if request.data else {}
    part = data.get("part")
    roi  = data.get("roi")
    if not part or not isinstance(roi, list) or len(roi) != 4:
        return jsonify({"success": False, "error": "Provide part and roi=[x0,y0,x1,y1]"}), 400
    x0, y0, x1, y1 = [int(v) for v in roi]
    if x0 >= x1 or y0 >= y1:
        return jsonify({"success": False, "error": "roi must have x0<x1 and y0<y1"}), 400
    config.ROI_DEFINITIONS[part] = [x0, y0, x1, y1]
    roi_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "roi_boundaries.json")
    try:
        with open(roi_path, "w") as f:
            json.dump(config.ROI_DEFINITIONS, f, indent=2)
        logging.info("ROI saved: %s → [%d,%d,%d,%d]", part, x0, y0, x1, y1)
        return jsonify({"success": True, "part": part, "roi": [x0, y0, x1, y1]})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 500


@app.route("/api/get_roi", methods=["GET"])
def api_get_roi():
    rois = {}
    all_parts = list(stable_detected_parts.keys())
    for part in all_parts:
        roi = _get_roi(part)
        if roi:
            rois[part] = roi
    return jsonify({"rois": rois, "threshold": config.BLACK_PIXEL_RATIO})


@app.route("/api/reset_roi", methods=["POST"])
def api_reset_roi():
    data = request.get_json(force=True) if request.data else {}
    part = data.get("part")
    if not part:
        return jsonify({"success": False, "error": "Provide part"}), 400
    config.ROI_DEFINITIONS.pop(part, None)
    roi_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "roi_boundaries.json")
    try:
        with open(roi_path, "w") as f:
            json.dump(config.ROI_DEFINITIONS, f, indent=2)
        return jsonify({"success": True, "part": part, "note": "reverted to ZONE_DEFINITIONS default"})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 500


@app.route("/api/set_vision_timeout", methods=["POST"])
def api_set_vision_timeout():
    data = request.get_json(force=True) if request.data else {}
    seconds = int(data.get("seconds", 0))
    config.VISION_WAIT_TIMEOUT = max(0, seconds)
    logging.info("Vision wait timeout set to %ds (0 = infinite)", config.VISION_WAIT_TIMEOUT)
    return jsonify({"success": True, "timeout": config.VISION_WAIT_TIMEOUT})


# ── Broadcast Loop ──────────────────────────────────────────────────────────

def background_broadcast():
    while True:
        rail.poll()
        
        # Collect active detections
        detected_list = []
        with detection_lock:
            for part, found in stable_detected_parts.items():
                if found: detected_list.append(part)
                
        # Collect latest tag coordinates snapshot
        with latest_tag_coords_lock:
            tag_coords_snapshot = dict(latest_tag_coords)
            
        # Collect latest raw tags snapshot
        with latest_raw_tags_lock:
            raw_tags_snapshot = list(latest_raw_tags)
            
        socketio.emit("robot_status", {
            **robot.state,
            "rail": rail.current_state,
            "vision_detected": detected_list,
            "latest_tag_coords": tag_coords_snapshot,
            "raw_tags": raw_tags_snapshot
        })
        socketio.sleep(0.1)


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
    socketio.start_background_task(background_broadcast)
    print("=" * 70)
    print("  Unified MechArm Assembly Cockpit -> http://localhost:5000")
    print("  Robotic Arm + Stepper Linear Rail + Camera CV Layer Integrated.")
    print("=" * 70)
    socketio.run(app, host="0.0.0.0", port=5000, debug=False, allow_unsafe_werkzeug=True)
