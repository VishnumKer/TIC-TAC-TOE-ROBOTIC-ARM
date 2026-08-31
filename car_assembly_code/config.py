# Shared settings, imported by both servers
import os
import json

# ── Which positions trigger a vision check ───────────────────────────────────
POSITION_PART_MAP = {
    "P1": "Chassis",
    "P2": "WheelBase",
    "P4": "Wheels",
    "P5": "Body",
}

# Parts to check per station — vision thread only runs detection for these parts
# when the rail is at that station. Empty = no detection (e.g. transit).
STATION_PARTS = {
    "P1": ["Chassis 1", "Chassis 2"],
    "P2": ["WheelBase 1a", "WheelBase 2a", "WheelBase 1b", "WheelBase 2b"],
    "P4": ["Wheels", "Wheel Slot 1", "Wheel Slot 2", "Wheel Slot 3", "Wheel Slot 4",
           "Wheel Slot 5", "Wheel Slot 6", "Wheel Slot 7", "Wheel Slot 8"],
    "P5": ["Body 1", "Body 2"],
}

# Pre-recorded arm scan poses per position (6 joint angles each)
SCAN_POSES = {
    "P1": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "P2": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "P4": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "P5": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
}

# Dynamic load of persistent scan poses if they exist
poses_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scan_poses.json")
if os.path.exists(poses_path):
    try:
        with open(poses_path) as f:
            SCAN_POSES.update(json.load(f))
    except Exception as e:
        print(f"Error loading scan_poses.json: {e}")

CONFIRM_FRAMES   = 5    # consecutive frames with detection before marking stable
DISAPPEAR_FRAMES = 10   # consecutive frames without detection before dropping stable

MAX_PICK_RETRIES  = 3    # re-scan attempts before emergency stop
RETRY_SETTLE_TIME = 2.0  # seconds to wait between retries

# Vision wait timeout: seconds to wait for missing part before error (0 = wait indefinitely)
VISION_WAIT_TIMEOUT = 0

# Indicator alert threshold: seconds of waiting for a missing part before
# triggering the Red LED (constant) + Buzzer (beep every 2 s) on the rail ESP32.
# Set to 0 to disable indicator alerts entirely.
INDICATOR_ALERT_TIMEOUT = 10  # seconds

P4_TOTAL_SLOTS = 8       # total wheel slots at P4

# ── Black colour detection (HSV) ─────────────────────────────────────────────
# Black plastic parts have V < 50. Ambient dim lighting has V ≈ 60-80, so a
# tighter V ceiling avoids false positives from dark room illumination.
BLACK_HSV_LOWER = (0,   0,   0)
BLACK_HSV_UPPER = (180, 255, 50)  # V < 50 = genuinely dark/black, not just dim
# Fraction of ROI pixels that must be black to count as "part present" (tune per environment)
BLACK_PIXEL_RATIO = 0.10

# Camera frame partition midpoints for slot coordinate mapping
FRAME_MID_X = 320
FRAME_MID_Y = 240

# Spatial zone definitions for per-station part slot assignment.
# Format: {station: {part_name: [x_min, y_min, x_max, y_max]}}
# Defaults use the frame midpoints above (640x480 camera).
# Calibrate via /api/update_zone and saved to zone_boundaries.json.
ZONE_DEFINITIONS = {
    "P1": {
        "Chassis 1": [320,   0, 640, 480],   # right half
        "Chassis 2": [  0,   0, 320, 480],   # left half
    },
    "P2": {
        "WheelBase 1a": [320,   0, 640, 240],  # right-top
        "WheelBase 1b": [320, 240, 640, 480],  # right-bottom
        "WheelBase 2a": [  0,   0, 320, 240],  # left-top
        "WheelBase 2b": [  0, 240, 320, 480],  # left-bottom
    },
    "P4": {
        "Wheel Slot 1": [  0,   0, 160, 240],
        "Wheel Slot 2": [160,   0, 320, 240],
        "Wheel Slot 3": [320,   0, 480, 240],
        "Wheel Slot 4": [480,   0, 640, 240],
        "Wheel Slot 5": [  0, 240, 160, 480],
        "Wheel Slot 6": [160, 240, 320, 480],
        "Wheel Slot 7": [320, 240, 480, 480],
        "Wheel Slot 8": [480, 240, 640, 480],
    },
    "P5": {
        "Body 1": [0, 0, 640, 480],
        "Body 2": [0, 0, 640, 480]
    },
}

_zones_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "zone_boundaries.json")
if os.path.exists(_zones_path):
    try:
        with open(_zones_path) as _zf:
            ZONE_DEFINITIONS.update(json.load(_zf))
    except Exception as _ze:
        print(f"Error loading zone_boundaries.json: {_ze}")

# ── Camera Connection Settings ───────────────────────────────────────────────
CAMERA_INDEX = 1   # 1 for USB camera on this system, was 0
import sys
CAMERA_BACKEND = "CAP_DSHOW" if sys.platform.startswith("win") else "CAP_V4L2" # Explicitly use V4L2 backend on Linux to avoid obsensor crashes

# ── Per-part ROI definitions ─────────────────────────────────────────────────
# Format: { part_name: [x_min, y_min, x_max, y_max] }
# Loaded from roi_boundaries.json; falls back to ZONE_DEFINITIONS entries when not set.
ROI_DEFINITIONS: dict = {}

_roi_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "roi_boundaries.json")
if os.path.exists(_roi_path):
    try:
        with open(_roi_path) as _rf:
            ROI_DEFINITIONS.update(json.load(_rf))
    except Exception as _re:
        print(f"Error loading roi_boundaries.json: {_re}")

# ── Network / Hardware Config ─────────────────────────────────────────────────
# Edit these to match your AP network. wifi_config.json and rail_config.json
# override ARM_IP/RAIL_IP if those files exist (set via dashboard).
ARM_IP         = "10.85.93.235"   # MechArm270 IP on SAP network
ARM_PORT       = 9000            # MechArm socket port (default 9000)
RAIL_IP        = "10.85.93.165"   # Rail ESP32 IP — check serial monitor on boot
ARM_SPEED      = 40            # Default arm move speed (1–100)
RAIL_SPEED_RPM = 300            # Default rail speed in RPM (max=420)

