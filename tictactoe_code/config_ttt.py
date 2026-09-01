# config_ttt.py  —  Tic Tac Toe Robot System Configuration
# All settings for TTT system. 100% offline, serial-only.

import os
import json
import sys

# ── Rail Preset Mapping ───────────────────────────────────────────────────────
# ESP32 serial commands: MOVE:BOARD1 / MOVE:BOARD2
RAIL_PRESET_MAP = {
    "B1": "BOARD1",
    "B2": "BOARD2",
}
BOARD_IDS = ["B1", "B2"]

# ── RGB LED Strip ─────────────────────────────────────────────────────────────
# 45 WS2812B LEDs under the rail, driven by ESP32 (GPIO 42).
RGB_LED_GPIO  = 42
RGB_LED_COUNT = 45

# LED effects available (sent as: LEDRGB:<effect>)
LED_EFFECTS = [
    "idle",           # slow breathing cyan   — no game active
    "scanning",       # spinning white        — arm at scan pose
    "human_turn",     # pulsing red           — waiting for human
    "robot_thinking", # fast blue pulse       — Minimax computing
    "robot_moving",   # blue chase along rail — arm executing move
    "win_robot",      # full blue flash       — robot wins
    "win_human",      # full red flash        — human wins
    "draw",           # alternating purple    — draw
    "alert",          # flashing red          — error / e-stop
    "rainbow",        # rainbow cycle         — aesthetic / attract
    "off",            # all off
]

# Solid color (sent as: LEDSOLID:<R>,<G>,<B>)
LED_DEFAULT_BRIGHTNESS = 128   # 0–255

# ── Arm Hardware (serial-only, offline) ───────────────────────────────────────
ARM_SERIAL_PORT     = "COM3"   # Windows default; auto-detected if None
ARM_SERIAL_BAUD     = 115200
ARM_SPEED           = 50       # default move speed (1–100)
ARM_SPEED_OVERRIDE  = None     # if set (1-100), overrides all per-waypoint speeds

# ── Rail Hardware ─────────────────────────────────────────────────────────────
RAIL_SPEED_RPM  = 300
RAIL_SERIAL_BAUD = 115200

# ── Motion Timing ─────────────────────────────────────────────────────────────
SCAN_SETTLE_TIME     = 1.5    # seconds after arm reaches scan_pose before camera reads
VACUUM_ON_DELAY_MS   = 1500   # pump settle time (ms)
VACUUM_OFF_DELAY_MS  = 300    # vent time after vacuum off (ms)
MOVE_TIMEOUT_SEC     = 25.0   # max seconds to wait for any single arm move

# ── Vision ────────────────────────────────────────────────────────────────────
CAMERA_INDEX     = 0          # USB webcam index (try 0 if 1 fails)
CAMERA_BACKEND   = "CAP_DSHOW" if sys.platform.startswith("win") else "CAP_V4L2"

CONFIRM_FRAMES   = 5          # consecutive frames to confirm token present
DISAPPEAR_FRAMES = 8          # consecutive frames to confirm token removed
TOKEN_PIXEL_RATIO = 0.10      # fraction of cell ROI that must match color

# Human token color: WHITE (HSV)
WHITE_HSV_LOWER = (0,   0,   200)
WHITE_HSV_UPPER = (180, 50,  255)

# Robot token color: BLUE (HSV)
BLUE_HSV_LOWER = (100, 100, 80)
BLUE_HSV_UPPER = (130, 255, 255)

# ── Game Settings ─────────────────────────────────────────────────────────────
DEFAULT_GAME_MODE = 1          # 1 = single board, 2 = dual board
INTERLEAVE_BOARDS = True       # in 2-game mode: alternate B1 → B2 → B1 …

# ── Board Cell ROIs ───────────────────────────────────────────────────────────
# Each cell: [x_min, y_min, x_max, y_max] in camera pixel coordinates.
# Default = placeholder zero-boxes. Calibrate via waypoint dashboard.
BOARD_CELL_ROIS: dict = {
    "B1": {f"cell_{i}": [0, 0, 100, 100] for i in range(9)},
    "B2": {f"cell_{i}": [0, 0, 100, 100] for i in range(9)},
}

# ── Required Waypoint Keys Per Board ─────────────────────────────────────────
# System & Safe poses:
#   - scan_pose: Camera overhead inspection position
#   - pick_safe: Safe transit pose near tray (pick side)
#   - place_safe: Safe transit pose near board (place side)
# Pick Sequence (4 tray slots for robot moves 1..4):
#   - tray_slot_{0..3}_approach -> tray_slot_{0..3}_pick (Vac ON) -> tray_slot_{0..3}_lift
# Place Sequence (per cell 0..8):
#   - cell_{i}_approach -> cell_{i}_place (Vac OFF) -> cell_{i}_lift (optional)
REQUIRED_WAYPOINT_KEYS = (
    ["scan_pose", "pick_safe", "place_safe"] +
    [f"tray_slot_{s}_{phase}" for s in range(4) for phase in ("approach", "pick", "lift")] +
    [f"cell_{i}_{phase}" for i in range(9) for phase in ("approach", "place", "lift")]
)

# ── Snapshot / Persistence Paths ─────────────────────────────────────────────
_BASE = os.path.dirname(os.path.abspath(__file__))

WAYPOINT_FILES = {
    "B1": os.path.join(_BASE, "ttt_waypoints_b1.json"),
    "B2": os.path.join(_BASE, "ttt_waypoints_b2.json"),
}
SETTINGS_PATH  = os.path.join(_BASE, "ttt_settings.json")
ARM_CONFIG_PATH = os.path.join(_BASE, "arm_config.json")
DANCE_CONFIG_PATH = os.path.join(_BASE, "ttt_dance.json")

# ── Load Overrides from ttt_settings.json ─────────────────────────────────────
def _load_settings():
    if os.path.exists(SETTINGS_PATH):
        try:
            with open(SETTINGS_PATH) as f:
                d = json.load(f)
            global ARM_SPEED, ARM_SPEED_OVERRIDE, RAIL_SPEED_RPM, DEFAULT_GAME_MODE
            global SCAN_SETTLE_TIME, VACUUM_ON_DELAY_MS, VACUUM_OFF_DELAY_MS
            global LED_DEFAULT_BRIGHTNESS
            global CAMERA_INDEX, TOKEN_PIXEL_RATIO
            global WHITE_HSV_LOWER, WHITE_HSV_UPPER
            global BLUE_HSV_LOWER, BLUE_HSV_UPPER, BOARD_CELL_ROIS

            ARM_SPEED              = d.get("arm_speed",            ARM_SPEED)
            # arm_speed_override: stored as int or null in settings
            raw_ov = d.get("arm_speed_override", None)
            ARM_SPEED_OVERRIDE     = int(raw_ov) if raw_ov is not None else None
            RAIL_SPEED_RPM         = d.get("rail_speed_rpm",       RAIL_SPEED_RPM)
            DEFAULT_GAME_MODE      = d.get("game_mode",            DEFAULT_GAME_MODE)
            SCAN_SETTLE_TIME       = d.get("scan_settle_time",     SCAN_SETTLE_TIME)
            VACUUM_ON_DELAY_MS     = d.get("vacuum_on_delay_ms",   VACUUM_ON_DELAY_MS)
            VACUUM_OFF_DELAY_MS    = d.get("vacuum_off_delay_ms",  VACUUM_OFF_DELAY_MS)
            LED_DEFAULT_BRIGHTNESS = d.get("led_brightness",       LED_DEFAULT_BRIGHTNESS)
            CAMERA_INDEX           = d.get("camera_index",         CAMERA_INDEX)
            TOKEN_PIXEL_RATIO      = d.get("token_pixel_ratio",    TOKEN_PIXEL_RATIO)

            if "white_hsv_lower" in d: WHITE_HSV_LOWER = tuple(d["white_hsv_lower"])
            if "white_hsv_upper" in d: WHITE_HSV_UPPER = tuple(d["white_hsv_upper"])
            if "blue_hsv_lower" in d: BLUE_HSV_LOWER = tuple(d["blue_hsv_lower"])
            if "blue_hsv_upper" in d: BLUE_HSV_UPPER = tuple(d["blue_hsv_upper"])
            if "board_cell_rois" in d:
                for b_id, cells in d["board_cell_rois"].items():
                    if b_id in BOARD_CELL_ROIS:
                        BOARD_CELL_ROIS[b_id].update(cells)
        except Exception as e:
            print(f"[config_ttt] Warning: could not load settings: {e}")

_load_settings()
