"""
server_ttt.py  —  Tic Tac Toe Robot Controller
================================================
Single-process Flask + SocketIO server (port 5000).
100% offline — USB serial only, no WiFi, no CDN.

Integrates:
  1. RobotManager   — MechArm270 via USB serial (reused from car assembly)
  2. RailManager    — ESP32 rail via USB serial  (reused from car assembly)
  3. RGBLEDManager  — 45-LED RGB strip via ESP32 serial (LEDRGB: commands)
  4. BoardVision    — Arm-camera HSV token detection
  5. TTTMoveExecutor — 3-phase pick+place orchestrator
  6. GameManager    — Minimax AI + dual-board game state
"""

import sys
import os
import time
import threading
import logging
import traceback
import queue
import json
import io
import cv2
import numpy as np
from flask import Flask, render_template, jsonify, request, Response, send_file
from flask_socketio import SocketIO, emit

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pymycobot import MechArm270
import config_ttt as cfg
from ttt_engine      import GameManager, GamePhase, HUMAN, ROBOT
from ttt_vision      import BoardVision
from ttt_move_executor import TTTMoveExecutor, TTTWaypointStore

# ── Logging ───────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)

# ── Flask / SocketIO ──────────────────────────────────────────────────────────
app = Flask(__name__, template_folder="templates", static_folder="static")
app.config["SECRET_KEY"] = "ttt_robot_secret"
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

import serial
import serial.tools.list_ports


# ═════════════════════════════════════════════════════════════════════════════
# RobotManager  (serial-only — identical architecture to car assembly)
# ═════════════════════════════════════════════════════════════════════════════
class RobotManager:
    JOINT_LIMITS = [
        (-170, 170), (-180, 180), (-180, 180),
        (-175, 175), (-170, 170), (-180, 180),
    ]

    def __init__(self):
        self.serial_port        = None
        self.mc                 = None
        self._transport         = "none"
        self.lock               = threading.RLock()
        self.cmd_queue          = queue.Queue()
        self._angles            = [0.0] * 6
        self._coords            = [150.0, 0.0, 100.0, 0.0, 0.0, 0.0]
        self._is_moving         = False
        self._powered           = False
        self._connected         = False
        self._last_error        = ""
        self._move_active       = False
        self._vacuum            = False
        self._hw_errors         = 0
        self._hw_next_err       = []
        self.coord_calibration  = [0.0] * 6
        self.angle_calibration  = [0.0] * 6
        self.config_path        = cfg.ARM_CONFIG_PATH
        self.completion_callbacks: list = []
        self._load_config()
        self._worker = threading.Thread(target=self._run_worker, daemon=True)
        self._worker.start()

    # ── Config ────────────────────────────────────────────────────────────
    def _load_config(self):
        try:
            if os.path.exists(self.config_path):
                with open(self.config_path) as f:
                    d = json.load(f)
                self.serial_port        = d.get("serial_port", self.serial_port)
                self.coord_calibration  = d.get("coord_calibration", [0.0] * 6)
                self.angle_calibration  = d.get("angle_calibration", [0.0] * 6)
        except Exception as e:
            logging.error("[ARM] Config load: %s", e)

    def save_config(self, serial_port=None):
        if serial_port is not None:
            self.serial_port = serial_port or None
        try:
            with open(self.config_path, "w") as f:
                json.dump({
                    "serial_port":       self.serial_port,
                    "coord_calibration": self.coord_calibration,
                    "angle_calibration": self.angle_calibration,
                }, f, indent=4)
        except Exception as e:
            logging.error("[ARM] Config save: %s", e)

    # ── Worker ────────────────────────────────────────────────────────────
    def _run_worker(self):
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
                    if elapsed > 0.8:
                        try:
                            moving = self.mc.is_moving()
                            if moving == 0:
                                self._move_active = False
                                self._finish_command(active_cmd)
                                active_cmd = None
                            elif elapsed > cfg.MOVE_TIMEOUT_SEC:
                                logging.warning("[ARM] Move timeout — forcing done.")
                                self._move_active = False
                                self._finish_command(active_cmd)
                                active_cmd = None
                        except Exception as e:
                            logging.warning("[ARM] is_moving() error: %s", e)
                            self._move_active = False
                            active_cmd = None
                        time.sleep(0.3)

                if not active_cmd:
                    try:
                        cmd = self.cmd_queue.get(timeout=0.1)
                        if not self._connected and cmd.get("action") != "reconnect":
                            logging.warning("[ARM] Dropping %s — not connected.", cmd.get("action"))
                            self._finish_command(cmd)
                            self.cmd_queue.task_done()
                        else:
                            success = self._execute_command(cmd)
                            if success and cmd["action"] in ("move_angles", "move_coords"):
                                active_cmd      = cmd
                                move_start_time = time.time()
                                self._move_active = True
                            else:
                                self._finish_command(cmd)
                            self.cmd_queue.task_done()
                            last_poll = 0
                    except queue.Empty:
                        pass

                if not self._move_active:
                    now2 = time.time()
                    if now2 - last_poll > 1.0:
                        self._poll_status()
                        last_poll = now2

                time.sleep(0.02)
            except Exception as e:
                logging.error("[ARM] Worker error: %s\n%s", e, traceback.format_exc())
                self._connected = False
                time.sleep(3.0)

    def _finish_command(self, cmd):
        data = {"action": cmd.get("action"), "cmd_id": cmd.get("cmd_id")}
        socketio.emit("command_complete", data)
        for cb in self.completion_callbacks:
            try:
                cb(cmd.get("cmd_id"))
            except Exception as e:
                logging.error("[ARM] Callback error: %s", e)
        with self.lock:
            self._is_moving = False

    def _attempt_connect(self):
        if self.mc:
            try:
                self.mc.close()
            except Exception:
                pass
            self.mc = None
        self._transport = "none"

        is_win = sys.platform.startswith("win")
        try:
            detected_ports = [p.device for p in serial.tools.list_ports.comports()]
        except Exception:
            detected_ports = []

        arm_ports = detected_ports if is_win else ["/dev/ttyACM1", "/dev/ttyUSB0", "/dev/ttyACM2"]
        ports = ([self.serial_port] if self.serial_port else []) + arm_ports
        seen = set()
        ports = [p for p in ports if not (p in seen or seen.add(p))]

        for port in ports:
            mc = None
            try:
                logging.info("[ARM] Trying %s …", port)
                mc = MechArm270(port, 115200)
                time.sleep(2.0)
                a = mc.get_angles()
                if a and isinstance(a, list) and len(a) >= 6 and a[0] != -1:
                    self.mc          = mc
                    self.serial_port = port
                    self._transport  = "serial"
                    with self.lock:
                        self._connected = True
                        self._powered   = True
                        self._angles    = [round(v, 2) for v in a[:6]]
                    logging.info("✅ Arm connected on %s.", port)
                    return
                raise Exception(f"get_angles() invalid: {a}")
            except Exception as e:
                logging.debug("[ARM] %s failed: %s", port, e)
                if mc:
                    try:
                        mc.close()
                    except Exception:
                        pass

        with self.lock:
            self._connected = False
            self._last_error = "No serial port found — will retry"
        logging.warning("⚠️  Arm not found. Will retry in 15s.")

    def _execute_command(self, cmd):
        action = cmd.get("action")
        params = cmd.get("params", {})
        try:
            if action == "move_angles":
                angles = [round(float(v), 2) for v in params["angles"]]
                angles = [angles[i] + self.angle_calibration[i] for i in range(6)]
                speed  = int(params["speed"])
                if not self._powered:
                    self._power_on_sequence()
                valid, msg = self._validate_angles(angles)
                if not valid:
                    with self.lock:
                        self._last_error = msg
                    return False
                with self.lock:
                    self._is_moving = True
                self.mc.send_angles(angles, speed)
                return True

            elif action == "move_coords":
                coords = [round(float(v), 2) for v in params["coords"]]
                coords = [coords[i] + self.coord_calibration[i] for i in range(6)]
                speed  = int(params["speed"])
                mode   = int(params.get("mode", 0))
                if not self._powered:
                    self._power_on_sequence()
                with self.lock:
                    self._is_moving = True
                self.mc.send_coords(coords, speed, mode)
                return True

            elif action == "power_on":
                self._power_on_sequence()

            elif action == "power_off":
                self.mc.release_all_servos()
                with self.lock:
                    self._powered = False
                return True

            elif action == "stop":
                if self.mc:
                    self.mc.stop()
                with self.lock:
                    self._is_moving = False

            elif action == "vacuum":
                state    = params.get("state", "off")
                delay_ms = int(params.get("delay", 0))
                if self.mc:
                    if state == "on":
                        self.mc.set_basic_output(5, 0)
                        self.mc.set_basic_output(2, 0)
                        time.sleep(1.5)
                        self.mc.set_basic_output(5, 1)
                    else:
                        self.mc.set_basic_output(5, 1)
                        self.mc.set_basic_output(2, 1)
                        time.sleep(0.3)
                with self.lock:
                    self._vacuum = (state == "on")
                if delay_ms > 0:
                    time.sleep(delay_ms / 1000.0)

            elif action == "reconnect":
                self._attempt_connect()

        except Exception as e:
            logging.error("[ARM] Cmd exec failed: %s", e)
            self._connected = False

    def _power_on_sequence(self):
        self.mc.power_on()
        time.sleep(0.5)
        a = self.mc.get_angles()
        if a and isinstance(a, list) and len(a) >= 6 and a[0] != -1:
            self.mc.set_fresh_mode(1)
            time.sleep(0.1)
            self.mc.send_angles([round(v, 2) for v in a[:6]], 10)
            time.sleep(0.5)
        self.mc.set_fresh_mode(0)
        with self.lock:
            self._powered = True

    def _poll_status(self):
        if not self.mc or not self._connected:
            return
        now = time.time()
        try:
            moving = self.mc.is_moving()
            with self.lock:
                self._is_moving = (moving == 1)
                is_moving_now   = self._is_moving
            if is_moving_now:
                return
            a = self.mc.get_angles()
            if a and isinstance(a, list) and len(a) >= 6 and a[0] != -1:
                with self.lock:
                    self._angles  = [round(v, 2) for v in a[:6]]
                    self._powered = True
            time.sleep(0.05)
            if not hasattr(self, "_last_slow_poll"):
                self._last_slow_poll = 0
            if now - self._last_slow_poll > 2.0:
                c = self.mc.get_coords()
                if c and isinstance(c, list) and len(c) >= 6 and c[0] != -1:
                    with self.lock:
                        self._coords = [round(v, 2) for v in c[:6]]
                self._last_slow_poll = now
        except Exception:
            with self.lock:
                self._connected = False

    def _validate_angles(self, angles):
        for i in range(6):
            lo, hi = self.JOINT_LIMITS[i]
            if not (lo <= angles[i] <= hi):
                return False, f"J{i+1}={angles[i]} out of [{lo},{hi}]"
        return True, "OK"

    # ── Public API ────────────────────────────────────────────────────────
    def send_angles(self, angles, speed=cfg.ARM_SPEED, cmd_id=None):
        self.cmd_queue.put({"action": "move_angles",
                            "params": {"angles": angles, "speed": speed},
                            "cmd_id": cmd_id})

    def send_coords(self, coords, speed=cfg.ARM_SPEED, mode=0, cmd_id=None):
        self.cmd_queue.put({"action": "move_coords",
                            "params": {"coords": coords, "speed": speed, "mode": mode},
                            "cmd_id": cmd_id})

    def set_vacuum(self, state, delay=0, cmd_id=None):
        self.cmd_queue.put({"action": "vacuum",
                            "params": {"state": state, "delay": delay},
                            "cmd_id": cmd_id})

    def lock_servos(self):
        self.cmd_queue.put({"action": "power_on"})

    def release_servos(self):
        self.cmd_queue.put({"action": "power_off"})

    def emergency_stop(self):
        while not self.cmd_queue.empty():
            try:
                self.cmd_queue.get_nowait()
            except Exception:
                pass
        self.cmd_queue.put({"action": "stop"})

    def reconnect(self, serial_port=None):
        self.save_config(serial_port=serial_port)
        self.cmd_queue.put({"action": "reconnect"})

    @property
    def state(self):
        with self.lock:
            return {
                "angles":           list(self._angles),
                "coords":           list(self._coords),
                "is_moving":        self._is_moving,
                "connected":        self._connected,
                "powered":          self._powered,
                "vacuum":           self._vacuum,
                "error":            self._last_error,
                "serial_port":      self.serial_port,
                "hw_errors":        self._hw_errors,
                "calibration": {
                    "coords":  list(self.coord_calibration),
                    "angles":  list(self.angle_calibration),
                },
                "timestamp": time.time(),
            }


# ═════════════════════════════════════════════════════════════════════════════
# RailManager  (serial-only)
# ═════════════════════════════════════════════════════════════════════════════
class RailManager:
    SERIAL_PORTS = ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0"]
    SERIAL_BAUD  = 115200


    def __init__(self):
        self.state = {
            "running": False, "homed": False, "point": "NONE",
            "absCm": 0.0, "rpm": 150, "tempC": -99.0,
            "board1Cm": 9.0, "board2Cm": 43.0, "connected": False,
        }
        self.lock             = threading.Lock()
        self._serial          = None
        self._serial_lock     = threading.Lock()
        self._serial_connected = False
        threading.Thread(target=self._connect_serial, daemon=True).start()

    def _connect_serial(self):
        try:
            import serial as _serial_mod
        except ImportError:
            logging.warning("⚠️  pyserial not installed.")
            return

        is_win = sys.platform.startswith("win")
        ports  = ([p.device for p in serial.tools.list_ports.comports()]
                  if is_win else self.SERIAL_PORTS)

        for port in ports:
            try:
                logging.info("[RAIL] Trying %s …", port)
                ser = _serial_mod.Serial(port, self.SERIAL_BAUD, timeout=1)
                ser.dtr = False
                time.sleep(0.5)
                ser.reset_input_buffer()
                verified = False
                start = time.time()
                while time.time() - start < 15.0:
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
                    logging.info("✅ Rail connected on %s.", port)
                    threading.Thread(target=self._serial_reader, daemon=True).start()
                    return
                else:
                    ser.close()
            except Exception as e:
                logging.debug("[RAIL] %s failed: %s", port, e)

        logging.warning("⚠️  Rail not found.")

    def _serial_reader(self):
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
                        "board1Cm":  data.get("board1Cm",  9.0),
                        "board2Cm":  data.get("board2Cm",  43.0),
                        "connected": True,
                    }
            except Exception as e:
                logging.warning("[RAIL] Reader error: %s", e)
                with self._serial_lock:
                    self._serial_connected = False
                    try:
                        if self._serial:
                            self._serial.close()
                    except Exception:
                        pass
                    self._serial = None
                time.sleep(3)
                threading.Thread(target=self._connect_serial, daemon=True).start()
                return

    def _send_serial(self, cmd: str) -> bool:
        with self._serial_lock:
            ser       = self._serial
            connected = self._serial_connected
        if ser and connected:
            try:
                ser.write((cmd + "\n").encode())
                return True
            except Exception as e:
                logging.warning("[RAIL] Serial send failed: %s", e)
                self._serial_connected = False
        return False

    def move_to_preset(self, name: str) -> bool:
        if not name or name == "NONE":
            return True
        sent = self._send_serial(f"MOVE:{name.upper()}")
        if not sent:
            logging.warning("[RAIL] Move dropped — serial unavailable.")
        return sent

    def set_preset_cm(self, name: str, cm: float) -> bool:
        return self._send_serial(f"SETPRESET:{name.upper()}:{cm}")

    def set_speed(self, rpm: int) -> bool:
        vmax = int(rpm * (500000 / 420))
        vmax = max(50000, min(500000, vmax))
        return self._send_serial(f"SPEED:{vmax}")

    def home(self) -> bool:
        return self._send_serial("HOME")

    def stop(self) -> bool:
        return self._send_serial("STOP")

    @property
    def current_state(self):
        with self.lock:
            return dict(self.state)


# ═════════════════════════════════════════════════════════════════════════════
# RGBLEDManager  — 45-LED strip via ESP32 serial
# ═════════════════════════════════════════════════════════════════════════════
class RGBLEDManager:
    """
    Sends LEDRGB:<effect> and LEDSOLID:<R>,<G>,<B> commands to the ESP32
    over the same serial connection as the rail.

    GPIO pin for the LED strip: cfg.RGB_LED_GPIO (None = TBD)
    """

    def __init__(self, rail: RailManager):
        self._rail       = rail
        self._last_effect = ""
        self._brightness  = cfg.LED_DEFAULT_BRIGHTNESS

    def _send(self, cmd: str, force: bool = False) -> bool:
        if not force and cmd == self._last_effect:
            return True   # deduplicate
        self._last_effect = cmd
        sent = self._rail._send_serial(cmd)
        if not sent:
            logging.warning("[LED] Serial unavailable — command dropped: %s", cmd)
        return sent

    # ── Game-state effects ────────────────────────────────────────────────
    def idle(self):            self._send("LEDRGB:idle")
    def scanning(self):        self._send("LEDRGB:scanning")
    def human_turn(self):      self._send("LEDRGB:human_turn")
    def robot_thinking(self):  self._send("LEDRGB:robot_thinking")
    def robot_moving(self):    self._send("LEDRGB:robot_moving")
    def win_robot(self):       self._send("LEDRGB:win_robot",  force=True)
    def win_human(self):       self._send("LEDRGB:win_human",  force=True)
    def draw(self):            self._send("LEDRGB:draw",        force=True)
    def alert(self):           self._send("LEDRGB:alert",       force=True)
    def rainbow(self):         self._send("LEDRGB:rainbow")
    def off(self):             self._send("LEDRGB:off")

    # ── Custom solid color ────────────────────────────────────────────────
    def set_solid(self, r: int, g: int, b: int) -> bool:
        r, g, b = max(0, min(255, r)), max(0, min(255, g)), max(0, min(255, b))
        cmd = f"LEDSOLID:{r},{g},{b}"
        self._last_effect = cmd
        return self._rail._send_serial(cmd)

    # ── Brightness ────────────────────────────────────────────────────────
    def set_brightness(self, value: int) -> bool:
        value = max(0, min(255, value))
        self._brightness = value
        return self._rail._send_serial(f"LEDBRIGHT:{value}")

    @property
    def current_effect(self) -> str:
        return self._last_effect

    @property
    def brightness(self) -> int:
        return self._brightness


# ═════════════════════════════════════════════════════════════════════════════
# Global instances
# ═════════════════════════════════════════════════════════════════════════════
robot    = RobotManager()
rail     = RailManager()
led      = RGBLEDManager(rail)
vision   = BoardVision()
game_mgr = GameManager()

wp_stores = {
    "B1": TTTWaypointStore("B1"),
    "B2": TTTWaypointStore("B2"),
}

executor = TTTMoveExecutor(robot, rail, wp_stores, socketio_ref=socketio)

# Register executor's completion callback with RobotManager
robot.completion_callbacks.append(executor.on_command_complete)

# ── Game loop thread control ──────────────────────────────────────────────────
_game_thread: threading.Thread | None = None
_game_stop_event = threading.Event()


# ═════════════════════════════════════════════════════════════════════════════
# Background status broadcaster
# ═════════════════════════════════════════════════════════════════════════════
def _status_broadcaster():
    """Broadcast hardware state and game state every second."""
    while True:
        try:
            socketio.emit("arm_status",  robot.state)
            socketio.emit("rail_status", rail.current_state)
            socketio.emit("led_status",  {
                "effect":     led.current_effect,
                "brightness": led.brightness,
                "gpio":       cfg.RGB_LED_GPIO,
                "count":      cfg.RGB_LED_COUNT,
            })
            socketio.emit("game_state_update", game_mgr.get_snapshot())
        except Exception as e:
            logging.debug("[Broadcaster] %s", e)
        time.sleep(1.0)

threading.Thread(target=_status_broadcaster, daemon=True).start()


# ═════════════════════════════════════════════════════════════════════════════
# Game Loop
# ═════════════════════════════════════════════════════════════════════════════
def _game_loop(board_id: str):
    """
    Main game loop for a single board.
    Runs in a daemon thread; stopped via _game_stop_event.
    """
    session = game_mgr.sessions[board_id]
    logging.info("[GameLoop %s] Starting.", board_id)

    while not _game_stop_event.is_set():
        snapshot = session.get_state_snapshot()

        # ── HUMAN_TURN: move arm to scan pose, wait for human move ────────
        if snapshot["phase"] == GamePhase.HUMAN_TURN:
            led.scanning()
            socketio.emit("scan_started", {"board_id": board_id})
            logging.info("[GameLoop %s] Moving to scan pose.", board_id)

            if not executor.go_to_scan_pose(board_id):
                if _game_stop_event.is_set():
                    break
                time.sleep(1.0)
                continue

            time.sleep(cfg.SCAN_SETTLE_TIME)
            if _game_stop_event.is_set():
                break

            led.human_turn()
            socketio.emit("vision_log",
                          {"level": "info",
                           "message": f"[{board_id}] Waiting for human move..."})

            # Poll until a new human token appears
            while not _game_stop_event.is_set():
                time.sleep(1.5)
                if _game_stop_event.is_set():
                    break

                robot_cells = [idx for idx, val in enumerate(session.grid) if val == "R"]
                new_state = vision.start_scan(board_id, robot_cells=robot_cells, timeout=8.0)
                if not new_state:
                    continue

                # Detect if any logically empty cell is now occupied by a white human token
                cell = next((i for i in range(9) if session.grid[i] == "" and new_state[i] == "H"), None)

                if cell is not None:
                    logging.info("[GameLoop %s] Human placed on cell %d.", board_id, cell)
                    socketio.emit("human_move_detected",
                                  {"board_id": board_id, "cell": cell})
                    result = session.apply_human_move(cell)
                    socketio.emit("game_state_update", game_mgr.get_snapshot())

                    if result.get("winner"):
                        _handle_game_over(board_id, result)
                    break

        # ── ROBOT_TURN: compute Minimax, execute move ─────────────────────
        elif snapshot["phase"] == GamePhase.ROBOT_TURN:
            led.robot_thinking()
            cell = session.get_robot_cell()
            socketio.emit("robot_thinking", {"board_id": board_id, "cell": cell})
            logging.info("[GameLoop %s] Robot playing cell %d.", board_id, cell)

            led.robot_moving()
            current_slot = session.robot_moves_count
            logging.info("[GameLoop %s] Robot picking from tray slot %d for cell %d.", board_id, current_slot, cell)
            success = executor.execute_robot_turn(board_id, cell, slot_idx=current_slot)

            if not success:
                logging.warning("[GameLoop %s] Move executor failed.", board_id)
                if _game_stop_event.is_set():
                    break
                time.sleep(1.0)
                continue

            result = session.apply_robot_move(cell)
            socketio.emit("game_state_update", game_mgr.get_snapshot())

            if result.get("winner"):
                _handle_game_over(board_id, result)
            else:
                led.human_turn()

        # ── GAME_OVER / IDLE ──────────────────────────────────────────────
        else:
            time.sleep(0.5)

    logging.info("[GameLoop %s] Stopped.", board_id)


def _handle_game_over(board_id: str, result: dict):
    winner   = result.get("winner")
    win_line = result.get("win_line")
    scores   = game_mgr.sessions[board_id].scores

    payload = {
        "board_id": board_id,
        "winner":   winner,
        "win_line": win_line,
        "scores":   scores,
    }
    socketio.emit("game_over", payload)

    if winner == ROBOT:
        led.win_robot()
    elif winner == HUMAN:
        led.win_human()
    else:
        led.draw()

    logging.info("[Game %s] Game over — winner: %s.", board_id, winner)


# ═════════════════════════════════════════════════════════════════════════════
# Flask Routes
# ═════════════════════════════════════════════════════════════════════════════

# ── Pages ─────────────────────────────────────────────────────────────────────
@app.route("/")
def index():
    return render_template("ttt_dashboard.html")

@app.route("/waypoints")
def waypoints_page():
    return render_template("ttt_waypoints.html")

@app.route("/calibration")
def calibration_page():
    return render_template("ttt_calibration.html")

# ── Camera Stream ─────────────────────────────────────────────────────────────
def _gen_camera_stream():
    while True:
        jpeg = vision.get_latest_jpeg()
        if jpeg:
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n")
        time.sleep(0.05)

@app.route("/video_feed")
def video_feed():
    return Response(_gen_camera_stream(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

# ── Game Control ──────────────────────────────────────────────────────────────
@app.route("/api/start_game", methods=["POST"])
def api_start_game():
    global _game_thread, _game_stop_event
    data     = request.get_json(silent=True) or {}
    board_id = data.get("board_id", "B1")
    mode     = int(data.get("mode", game_mgr.mode))

    game_mgr.set_mode(mode)

    # Check waypoints calibrated
    missing = wp_stores[board_id].missing_keys()
    if missing:
        return jsonify({"success": False,
                        "error": f"Missing waypoints: {missing[:5]}..."})

    if not game_mgr.start_game(board_id):
        return jsonify({"success": False, "error": "Game already active"})

    # Start game loop thread
    _game_stop_event.clear()
    _game_thread = threading.Thread(target=_game_loop, args=(board_id,), daemon=True)
    _game_thread.start()

    led.idle()
    socketio.emit("game_state_update", game_mgr.get_snapshot())
    return jsonify({"success": True, "board_id": board_id, "mode": mode})

@app.route("/api/reset_game", methods=["POST"])
def api_reset_game():
    global _game_thread, _game_stop_event
    data     = request.get_json(silent=True) or {}
    board_id = data.get("board_id", None)

    _game_stop_event.set()
    executor.stop()
    game_mgr.reset_game(board_id)
    led.idle()
    socketio.emit("game_state_update", game_mgr.get_snapshot())
    return jsonify({"success": True})

@app.route("/api/set_mode", methods=["POST"])
def api_set_mode():
    data = request.get_json(silent=True) or {}
    mode = int(data.get("mode", 1))
    try:
        game_mgr.set_mode(mode)
        return jsonify({"success": True, "mode": mode})
    except ValueError as e:
        return jsonify({"success": False, "error": str(e)})

@app.route("/api/game_state")
def api_game_state():
    return jsonify(game_mgr.get_snapshot())

# ── Hardware Control ──────────────────────────────────────────────────────────
@app.route("/api/emergency_stop", methods=["POST"])
def api_emergency_stop():
    _game_stop_event.set()
    executor.stop()
    led.alert()
    return jsonify({"success": True})

@app.route("/api/robot_state")
def api_robot_state():
    return jsonify({
        "arm":  robot.state,
        "rail": rail.current_state,
        "led":  {
            "effect":     led.current_effect,
            "brightness": led.brightness,
            "gpio":       cfg.RGB_LED_GPIO,
            "count":      cfg.RGB_LED_COUNT,
        },
    })

@app.route("/api/reconnect", methods=["POST"])
def api_reconnect():
    data = request.get_json(silent=True) or {}
    port = data.get("serial_port")
    robot.reconnect(serial_port=port)
    return jsonify({"success": True})

@app.route("/api/home_rail", methods=["POST"])
def api_home_rail():
    rail.home()
    return jsonify({"success": True})

@app.route("/api/move_rail", methods=["POST"])
def api_move_rail():
    data = request.get_json(silent=True) or {}
    preset = data.get("preset", "HOME")
    success = rail.move_to_preset(preset)
    return jsonify({"success": success})

@app.route("/api/set_rail_preset", methods=["POST"])
def api_set_rail_preset():
    data = request.get_json(silent=True) or {}
    name = data.get("name")
    cm = data.get("cm")
    if not name or cm is None:
        return jsonify({"success": False, "error": "Missing name or cm"}), 400
    try:
        cm_float = float(cm)
        success = rail.set_preset_cm(name, cm_float)
        return jsonify({"success": success})
    except ValueError:
        return jsonify({"success": False, "error": "Invalid cm value"}), 400

# ── LED Control ───────────────────────────────────────────────────────────────
@app.route("/api/led/<effect>", methods=["POST"])
def api_led_effect(effect: str):
    effect = effect.lower().strip()
    if effect not in cfg.LED_EFFECTS:
        return jsonify({"success": False, "error": f"Unknown effect: {effect}"})
    getattr(led, effect, led.idle)()
    return jsonify({"success": True, "effect": effect})

@app.route("/api/led/solid", methods=["POST"])
def api_led_solid():
    data = request.get_json(silent=True) or {}
    r = int(data.get("r", 255))
    g = int(data.get("g", 255))
    b = int(data.get("b", 255))
    led.set_solid(r, g, b)
    return jsonify({"success": True, "r": r, "g": g, "b": b})

@app.route("/api/led/brightness", methods=["POST"])
def api_led_brightness():
    data  = request.get_json(silent=True) or {}
    value = int(data.get("value", 128))
    led.set_brightness(value)
    # Persist to settings
    _update_setting("led_brightness", value)
    return jsonify({"success": True, "brightness": value})

@app.route("/api/led/effects")
def api_led_effects():
    return jsonify({"effects": cfg.LED_EFFECTS})

# ── Waypoint Management ───────────────────────────────────────────────────────
@app.route("/api/waypoints/<board_id>")
def api_get_waypoints(board_id: str):
    store = wp_stores.get(board_id)
    if not store:
        return jsonify({"success": False, "error": "Unknown board"}), 404
    return jsonify({
        "success":    True,
        "board_id":   board_id,
        "waypoints":  store.as_dict(),
        "calibration_status": store.calibration_status(),
        "missing_keys": store.missing_keys(),
    })

@app.route("/api/waypoints/<board_id>", methods=["POST"])
def api_set_waypoint(board_id: str):
    store = wp_stores.get(board_id)
    if not store:
        return jsonify({"success": False, "error": "Unknown board"}), 404
    data = request.get_json(silent=True) or {}
    key  = data.get("key")
    if not key:
        return jsonify({"success": False, "error": "Missing key"}), 400
    store.set(key, data)
    return jsonify({"success": True, "key": key,
                    "calibration_status": store.calibration_status()})

@app.route("/api/waypoints/<board_id>/<key>", methods=["DELETE"])
def api_delete_waypoint(board_id: str, key: str):
    store = wp_stores.get(board_id)
    if not store:
        return jsonify({"success": False, "error": "Unknown board"}), 404
    return jsonify({"success": store.delete(key)})

@app.route("/api/record_waypoint", methods=["POST"])
def api_record_waypoint():
    """Record the arm's current angles and coordinates as a named waypoint."""
    data      = request.get_json(silent=True) or {}
    board_id  = data.get("board_id", "B1")
    key       = data.get("key", "")
    speed     = int(data.get("speed", cfg.ARM_SPEED))
    vacuum    = data.get("vacuum", "none")
    delay_ms  = int(data.get("delay_ms", 0))
    move_type = data.get("move_type", "")
    wp_type   = data.get("type", "angles")
    cell_id   = data.get("cell_id", None)

    if not key:
        return jsonify({"success": False, "error": "key is required"}), 400

    store = wp_stores.get(board_id)
    if not store:
        return jsonify({"success": False, "error": "Unknown board"}), 404

    with robot.lock:
        angles = list(robot._angles)
        coords = list(robot._coords)

    # Use explicit angles/coords if provided in payload, else use live robot readouts
    custom_angles = data.get("angles")
    custom_coords = data.get("coords")
    if custom_angles and len(custom_angles) == 6:
        angles = [round(float(v), 2) for v in custom_angles]
    if custom_coords and len(custom_coords) == 6:
        coords = [round(float(v), 2) for v in custom_coords]

    waypoint = {
        "key":              key,
        "name":             data.get("name", key),
        "board_id":         board_id,
        "cell_id":          cell_id,
        "move_type":        move_type,
        "type":             wp_type,
        "data":             angles,
        "coords":           coords,
        "speed":            speed,
        "vacuum":           vacuum,
        "vacuum_delay_ms":  300,
        "delay_ms":         delay_ms,
        "rail_preset":      cfg.RAIL_PRESET_MAP.get(board_id, "BOARD1"),
        "timestamp":        time.time(),
    }
    store.set(key, waypoint)
    return jsonify({"success": True, "key": key, "angles": angles, "coords": coords,
                    "calibration_status": store.calibration_status()})

@app.route("/api/jog_waypoint", methods=["POST"])
def api_jog_waypoint():
    """Send arm to a specific keyed waypoint (by angles or coords)."""
    data     = request.get_json(silent=True) or {}
    board_id = data.get("board_id", "B1")
    key      = data.get("key", "")
    mode     = data.get("mode")  # "angles" or "coords", None = use waypoint type
    store    = wp_stores.get(board_id)
    if not store:
        return jsonify({"success": False, "error": "Unknown board"}), 404
    wp = store.get(key)
    if not wp:
        return jsonify({"success": False, "error": f"Waypoint '{key}' not found"}), 404

    speed = int(data.get("speed", wp.get("speed", cfg.ARM_SPEED)))
    wp_type = mode or wp.get("type", "angles")

    if wp_type == "coords" and wp.get("coords"):
        robot.send_coords(wp["coords"], speed=speed)
    elif wp.get("data"):
        robot.send_angles(wp["data"], speed=speed)
    else:
        return jsonify({"success": False, "error": "No valid position data in waypoint"}), 400

    return jsonify({"success": True, "key": key, "mode": wp_type})

@app.route("/api/test_cell", methods=["POST"])
def api_test_cell():
    """
    Run a dry-run pick+place sequence for one cell from a specified tray slot.
    Goes: pick_safe → tray_slot_S_approach → pick (Vac ON) → lift
         → pick_safe → place_safe → cell_N_approach → place (Vac OFF) → lift
         → place_safe → scan_pose
    """
    data     = request.get_json(silent=True) or {}
    board_id = data.get("board_id", "B1")
    cell_idx = int(data.get("cell_idx", 0))
    slot_idx = int(data.get("slot_idx", 0))

    def _run():
        executor.execute_robot_turn(board_id, cell_idx, slot_idx=slot_idx)

    threading.Thread(target=_run, daemon=True).start()
    return jsonify({"success": True, "board_id": board_id, "cell_idx": cell_idx, "slot_idx": slot_idx})

# ── Vision ROI Calibration ────────────────────────────────────────────────────
@app.route("/api/cell_rois/<board_id>")
def api_get_rois(board_id: str):
    rois = cfg.BOARD_CELL_ROIS.get(board_id, {})
    return jsonify({"success": True, "board_id": board_id, "rois": rois})

@app.route("/api/cell_rois/<board_id>", methods=["POST"])
def api_set_roi(board_id: str):
    data     = request.get_json(silent=True) or {}
    cell_idx = int(data.get("cell_idx", 0))
    roi      = data.get("roi", [0, 0, 100, 100])
    vision.update_roi(board_id, cell_idx, roi)
    # Persist to settings JSON
    _update_roi_config(board_id, cell_idx, roi)
    return jsonify({"success": True})

# ── Arm Direct Control (manual jog for calibration) ───────────────────────────
@app.route("/api/move_angles", methods=["POST"])
def api_move_angles():
    data   = request.get_json(silent=True) or {}
    angles = data.get("angles", [0] * 6)
    speed  = int(data.get("speed", cfg.ARM_SPEED))
    robot.send_angles(angles, speed=speed)
    return jsonify({"success": True})

@app.route("/api/vacuum", methods=["POST"])
def api_vacuum():
    data  = request.get_json(silent=True) or {}
    state = data.get("state", "off")
    delay = int(data.get("delay", 0))
    robot.set_vacuum(state, delay=delay)
    return jsonify({"success": True, "state": state})

@app.route("/api/power_on", methods=["POST"])
def api_power_on():
    robot.lock_servos()
    return jsonify({"success": True})

@app.route("/api/power_off", methods=["POST"])
def api_power_off():
    robot.release_servos()
    return jsonify({"success": True})

# ── Settings ──────────────────────────────────────────────────────────────────
@app.route("/api/settings")
def api_get_settings():
    try:
        existing = {}
        if os.path.exists(cfg.SETTINGS_PATH):
            with open(cfg.SETTINGS_PATH) as f:
                existing = json.load(f)
        
        # Build complete dictionary of active settings
        defaults = {
            "arm_speed": cfg.ARM_SPEED,
            "rail_speed_rpm": cfg.RAIL_SPEED_RPM,
            "game_mode": cfg.DEFAULT_GAME_MODE,
            "scan_settle_time": cfg.SCAN_SETTLE_TIME,
            "vacuum_on_delay_ms": cfg.VACUUM_ON_DELAY_MS,
            "vacuum_off_delay_ms": cfg.VACUUM_OFF_DELAY_MS,
            "led_brightness": cfg.LED_DEFAULT_BRIGHTNESS,
            "camera_index": cfg.CAMERA_INDEX,
            "token_pixel_ratio": cfg.TOKEN_PIXEL_RATIO,
            "white_hsv_lower": list(cfg.WHITE_HSV_LOWER),
            "white_hsv_upper": list(cfg.WHITE_HSV_UPPER),
            "blue_hsv_lower": list(cfg.BLUE_HSV_LOWER),
            "blue_hsv_upper": list(cfg.BLUE_HSV_UPPER),
        }
        defaults.update(existing)
        return jsonify(defaults)
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route("/api/settings", methods=["POST"])
def api_save_settings():
    data = request.get_json(silent=True) or {}
    try:
        existing = {}
        if os.path.exists(cfg.SETTINGS_PATH):
            with open(cfg.SETTINGS_PATH) as f:
                existing = json.load(f)
        existing.update(data)
        with open(cfg.SETTINGS_PATH, "w") as f:
            json.dump(existing, f, indent=2)
        cfg._load_settings()
        return jsonify({"success": True})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})


# ── Helpers ───────────────────────────────────────────────────────────────────
def _update_setting(key: str, value) -> None:
    try:
        existing = {}
        if os.path.exists(cfg.SETTINGS_PATH):
            with open(cfg.SETTINGS_PATH) as f:
                existing = json.load(f)
        existing[key] = value
        with open(cfg.SETTINGS_PATH, "w") as f:
            json.dump(existing, f, indent=2)
    except Exception as e:
        logging.warning("[Settings] Update failed: %s", e)


def _update_roi_config(board_id: str, cell_idx: int, roi: list) -> None:
    """Persist ROI update back to settings file."""
    try:
        existing = {}
        if os.path.exists(cfg.SETTINGS_PATH):
            with open(cfg.SETTINGS_PATH) as f:
                existing = json.load(f)
        rois = existing.setdefault("board_cell_rois", {})
        rois.setdefault(board_id, {})[f"cell_{cell_idx}"] = roi
        with open(cfg.SETTINGS_PATH, "w") as f:
            json.dump(existing, f, indent=2)
        cfg._load_settings()
    except Exception as e:
        logging.warning("[ROI] Persist failed: %s", e)


@app.after_request
def add_header(response):
    """Prevent caching of API responses."""
    if response.headers.get("Content-Type") == "application/json":
        response.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
        response.headers["Pragma"] = "no-cache"
        response.headers["Expires"] = "0"
    return response


# ═════════════════════════════════════════════════════════════════════════════
# SocketIO Events
# ═════════════════════════════════════════════════════════════════════════════
@socketio.on("connect")
def on_connect():
    emit("arm_status",        robot.state)
    emit("rail_status",       rail.current_state)
    emit("game_state_update", game_mgr.get_snapshot())
    emit("led_status", {
        "effect":     led.current_effect,
        "brightness": led.brightness,
        "gpio":       cfg.RGB_LED_GPIO,
        "count":      cfg.RGB_LED_COUNT,
    })
    logging.info("[SocketIO] Client connected.")

@socketio.on("request_emergency_stop")
def on_estop():
    _game_stop_event.set()
    executor.stop()
    led.alert()
    emit("status", {"message": "Emergency stop triggered."})


# ═════════════════════════════════════════════════════════════════════════════
# Startup
# ═════════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    led.idle()
    logging.info("🤖 TTT Robot Server starting on http://0.0.0.0:5000")
    socketio.run(app, host="0.0.0.0", port=5000, debug=False, use_reloader=False)
