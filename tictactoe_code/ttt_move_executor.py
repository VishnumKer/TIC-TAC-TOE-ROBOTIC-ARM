"""
ttt_move_executor.py  —  3-Phase Motion Orchestrator
=====================================================
Handles all arm + rail motion for the TTT robot.

Safe position is ONLY used when the rail must travel between boards.
Sequence for a robot move:

  Same-board (no rail move needed):
    scan_pose → tray_approach → tray_pick → tray_lift
    → cell_N_approach → cell_N_place → cell_N_lift → scan_pose

  Cross-board (rail must move):
    safe_position         [arm retracted — rail will move]
    → RAIL MOVE
    → tray_approach (new board) → tray_pick → tray_lift
    → cell_N_approach → cell_N_place → cell_N_lift → scan_pose
"""

from __future__ import annotations
import json
import os
import time
import threading
import logging
import queue

import config_ttt as cfg


# ─────────────────────────────────────────────────────────────────────────────
# WaypointStore  (per-board, keyed dict)
# ─────────────────────────────────────────────────────────────────────────────
class TTTWaypointStore:
    """Loads and persists keyed waypoints for one board."""

    def __init__(self, board_id: str):
        self.board_id = board_id
        self.filepath = cfg.WAYPOINT_FILES[board_id]
        self._data: dict = {}
        self._lock = threading.Lock()
        self.load()

    def load(self) -> None:
        if os.path.exists(self.filepath):
            try:
                with open(self.filepath) as f:
                    self._data = json.load(f)
                logging.info("[WP %s] Loaded %d waypoints.", self.board_id, len(self._data))
            except Exception as e:
                logging.error("[WP %s] Load error: %s", self.board_id, e)
                self._data = {}
        else:
            self._data = {}

    def save(self) -> None:
        try:
            with open(self.filepath, "w") as f:
                json.dump(self._data, f, indent=2)
        except Exception as e:
            logging.error("[WP %s] Save error: %s", self.board_id, e)

    def get(self, key: str) -> dict | None:
        with self._lock:
            return self._data.get(key)

    def set(self, key: str, waypoint: dict) -> None:
        with self._lock:
            waypoint["key"]      = key
            waypoint["board_id"] = self.board_id
            self._data[key]      = waypoint
        self.save()

    def delete(self, key: str) -> bool:
        with self._lock:
            if key in self._data:
                del self._data[key]
                self.save()
                return True
        return False

    def all_keys(self) -> list[str]:
        with self._lock:
            return list(self._data.keys())

    def as_dict(self) -> dict:
        with self._lock:
            return dict(self._data)

    def calibration_status(self) -> dict:
        """Return per-cell and per-slot calibration completeness for dashboard display."""
        status = {}
        for i in range(9):
            phases = {
                "approach": self.get(f"cell_{i}_approach") is not None,
                "place":    self.get(f"cell_{i}_place")    is not None,
                "lift":     self.get(f"cell_{i}_lift")     is not None,
            }
            phases["complete"] = phases["approach"] and phases["place"]
            status[f"cell_{i}"] = phases

        # 4 Tray slots
        for s in range(4):
            slot_phases = {
                "approach": (self.get(f"tray_slot_{s}_approach") is not None) or (self.get("tray_approach") is not None),
                "pick":     (self.get(f"tray_slot_{s}_pick") is not None)     or (self.get("tray_pick") is not None),
                "lift":     (self.get(f"tray_slot_{s}_lift") is not None)     or (self.get("tray_lift") is not None),
            }
            slot_phases["complete"] = slot_phases["approach"] and slot_phases["pick"]
            status[f"tray_slot_{s}"] = slot_phases

        status["scan_pose"]   = self.get("scan_pose")  is not None
        status["pick_safe"]   = (self.get("pick_safe") is not None) or (self.get("safe_position") is not None)
        status["place_safe"]  = (self.get("place_safe") is not None) or (self.get("safe_position") is not None)

        # Ready check: requires scan_pose, pick_safe, place_safe, all 4 tray slots (or fallback tray_pick), and all 9 cells
        slots_ready = all(status[f"tray_slot_{s}"]["complete"] for s in range(4)) or (self.get("tray_pick") is not None)
        status["ready"] = all([
            status["scan_pose"], status["pick_safe"], status["place_safe"],
            slots_ready,
            all(status[f"cell_{i}"]["complete"] for i in range(9)),
        ])
        return status

    def missing_keys(self) -> list[str]:
        missing = []
        for key in cfg.REQUIRED_WAYPOINT_KEYS:
            if self.get(key) is None:
                # Check backward compatible aliases
                if key == "pick_safe" and self.get("safe_position"):
                    continue
                if key == "place_safe" and self.get("safe_position"):
                    continue
                if key.startswith("tray_slot_") and self.get("tray_pick"):
                    continue
                missing.append(key)
        return missing


# ─────────────────────────────────────────────────────────────────────────────
# TTTMoveExecutor
# ─────────────────────────────────────────────────────────────────────────────
class TTTMoveExecutor:
    """
    Orchestrates arm + rail movements for TTT game play.
    Follows:
      Pick Sequence from Tray Slot (tray_slot_S_approach -> tray_slot_S_pick [Vac ON] -> tray_slot_S_lift)
      -> Pick Safe Position (pick_safe)
      -> Place Safe Position (place_safe)
      -> Place Sequence (cell_i_approach -> cell_i_place [Vac OFF] -> cell_i_lift)
      -> Place Safe Position (place_safe)
      -> Scan Pose (scan_pose)
    """

    def __init__(self, robot, rail, stores: dict[str, TTTWaypointStore],
                 socketio_ref=None):
        self.robot     = robot
        self.rail      = rail
        self.stores    = stores        # {"B1": TTTWaypointStore, "B2": TTTWaypointStore}
        self._sio      = socketio_ref  # for emitting progress events
        self._lock     = threading.Lock()
        self._stop_evt = threading.Event()

        # Track which rail station the arm is currently at
        self._current_board: str | None = None

        # Command-done synchronisation
        self._cmd_done_event = threading.Event()
        self._pending_cmd_id : str | None = None

    # ── Public: game moves ────────────────────────────────────────────────
    def execute_robot_turn(self, board_id: str, cell_idx: int,
                           slot_idx: int = 0, progress_cb=None) -> bool:
        """
        Full pick-and-place robot turn using one of the 4 tray slots:
          1. Cross-board transit if needed (place_safe -> rail_move)
          2. Move to pick_safe
          3. Pick Sequence from Tray Slot S (0..3):
             tray_slot_S_approach -> tray_slot_S_pick (Vac ON) -> tray_slot_S_lift
          4. Move to pick_safe
          5. Move to place_safe
          6. Place Sequence: cell_N_approach -> cell_N_place (Vac OFF) -> cell_N_lift
          7. Move to place_safe
          8. Return to scan_pose
        """
        self._stop_evt.clear()
        store = self.stores[board_id]
        slot  = slot_idx % 4  # 4 slots in the tray (0, 1, 2, 3)

        # Slot-specific waypoint keys with fallback to generic tray keys
        appr_key = f"tray_slot_{slot}_approach" if store.get(f"tray_slot_{slot}_approach") else "tray_approach"
        pick_key = f"tray_slot_{slot}_pick"     if store.get(f"tray_slot_{slot}_pick")     else "tray_pick"
        lift_key = f"tray_slot_{slot}_lift"     if store.get(f"tray_slot_{slot}_lift")     else "tray_lift"

        def emit(phase: str, detail: str = ""):
            if self._sio:
                import server_ttt as _srv
                self._sio.emit("robot_move_start",
                               {"board_id": board_id, "cell": cell_idx, "slot": slot,
                                "phase": phase, "detail": detail,
                                "seq": _srv._next_seq()})
            if progress_cb:
                progress_cb(phase, detail)
            logging.info("[Executor %s] Phase: %s %s", board_id, phase, detail)

        try:
            # ── 1. Safe travel if switching rail stations ─────────────────
            if self._current_board and self._current_board != board_id:
                emit("place_safe", "Retracting to Place Safe position for rail travel")
                if not self._move_to("place_safe", self._current_board):
                    return False
                if self._stop_evt.is_set():
                    return False
                emit("rail_move", f"Moving rail to {board_id}")
                if not self._move_rail_to(board_id):
                    return False
            elif self._current_board is None:
                emit("rail_move", f"Moving rail to {board_id}")
                self._move_rail_to(board_id)

            if self._stop_evt.is_set():
                return False

            # ── 2. Move to Pick Safe Position ─────────────────────────────
            emit("pick_safe", "Moving to Pick Safe Position")
            if not self._move_to("pick_safe", board_id):
                return False
            if self._stop_evt.is_set():
                return False

            # ── 3. Pick Sequence (from Tray Slot S) ───────────────────────
            emit("tray_approach", f"Approaching Tray Slot {slot}")
            if not self._move_to(appr_key, board_id):
                return False
            if self._stop_evt.is_set():
                return False

            emit("tray_pick", f"Picking token from Slot {slot} — vacuum ON")
            if not self._move_to(pick_key, board_id):
                return False
            self._set_vacuum("on")
            if self._stop_evt.is_set():
                return False

            emit("tray_lift", f"Lifting token from Slot {slot}")
            if not self._move_to(lift_key, board_id):
                return False
            if self._stop_evt.is_set():
                return False

            # ── 4. Retract to Pick Safe Position ──────────────────────────
            emit("pick_safe", "Retracting to Pick Safe Position")
            if not self._move_to("pick_safe", board_id):
                return False
            if self._stop_evt.is_set():
                return False

            # ── 5. Move to Place Safe Position ────────────────────────────
            emit("place_safe", "Moving to Place Safe Position")
            if not self._move_to("place_safe", board_id):
                return False
            if self._stop_evt.is_set():
                return False

            # ── 6. Place Sequence (at Cell N) ─────────────────────────────
            emit("cell_approach", f"Approaching cell {cell_idx}")
            if not self._move_to(f"cell_{cell_idx}_approach", board_id):
                return False
            if self._stop_evt.is_set():
                return False

            emit("cell_place", f"Placing token on cell {cell_idx} — vacuum OFF")
            if not self._move_to(f"cell_{cell_idx}_place", board_id):
                return False
            self._set_vacuum("off")
            if self._stop_evt.is_set():
                return False

            # Lift waypoint if taught, else continue
            if store.get(f"cell_{cell_idx}_lift"):
                emit("cell_lift", f"Lifting from cell {cell_idx}")
                if not self._move_to(f"cell_{cell_idx}_lift", board_id):
                    return False
                if self._stop_evt.is_set():
                    return False

            # ── 7. Retract to Place Safe Position ─────────────────────────
            emit("place_safe", "Retracting to Place Safe Position")
            if not self._move_to("place_safe", board_id):
                return False
            if self._stop_evt.is_set():
                return False

            # ── 8. Move to Scan Pose for Camera Vision ────────────────────
            emit("scan_pose", "Moving to scan pose for camera vision")
            if not self._move_to("scan_pose", board_id):
                return False

            self._current_board = board_id
            if self._sio:
                self._sio.emit("robot_move_done",
                               {"board_id": board_id, "cell": cell_idx, "slot": slot})
            return True

        except Exception as e:
            logging.error("[Executor] execute_robot_turn error: %s", e)
            return False

    def go_to_scan_pose(self, board_id: str) -> bool:
        """Move arm to scan pose (used before triggering vision)."""
        self._stop_evt.clear()
        if self._current_board != board_id:
            if self._current_board is not None:
                self._move_to("place_safe", self._current_board)
            self._move_rail_to(board_id)
        return self._move_to("scan_pose", board_id)

    def go_to_safe(self, board_id: str, safe_type: str = "place") -> bool:
        """Retract arm to safe position (before rail travel or standby)."""
        self._stop_evt.clear()
        key = "pick_safe" if safe_type == "pick" else "place_safe"
        return self._move_to(key, board_id)

    def stop(self) -> None:
        """Signal stop to interrupt any running motion sequence."""
        self._stop_evt.set()
        self.robot.emergency_stop()
        # Unblock any pending wait
        self._cmd_done_event.set()

    # ── Command completion callback (registered with RobotManager) ────────
    def on_command_complete(self, cmd_id: str | None) -> None:
        if cmd_id == self._pending_cmd_id:
            self._cmd_done_event.set()

    # ── Internal helpers ──────────────────────────────────────────────────
    def _move_to(self, key: str, board_id: str) -> bool:
        """Fetch waypoint by key, send to arm (angles or coords), wait for completion."""
        store = self.stores.get(board_id)
        if store is None:
            logging.error("[Executor] Unknown board_id: %s", board_id)
            return False

        wp = store.get(key)
        # Fallback aliases for safe positions
        if wp is None:
            if key in ("pick_safe", "place_safe"):
                wp = store.get("safe_position")
            elif key == "safe_position":
                wp = store.get("place_safe") or store.get("pick_safe")

        if wp is None:
            logging.error("[Executor] Waypoint '%s' not found for board %s.", key, board_id)
            return False

        speed  = cfg.ARM_SPEED_OVERRIDE if cfg.ARM_SPEED_OVERRIDE else wp.get("speed", cfg.ARM_SPEED)
        cmd_id = f"ttt_{key}_{time.time():.3f}"
        self._cmd_done_event.clear()
        self._pending_cmd_id = cmd_id

        # Check if waypoint is coordinate-based or angle-based
        wp_type = wp.get("type", "angles")
        coords  = wp.get("coords")
        angles  = wp.get("data")

        if wp_type == "coords" and coords and len(coords) == 6:
            logging.info("[Executor] Moving to '%s' (coords=%s, board=%s, speed=%d).", key, coords, board_id, speed)
            self.robot.send_coords(coords, speed=speed, cmd_id=cmd_id)
        elif angles and len(angles) == 6:
            logging.info("[Executor] Moving to '%s' (angles=%s, board=%s, speed=%d).", key, angles, board_id, speed)
            self.robot.send_angles(angles, speed=speed, cmd_id=cmd_id)
        else:
            logging.error("[Executor] Invalid data for waypoint '%s'.", key)
            return False

        # Wait for completion or stop signal
        done = self._cmd_done_event.wait(timeout=cfg.MOVE_TIMEOUT_SEC)
        self._pending_cmd_id = None

        if not done:
            logging.warning("[Executor] Move timeout for key '%s'.", key)
            return False
        if self._stop_evt.is_set():
            return False
        return True

    def _set_vacuum(self, state: str) -> None:
        """Activate or deactivate vacuum. The delay is enforced by the worker
        queue action itself — no extra sleep needed here."""
        delay_ms = (cfg.VACUUM_ON_DELAY_MS if state == "on"
                    else cfg.VACUUM_OFF_DELAY_MS)
        self.robot.set_vacuum(state, delay=delay_ms)
        # Wait for the vacuum to settle (the worker handles this inline)
        time.sleep(delay_ms / 1000.0)
        logging.info("[Executor] Vacuum %s.", state.upper())

    def _move_rail_to(self, board_id: str) -> bool:
        """Send rail move command and wait for arrival."""
        preset = cfg.RAIL_PRESET_MAP.get(board_id)
        if not preset:
            logging.error("[Executor] No rail preset for board %s.", board_id)
            return False
        logging.info("[Executor] Rail → %s (%s).", board_id, preset)
        self.rail.move_to_preset(preset)
        # Poll until rail reports it is at the target preset
        timeout = 30.0
        start   = time.time()
        while time.time() - start < timeout:
            if self._stop_evt.is_set():
                return False
            rail_state = self.rail.current_state
            if rail_state.get("point", "").upper() == preset.upper() \
                    and not rail_state.get("running", True):
                logging.info("[Executor] Rail arrived at %s.", preset)
                self._current_board = board_id
                return True
            time.sleep(0.3)
        logging.warning("[Executor] Rail move to %s timed out.", preset)
        self._current_board = board_id  # assume arrived
        return True
