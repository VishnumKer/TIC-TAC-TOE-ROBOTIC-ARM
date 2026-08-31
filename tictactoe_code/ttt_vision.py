"""
ttt_vision.py  —  Board Vision Module
=======================================
Camera is mounted on the robotic arm.
The arm travels to a dedicated `scan_pose` before vision is triggered.
Once triggered, this module captures CONFIRM_FRAMES of stable data
and reports the full 9-cell board state as ["H"/"R"/""] per cell.

Color scheme:
  RED  tokens → Human  (H)
  BLUE tokens → Robot  (R)
"""

from __future__ import annotations
import cv2
import numpy as np
import threading
import time
import logging
import io

import config_ttt as cfg


# ─────────────────────────────────────────────────────────────────────────────
# BoardVision
# ─────────────────────────────────────────────────────────────────────────────
class BoardVision:
    """
    Runs a background camera capture loop.
    On demand (triggered by move executor), performs a stable-frame
    scan of all 9 board cells and returns their occupancy.
    """

    def __init__(self):
        self._cap: cv2.VideoCapture | None = None
        self._cap_lock     = threading.Lock()
        self._frame_lock   = threading.Lock()
        self._latest_frame: np.ndarray | None = None
        self._latest_annotated_jpg: bytes | None = None
        self._scan_active    = False          # True while scanning
        self._scan_board_id  = "B1"
        self._scan_result    : list[str] | None = None   # 9-element result
        self._scan_event     = threading.Event()          # fires when scan done
        self._stop_event     = threading.Event()

        # Per-cell consecutive-frame counters {board_id: {cell_i: {H: int, R: int}}}
        self._cell_counts: dict[str, list[dict]] = {
            bid: [{"H": 0, "R": 0} for _ in range(9)]
            for bid in cfg.BOARD_IDS
        }
        self._robot_cells: dict[str, set[int]] = {
            bid: set() for bid in cfg.BOARD_IDS
        }

        # Background capture thread
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    # ── Camera open / close ───────────────────────────────────────────────
    def _open_camera(self) -> bool:
        backend = getattr(cv2, cfg.CAMERA_BACKEND, cv2.CAP_ANY)
        cap = cv2.VideoCapture(cfg.CAMERA_INDEX, backend)
        if not cap.isOpened():
            cap = cv2.VideoCapture(cfg.CAMERA_INDEX)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_FPS, 30)
            with self._cap_lock:
                self._cap = cap
            logging.info("[Vision] Camera opened (index=%d).", cfg.CAMERA_INDEX)
            return True
        logging.warning("[Vision] Could not open camera index %d.", cfg.CAMERA_INDEX)
        return False

    # ── Background capture loop ───────────────────────────────────────────
    def _capture_loop(self):
        while not self._stop_event.is_set():
            with self._cap_lock:
                cap = self._cap
            if cap is None or not cap.isOpened():
                if not self._open_camera():
                    time.sleep(3.0)
                continue

            ret, frame = cap.read()
            if not ret:
                logging.warning("[Vision] Frame read failed — retrying.")
                time.sleep(0.5)
                continue

            with self._frame_lock:
                self._latest_frame = frame.copy()

            if self._scan_active:
                self._process_frame(frame, self._scan_board_id)
            else:
                # Idle: still annotate for live stream
                ann = self._annotate_frame(frame, self._scan_board_id, scanning=False)
                ok, buf = cv2.imencode(".jpg", ann, [cv2.IMWRITE_JPEG_QUALITY, 75])
                if ok:
                    with self._frame_lock:
                        self._latest_annotated_jpg = buf.tobytes()

    # ── Public API ────────────────────────────────────────────────────────
    def start_scan(self, board_id: str, robot_cells: list[int] | None = None, timeout: float = 10.0) -> list[str] | None:
        """
        Trigger a scan of the given board. Blocks until CONFIRM_FRAMES
        of stable data are collected or timeout is reached.
        Returns 9-element list ["H"/"R"/""] or None on timeout.
        """
        if board_id not in cfg.BOARD_IDS:
            logging.error("[Vision] Unknown board_id: %s", board_id)
            return None

        # Reset counters for fresh scan
        self._cell_counts[board_id] = [{"H": 0, "R": 0} for _ in range(9)]
        self._robot_cells[board_id] = set(robot_cells or [])
        for idx in self._robot_cells[board_id]:
            self._cell_counts[board_id][idx]["R"] = cfg.CONFIRM_FRAMES
            
        self._scan_result = None
        self._scan_event.clear()
        self._scan_board_id = board_id
        self._scan_active   = True

        logging.info("[Vision] Scan started for board %s.", board_id)
        fired = self._scan_event.wait(timeout=timeout)
        self._scan_active = False

        if fired and self._scan_result is not None:
            logging.info("[Vision] Scan done: %s", self._scan_result)
            return list(self._scan_result)
        else:
            # Timeout — return best-effort reading
            logging.warning("[Vision] Scan timeout — returning partial result.")
            return self._build_state_from_counts(board_id)

    def get_latest_jpeg(self) -> bytes | None:
        """Return latest annotated JPEG for live stream."""
        with self._frame_lock:
            return self._latest_annotated_jpg

    def diff_board(self, old_state: list[str], new_state: list[str]) -> int | None:
        """
        Return cell index that changed from empty to occupied (human's move),
        or None if no single new human cell found.
        """
        changed = [i for i in range(9)
                   if old_state[i] == "" and new_state[i] == "H"]
        return changed[0] if len(changed) == 1 else None

    def update_roi(self, board_id: str, cell_idx: int, roi: list[int]) -> None:
        """Update cell ROI at runtime (called from dashboard calibration)."""
        key = f"cell_{cell_idx}"
        cfg.BOARD_CELL_ROIS[board_id][key] = roi
        logging.info("[Vision] ROI updated: %s %s → %s", board_id, key, roi)

    def stop(self) -> None:
        self._stop_event.set()

    # ── Frame processing ─────────────────────────────────────────────────
    def _process_frame(self, frame: np.ndarray, board_id: str) -> None:
        """Classify each cell in the frame and update counters."""
        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        rois = cfg.BOARD_CELL_ROIS.get(board_id, {})
        counts = self._cell_counts[board_id]

        for i in range(9):
            if i in self._robot_cells.get(board_id, set()):
                counts[i]["R"] = cfg.CONFIRM_FRAMES
                counts[i]["H"] = 0
                continue

            key = f"cell_{i}"
            roi = rois.get(key)
            if not roi:
                continue
            x0, y0, x1, y1 = roi
            if x0 >= x1 or y0 >= y1:
                continue

            color = self._classify_cell(frame_hsv, x0, y0, x1, y1)
            for player in ("H", "R"):
                if color == player:
                    counts[i][player] = min(counts[i][player] + 1, cfg.CONFIRM_FRAMES + 2)
                else:
                    counts[i][player] = max(counts[i][player] - 1, 0)

        # Check if all cells are stable
        if all(
            max(counts[i]["H"], counts[i]["R"]) >= cfg.CONFIRM_FRAMES
            or (counts[i]["H"] == 0 and counts[i]["R"] == 0)
            for i in range(9)
        ):
            self._scan_result = self._build_state_from_counts(board_id)
            self._scan_event.set()

        # Annotate for stream
        ann = self._annotate_frame(frame, board_id, scanning=True)
        ok, buf = cv2.imencode(".jpg", ann, [cv2.IMWRITE_JPEG_QUALITY, 75])
        if ok:
            with self._frame_lock:
                self._latest_annotated_jpg = buf.tobytes()

    def _classify_cell(self, hsv: np.ndarray, x0: int, y0: int,
                        x1: int, y1: int) -> str:
        """Return 'H', 'R', or '' for the circular region."""
        region = hsv[y0:y1, x0:x1]
        h, w = region.shape[:2]
        if h == 0 or w == 0:
            return ""

        # Create a circular mask for the region
        mask_circle = np.zeros((h, w), dtype=np.uint8)
        cx_rel, cy_rel = w // 2, h // 2
        r = min(w, h) // 2
        cv2.circle(mask_circle, (cx_rel, cy_rel), r, 255, -1)

        area = int(np.pi * r * r)
        if area == 0:
            return ""

        # White mask (Human)
        mask_w = cv2.inRange(region,
                             np.array(cfg.WHITE_HSV_LOWER, dtype=np.uint8),
                             np.array(cfg.WHITE_HSV_UPPER, dtype=np.uint8))
        mask_w = cv2.bitwise_and(mask_w, mask_circle)
        white_ratio = cv2.countNonZero(mask_w) / area

        if white_ratio >= cfg.TOKEN_PIXEL_RATIO:
            return "H"
        return ""

    def _build_state_from_counts(self, board_id: str) -> list[str]:
        counts = self._cell_counts[board_id]
        state  = []
        for i in range(9):
            h = counts[i]["H"]
            r = counts[i]["R"]
            if h >= cfg.CONFIRM_FRAMES and h >= r:
                state.append("H")
            elif r >= cfg.CONFIRM_FRAMES:
                state.append("R")
            else:
                state.append("")
        return state

    # ── Annotation ────────────────────────────────────────────────────────
    def _annotate_frame(self, frame: np.ndarray, board_id: str,
                        scanning: bool) -> np.ndarray:
        out  = frame.copy()
        rois = cfg.BOARD_CELL_ROIS.get(board_id, {})
        counts = self._cell_counts.get(board_id, [{"H": 0, "R": 0}] * 9)

        for i in range(9):
            key = f"cell_{i}"
            roi = rois.get(key)
            if not roi:
                continue
            x0, y0, x1, y1 = roi
            
            # Calculate circle center and radius
            cx = (x0 + x1) // 2
            cy = (y0 + y1) // 2
            r = (x1 - x0) // 2
            if r <= 0:
                continue

            h = counts[i]["H"] if i < len(counts) else 0
            r_count = counts[i]["R"] if i < len(counts) else 0

            if h >= cfg.CONFIRM_FRAMES:
                color = (240, 240, 240)    # White color -> Human
                label = "H"
            elif r_count >= cfg.CONFIRM_FRAMES:
                color = (220, 80, 0)   # Blue color -> Robot
                label = "R"
            else:
                color = (180, 180, 180)
                label = str(i)

            # Draw circle instead of rectangle!
            cv2.circle(out, (cx, cy), r, color, 2)
            cv2.putText(out, label, (cx - 5, cy + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            if scanning:
                # Progress indicator as an inner green circle
                prog = min(max(h, r_count) / cfg.CONFIRM_FRAMES, 1.0)
                if prog > 0:
                    inner_r = max(int(r * prog), 1)
                    cv2.circle(out, (cx, cy), inner_r, (0, 220, 60), 1)

        # Board label
        label_text = f"Board {board_id}  {'[SCANNING]' if scanning else ''}"
        cv2.putText(out, label_text, (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        return out
