"""
ttt_engine.py  —  Tic Tac Toe Game Logic
==========================================
Pure Python — zero hardware dependencies.
Contains: Board, MinimaxAI, GameSession, GameManager.
"""

from __future__ import annotations
import copy
import time
import threading
import logging

# ── Win conditions (index triples) ───────────────────────────────────────────
WIN_LINES = [
    (0, 1, 2), (3, 4, 5), (6, 7, 8),   # rows
    (0, 3, 6), (1, 4, 7), (2, 5, 8),   # cols
    (0, 4, 8), (2, 4, 6),               # diagonals
]

# Player tokens
HUMAN = "H"   # Human  (red token)
ROBOT = "R"   # Robot  (blue token)
EMPTY = ""


# ─────────────────────────────────────────────────────────────────────────────
# Board
# ─────────────────────────────────────────────────────────────────────────────
class Board:
    """
    Immutable-ish 3×3 board.
    grid[0..8]: "" | "H" | "R"

    Layout (cell indices):
        0 | 1 | 2
        ---------
        3 | 4 | 5
        ---------
        6 | 7 | 8
    """

    def __init__(self, grid: list[str] | None = None):
        self.grid: list[str] = grid if grid is not None else [EMPTY] * 9

    def copy(self) -> "Board":
        return Board(list(self.grid))

    def mark(self, cell: int, player: str) -> None:
        """Mark a cell for a player. Raises ValueError if cell is occupied."""
        if not (0 <= cell <= 8):
            raise ValueError(f"Cell {cell} out of range 0-8")
        if self.grid[cell] != EMPTY:
            raise ValueError(f"Cell {cell} is already occupied by '{self.grid[cell]}'")
        self.grid[cell] = player

    def available_cells(self) -> list[int]:
        return [i for i, v in enumerate(self.grid) if v == EMPTY]

    def winner(self) -> str | None:
        """Return 'H', 'R', or None."""
        for a, b, c in WIN_LINES:
            if self.grid[a] != EMPTY and self.grid[a] == self.grid[b] == self.grid[c]:
                return self.grid[a]
        return None

    def winning_line(self) -> list[int] | None:
        """Return list of 3 cell indices forming the winning line, or None."""
        for a, b, c in WIN_LINES:
            if self.grid[a] != EMPTY and self.grid[a] == self.grid[b] == self.grid[c]:
                return [a, b, c]
        return None

    def is_draw(self) -> bool:
        return self.winner() is None and not self.available_cells()

    def is_terminal(self) -> bool:
        return self.winner() is not None or self.is_draw()

    def __repr__(self) -> str:
        g = [v if v else "." for v in self.grid]
        return f"{g[0]}|{g[1]}|{g[2]}\n{g[3]}|{g[4]}|{g[5]}\n{g[6]}|{g[7]}|{g[8]}"


# ─────────────────────────────────────────────────────────────────────────────
# Minimax AI (Alpha-Beta Pruning)
# ─────────────────────────────────────────────────────────────────────────────
class MinimaxAI:
    """
    Unbeatable Minimax AI with alpha-beta pruning.
    Robot (R) is the maximising player.
    Human (H) is the minimising player.
    """

    def best_move(self, board: Board) -> int:
        """Return the optimal cell index (0-8) for the ROBOT."""
        available = board.available_cells()
        if not available:
            raise ValueError("No available cells — board is full.")

        best_score = float("-inf")
        best_cell  = available[0]

        for cell in available:
            b = board.copy()
            b.mark(cell, ROBOT)
            score = self._minimax(b, depth=0, is_max=False,
                                  alpha=float("-inf"), beta=float("inf"))
            if score > best_score:
                best_score = score
                best_cell  = cell

        return best_cell

    def _minimax(self, board: Board, depth: int, is_max: bool,
                 alpha: float, beta: float) -> int:
        winner = board.winner()
        if winner == ROBOT:
            return 10 - depth
        if winner == HUMAN:
            return depth - 10
        if board.is_draw():
            return 0

        available = board.available_cells()

        if is_max:
            best = float("-inf")
            for cell in available:
                b = board.copy()
                b.mark(cell, ROBOT)
                score = self._minimax(b, depth + 1, False, alpha, beta)
                best  = max(best, score)
                alpha = max(alpha, best)
                if beta <= alpha:
                    break
            return best
        else:
            best = float("inf")
            for cell in available:
                b = board.copy()
                b.mark(cell, HUMAN)
                score = self._minimax(b, depth + 1, True, alpha, beta)
                best  = min(best, score)
                beta  = min(beta, best)
                if beta <= alpha:
                    break
            return best


# ─────────────────────────────────────────────────────────────────────────────
# GameSession — one game on one board
# ─────────────────────────────────────────────────────────────────────────────
class GamePhase:
    IDLE        = "IDLE"
    HUMAN_TURN  = "HUMAN_TURN"
    ROBOT_TURN  = "ROBOT_TURN"
    GAME_OVER   = "GAME_OVER"


class GameSession:
    """
    Manages one complete Tic Tac Toe game on a single board.
    Thread-safe via internal lock.
    By default Human plays first (HUMAN_TURN), but robot_first=True makes Robot go first.
    Robot plays at most 4 moves per game (from 4 physical tray slots).
    """

    def __init__(self, board_id: str):
        self.board_id          = board_id
        self.board             = Board()
        self.phase             = GamePhase.IDLE
        self.winner_val        : str | None = None     # "H" | "R" | "DRAW" | None
        self.win_line          : list[int] | None = None
        self.move_log          : list[dict] = []
        self.scores            = {"human_wins": 0, "robot_wins": 0, "draws": 0, "rounds": 0}
        self.robot_moves_count = 0                     # tracks which of the 4 tray slots to use (0..3)
        self._lock             = threading.Lock()
        self._ai               = MinimaxAI()

    # ── State helpers ──────────────────────────────────────────────────────
    @property
    def grid(self) -> list[str]:
        with self._lock:
            return list(self.board.grid)

    def get_state_snapshot(self) -> dict:
        with self._lock:
            return {
                "board_id":          self.board_id,
                "grid":              list(self.board.grid),
                "phase":             self.phase,
                "winner":            self.winner_val,
                "win_line":          self.win_line,
                "scores":            dict(self.scores),
                "move_log":          list(self.move_log),
                "robot_moves_count": self.robot_moves_count,
            }

    # ── Game control ───────────────────────────────────────────────────────
    def start(self, robot_first: bool = False) -> bool:
        """Transition from IDLE/GAME_OVER → HUMAN_TURN or ROBOT_TURN depending on robot_first."""
        with self._lock:
            if self.phase not in (GamePhase.IDLE, GamePhase.GAME_OVER):
                return False
            self.board             = Board()
            self.phase             = GamePhase.ROBOT_TURN if robot_first else GamePhase.HUMAN_TURN
            self.winner_val        = None
            self.win_line          = None
            self.move_log          = []
            self.robot_moves_count = 0
        first = "Robot" if robot_first else "Human"
        logging.info("[TTT %s] Game started — %s plays first.", self.board_id, first)
        return True

    def reset(self) -> None:
        """Hard reset to IDLE without updating scores."""
        with self._lock:
            self.board             = Board()
            self.phase             = GamePhase.IDLE
            self.winner_val        = None
            self.win_line          = None
            self.move_log          = []
            self.robot_moves_count = 0
        logging.info("[TTT %s] Game reset to IDLE.", self.board_id)

    # ── Move application ───────────────────────────────────────────────────
    def apply_human_move(self, cell: int) -> dict:
        """
        Mark the human's move and transition to ROBOT_TURN (or GAME_OVER).
        Returns result dict with keys: success, phase, winner, win_line, error.
        """
        with self._lock:
            if self.phase != GamePhase.HUMAN_TURN:
                return {"success": False, "error": f"Not human's turn (phase={self.phase})"}
            if self.board.grid[cell] != EMPTY:
                return {"success": False, "error": f"Cell {cell} already occupied"}

            self.board.mark(cell, HUMAN)
            self.move_log.append({"player": HUMAN, "cell": cell, "ts": time.time()})
            logging.info("[TTT %s] Human played cell %d.", self.board_id, cell)

            return self._check_terminal_and_advance(next_phase=GamePhase.ROBOT_TURN)

    def apply_robot_move(self, cell: int) -> dict:
        """
        Mark the robot's move, increment robot_moves_count, and transition to HUMAN_TURN (or GAME_OVER).
        """
        with self._lock:
            if self.phase != GamePhase.ROBOT_TURN:
                return {"success": False, "error": f"Not robot's turn (phase={self.phase})"}
            if self.board.grid[cell] != EMPTY:
                return {"success": False, "error": f"Cell {cell} already occupied"}

            self.board.mark(cell, ROBOT)
            self.move_log.append({"player": ROBOT, "cell": cell,
                                  "slot_used": self.robot_moves_count, "ts": time.time()})
            self.robot_moves_count += 1
            logging.info("[TTT %s] Robot played cell %d (slot %d). Total robot moves: %d.",
                         self.board_id, cell, (self.robot_moves_count - 1), self.robot_moves_count)

            return self._check_terminal_and_advance(next_phase=GamePhase.HUMAN_TURN)

    def get_robot_cell(self) -> int:
        """Return Minimax best move. Call only when phase == ROBOT_TURN."""
        with self._lock:
            board_copy = self.board.copy()
        cell = self._ai.best_move(board_copy)
        logging.info("[TTT %s] Minimax chose cell %d.", self.board_id, cell)
        return cell

    # ── Internal ───────────────────────────────────────────────────────────
    def _check_terminal_and_advance(self, next_phase: str) -> dict:
        """Must be called with self._lock held."""
        w = self.board.winner()
        if w:
            self.winner_val = w
            self.win_line   = self.board.winning_line()
            self.phase      = GamePhase.GAME_OVER
            self._update_scores(w)
            logging.info("[TTT %s] Game over — winner: %s.", self.board_id, w)
            return {"success": True, "phase": GamePhase.GAME_OVER,
                    "winner": w, "win_line": self.win_line}
        if self.board.is_draw():
            self.winner_val = "DRAW"
            self.win_line   = None
            self.phase      = GamePhase.GAME_OVER
            self._update_scores("DRAW")
            logging.info("[TTT %s] Game over — draw.", self.board_id)
            return {"success": True, "phase": GamePhase.GAME_OVER,
                    "winner": "DRAW", "win_line": None}

        self.phase = next_phase
        return {"success": True, "phase": next_phase, "winner": None, "win_line": None}

    def _update_scores(self, result: str) -> None:
        """Must be called with self._lock held."""
        self.scores["rounds"] += 1
        if result == HUMAN:
            self.scores["human_wins"] += 1
        elif result == ROBOT:
            self.scores["robot_wins"] += 1
        else:
            self.scores["draws"] += 1


# ─────────────────────────────────────────────────────────────────────────────
# GameManager — orchestrates 1 or 2 simultaneous sessions
# ─────────────────────────────────────────────────────────────────────────────
class GameManager:
    """
    Manages 1 or 2 GameSession objects.
    In 2-game mode the robot interleaves turns: B1 -> B2 -> B1 -> ...
    """

    def __init__(self):
        self.sessions: dict[str, GameSession] = {
            "B1": GameSession("B1"),
            "B2": GameSession("B2"),
        }
        self._mode                  = 1       # 1 or 2
        self.selected_single_board  = "B1"    # "B1" or "B2" when mode == 1
        self._lock                  = threading.Lock()
        self._b2_active             = False   # True when 2-game mode AND B2 game started
        self.robot_first            = False   # If True, robot makes the first move each game

    # ── Mode ───────────────────────────────────────────────────────────────
    @property
    def mode(self) -> int:
        return self._mode

    def set_mode(self, mode: int, target_board: str = "B1") -> None:
        if mode not in (1, 2):
            raise ValueError("mode must be 1 or 2")
        with self._lock:
            self._mode = mode
            if mode == 1:
                if target_board in ("B1", "B2"):
                    self.selected_single_board = target_board
                inactive = "B2" if self.selected_single_board == "B1" else "B1"
                self.sessions[inactive].reset()
                self._b2_active = False
            else:
                self._b2_active = True
        logging.info("[GameManager] Mode set to %d (selected_board=%s).", mode, self.selected_single_board)

    def set_robot_first(self, enabled: bool) -> None:
        """Set whether the robot makes the first move each game."""
        with self._lock:
            self.robot_first = bool(enabled)
        logging.info("[GameManager] Robot-first mode: %s.", self.robot_first)

    def active_board_ids(self) -> list[str]:
        """Return list of board IDs currently in an active (non-IDLE) game."""
        with self._lock:
            ids = [self.selected_single_board] if self._mode == 1 else ["B1", "B2"]
        return [bid for bid in ids
                if self.sessions[bid].phase not in (GamePhase.IDLE, GamePhase.GAME_OVER)]

    def start_game(self, board_id: str = "B1", robot_first: bool | None = None) -> bool:
        """Start (or restart a completed) game on the given board.

        robot_first: If None (default), uses the manager's stored robot_first flag.
                     Pass True/False to override for this game only.
        """
        with self._lock:
            if self._mode == 1 and board_id in ("B1", "B2"):
                self.selected_single_board = board_id
                inactive = "B2" if board_id == "B1" else "B1"
                self.sessions[inactive].reset()
        session = self.sessions.get(board_id)
        if session is None:
            return False
        if session.phase == GamePhase.GAME_OVER:
            session.reset()
        rf = self.robot_first if robot_first is None else robot_first
        return session.start(robot_first=rf)

    def reset_game(self, board_id: str | None = None) -> None:
        """Reset one board or all boards."""
        if board_id:
            self.sessions.get(board_id, GameSession(board_id)).reset()
        else:
            for s in self.sessions.values():
                s.reset()

    def next_robot_board(self, preferred_first: str | None = None) -> str | None:
        """
        Return the board_id where the robot needs to move next, or None.
        In 2-game mode, interleaves B1/B2 (prioritizing preferred_first if specified).
        In 1-game mode, returns selected board if ROBOT_TURN.
        """
        with self._lock:
            if self._mode == 2:
                order = ["B1", "B2"]
                if preferred_first in order:
                    order = [preferred_first] + [b for b in order if b != preferred_first]
            else:
                order = [self.selected_single_board]
        for bid in order:
            s = self.sessions[bid]
            if s.phase == GamePhase.ROBOT_TURN:
                return bid
        return None

    def get_snapshot(self) -> dict:
        return {
            "mode":                  self._mode,
            "selected_single_board": self.selected_single_board,
            "robot_first":           self.robot_first,
            "boards": {bid: s.get_state_snapshot() for bid, s in self.sessions.items()},
        }


# ─────────────────────────────────────────────────────────────────────────────
# Self-test (run directly: python ttt_engine.py)
# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    print("=== Minimax self-play test ===")
    ai = MinimaxAI()
    board = Board()
    turn  = HUMAN   # Human goes first

    while not board.is_terminal():
        if turn == ROBOT:
            cell = ai.best_move(board)
            board.mark(cell, ROBOT)
            print(f"Robot -> cell {cell}\n{board}\n")
        else:
            avail = board.available_cells()
            # Human plays first available (worst strategy) to stress-test AI
            cell = avail[0]
            board.mark(cell, HUMAN)
            print(f"Human -> cell {cell}\n{board}\n")
        turn = HUMAN if turn == ROBOT else ROBOT

    w = board.winner()
    print(f"Result: {'Draw' if not w else ('Robot wins' if w == ROBOT else 'Human wins')}")
    assert w in (ROBOT, None), "FAIL: Robot must never lose!"
    print("PASS - Robot never loses [OK]")
