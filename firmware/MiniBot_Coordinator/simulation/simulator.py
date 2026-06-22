"""
simulation/simulator.py  —  MiniBot Chess Swarm Coordinator

Software-in-the-loop motion simulator.

When simulator mode is active in the GUI, MoveCommands are passed here
instead of being sent over serial.  A QTimer drives a motion tick at
SIMULATOR.UPDATE_INTERVAL_MS.  Each tick advances every active piece
toward its target at a configurable speed, then:

  1. Boundary enforcement: piece centre is clamped to the table limits
     (playing area + all four border margins), such that the piece body
     never crosses the outer hard line.

  2. Piece–piece collision detection: if the desired new position would
     place a piece closer than (2 × radius + margin) to any other piece,
     the move for that piece is blocked for this tick (the piece holds
     its current position).  Static (non-moving) pieces are checked
     against a snapshot taken at the start of the tick; this avoids
     order-dependency within a single tick.

Signals:
  position_updated(int, float, float, float)  — id, x_mm, y_mm, theta_deg
  move_complete(int)                           — id when a piece reaches target
  log_message(str)                             — human-readable event string
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

from PyQt6.QtCore import QObject, QTimer, pyqtSignal, pyqtSlot

from config import PIECES, SIMULATOR
from models.piece import BoardState
from planning.base_planner import MoveCommand


class MotionSimulator(QObject):
    """Simulates robot motion for all active MoveCommands.

    Thread safety:
        Intended to run on the GUI thread (driven by QTimer).
        All calls to queue_moves() must also be on the GUI thread.
    """

    position_updated = pyqtSignal(
        int, float, float, float, float
    )  # id, x, y, theta, battery_v(0)
    move_complete = pyqtSignal(int)  # id
    log_message = pyqtSignal(str)

    def __init__(
        self,
        board_state: BoardState,
        speed_mm_s: float = SIMULATOR.DEFAULT_SPEED_MM_S,
        parent: Optional[QObject] = None,
    ) -> None:
        super().__init__(parent)
        self._board = board_state
        self._speed = speed_mm_s
        self._active: Dict[int, MoveCommand] = {}  # piece_id → command
        # Two-phase motion: 'rotate' then 'translate'
        self._phase: Dict[int, str] = {}  # piece_id → 'rotate' | 'translate'
        self._rotate_to: Dict[int, float] = {}  # piece_id → chosen heading (deg)

        self._collision_enabled: bool = True

        self._timer = QTimer(self)
        self._timer.setInterval(SIMULATOR.UPDATE_INTERVAL_MS)
        self._timer.timeout.connect(self._tick)

        # Boundary constants (piece centre limits)
        r = float(PIECES.CIRCLE_RADIUS_MM)
        m = float(SIMULATOR.COLLISION_MARGIN_MM)
        self._x_min = float(SIMULATOR.X_MIN_MM) + r
        self._x_max = float(SIMULATOR.X_MAX_MM) - r
        self._y_min = float(SIMULATOR.Y_MIN_MM) + r
        self._y_max = float(SIMULATOR.Y_MAX_MM) - r
        self._collision_dist = 2.0 * r + m

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def speed_mm_s(self) -> float:
        return self._speed

    @speed_mm_s.setter
    def speed_mm_s(self, value: float) -> None:
        self._speed = max(1.0, value)

    def queue_moves(self, commands: List[MoveCommand]) -> None:
        """Accept a list of MoveCommands to simulate.

        Commands are keyed by piece_id; a new command for an already-moving
        piece replaces the previous target immediately.
        """
        for cmd in commands:
            self._active[cmd.piece_id] = cmd
            self._phase[cmd.piece_id] = "rotate"
            self._rotate_to.pop(cmd.piece_id, None)  # recompute on first tick
        if self._active and not self._timer.isActive():
            self._timer.start()

    def stop_all(self) -> None:
        """Cancel all active moves and stop the timer."""
        self._active.clear()
        self._phase.clear()
        self._rotate_to.clear()
        self._timer.stop()

    @property
    def is_running(self) -> bool:
        return self._timer.isActive()

    @property
    def collision_enabled(self) -> bool:
        return self._collision_enabled

    @collision_enabled.setter
    def collision_enabled(self, value: bool) -> None:
        self._collision_enabled = value

    # ------------------------------------------------------------------
    # Timer tick
    # ------------------------------------------------------------------

    @pyqtSlot()
    def _tick(self) -> None:
        dt = SIMULATOR.UPDATE_INTERVAL_MS / 1000.0
        step = self._speed * dt
        rot_step = SIMULATOR.ROTATION_SPEED_DEG_S * dt

        # Snapshot of all NON-moving piece positions for collision checks.
        snapshot: Dict[int, Tuple[float, float]] = {}
        for piece in self._board.active_pieces():
            if piece.piece_id not in self._active:
                snapshot[piece.piece_id] = (piece.x_mm, piece.y_mm)

        arrived_ids: List[int] = []

        for pid, cmd in list(self._active.items()):
            piece = self._board.get_piece(pid)
            if piece is None or piece.is_captured:
                arrived_ids.append(pid)
                continue

            cur_x, cur_y = piece.x_mm, piece.y_mm
            dx = cmd.target_x_mm - cur_x
            dy = cmd.target_y_mm - cur_y
            dist = math.hypot(dx, dy)

            # Default: hold position and orientation
            new_x, new_y = cur_x, cur_y
            new_theta = piece.orientation_deg

            phase = self._phase.get(pid, "rotate")

            # ── Phase 1: Rotate to face target (forward or backward) ─────
            if phase == "rotate":
                # Lazily compute the rotation target on the first tick.
                if pid not in self._rotate_to:
                    if dist > 1.0:
                        fwd = math.degrees(math.atan2(dy, dx)) % 360.0
                        bwd = (fwd + 180.0) % 360.0
                        diff_fwd = abs(
                            (fwd - piece.orientation_deg + 180.0) % 360.0 - 180.0
                        )
                        diff_bwd = abs(
                            (bwd - piece.orientation_deg + 180.0) % 360.0 - 180.0
                        )
                        self._rotate_to[pid] = fwd if diff_fwd <= diff_bwd else bwd
                    else:
                        # Already essentially at target; no rotation needed.
                        self._rotate_to[pid] = piece.orientation_deg

                rotate_target = self._rotate_to[pid]
                heading_diff = (
                    rotate_target - piece.orientation_deg + 180.0
                ) % 360.0 - 180.0

                if abs(heading_diff) <= rot_step:
                    # Snap to target heading and advance to translate phase.
                    new_theta = rotate_target
                    self._phase[pid] = "translate"
                else:
                    new_theta = (
                        piece.orientation_deg + math.copysign(rot_step, heading_diff)
                    ) % 360.0
                # Hold position while rotating.

            # ── Phase 2: Translate to target, preserving orientation ─────
            elif phase == "translate":
                # Preserve the heading chosen during the rotate phase.
                new_theta = self._rotate_to.get(pid, piece.orientation_deg)

                if dist <= step:
                    new_x, new_y = cmd.target_x_mm, cmd.target_y_mm
                else:
                    new_x = cur_x + (dx / dist) * step
                    new_y = cur_y + (dy / dist) * step

            # ── 1. Boundary enforcement (clamp to table limits) ──────────
            clamped_x = max(self._x_min, min(self._x_max, new_x))
            clamped_y = max(self._y_min, min(self._y_max, new_y))
            if (clamped_x, clamped_y) != (new_x, new_y):
                self.log_message.emit(
                    f"SIM: 0x{pid:02X} boundary clamped "
                    f"({new_x:.1f},{new_y:.1f})→({clamped_x:.1f},{clamped_y:.1f})"
                )
                new_x, new_y = clamped_x, clamped_y

            # ── 2. Piece–piece collision check (vs. static snapshot) ─────
            blocked = False
            if self._collision_enabled:
                for other_pid, (ox, oy) in snapshot.items():
                    if math.hypot(new_x - ox, new_y - oy) < self._collision_dist:
                        self.log_message.emit(
                            f"SIM: 0x{pid:02X} blocked by 0x{other_pid:02X}"
                        )
                        blocked = True
                        break

            if blocked:
                # Hold position this tick but allow in-place rotation
                new_x, new_y = cur_x, cur_y

            # ── Arrival check ─────────────────────────────────────────────
            # Arrived when translate phase completes (position reached).
            pos_done = (
                self._phase.get(pid) == "translate"
                and math.hypot(new_x - cmd.target_x_mm, new_y - cmd.target_y_mm) <= 0.5
            )
            if pos_done:
                arrived_ids.append(pid)

            # ── Commit to board state and emit ───────────────────────────
            self._board.update_piece_position(pid, new_x, new_y, new_theta)
            self.position_updated.emit(pid, new_x, new_y, new_theta, 0.0)

        # Clean up finished moves
        for pid in arrived_ids:
            self._active.pop(pid, None)
            self._phase.pop(pid, None)
            self._rotate_to.pop(pid, None)
            self.move_complete.emit(pid)
            self.log_message.emit(f"SIM: 0x{pid:02X} arrived at target")

        if not self._active:
            self._timer.stop()
