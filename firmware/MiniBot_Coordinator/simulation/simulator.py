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

# Trajectory playback: advance the global time cursor by at most this many time
# slices per tick, so no waypoint is skipped and the collision-free lockstep is
# preserved. Segments shorter than _MIN_SEGMENT_MM carry no meaningful heading.
_MAX_CURSOR_ADVANCE = 1.0
_MIN_SEGMENT_MM = 0.5
_FINAL_HEADING_TOL_DEG = 0.5


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

        # Time-synchronized trajectory playback (used for planners that emit a
        # continuous, collision-free-by-construction path). When active, this
        # takes over the tick and the per-command logic above is bypassed.
        self._trajectory: Optional[Dict[int, List[Tuple[float, float]]]] = None
        self._traj_len: int = 0
        self._traj_cursor: float = 0.0
        self._traj_final_theta: Dict[int, float] = {}
        self._traj_finalizing: bool = False

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

    def play_trajectory(
        self,
        paths: Dict[int, List[Tuple[float, float]]],
        final_theta: Dict[int, float],
    ) -> None:
        """Play a time-synchronized, collision-free trajectory continuously.

        paths[pid] is that piece's position at each of L identical-length time
        slices (a stopped piece repeats its last position); final_theta[pid] is
        the heading in degrees to face once playback ends. Every piece advances in
        lockstep along a single global cursor, so the collision-free spacing the
        planner guarantees at each slice is preserved between slices too. This
        replaces the discrete rotate-then-translate, wave-barrier motion so a
        continuous planner's arcs render smoothly.
        """
        if not paths:
            return
        self._active.clear()
        self._phase.clear()
        self._rotate_to.clear()
        self._trajectory = {pid: list(pts) for pid, pts in paths.items()}
        self._traj_len = max(len(pts) for pts in self._trajectory.values())
        self._traj_final_theta = dict(final_theta)
        self._traj_cursor = 0.0
        self._traj_finalizing = False
        if not self._timer.isActive():
            self._timer.start()

    def stop_all(self) -> None:
        """Cancel all active moves and stop the timer."""
        self._active.clear()
        self._phase.clear()
        self._rotate_to.clear()
        self._trajectory = None
        self._traj_final_theta = {}
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
        # Continuous trajectory playback takes over the tick when active; the
        # per-command rotate/translate logic below is left intact for the
        # coordinator's other (discrete) planners.
        if self._trajectory is not None:
            self._tick_trajectory()
            return

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
                    if cmd.target_theta is not None:
                        # Position reached; correct to the commanded final heading.
                        self._phase[pid] = "final_rotate"
                else:
                    new_x = cur_x + (dx / dist) * step
                    new_y = cur_y + (dy / dist) * step

            # ── Phase 3: Rotate in place to the commanded final heading ──
            elif phase == "final_rotate":
                new_x, new_y = cur_x, cur_y
                heading_diff = (
                    cmd.target_theta - piece.orientation_deg + 180.0
                ) % 360.0 - 180.0
                if abs(heading_diff) <= rot_step:
                    new_theta = cmd.target_theta % 360.0
                else:
                    new_theta = (
                        piece.orientation_deg + math.copysign(rot_step, heading_diff)
                    ) % 360.0

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
            # Arrived once the target position is reached, and (when a final
            # heading was commanded) the piece has also turned to face it.
            ph = self._phase.get(pid)
            pos_reached = (
                math.hypot(new_x - cmd.target_x_mm, new_y - cmd.target_y_mm) <= 0.5
            )
            if ph == "translate" and pos_reached and cmd.target_theta is None:
                arrived_ids.append(pid)
            elif ph == "final_rotate":
                heading_reached = (
                    abs((cmd.target_theta - new_theta + 180.0) % 360.0 - 180.0) <= 0.5
                )
                if pos_reached and heading_reached:
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

    def _tick_trajectory(self) -> None:
        """Advance one continuous-playback tick along the global time cursor.

        Every piece moves in lockstep: the cursor advances so the fastest piece
        travels at ~speed mm/s (capped so no time slice is skipped), and each
        piece is linear-interpolated along its own segment and faces its travel
        direction. Once the cursor reaches the last slice, pieces hold on their
        final square and turn in place to the commanded heading, then playback
        ends with a move_complete for every piece.
        """
        dt = SIMULATOR.UPDATE_INTERVAL_MS / 1000.0
        rot_step = SIMULATOR.ROTATION_SPEED_DEG_S * dt
        paths = self._trajectory
        last = self._traj_len - 1

        if not self._traj_finalizing:
            # A tick advances the cursor by at most one slice, so it can touch the
            # current segment and the next; size the step from the longer of the
            # two so no piece moves more than `step` this tick.
            i = min(int(self._traj_cursor), last)
            b = min(i + 1, last)
            d = min(i + 2, last)
            seg_max = 0.0
            for pts in paths.values():
                seg_max = max(
                    seg_max,
                    math.hypot(pts[b][0] - pts[i][0], pts[b][1] - pts[i][1]),
                    math.hypot(pts[d][0] - pts[b][0], pts[d][1] - pts[b][1]),
                )
            step = self._speed * dt
            advance = _MAX_CURSOR_ADVANCE if seg_max <= _MIN_SEGMENT_MM else step / seg_max
            self._traj_cursor += min(advance, _MAX_CURSOR_ADVANCE)
            if self._traj_cursor >= last:
                self._traj_cursor = float(last)
                self._traj_finalizing = True

            c = self._traj_cursor
            i = min(int(c), last)
            frac = c - i
            i2 = min(i + 1, last)
            for pid, pts in paths.items():
                ax, ay = pts[i]
                bx, by = pts[i2]
                nx = max(self._x_min, min(self._x_max, ax + (bx - ax) * frac))
                ny = max(self._y_min, min(self._y_max, ay + (by - ay) * frac))
                sdx, sdy = bx - ax, by - ay
                if math.hypot(sdx, sdy) > _MIN_SEGMENT_MM:
                    theta = math.degrees(math.atan2(sdy, sdx)) % 360.0
                else:
                    piece = self._board.get_piece(pid)
                    theta = piece.orientation_deg if piece is not None else 0.0
                self._board.update_piece_position(pid, nx, ny, theta)
                self.position_updated.emit(pid, nx, ny, theta, 0.0)
            return

        # Finalizing: hold on the last waypoint, turn in place to the final heading.
        all_aligned = True
        for pid, pts in paths.items():
            fx, fy = pts[last]
            piece = self._board.get_piece(pid)
            cur_theta = piece.orientation_deg if piece is not None else 0.0
            tgt = self._traj_final_theta.get(pid)
            if tgt is None:
                theta = cur_theta
            else:
                diff = (tgt - cur_theta + 180.0) % 360.0 - 180.0
                if abs(diff) <= max(rot_step, _FINAL_HEADING_TOL_DEG):
                    theta = tgt % 360.0
                else:
                    theta = (cur_theta + math.copysign(rot_step, diff)) % 360.0
                    all_aligned = False
            self._board.update_piece_position(pid, fx, fy, theta)
            self.position_updated.emit(pid, fx, fy, theta, 0.0)

        if all_aligned:
            ids = list(paths.keys())
            self._trajectory = None
            self._traj_final_theta = {}
            self._timer.stop()
            for pid in ids:
                self.move_complete.emit(pid)
            self.log_message.emit("SIM: trajectory playback complete")
