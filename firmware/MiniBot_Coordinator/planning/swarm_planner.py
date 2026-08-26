"""
planning/swarm_planner.py  —  MiniBot Chess Swarm Coordinator

SwarmPlanner: a complete, collision-free multi-robot planner. It plans the
choreography with PIBT (priority inheritance with backtracking) refined by
MAPF-LNS for lower total travel, then forward-simulates the plan through a
Buffered-Voronoi-Cell executor and emits each piece's actual collision-free
trajectory as wave-ordered MoveCommands. Because the robots follow waypoints
open-loop, replaying the executor's trajectory (rather than raw cell steps) is
what makes the on-board motion collision-free.

The vendored pipeline lives in the self-contained ``planning.swarm`` subpackage.
See ``planning/swarm/README.md`` for the recommended 57.15 mm board size.
"""
from __future__ import annotations

import math
from typing import Callable, Dict, List, Optional, Tuple

from config import BOARD, PIECES
from planning.base_planner import BasePlanner, MoveCommand
from planning.swarm.assignment import min_cost_assignment
from planning.swarm.drive import Bot, HybridDrive, run_and_sample
from planning.swarm.executor import DT_S
from planning.swarm.grid import CellGrid
from planning.swarm.lns import lns_plan
from planning.swarm.pibt import pibt_plan

# Recommended square size: 2.25 in tournament square. Wider than their 50 mm gives
# the collision-free (buffered Voronoi) motion the clearance it needs; see README.
RECOMMENDED_SQUARE_MM = 57.15

# One staging column each side of the 8x8 board, enough for the off-board extra
# queens (0x11, 0x22). A second column would place cell centers close enough to
# the table edge that a piece's body clips the boundary, and the coordinator's
# simulator (and real hardware) would clamp it there and never reach the target.
_BORDER_COLS = 1

# Heading each color faces once parked (their convention: white up, black down).
_WHITE_HEADING_DEG = 90.0
_BLACK_HEADING_DEG = 270.0

_DEFAULT_SAMPLE_MS = 200.0
_STEP_CAP = 20000

# A piece center must stay at least its radius (plus a hair) inside the physical
# table edges, or the coordinator/hardware clamps it and it never reaches the
# waypoint. The executor can transiently push a piece into the border while
# avoiding a crowd, so clamp every emitted waypoint back into the reachable box.
_EDGE_MARGIN_MM = 0.5


class SwarmPlanner(BasePlanner):
    """PIBT + LNS choreography executed through BVC, exported as MoveCommand waves."""

    def __init__(self, use_lns: bool = True, sample_ms: float = _DEFAULT_SAMPLE_MS,
                 interchangeable: bool = True):
        # Grid pitch matches the board actually in use, so the planner is a correct
        # drop-in on their current 50 mm board; RECOMMENDED_SQUARE_MM documents the
        # size we advise moving to.
        self._square_mm = float(BOARD.SQUARE_SIZE_MM)
        self._planner = lns_plan if use_lns else pibt_plan
        self._sample_ms = sample_ms
        # Public so callers can disable it for a single manual move (where the
        # specific clicked piece must go to the target, not a swapped same-type one).
        self.interchangeable = interchangeable
        r = float(PIECES.CIRCLE_RADIUS_MM) + _EDGE_MARGIN_MM
        self._x_min = -float(BOARD.BORDER_LEFT_MM) + r
        self._x_max = float(BOARD.PLAYING_AREA_MM) + float(BOARD.BORDER_RIGHT_MM) - r
        self._y_min = -float(BOARD.BORDER_BOTTOM_MM) + r
        self._y_max = float(BOARD.PLAYING_AREA_MM) + float(BOARD.BORDER_TOP_MM) - r
        self._grid = self._build_grid()

    def _clamp(self, x: float, y: float):
        """Keep a waypoint inside the reachable table box (piece body within walls)."""
        return (min(self._x_max, max(self._x_min, x)),
                min(self._y_max, max(self._y_min, y)))

    def _reassign_interchangeable(self, ids, piece_positions, goal_xy):
        """Swap targets within same-color, same-rank groups to minimize travel.

        Same-type pieces are interchangeable, so any pawn may take any pawn square.
        Optimally rematching each group beats the fixed one-square-per-id home map.
        """
        groups = {}
        for pid in ids:
            color = "w" if pid in PIECES.WHITE_IDS else "b"
            groups.setdefault((color, PIECES.PIECE_RANKS.get(pid, "?")), []).append(pid)
        result = dict(goal_xy)
        for members in groups.values():
            if len(members) < 2:
                continue
            tgts = [goal_xy[p] for p in members]
            cost = [[math.hypot(piece_positions[p][0] - tx, piece_positions[p][1] - ty)
                     for tx, ty in tgts] for p in members]
            assignment = min_cost_assignment(cost)
            for i, p in enumerate(members):
                result[p] = tgts[assignment[i]]
        return result

    @property
    def name(self) -> str:
        return "Swarm (PIBT+LNS)"

    @property
    def produces_trajectory(self) -> bool:
        """This planner emits a fine, time-synchronized, collision-free path."""
        return True

    def _build_grid(self) -> CellGrid:
        """Grid over the 8x8 play squares plus staging columns each side."""
        s = self._square_mm
        specs = [
            (col, row, (col + 0.5) * s, (row + 0.5) * s)
            for col in range(-_BORDER_COLS, BOARD.NUM_SQUARES + _BORDER_COLS)
            for row in range(BOARD.NUM_SQUARES)
        ]
        return CellGrid(specs, s)

    def _heading_rad(self, piece_id: int) -> float:
        """Target heading in radians for a piece, by color."""
        deg = _WHITE_HEADING_DEG if piece_id in PIECES.WHITE_IDS else _BLACK_HEADING_DEG
        return math.radians(deg)

    def plan_moves(
        self,
        piece_positions: Dict[int, Tuple[float, float]],
        targets: Dict[int, Tuple[float, float]],
        orientations: Optional[Dict[int, float]] = None,
        validator: Optional[Callable[[int, float, float], bool]] = None,
    ) -> List[MoveCommand]:
        """Plan collision-free waves for every piece that has both a start and target.

        Runs PIBT+LNS to a cell choreography, forward-simulates it through the BVC
        executor, and returns the sampled trajectory as wave-ordered MoveCommands.
        The chess-rules validator, if given, drops any piece whose target it rejects.
        """
        ids = [pid for pid in targets if pid in piece_positions]
        if validator is not None:
            ids = [pid for pid in ids
                   if validator(pid, targets[pid][0], targets[pid][1])]
        if not ids:
            return []

        goal_xy = {pid: (float(targets[pid][0]), float(targets[pid][1])) for pid in ids}
        if self.interchangeable:
            goal_xy = self._reassign_interchangeable(ids, piece_positions, goal_xy)
        orient = {pid: self._heading_rad(pid) for pid in ids}
        start_theta = orientations or {}
        bots = [
            Bot(pid, float(piece_positions[pid][0]), float(piece_positions[pid][1]),
                math.radians(start_theta.get(pid, 0.0)) if pid in start_theta
                else orient[pid])
            for pid in ids
        ]

        drive = HybridDrive(self._grid, goal_xy, ids, orient, self._planner)
        drive.reset(bots)
        sample_every = max(1, round(self._sample_ms / (DT_S * 1000.0)))
        waves = run_and_sample(drive, bots, sample_every, _STEP_CAP)

        duration_ms = int(round(sample_every * DT_S * 1000.0))
        commands = self._waves_to_commands(waves, duration_ms)
        commands.extend(self._final_wave(ids, goal_xy, orient,
                                         len(waves) + 1, duration_ms))
        return commands

    def _waves_to_commands(self, waves: list, duration_ms: int) -> List[MoveCommand]:
        """Turn sampled trajectory waves into wave-ordered MoveCommands."""
        commands: List[MoveCommand] = []
        for wave, snap in enumerate(waves, start=1):
            for pid, (x, y, _theta) in snap.items():
                # Intermediate waypoints leave heading free (the piece faces its
                # direction of travel); only the final wave commands the target
                # heading, so pieces do not stop to rotate at every waypoint.
                cx, cy = self._clamp(x, y)
                commands.append(MoveCommand(
                    piece_id=pid,
                    target_x_mm=cx,
                    target_y_mm=cy,
                    target_theta=None,
                    duration_ms=duration_ms,
                    sequence_num=wave,
                    planner_debug="swarm",
                ))
        return commands

    def _final_wave(self, ids, goal_xy, orient, sequence_num, duration_ms):
        """A closing wave placing every piece exactly on its target, facing forward.

        Guarantees each piece's last waypoint is its exact target (not just the
        planning cell center) and that non-moving pieces are still commanded home.
        Targets are distinct board squares, so this wave is collision-free.
        """
        return [
            MoveCommand(
                piece_id=pid,
                target_x_mm=self._clamp(*goal_xy[pid])[0],
                target_y_mm=self._clamp(*goal_xy[pid])[1],
                target_theta=math.degrees(orient[pid]) % 360.0,
                duration_ms=duration_ms,
                sequence_num=sequence_num,
                planner_debug="swarm-final",
            )
            for pid in ids
        ]
