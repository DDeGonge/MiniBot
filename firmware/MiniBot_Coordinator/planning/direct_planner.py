"""
planning/direct_planner.py  —  MiniBot Chess Swarm Coordinator

DirectPlanner: trivially sends every piece straight to its target in one wave
per piece, with no collision avoidance.

Intended for single-piece debugging only.  Do not use for multi-piece moves.
"""

from __future__ import annotations

from typing import Callable, Dict, List, Optional, Tuple

from planning.base_planner import BasePlanner, MoveCommand


class DirectPlanner(BasePlanner):
    """Send every piece directly to its goal — no collision avoidance."""

    @property
    def name(self) -> str:
        return "Direct (debug only)"

    def plan_moves(
        self,
        piece_positions: Dict[int, Tuple[float, float]],
        targets: Dict[int, Tuple[float, float]],
        orientations: Optional[Dict[int, float]] = None,
        validator: Optional[Callable[[int, float, float], bool]] = None,
    ) -> List[MoveCommand]:
        del orientations
        commands: List[MoveCommand] = []
        for seq, (pid, (tx, ty)) in enumerate(targets.items()):
            if pid not in piece_positions:
                continue
            sx, sy = piece_positions[pid]
            import math

            dist = math.hypot(tx - sx, ty - sy)
            commands.append(
                MoveCommand(
                    piece_id=pid,
                    target_x_mm=tx,
                    target_y_mm=ty,
                    duration_ms=self.duration_for_distance(dist),
                    sequence_num=seq,
                    planner_debug="direct",
                )
            )
        return commands
