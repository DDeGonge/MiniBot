"""Self test for continuous trajectory playback in the simulator.

Run from the coordinator root:

    python planning/swarm/_selftest_playback.py

Scatters all 34 pieces, plans them home with the SwarmPlanner, reconstructs the
per-piece time-synced paths exactly as MainWindow does, plays them through the
patched MotionSimulator, and checks the playback completes, covers every home,
stays collision-free through the continuous interpolation, and moves smoothly.
"""
import math
import os
import random
import sys

from PyQt6.QtWidgets import QApplication

sys.path.insert(0, os.getcwd())

from config import PIECES                                 # noqa: E402
from models.piece import BoardState                       # noqa: E402
from simulation.simulator import MotionSimulator          # noqa: E402
from planning.swarm_planner import SwarmPlanner           # noqa: E402

_SPEED_MM_S = 100.0
_PIECE_DIAMETER_MM = 2.0 * PIECES.CIRCLE_RADIUS_MM
_CONTINUITY_BOUND_MM = 8.0   # > step (speed * tick) with margin


def _reconstruct(board, commands):
    """Mirror MainWindow._play_trajectory_sim path/final-heading reconstruction."""
    waves = {}
    for c in commands:
        waves.setdefault(c.sequence_num, {})[c.piece_id] = c
    wave_nums = sorted(waves)
    paths, final_theta = {}, {}
    for pid in {c.piece_id for c in commands}:
        piece = board.get_piece(pid)
        prev = (piece.x_mm, piece.y_mm)
        seq = [prev]
        for wnum in wave_nums:
            cmd = waves[wnum].get(pid)
            if cmd is not None:
                prev = (cmd.target_x_mm, cmd.target_y_mm)
                if cmd.target_theta is not None:
                    final_theta[pid] = cmd.target_theta
            seq.append(prev)
        paths[pid] = seq
    return paths, final_theta


def main() -> None:
    QApplication([])
    board = BoardState()
    sim = MotionSimulator(board)
    sim.speed_mm_s = _SPEED_MM_S

    ids = list(PIECES.HOME_POSITIONS)
    homes = {p: (x, y) for p, (x, y, _t) in PIECES.HOME_POSITIONS.items()}
    random.seed(1)
    placed = {}
    for pid in ids:
        for _ in range(10000):
            x = random.uniform(30, 427)
            y = random.uniform(30, 427)
            if all(math.hypot(x - a, y - b) >= 40 for a, b in placed.values()):
                placed[pid] = (x, y)
                break
        board.update_piece_position(pid, placed[pid][0], placed[pid][1], 0.0)

    commands = SwarmPlanner().plan_moves(placed, homes)
    paths, final_theta = _reconstruct(board, commands)
    sim.play_trajectory(paths, final_theta)

    prev_pos = {pid: (board.get_piece(pid).x_mm, board.get_piece(pid).y_mm) for pid in ids}
    min_gap = float("inf")
    max_jump = 0.0
    ticks = 0
    while sim._trajectory is not None and ticks < 100000:
        sim._tick()
        ticks += 1
        cur = {pid: (board.get_piece(pid).x_mm, board.get_piece(pid).y_mm) for pid in ids}
        for pid in ids:
            max_jump = max(max_jump, math.hypot(cur[pid][0] - prev_pos[pid][0],
                                                cur[pid][1] - prev_pos[pid][1]))
        prev_pos = cur
        pts = list(cur.values())
        for i in range(len(pts)):
            for j in range(i + 1, len(pts)):
                min_gap = min(min_gap, math.hypot(pts[i][0] - pts[j][0],
                                                  pts[i][1] - pts[j][1]))

    completed = sim._trajectory is None
    finals = [(board.get_piece(pid).x_mm, board.get_piece(pid).y_mm) for pid in ids]
    covered = sum(1 for hx, hy in homes.values()
                  if any(math.hypot(fx - hx, fy - hy) < 5.0 for fx, fy in finals))
    thetas_ok = all(
        abs((final_theta.get(pid, board.get_piece(pid).orientation_deg)
             - board.get_piece(pid).orientation_deg + 180.0) % 360.0 - 180.0) <= 1.0
        for pid in ids)

    assert completed, "playback did not complete"
    assert covered == 34, f"only {covered}/34 homes covered"
    assert min_gap >= _PIECE_DIAMETER_MM - 1.0, f"min gap {min_gap:.1f} < {_PIECE_DIAMETER_MM}"
    assert max_jump <= _CONTINUITY_BOUND_MM, f"jump {max_jump:.1f} > {_CONTINUITY_BOUND_MM}"
    assert thetas_ok, "some piece did not reach its final heading"
    print(f"PASS: completed in {ticks} ticks; homes covered {covered}/34; "
          f"min pairwise gap {min_gap:.1f} mm (>= {_PIECE_DIAMETER_MM:.0f}); "
          f"max per-tick jump {max_jump:.1f} mm (<= {_CONTINUITY_BOUND_MM}); "
          f"final headings reached.")


if __name__ == "__main__":
    main()
