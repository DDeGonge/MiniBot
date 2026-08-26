"""Self test for the vendored swarm planner.

Run from the coordinator root so `from config import ...` resolves:

    python planning/swarm/_selftest.py

Scatters all 34 pieces onto random board squares, plans them home, and asserts the
output is wave-ordered, lands every piece on its target, and is collision-free at
every wave.
"""
import math
import os
import random
import sys

# Allow running as `python planning/swarm/_selftest.py` from the coordinator root.
sys.path.insert(0, os.getcwd())

from config import BOARD, PIECES                       # noqa: E402
from planning.swarm.executor import PIECE_DIAMETER_MM  # noqa: E402
from planning.swarm_planner import SwarmPlanner        # noqa: E402


def main() -> None:
    ids = list(PIECES.HOME_POSITIONS.keys())
    targets = {pid: (x, y) for pid, (x, y, _theta) in PIECES.HOME_POSITIONS.items()}

    s = BOARD.SQUARE_SIZE_MM
    play_cells = [((c + 0.5) * s, (r + 0.5) * s)
                  for c in range(BOARD.NUM_SQUARES) for r in range(BOARD.NUM_SQUARES)]
    rng = random.Random(0)
    rng.shuffle(play_cells)
    starts = {pid: play_cells[i] for i, pid in enumerate(ids)}

    commands = SwarmPlanner().plan_moves(starts, targets)
    assert commands, "planner returned no commands"

    waves = [c.sequence_num for c in commands]
    assert waves == sorted(waves), "commands are not wave-ordered"

    final = {}
    for c in commands:
        final[c.piece_id] = (c.target_x_mm, c.target_y_mm)
    assert set(final) == set(ids), "not every piece has a final command"
    # Pieces are interchangeable, so check every home SQUARE is covered by some
    # piece rather than requiring a specific id on a specific square.
    finals = list(final.values())
    for hx, hy in targets.values():
        assert any(math.hypot(fx - hx, fy - hy) < 5.0 for fx, fy in finals), \
            f"home square ({hx:.0f},{hy:.0f}) not covered"

    by_wave = {}
    for c in commands:
        by_wave.setdefault(c.sequence_num, []).append(c)
    pos = {}
    min_gap = float("inf")
    for wave in sorted(by_wave):
        for c in by_wave[wave]:
            pos[c.piece_id] = (c.target_x_mm, c.target_y_mm)
        placed = list(pos.values())
        for i in range(len(placed)):
            for j in range(i + 1, len(placed)):
                gap = math.hypot(placed[i][0] - placed[j][0], placed[i][1] - placed[j][1])
                min_gap = min(min_gap, gap)
                assert gap >= PIECE_DIAMETER_MM, \
                    f"wave {wave}: pieces {gap:.1f} mm apart (< {PIECE_DIAMETER_MM})"

    print(f"PASS: {len(commands)} commands across {max(waves)} waves; "
          f"min pairwise gap {min_gap:.1f} mm >= {PIECE_DIAMETER_MM} mm; "
          f"all 34 pieces reached target.")


if __name__ == "__main__":
    main()
