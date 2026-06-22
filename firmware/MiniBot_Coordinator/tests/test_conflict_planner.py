"""
tests/test_conflict_planner.py  —  MiniBot Chess Swarm Coordinator

Standalone test / diagnostic suite for ConflictPlanner (dev copy of
EnhancedConflictPlanner).  Run without fixing anything; just observe which
tests pass and what the StepSimulator reports.

Usage:
    python tests/test_conflict_planner.py
    python tests/test_conflict_planner.py back_rank_home   # run one test

Each test prints PASS / FAIL / ERROR.

A pure-Python StepSimulator reproduces the rotate-then-translate motion model
from simulation/simulator.py and checks for collisions and boundary violations.

NOTE ON COLLISION THRESHOLD
  The planner commits segments at _CLEARANCE=33 mm centre-to-centre, but the
  StepSimulator fires a collision at COLLISION_DIST = 2 × R = 31 mm.  This is
  intentional: it exposes cases where the planner's safety margin is too thin
  once real rotation sweeps are added.
"""

from __future__ import annotations

import math
import sys
import os
import random

# Ensure the project root is on the path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import PIECES, SIMULATOR, PLANNING
from planning.base_planner import MoveCommand
from planning.conflict_planner import ConflictPlanner, _optimise_targets_optimal

# ---------------------------------------------------------------------------
# Shared constants (mirrors config values)
# ---------------------------------------------------------------------------

_R = float(PIECES.CIRCLE_RADIUS_MM)  # 15.5 mm
_X_LO = float(SIMULATOR.X_MIN_MM) + _R  # -84.5
_X_HI = float(SIMULATOR.X_MAX_MM) - _R  #  484.5
_Y_LO = float(SIMULATOR.Y_MIN_MM) + _R  #  -9.5
_Y_HI = float(SIMULATOR.Y_MAX_MM) - _R  #  409.5
_ARRIVE_EPS = float(getattr(PLANNING, "CONFLICT_ARRIVAL_EPS_MM", 2.0))

# ---------------------------------------------------------------------------
# Pure-Python step simulator
# ---------------------------------------------------------------------------

from dataclasses import dataclass, field
from typing import Dict, List, Tuple


@dataclass
class SimResult:
    success: bool
    collision_events: List[str] = field(default_factory=list)
    boundary_events: List[str] = field(default_factory=list)
    final_positions: Dict[int, Tuple[float, float]] = field(default_factory=dict)


class StepSimulator:
    """Simulate planned move commands and check for collisions / boundary violations.

    Motion model:
      - Rotate in place to face target, then translate in a straight line.
      - All pieces in the same wave advance simultaneously each tick.
      - Collision threshold: centre-to-centre < 2 * CIRCLE_RADIUS_MM (31 mm).
      - Boundary: piece centre must stay within table limits.
    """

    SPEED_MM_S = 80.0
    ROT_DEG_S = math.degrees(2.0)  # ~114.6 °/s
    DT_MS = 50  # 20 Hz
    COLLISION_DIST = 2.0 * _R  # 31 mm  (intentionally no extra margin)
    WAVE_TIMEOUT_S = 60.0  # safety cap per wave

    def __init__(self, positions: Dict[int, Tuple[float, float]]) -> None:
        self._pos: Dict[int, Tuple[float, float]] = {
            pid: (float(x), float(y)) for pid, (x, y) in positions.items()
        }
        self._theta: Dict[int, float] = {pid: 0.0 for pid in positions}

    def run(self, commands: List[MoveCommand]) -> SimResult:
        result = SimResult(success=True)

        # Group commands by sequence_num
        waves: Dict[int, List[MoveCommand]] = {}
        for cmd in commands:
            waves.setdefault(cmd.sequence_num, []).append(cmd)

        for wave_idx in sorted(waves.keys()):
            wave_cmds = waves[wave_idx]
            ok = self._run_wave(wave_cmds, result)
            if not ok:
                result.success = False

        result.final_positions = dict(self._pos)
        return result

    def _run_wave(self, cmds: List[MoveCommand], result: SimResult) -> bool:
        """Simulate one wave until all pieces reach their targets or timeout."""
        dt = self.DT_MS / 1000.0
        step_mm = self.SPEED_MM_S * dt
        step_deg = self.ROT_DEG_S * dt
        timeout_ticks = int(self.WAVE_TIMEOUT_S / dt)

        # per-piece state
        phase: Dict[int, str] = {cmd.piece_id: "rotate" for cmd in cmds}
        rotate_to: Dict[int, float] = {}
        targets_mm: Dict[int, Tuple[float, float]] = {
            cmd.piece_id: (cmd.target_x_mm, cmd.target_y_mm) for cmd in cmds
        }
        active_pids = set(phase.keys())
        success = True

        for _tick in range(timeout_ticks):
            if not active_pids:
                break

            new_pos = {}
            new_theta = {}

            for pid in list(active_pids):
                cur_x, cur_y = self._pos[pid]
                tx, ty = targets_mm[pid]
                dx, dy = tx - cur_x, ty - cur_y
                dist = math.hypot(dx, dy)
                cur_th = self._theta[pid]
                ph = phase[pid]

                if ph == "rotate":
                    if dist > 1.0:
                        if pid not in rotate_to:
                            fwd = math.degrees(math.atan2(dy, dx)) % 360.0
                            bwd = (fwd + 180.0) % 360.0
                            df = abs((fwd - cur_th + 180.0) % 360.0 - 180.0)
                            db = abs((bwd - cur_th + 180.0) % 360.0 - 180.0)
                            rotate_to[pid] = fwd if df <= db else bwd
                        rt = rotate_to[pid]
                        hdiff = (rt - cur_th + 180.0) % 360.0 - 180.0
                        if abs(hdiff) <= step_deg:
                            new_theta[pid] = rt
                            phase[pid] = "translate"
                        else:
                            new_theta[pid] = (
                                cur_th + math.copysign(step_deg, hdiff)
                            ) % 360.0
                        new_pos[pid] = (cur_x, cur_y)  # hold while rotating
                    else:
                        phase[pid] = "translate"
                        new_pos[pid] = (cur_x, cur_y)
                        new_theta[pid] = cur_th

                if phase[pid] == "translate":
                    new_theta[pid] = rotate_to.get(pid, cur_th)
                    dx2, dy2 = tx - cur_x, ty - cur_y
                    dist2 = math.hypot(dx2, dy2)
                    if dist2 <= step_mm:
                        new_pos[pid] = (tx, ty)
                        active_pids.discard(pid)
                    else:
                        new_pos[pid] = (
                            cur_x + (dx2 / dist2) * step_mm,
                            cur_y + (dy2 / dist2) * step_mm,
                        )

            # Apply updates
            for pid, pos in new_pos.items():
                self._pos[pid] = pos
            for pid, th in new_theta.items():
                self._theta[pid] = th

            # Boundary check (all tracked pieces)
            for pid, (px, py) in self._pos.items():
                if not (_X_LO <= px <= _X_HI and _Y_LO <= py <= _Y_HI):
                    msg = f"boundary: 0x{pid:02X} at ({px:.1f},{py:.1f})"
                    if msg not in result.boundary_events:
                        result.boundary_events.append(msg)

            # Collision check (all pairs)
            all_pids = list(self._pos.keys())
            for i in range(len(all_pids)):
                for j in range(i + 1, len(all_pids)):
                    pa, pb = all_pids[i], all_pids[j]
                    if _dist2(self._pos[pa], self._pos[pb]) < self.COLLISION_DIST:
                        msg = f"collision: 0x{pa:02X} <-> 0x{pb:02X}"
                        if msg not in result.collision_events:
                            result.collision_events.append(msg)

        if active_pids:
            result.collision_events.append(
                f'wave timeout: pieces {[f"0x{p:02X}" for p in sorted(active_pids)]} did not arrive'
            )
            success = False

        return success


def _dist2(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


# ---------------------------------------------------------------------------
# Helper: check all pieces are at their goals
# ---------------------------------------------------------------------------


def all_at_goals(
    final: Dict[int, Tuple[float, float]],
    goals: Dict[int, Tuple[float, float]],
    eps: float = 4.0,
) -> bool:
    for pid, (gx, gy) in goals.items():
        if pid not in final:
            return False
        if _dist2(final[pid], (gx, gy)) > eps:
            return False
    return True


# ---------------------------------------------------------------------------
# Test helpers
# ---------------------------------------------------------------------------


def _make_planner() -> ConflictPlanner:
    return ConflictPlanner()


def _run_and_check(
    positions: Dict[int, Tuple[float, float]],
    targets: Dict[int, Tuple[float, float]],
    check_no_collision: bool = True,
    check_goals: bool = True,
    label: str = "",
) -> Tuple[SimResult, List[MoveCommand]]:
    planner = _make_planner()
    commands = planner.plan_moves(positions, targets)

    # Determine the effective goals after brute-force target-assignment optimisation.
    # ConflictPlanner internally uses a greedy NN assignment; we verify against the
    # optimal assignment so mismatches are surfaced as diagnostic information.
    effective_targets = _optimise_targets_optimal(
        {pid: (float(x), float(y)) for pid, (x, y) in positions.items()},
        {pid: (float(x), float(y)) for pid, (x, y) in targets.items()},
    )

    # If all effective targets equal current positions, no movement needed
    trivially_done = all(
        _dist2(positions[pid], effective_targets[pid]) <= _ARRIVE_EPS
        for pid in effective_targets
        if pid in positions
    )
    if trivially_done:
        assert not commands or all(
            _dist2(
                positions.get(cmd.piece_id, (0, 0)), (cmd.target_x_mm, cmd.target_y_mm)
            )
            <= _ARRIVE_EPS
            for cmd in commands
        ), f"{label}: trivially-done but commands contain non-trivial moves"
        return SimResult(success=True, final_positions=dict(positions)), commands

    assert commands, f"{label}: planner returned no commands"

    sim = StepSimulator(positions)
    result = sim.run(commands)

    if check_no_collision:
        assert (
            not result.collision_events
        ), f"{label}: collision(s): {result.collision_events[:3]}"
    if check_goals:
        assert all_at_goals(
            result.final_positions, effective_targets
        ), f"{label}: pieces did not reach goals. " + ", ".join(
            f'0x{pid:02X}: got ({result.final_positions.get(pid, ("?","?"))[0]:.0f},'
            f'{result.final_positions.get(pid, ("?","?"))[1]:.0f}) '
            f"want ({gx:.0f},{gy:.0f})"
            for pid, (gx, gy) in effective_targets.items()
            if pid not in result.final_positions
            or _dist2(result.final_positions[pid], (gx, gy)) > 4.0
        )
    return result, commands


# ---------------------------------------------------------------------------
# Test 1: single piece in open space
# ---------------------------------------------------------------------------


def test_single_piece_open() -> None:
    pos = {0x01: (25.0, 25.0)}
    tgt = {0x01: (375.0, 375.0)}
    _run_and_check(pos, tgt, label="single_piece_open")


# ---------------------------------------------------------------------------
# Test 2: two pieces with crossing paths
# ---------------------------------------------------------------------------


def test_two_pieces_crossing() -> None:
    # Use non-pawn IDs (rook + bishop) so target swap doesn't apply
    pos = {0x09: (25.0, 200.0), 0x0B: (200.0, 25.0)}
    tgt = {0x09: (375.0, 200.0), 0x0B: (200.0, 375.0)}
    _run_and_check(pos, tgt, label="two_pieces_crossing")


# ---------------------------------------------------------------------------
# Test 3: two-piece swap (classic cyclic deadlock)
# Use unique pieces (king + queen) so target swap doesn't trivialize it
# ---------------------------------------------------------------------------


def test_two_piece_swap() -> None:
    pos = {0x0D: (50.0, 50.0), 0x0C: (350.0, 350.0)}
    tgt = {0x0D: (350.0, 350.0), 0x0C: (50.0, 50.0)}
    _run_and_check(pos, tgt, label="two_piece_swap")


# ---------------------------------------------------------------------------
# Test 4: four-piece cycle (A->B->C->D->A positions)
# Use unique pieces (kings/queens across colors) to prevent trivial optimisation
# ---------------------------------------------------------------------------


def test_four_piece_cycle() -> None:
    pos = {
        0x0D: (50.0, 50.0),
        0x0C: (350.0, 50.0),
        0x1E: (350.0, 350.0),
        0x1D: (50.0, 350.0),
    }
    tgt = {
        0x0D: (350.0, 50.0),
        0x0C: (350.0, 350.0),
        0x1E: (50.0, 350.0),
        0x1D: (50.0, 50.0),
    }
    _run_and_check(pos, tgt, label="four_piece_cycle")


# ---------------------------------------------------------------------------
# Test 5: eight pawns advance one rank (should be one parallel wave)
# ---------------------------------------------------------------------------


def test_eight_pawns_advance() -> None:
    S = 50.0
    pos = {(0x01 + i): ((i + 0.5) * S, 75.0) for i in range(8)}
    tgt = {pid: (x, 175.0) for pid, (x, _) in pos.items()}
    result, commands = _run_and_check(pos, tgt, label="eight_pawns_advance")

    wave0_pids = {cmd.piece_id for cmd in commands if cmd.sequence_num == 0}
    assert (
        len(wave0_pids) >= 4
    ), f"eight_pawns_advance: expected >=4 pieces in wave 0, got {len(wave0_pids)}: {wave0_pids}"


# ---------------------------------------------------------------------------
# Test 6: white back rank + pawns return home from random positions
# ---------------------------------------------------------------------------


def test_back_rank_home() -> None:
    random.seed(42)
    home = PIECES.HOME_POSITIONS
    pids = list(range(0x01, 0x11))  # 0x01-0x10 (white pieces)
    r = _R
    pos: Dict[int, Tuple[float, float]] = {}
    placed: List[Tuple[float, float]] = []
    for pid in pids:
        for _ in range(10_000):
            x = random.uniform(-84.0, 484.0)
            y = random.uniform(-9.0, 409.0)
            if all(_dist2((x, y), p) >= 2 * r + 5 for p in placed):
                pos[pid] = (x, y)
                placed.append((x, y))
                break
        else:
            hx, hy, _ = home[pid]
            pos[pid] = (float(hx) + 1.0, float(hy) + 1.0)
            placed.append(pos[pid])

    tgt = {pid: (float(home[pid][0]), float(home[pid][1])) for pid in pids}
    _run_and_check(pos, tgt, label="back_rank_home")


# ---------------------------------------------------------------------------
# Test 7: all 32 on-board pieces scrambled back to home
# ---------------------------------------------------------------------------


def test_full_board_home() -> None:
    random.seed(123)
    home = PIECES.HOME_POSITIONS
    pids = [pid for pid in home if home[pid][0] >= 0]
    r = _R
    pos: Dict[int, Tuple[float, float]] = {}
    placed: List[Tuple[float, float]] = []
    for pid in pids:
        for _ in range(50_000):
            x = random.uniform(-84.0, 484.0)
            y = random.uniform(-9.0, 409.0)
            if all(_dist2((x, y), p) >= 2 * r + 5 for p in placed):
                pos[pid] = (x, y)
                placed.append((x, y))
                break
        else:
            hx, hy, _ = home[pid]
            pos[pid] = (float(hx), float(hy))
            placed.append(pos[pid])

    tgt = {
        pid: (float(home[pid][0]), float(home[pid][1])) for pid in pids if pid in pos
    }
    _run_and_check(pos, tgt, label="full_board_home")


# ---------------------------------------------------------------------------
# Test 8: staging parking — 6 pieces tightly packed, 2 endpoints must cross
# ---------------------------------------------------------------------------


def test_staging_parking() -> None:
    spacing = 34.0  # just above clearance (33 mm)
    pids = [0x0D, 0x0C, 0x09, 0x10, 0x1E, 0x1D]
    pos: Dict[int, Tuple[float, float]] = {}
    for i, pid in enumerate(pids):
        pos[pid] = (80.0 + i * spacing, 200.0)

    tgt = {
        pids[0]: (80.0 + 5 * spacing, 200.0),
        pids[5]: (80.0, 200.0),
    }
    _run_and_check(pos, tgt, label="staging_parking")


# ---------------------------------------------------------------------------
# Test 9: minimum segment length check (full board home)
# ---------------------------------------------------------------------------


def test_min_segment_check() -> None:
    random.seed(77)
    home = PIECES.HOME_POSITIONS
    pids = [pid for pid in home if home[pid][0] >= 0]
    r = _R
    pos: Dict[int, Tuple[float, float]] = {}
    placed: List[Tuple[float, float]] = []
    for pid in pids:
        for _ in range(50_000):
            x = random.uniform(-84.0, 484.0)
            y = random.uniform(-9.0, 409.0)
            if all(_dist2((x, y), p) >= 2 * r + 5 for p in placed):
                pos[pid] = (x, y)
                placed.append((x, y))
                break
        else:
            hx, hy, _ = home[pid]
            pos[pid] = (float(hx), float(hy))
            placed.append(pos[pid])

    tgt = {
        pid: (float(home[pid][0]), float(home[pid][1])) for pid in pids if pid in pos
    }

    planner = _make_planner()
    commands = planner.plan_moves(pos, tgt)

    MIN_SEG = float(getattr(PLANNING, "CONFLICT_MIN_SEGMENT_MM", 20.0))
    cur: Dict[int, Tuple[float, float]] = dict(pos)
    waves: Dict[int, List[MoveCommand]] = {}
    for cmd in commands:
        waves.setdefault(cmd.sequence_num, []).append(cmd)

    violations: List[str] = []
    for wave_idx in sorted(waves.keys()):
        for cmd in waves[wave_idx]:
            pid = cmd.piece_id
            if pid in cur:
                d = _dist2(cur[pid], (cmd.target_x_mm, cmd.target_y_mm))
                if d < MIN_SEG - 0.5:
                    dock_eps = float(getattr(PLANNING, "CONFLICT_DOCK_EPS_MM", 8.0))
                    if d < dock_eps:
                        violations.append(
                            f"0x{pid:02X} seg={d:.1f}mm at seq={wave_idx} (too short)"
                        )
            cur[pid] = (cmd.target_x_mm, cmd.target_y_mm)

    assert not violations, f"min_segment_check: {violations[:5]}"


# ---------------------------------------------------------------------------
# Test 10: parallel efficiency for eight-pawn advance
# ---------------------------------------------------------------------------


def test_parallel_efficiency() -> None:
    S = 50.0
    pos = {(0x01 + i): ((i + 0.5) * S, 75.0) for i in range(8)}
    tgt = {pid: (x, 175.0) for pid, (x, _) in pos.items()}

    planner = _make_planner()
    commands = planner.plan_moves(pos, tgt)

    wave0_pids = {cmd.piece_id for cmd in commands if cmd.sequence_num == 0}
    assert (
        len(wave0_pids) >= 4
    ), f"parallel_efficiency: expected >=4 pieces in wave 0, got {len(wave0_pids)}"


# ---------------------------------------------------------------------------
# Test 11: target swap — two pawns with swapped goals
# ---------------------------------------------------------------------------


def test_target_swap_optimal() -> None:
    pos = {0x01: (25.0, 75.0), 0x02: (375.0, 75.0)}
    naive_goals = {0x01: (375.0, 75.0), 0x02: (25.0, 75.0)}

    optimised = _optimise_targets_optimal(pos, naive_goals)

    naive_total = sum(_dist2(pos[pid], naive_goals[pid]) for pid in pos)
    opt_total = sum(_dist2(pos[pid], optimised[pid]) for pid in pos)
    assert opt_total < naive_total - 100, (
        f"target_swap_optimal: optimised dist {opt_total:.0f} not much less than "
        f"naive {naive_total:.0f}"
    )


# ---------------------------------------------------------------------------
# Test 12: target swap no worse than naive
# ---------------------------------------------------------------------------


def test_target_swap_no_worse() -> None:
    random.seed(55)
    S = 50.0
    pids = list(range(0x01, 0x09))
    placed: List[Tuple[float, float]] = []
    pos: Dict[int, Tuple[float, float]] = {}
    for pid in pids:
        for _ in range(10_000):
            x = random.uniform(0.0, 400.0)
            y = random.uniform(0.0, 400.0)
            if all(_dist2((x, y), p) >= 2 * _R + 5 for p in placed):
                pos[pid] = (x, y)
                placed.append((x, y))
                break
        else:
            pos[pid] = (float(pids.index(pid) + 1) * S - S / 2, 75.0)
            placed.append(pos[pid])

    home = PIECES.HOME_POSITIONS
    naive_goals = {pid: (float(home[pid][0]), float(home[pid][1])) for pid in pids}

    naive_total = sum(_dist2(pos[pid], naive_goals[pid]) for pid in pids)
    optimised = _optimise_targets_optimal(pos, naive_goals)
    opt_total = sum(_dist2(pos[pid], optimised[pid]) for pid in pids)

    assert (
        opt_total <= naive_total + 1.0
    ), f"target_swap_no_worse: optimised {opt_total:.0f} > naive {naive_total:.0f}"


# ---------------------------------------------------------------------------
# Test runner
# ---------------------------------------------------------------------------

TESTS = [
    ("single_piece_open", test_single_piece_open),
    ("two_pieces_crossing", test_two_pieces_crossing),
    ("two_piece_swap", test_two_piece_swap),
    ("four_piece_cycle", test_four_piece_cycle),
    ("eight_pawns_advance", test_eight_pawns_advance),
    ("back_rank_home", test_back_rank_home),
    ("full_board_home", test_full_board_home),
    ("staging_parking", test_staging_parking),
    ("min_segment_check", test_min_segment_check),
    ("parallel_efficiency", test_parallel_efficiency),
    ("target_swap_optimal", test_target_swap_optimal),
    ("target_swap_no_worse", test_target_swap_no_worse),
]


def run_test(name: str, fn) -> bool:
    try:
        fn()
        print(f"PASS  {name}")
        return True
    except AssertionError as e:
        print(f"FAIL  {name}: {e}")
        return False
    except Exception as e:
        import traceback

        print(f"ERROR {name}: {e}")
        traceback.print_exc()
        return False


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="ConflictPlanner diagnostic test suite"
    )
    parser.add_argument(
        "filter",
        nargs="?",
        default="",
        help="Run only tests whose name contains this substring",
    )
    args = parser.parse_args()

    passed = 0
    failed = 0
    for name, fn in TESTS:
        if args.filter and args.filter.lower() not in name.lower():
            continue
        ok = run_test(name, fn)
        if ok:
            passed += 1
        else:
            failed += 1

    total = passed + failed
    print(f"\n{passed}/{total} passed", ("OK" if failed == 0 else "FAIL"))
    sys.exit(0 if failed == 0 else 1)
