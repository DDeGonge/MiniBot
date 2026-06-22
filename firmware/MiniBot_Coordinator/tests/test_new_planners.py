"""
tests/test_new_planners.py  —  MiniBot Chess Swarm Coordinator

Diagnostic test suite for MakeWayPlanner and StagingPlanner.
Runs the same 12 scenarios as test_conflict_planner.py against both planners
so results can be compared directly.

Usage:
    python tests/test_new_planners.py                    # both planners
    python tests/test_new_planners.py makeway           # MakeWayPlanner only
    python tests/test_new_planners.py staging           # StagingPlanner only
    python tests/test_new_planners.py makeway back_rank # filter by test name
"""

from __future__ import annotations

import math
import sys
import os
import random

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import PIECES, SIMULATOR, PLANNING
from planning.base_planner import MoveCommand
from planning.enhanced_conflict_planner import EnhancedConflictPlanner
from planning.enhanced_conflict_planner import _optimise_targets_optimal

from dataclasses import dataclass, field
from typing import Dict, List, Tuple, Optional

# ---------------------------------------------------------------------------
# Shared constants
# ---------------------------------------------------------------------------

_R = float(PIECES.CIRCLE_RADIUS_MM)
_X_LO = float(SIMULATOR.X_MIN_MM) + _R
_X_HI = float(SIMULATOR.X_MAX_MM) - _R
_Y_LO = float(SIMULATOR.Y_MIN_MM) + _R
_Y_HI = float(SIMULATOR.Y_MAX_MM) - _R
_ARRIVE_EPS = float(getattr(PLANNING, "CONFLICT_ARRIVAL_EPS_MM", 2.0))


# ---------------------------------------------------------------------------
# Step simulator (identical to test_conflict_planner.py)
# ---------------------------------------------------------------------------


@dataclass
class SimResult:
    success: bool
    collision_events: List[str] = field(default_factory=list)
    boundary_events: List[str] = field(default_factory=list)
    final_positions: Dict[int, Tuple[float, float]] = field(default_factory=dict)


class StepSimulator:
    SPEED_MM_S = 80.0
    ROT_DEG_S = math.degrees(2.0)
    DT_MS = 50
    COLLISION_DIST = 2.0 * _R
    WAVE_TIMEOUT_S = 60.0

    def __init__(self, positions: Dict[int, Tuple[float, float]]) -> None:
        self._pos = {pid: (float(x), float(y)) for pid, (x, y) in positions.items()}
        self._theta = {pid: 0.0 for pid in positions}

    def run(self, commands: List[MoveCommand]) -> SimResult:
        result = SimResult(success=True)
        waves: Dict[int, List[MoveCommand]] = {}
        for cmd in commands:
            waves.setdefault(cmd.sequence_num, []).append(cmd)
        for wave_idx in sorted(waves.keys()):
            if not self._run_wave(waves[wave_idx], result):
                result.success = False
        result.final_positions = dict(self._pos)
        return result

    def _run_wave(self, cmds, result):
        dt = self.DT_MS / 1000.0
        step_mm = self.SPEED_MM_S * dt
        step_deg = self.ROT_DEG_S * dt
        timeout_ticks = int(self.WAVE_TIMEOUT_S / dt)
        phase = {cmd.piece_id: "rotate" for cmd in cmds}
        rotate_to: Dict[int, float] = {}
        targets = {cmd.piece_id: (cmd.target_x_mm, cmd.target_y_mm) for cmd in cmds}
        active = set(phase.keys())
        success = True

        for _ in range(timeout_ticks):
            if not active:
                break
            new_pos = {}
            new_theta = {}
            for pid in list(active):
                cx, cy = self._pos[pid]
                tx, ty = targets[pid]
                dx, dy = tx - cx, ty - cy
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
                        new_pos[pid] = (cx, cy)
                    else:
                        phase[pid] = "translate"
                        new_pos[pid] = (cx, cy)
                        new_theta[pid] = cur_th

                if phase[pid] == "translate":
                    new_theta[pid] = rotate_to.get(pid, cur_th)
                    dx2, dy2 = tx - cx, ty - cy
                    d2 = math.hypot(dx2, dy2)
                    if d2 <= step_mm:
                        new_pos[pid] = (tx, ty)
                        active.discard(pid)
                    else:
                        new_pos[pid] = (
                            cx + dx2 / d2 * step_mm,
                            cy + dy2 / d2 * step_mm,
                        )

            for pid, p in new_pos.items():
                self._pos[pid] = p
            for pid, t in new_theta.items():
                self._theta[pid] = t

            for pid, (px, py) in self._pos.items():
                if not (_X_LO <= px <= _X_HI and _Y_LO <= py <= _Y_HI):
                    msg = f"boundary: 0x{pid:02X} at ({px:.1f},{py:.1f})"
                    if msg not in result.boundary_events:
                        result.boundary_events.append(msg)

            all_pids = list(self._pos.keys())
            for i in range(len(all_pids)):
                for j in range(i + 1, len(all_pids)):
                    pa, pb = all_pids[i], all_pids[j]
                    if _d2(self._pos[pa], self._pos[pb]) < self.COLLISION_DIST:
                        msg = f"collision: 0x{pa:02X} <-> 0x{pb:02X}"
                        if msg not in result.collision_events:
                            result.collision_events.append(msg)

        if active:
            result.collision_events.append(
                f'wave timeout: {[f"0x{p:02X}" for p in sorted(active)]} did not arrive'
            )
            success = False
        return success


def _d2(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def all_at_goals(final, goals, eps=4.0):
    return all(pid in final and _d2(final[pid], g) <= eps for pid, g in goals.items())


# ---------------------------------------------------------------------------
# Runner helpers
# ---------------------------------------------------------------------------


def _run_and_check(
    planner, positions, targets, check_collision=True, check_goals=True, label=""
):
    commands = planner.plan_moves(positions, targets)

    effective = _optimise_targets_optimal(
        {pid: (float(x), float(y)) for pid, (x, y) in positions.items()},
        {pid: (float(x), float(y)) for pid, (x, y) in targets.items()},
    )
    trivially_done = all(
        _d2(positions[pid], effective[pid]) <= _ARRIVE_EPS
        for pid in effective
        if pid in positions
    )
    if trivially_done:
        return SimResult(success=True, final_positions=dict(positions)), commands

    assert commands, f"{label}: planner returned no commands"
    sim = StepSimulator(positions)
    result = sim.run(commands)

    if check_collision:
        assert (
            not result.collision_events
        ), f"{label}: collision(s): {result.collision_events[:3]}"
    if check_goals:
        assert all_at_goals(
            result.final_positions, effective
        ), f"{label}: pieces did not reach goals. " + ", ".join(
            f'0x{pid:02X}: got ({result.final_positions.get(pid,("?","?"))[0]:.0f},'
            f'{result.final_positions.get(pid,("?","?"))[1]:.0f}) '
            f"want ({gx:.0f},{gy:.0f})"
            for pid, (gx, gy) in effective.items()
            if pid not in result.final_positions
            or _d2(result.final_positions[pid], (gx, gy)) > 4.0
        )
    return result, commands


# ---------------------------------------------------------------------------
# Test definitions (same 12 scenarios as test_conflict_planner.py)
# ---------------------------------------------------------------------------


def _test_single_piece_open(p):
    _run_and_check(
        p, {0x01: (25.0, 25.0)}, {0x01: (375.0, 375.0)}, label="single_piece_open"
    )


def _test_two_pieces_crossing(p):
    _run_and_check(
        p,
        {0x09: (25.0, 200.0), 0x0B: (200.0, 25.0)},
        {0x09: (375.0, 200.0), 0x0B: (200.0, 375.0)},
        label="two_pieces_crossing",
    )


def _test_two_piece_swap(p):
    _run_and_check(
        p,
        {0x0D: (50.0, 50.0), 0x0C: (350.0, 350.0)},
        {0x0D: (350.0, 350.0), 0x0C: (50.0, 50.0)},
        label="two_piece_swap",
    )


def _test_four_piece_cycle(p):
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
    _run_and_check(p, pos, tgt, label="four_piece_cycle")


def _test_eight_pawns_advance(p):
    S = 50.0
    pos = {(0x01 + i): ((i + 0.5) * S, 75.0) for i in range(8)}
    tgt = {pid: (x, 175.0) for pid, (x, _) in pos.items()}
    _, commands = _run_and_check(p, pos, tgt, label="eight_pawns_advance")
    w0 = {c.piece_id for c in commands if c.sequence_num == 0}
    assert len(w0) >= 4, f"eight_pawns_advance: expected >=4 in wave 0, got {len(w0)}"


def _test_back_rank_home(p):
    random.seed(42)
    home = PIECES.HOME_POSITIONS
    pids = list(range(0x01, 0x11))
    r = _R
    pos = {}
    placed = []
    for pid in pids:
        for _ in range(10_000):
            x = random.uniform(-84.0, 484.0)
            y = random.uniform(-9.0, 409.0)
            if all(_d2((x, y), q) >= 2 * r + 5 for q in placed):
                pos[pid] = (x, y)
                placed.append((x, y))
                break
        else:
            hx, hy, _ = home[pid]
            pos[pid] = (float(hx) + 1.0, float(hy) + 1.0)
            placed.append(pos[pid])
    tgt = {pid: (float(home[pid][0]), float(home[pid][1])) for pid in pids}
    _run_and_check(p, pos, tgt, label="back_rank_home")


def _test_full_board_home(p):
    random.seed(123)
    home = PIECES.HOME_POSITIONS
    pids = [pid for pid in home if home[pid][0] >= 0]
    r = _R
    pos = {}
    placed = []
    for pid in pids:
        for _ in range(50_000):
            x = random.uniform(-84.0, 484.0)
            y = random.uniform(-9.0, 409.0)
            if all(_d2((x, y), q) >= 2 * r + 5 for q in placed):
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
    _run_and_check(p, pos, tgt, label="full_board_home")


def _test_staging_parking(p):
    spacing = 34.0
    pids = [0x0D, 0x0C, 0x09, 0x10, 0x1E, 0x1D]
    pos = {pid: (80.0 + i * spacing, 200.0) for i, pid in enumerate(pids)}
    tgt = {pids[0]: (80.0 + 5 * spacing, 200.0), pids[5]: (80.0, 200.0)}
    _run_and_check(p, pos, tgt, label="staging_parking")


def _test_min_segment_check(p):
    random.seed(77)
    home = PIECES.HOME_POSITIONS
    pids = [pid for pid in home if home[pid][0] >= 0]
    r = _R
    pos = {}
    placed = []
    for pid in pids:
        for _ in range(50_000):
            x = random.uniform(-84.0, 484.0)
            y = random.uniform(-9.0, 409.0)
            if all(_d2((x, y), q) >= 2 * r + 5 for q in placed):
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
    commands = p.plan_moves(pos, tgt)
    MIN_SEG = float(getattr(PLANNING, "CONFLICT_MIN_SEGMENT_MM", 20.0))
    cur = dict(pos)
    waves = {}
    for cmd in commands:
        waves.setdefault(cmd.sequence_num, []).append(cmd)
    viol = []
    for seq in sorted(waves):
        for cmd in waves[seq]:
            pid = cmd.piece_id
            if pid in cur:
                d = _d2(cur[pid], (cmd.target_x_mm, cmd.target_y_mm))
                if d < MIN_SEG - 0.5:
                    dock_eps = float(getattr(PLANNING, "CONFLICT_DOCK_EPS_MM", 8.0))
                    if d < dock_eps:
                        viol.append(f"0x{pid:02X} seg={d:.1f}mm seq={seq}")
            cur[pid] = (cmd.target_x_mm, cmd.target_y_mm)
    assert not viol, f"min_segment_check: {viol[:5]}"


def _test_parallel_efficiency(p):
    S = 50.0
    pos = {(0x01 + i): ((i + 0.5) * S, 75.0) for i in range(8)}
    tgt = {pid: (x, 175.0) for pid, (x, _) in pos.items()}
    commands = p.plan_moves(pos, tgt)
    w0 = {c.piece_id for c in commands if c.sequence_num == 0}
    assert len(w0) >= 4, f"parallel_efficiency: expected >=4 in wave 0, got {len(w0)}"


def _test_target_swap_optimal(p):
    pos = {0x01: (25.0, 75.0), 0x02: (375.0, 75.0)}
    naive = {0x01: (375.0, 75.0), 0x02: (25.0, 75.0)}
    opt = _optimise_targets_optimal(pos, naive)
    naive_t = sum(_d2(pos[pid], naive[pid]) for pid in pos)
    opt_t = sum(_d2(pos[pid], opt[pid]) for pid in pos)
    assert (
        opt_t < naive_t - 100
    ), f"target_swap_optimal: opt={opt_t:.0f} naive={naive_t:.0f}"


def _test_target_swap_no_worse(p):
    random.seed(55)
    S = 50.0
    pids = list(range(0x01, 0x09))
    placed = []
    pos = {}
    for pid in pids:
        for _ in range(10_000):
            x = random.uniform(0.0, 400.0)
            y = random.uniform(0.0, 400.0)
            if all(_d2((x, y), q) >= 2 * _R + 5 for q in placed):
                pos[pid] = (x, y)
                placed.append((x, y))
                break
        else:
            pos[pid] = (float(pids.index(pid) + 1) * S - S / 2, 75.0)
            placed.append(pos[pid])
    home = PIECES.HOME_POSITIONS
    naive = {pid: (float(home[pid][0]), float(home[pid][1])) for pid in pids}
    naive_t = sum(_d2(pos[pid], naive[pid]) for pid in pids)
    opt = _optimise_targets_optimal(pos, naive)
    opt_t = sum(_d2(pos[pid], opt[pid]) for pid in pids)
    assert (
        opt_t <= naive_t + 1.0
    ), f"target_swap_no_worse: opt={opt_t:.0f} naive={naive_t:.0f}"


# ---------------------------------------------------------------------------
# Test registry and runner
# ---------------------------------------------------------------------------

TESTS = [
    ("single_piece_open", _test_single_piece_open),
    ("two_pieces_crossing", _test_two_pieces_crossing),
    ("two_piece_swap", _test_two_piece_swap),
    ("four_piece_cycle", _test_four_piece_cycle),
    ("eight_pawns_advance", _test_eight_pawns_advance),
    ("back_rank_home", _test_back_rank_home),
    ("full_board_home", _test_full_board_home),
    ("staging_parking", _test_staging_parking),
    ("min_segment_check", _test_min_segment_check),
    ("parallel_efficiency", _test_parallel_efficiency),
    ("target_swap_optimal", _test_target_swap_optimal),
    ("target_swap_no_worse", _test_target_swap_no_worse),
]

PLANNER_REGISTRY = {
    "makeway": ("EnhancedConflictPlanner", EnhancedConflictPlanner),
}


def run_test(name, fn, planner_instance):
    try:
        fn(planner_instance)
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
        description="EnhancedConflictPlanner diagnostic suite"
    )
    parser.add_argument(
        "planner",
        nargs="?",
        default="",
        help='Planner to test: "makeway", "staging", or empty for both',
    )
    parser.add_argument(
        "filter",
        nargs="?",
        default="",
        help="Run only tests whose name contains this substring",
    )
    args = parser.parse_args()

    planner_filter = args.planner.lower()
    test_filter = args.filter.lower()

    # If planner_filter looks like a test name rather than a planner key, treat
    # it as a test filter and run both planners.
    if planner_filter and planner_filter not in PLANNER_REGISTRY:
        test_filter = planner_filter
        planner_filter = ""

    planners_to_run = {
        k: v
        for k, v in PLANNER_REGISTRY.items()
        if not planner_filter or k == planner_filter
    }

    grand_passed = grand_failed = 0
    for pkey, (pname, PlannerClass) in planners_to_run.items():
        planner_instance = PlannerClass()
        print(f"\n=== {pname} ===")
        passed = failed = 0
        for name, fn in TESTS:
            if test_filter and test_filter not in name:
                continue
            ok = run_test(name, fn, planner_instance)
            if ok:
                passed += 1
            else:
                failed += 1
        total = passed + failed
        print(f"{passed}/{total} passed", "OK" if failed == 0 else "FAIL")
        grand_passed += passed
        grand_failed += failed

    if len(planners_to_run) > 1:
        grand_total = grand_passed + grand_failed
        print(f"\n=== TOTAL: {grand_passed}/{grand_total} passed ===")
    sys.exit(0 if grand_failed == 0 else 1)
