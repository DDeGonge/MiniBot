"""
tests/test_stress.py  —  MiniBot Chess Swarm Coordinator

Randomised stress test for EnhancedConflictPlanner.
Runs many random-seed scenarios across three difficulty levels and reports
any seeds that produce oscillations, stuck pieces, or collisions.

Usage:
    python tests/test_stress.py                    # default: 100 seeds, all modes
    python tests/test_stress.py --seeds 500        # run more seeds
    python tests/test_stress.py --mode full        # only full-board tests
    python tests/test_stress.py --mode half        # only half-board tests
    python tests/test_stress.py --mode pairs       # only pair-swap tests
    python tests/test_stress.py --seed 42          # replay a specific seed
    python tests/test_stress.py --verbose          # print per-seed results
    python tests/test_stress.py --seeds 200 --jobs 4   # parallel (if available)
"""

from __future__ import annotations

import argparse
import math
import os
import random
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import PIECES, SIMULATOR, PLANNING
from planning.base_planner import MoveCommand
from planning.enhanced_conflict_planner import (
    EnhancedConflictPlanner,
    _optimise_targets_optimal,
)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

_R = float(PIECES.CIRCLE_RADIUS_MM)
_X_LO = float(SIMULATOR.X_MIN_MM) + _R
_X_HI = float(SIMULATOR.X_MAX_MM) - _R
_Y_LO = float(SIMULATOR.Y_MIN_MM) + _R
_Y_HI = float(SIMULATOR.Y_MAX_MM) - _R
_ARRIVE = float(getattr(PLANNING, "CONFLICT_ARRIVAL_EPS_MM", 2.0))
_MIN_GAP = 2 * _R + 2.0  # minimum centre-to-centre gap when placing pieces


# ---------------------------------------------------------------------------
# Embedded step simulator (same as test_new_planners.py)
# ---------------------------------------------------------------------------


@dataclass
class SimResult:
    success: bool
    collision_events: List[str] = field(default_factory=list)
    boundary_events: List[str] = field(default_factory=list)
    final_positions: Dict[int, Tuple[float, float]] = field(default_factory=dict)
    # wave index where first collision occurred, -1 = none
    first_collision_wave: int = -1


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
            pre_events = len(result.collision_events)
            if not self._run_wave(waves[wave_idx], result):
                result.success = False
            if (
                result.first_collision_wave == -1
                and len(result.collision_events) > pre_events
            ):
                result.first_collision_wave = wave_idx
        result.final_positions = dict(self._pos)
        return result

    def _run_wave(self, cmds, result):
        dt = self.DT_MS / 1000.0
        step_mm = self.SPEED_MM_S * dt
        step_deg = self.ROT_DEG_S * dt
        timeout_t = int(self.WAVE_TIMEOUT_S / dt)
        phase = {cmd.piece_id: "rotate" for cmd in cmds}
        rotate_to: Dict[int, float] = {}
        targets = {cmd.piece_id: (cmd.target_x_mm, cmd.target_y_mm) for cmd in cmds}
        active = set(phase.keys())

        for _ in range(timeout_t):
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
                    msg = f"boundary 0x{pid:02X} at ({px:.1f},{py:.1f})"
                    if msg not in result.boundary_events:
                        result.boundary_events.append(msg)

            all_pids = list(self._pos.keys())
            for i in range(len(all_pids)):
                for j in range(i + 1, len(all_pids)):
                    pa, pb = all_pids[i], all_pids[j]
                    d = math.hypot(
                        self._pos[pa][0] - self._pos[pb][0],
                        self._pos[pa][1] - self._pos[pb][1],
                    )
                    if d < self.COLLISION_DIST:
                        msg = f"0x{pa:02X}<->0x{pb:02X}"
                        if msg not in result.collision_events:
                            result.collision_events.append(msg)

        if active:
            for pid in sorted(active):
                tx, ty = targets[pid]
                cx, cy = self._pos[pid]
                result.collision_events.append(
                    f"timeout 0x{pid:02X} stuck at ({cx:.0f},{cy:.0f}) "
                    f"want ({tx:.0f},{ty:.0f})"
                )
            return False
        return True


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _all_at_goals(final, goals, eps=4.0):
    return all(pid in final and _dist(final[pid], g) <= eps for pid, g in goals.items())


# ---------------------------------------------------------------------------
# Random position generators
# ---------------------------------------------------------------------------


def _random_positions(
    rng: random.Random, pids: List[int], max_tries: int = 50_000
) -> Dict[int, Tuple[float, float]]:
    """Place each piece at a random collision-free position within the arena."""
    pos: Dict[int, Tuple[float, float]] = {}
    placed: List[Tuple[float, float]] = []
    home = PIECES.HOME_POSITIONS

    for pid in pids:
        placed_ok = False
        for _ in range(max_tries):
            x = rng.uniform(_X_LO, _X_HI)
            y = rng.uniform(_Y_LO, _Y_HI)
            if all(_dist((x, y), q) >= _MIN_GAP for q in placed):
                pos[pid] = (x, y)
                placed.append((x, y))
                placed_ok = True
                break
        if not placed_ok:
            # Fall back to home position with a small offset so it's never exact
            hx, hy, _ = home[pid]
            pos[pid] = (float(hx) + 1.0, float(hy) + 1.0)
            placed.append(pos[pid])

    return pos


def _home_targets(pids: List[int]) -> Dict[int, Tuple[float, float]]:
    home = PIECES.HOME_POSITIONS
    return {pid: (float(home[pid][0]), float(home[pid][1])) for pid in pids}


# ---------------------------------------------------------------------------
# Failure record
# ---------------------------------------------------------------------------


@dataclass
class Failure:
    seed: int
    mode: str
    reason: str  # 'stuck' | 'collision' | 'boundary' | 'exception'
    detail: str


# ---------------------------------------------------------------------------
# Single-seed runner
# ---------------------------------------------------------------------------


def run_seed(
    planner: EnhancedConflictPlanner,
    seed: int,
    mode: str,
    debug: bool = False,
) -> Optional[Failure]:
    """Run one scenario and return a Failure if something went wrong."""
    rng = random.Random(seed)
    home = PIECES.HOME_POSITIONS

    # Choose piece set based on mode
    if mode == "full":
        pids = [pid for pid in home if home[pid][0] >= 0]
    elif mode == "half":
        # White pieces only (0x01-0x10)
        pids = list(range(0x01, 0x11))
    elif mode == "back_rank":
        # Back-rank + pawn swarm (all white)
        pids = list(range(0x01, 0x11))
    elif mode == "pairs":
        # 4 random pieces from each colour
        white = list(range(0x01, 0x11))
        black = [pid for pid in home if pid > 0x10 and home[pid][0] >= 0]
        rng.shuffle(white)
        rng.shuffle(black)
        pids = white[:4] + black[:4]
    elif mode == "pawns":
        # All 8 white pawns + all 8 black pawns
        pids = [
            pid
            for pid in home
            if home[pid][0] >= 0
            and getattr(PIECES, "PIECE_RANKS", {}).get(pid, "") == "pawn"
        ]
        if len(pids) < 4:
            # Fallback: first 16 pieces
            pids = [pid for pid in home if home[pid][0] >= 0][:16]
    else:
        raise ValueError(f"Unknown mode: {mode}")

    pos = _random_positions(rng, pids)
    tgts = _home_targets(pids)

    try:
        commands = planner.plan_moves(pos, tgts)
    except Exception as exc:
        return Failure(
            seed=seed,
            mode=mode,
            reason="exception",
            detail=f"{type(exc).__name__}: {exc}",
        )

    if not commands:
        # Check if already at goals
        effective = _optimise_targets_optimal(
            {p: (float(x), float(y)) for p, (x, y) in pos.items()},
            {p: (float(x), float(y)) for p, (x, y) in tgts.items()},
        )
        if _all_at_goals(pos, effective):
            return None  # trivially done
        return Failure(
            seed=seed,
            mode=mode,
            reason="stuck",
            detail="planner returned no commands but pieces not at goals",
        )

    sim = StepSimulator(pos)
    result = sim.run(commands)

    effective = _optimise_targets_optimal(
        {p: (float(x), float(y)) for p, (x, y) in pos.items()},
        {p: (float(x), float(y)) for p, (x, y) in tgts.items()},
    )

    if result.collision_events:
        detail = "; ".join(result.collision_events[:5])
        if debug:
            _print_plan_debug(pos, tgts, commands, result)
        return Failure(seed=seed, mode=mode, reason="collision", detail=detail)

    if result.boundary_events:
        detail = "; ".join(result.boundary_events[:3])
        if debug:
            _print_plan_debug(pos, tgts, commands, result)
        return Failure(seed=seed, mode=mode, reason="boundary", detail=detail)

    if not _all_at_goals(result.final_positions, effective):
        stuck = [
            f'0x{pid:02X}: got ({result.final_positions.get(pid, ("?","?"))[0]:.0f},'
            f'{result.final_positions.get(pid, ("?","?"))[1]:.0f}) '
            f"want ({gx:.0f},{gy:.0f})"
            for pid, (gx, gy) in effective.items()
            if pid not in result.final_positions
            or _dist(result.final_positions[pid], (gx, gy)) > 4.0
        ]
        stuck_pids = {
            pid
            for pid, (gx, gy) in effective.items()
            if pid not in result.final_positions
            or _dist(result.final_positions[pid], (gx, gy)) > 4.0
        }
        if debug:
            _print_plan_debug(pos, tgts, commands, result, stuck_pids=stuck_pids)
        return Failure(
            seed=seed, mode=mode, reason="stuck", detail="; ".join(stuck[:5])
        )

    return None  # all good


# ---------------------------------------------------------------------------
# Debug helpers
# ---------------------------------------------------------------------------


def _print_plan_debug(
    positions: Dict[int, Tuple[float, float]],
    targets: Dict[int, Tuple[float, float]],
    commands: List[MoveCommand],
    result: SimResult,
    stuck_pids: Optional[set] = None,
) -> None:
    """Print a human-readable summary of the plan and collision context."""
    # Which pieces are involved in collisions or stuck?
    import re

    involved: set = set()
    for evt in result.collision_events:
        for match in re.findall(r"0x[0-9A-Fa-f]+", evt):
            try:
                involved.add(int(match, 16))
            except ValueError:
                pass
    if stuck_pids:
        involved |= stuck_pids

    print(f"  Starting positions:")
    for pid, (x, y) in sorted(positions.items()):
        mark = " <<< INVOLVED" if pid in involved else ""
        print(
            f"    0x{pid:02X}: ({x:.1f}, {y:.1f}) -> target ({targets.get(pid,(0,0))[0]:.1f}, {targets.get(pid,(0,0))[1]:.1f}){mark}"
        )

    first_w = result.first_collision_wave
    print(f"  First collision in wave {first_w}. All commands for involved pieces:")
    for cmd in commands:
        if cmd.piece_id in involved:
            print(
                f"    wave={cmd.sequence_num} 0x{cmd.piece_id:02X} -> ({cmd.target_x_mm:.1f},{cmd.target_y_mm:.1f}) [{cmd.planner_debug}]"
            )
    print(f"  Commands near wave {first_w} (all pieces):")
    for cmd in commands:
        if abs(cmd.sequence_num - first_w) <= 3:
            inv = " <<<" if cmd.piece_id in involved else ""
            print(
                f"    wave={cmd.sequence_num} 0x{cmd.piece_id:02X} -> ({cmd.target_x_mm:.1f},{cmd.target_y_mm:.1f}) [{cmd.planner_debug}]{inv}"
            )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

MODES = ["full", "half", "pairs", "pawns"]

DEFAULT_SEEDS = 100
DEFAULT_MODES = MODES


def main():
    parser = argparse.ArgumentParser(
        description="Randomised stress test for EnhancedConflictPlanner"
    )
    parser.add_argument(
        "--seeds",
        type=int,
        default=DEFAULT_SEEDS,
        help=f"Number of random seeds to test (default {DEFAULT_SEEDS})",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Replay a single specific seed (runs all modes unless --mode set)",
    )
    parser.add_argument(
        "--mode",
        choices=MODES + ["all"],
        default="all",
        help="Scenario type (default: all)",
    )
    parser.add_argument(
        "--verbose",
        "-v",
        action="store_true",
        help="Print PASS for each seed, not just failures",
    )
    parser.add_argument(
        "--fail-fast", action="store_true", help="Stop after first failure"
    )
    parser.add_argument(
        "--debug",
        "-d",
        action="store_true",
        help="Print planned command list for every failure",
    )
    parser.add_argument(
        "--seed-offset",
        type=int,
        default=0,
        help="Add this offset to every seed (for parallel shards)",
    )
    args = parser.parse_args()

    modes_to_run = MODES if args.mode == "all" else [args.mode]

    if args.seed is not None:
        seeds = [args.seed]
    else:
        seeds = list(range(args.seed_offset, args.seed_offset + args.seeds))

    planner = EnhancedConflictPlanner()
    failures: List[Failure] = []
    total = 0
    t_start = time.time()

    print(
        f"Running {len(seeds)} seed(s) × {len(modes_to_run)} mode(s) "
        f"= {len(seeds)*len(modes_to_run)} scenario(s)"
    )
    print()

    for seed in seeds:
        for mode in modes_to_run:
            total += 1
            fail = run_seed(planner, seed, mode, debug=args.debug)
            if fail:
                failures.append(fail)
                tag = f"FAIL  [{mode:8s}] seed={seed:5d}  {fail.reason}: {fail.detail}"
                print(tag)
                if args.fail_fast:
                    break
            elif args.verbose:
                print(f"pass  [{mode:8s}] seed={seed}")
        if args.fail_fast and failures:
            break

    elapsed = time.time() - t_start

    # Summary
    print()
    print("=" * 60)
    passed = total - len(failures)
    print(
        f"{passed}/{total} passed  ({len(failures)} failure(s))  "
        f"{elapsed:.1f}s  ({elapsed/total*1000:.0f}ms/test)"
    )

    if failures:
        print()
        print("Failure breakdown:")
        by_reason: Dict[str, int] = {}
        by_mode: Dict[str, int] = {}
        for f in failures:
            by_reason[f.reason] = by_reason.get(f.reason, 0) + 1
            by_mode[f.mode] = by_mode.get(f.mode, 0) + 1
        for reason, count in sorted(by_reason.items(), key=lambda x: -x[1]):
            print(f"  {reason:12s}: {count}")
        print()
        for mode, count in sorted(by_mode.items(), key=lambda x: -x[1]):
            print(f"  {mode:12s}: {count}")
        print()
        print("To replay a failure:")
        for f in failures[:5]:
            print(
                f"  python tests/test_stress.py --seed {f.seed} --mode {f.mode} --verbose"
            )

    sys.exit(0 if not failures else 1)


if __name__ == "__main__":
    main()
