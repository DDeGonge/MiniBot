"""Forward-simulate the PIBT/LNS choreography through the BVC executor.

The planner decides each piece's next square; this drive loop executes the
transitions as continuous collision-free motion, advancing to the next joint
configuration only once every piece has reached its current one, latching pieces
home on the final step, and turning them in place to face forward. If a transition
stalls it replans from the pieces' actual squares. It records each piece's
trajectory so the caller can emit it as waypoints, which is how our sim's
collision-free behavior reaches robots that only follow waypoints.
"""
import math

from planning.swarm.executor import (
    DT_S, GOAL_TOL_MM, MAX_WHEEL_SPEED_MMPS, NEIGHBOR_RANGE_MM, PIECE_DIAMETER_MM,
    PROJECTION_ITERS, RADIUS_MM, REACH_TOL_MM, WHEELBASE_MM, drive_bot_to, step_pose,
)

_STALL_LIMIT_STEPS = 400
_MAX_REPLAN_FAILS = 3
_ORIENT_TOL_RAD = 0.05
_MOVED_EPS_MM = 1.0


class Bot:
    """Lightweight mutable piece state used by the executor and drive loop."""

    __slots__ = ("id", "x", "y", "theta")

    def __init__(self, piece_id: int, x: float, y: float, theta: float):
        self.id = piece_id
        self.x = x
        self.y = y
        self.theta = theta


def _at(x, y, gx, gy, tol) -> bool:
    """True if (x, y) is within tol of (gx, gy)."""
    return math.hypot(x - gx, y - gy) < tol


def _wrap(angle: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class HybridDrive:
    """Executes a PIBT/LNS plan through BVC, replanning on stalls, and settles pieces."""

    def __init__(self, grid, goal_xy: dict, ids: list, orient_target: dict, planner):
        self._grid = grid
        self._goal_xy = goal_xy
        self._ids = ids
        self._index = {bid: i for i, bid in enumerate(ids)}
        self._orient = orient_target
        self._planner = planner
        self._goal_cells = None
        self._plan = None
        self._idx = 0
        self._since = 0
        self._fails = 0
        self._home: set = set()
        self._finished = False

    def reset(self, bots) -> None:
        """Snap starts/goals to distinct cells and plan the choreography."""
        self._goal_cells = self._distinct_cells([self._goal_xy[b.id] for b in bots])
        starts = self._distinct_cells([(b.x, b.y) for b in bots])
        self._plan = self._planner(starts, self._goal_cells, self._grid)
        self._idx = 1 if self._plan and len(self._plan) > 1 else 0
        self._finished = self._plan is not None and len(self._plan) <= 1

    def _distinct_cells(self, points: list) -> list:
        """Snap each point to a distinct cell (nearest free cell on collision)."""
        used = set()
        out = []
        for x, y in points:
            cell = self._grid.nearest(x, y)
            if cell in used:
                cell = self._nearest_free(x, y, used)
            used.add(cell)
            out.append(cell)
        return out

    def _nearest_free(self, x: float, y: float, used: set) -> int:
        """The closest cell to (x, y) not already taken in this assignment pass."""
        best_id, best_d = 0, math.inf
        for cid in range(self._grid.count):
            if cid in used:
                continue
            cx, cy = self._grid.xy(cid)
            d = (cx - x) ** 2 + (cy - y) ** 2
            if d < best_d:
                best_d, best_id = d, cid
        return best_id

    def _aligned(self, bot) -> bool:
        """True if the piece faces its target heading within tolerance."""
        if bot.id not in self._orient:
            return True
        return abs(_wrap(self._orient[bot.id] - bot.theta)) <= _ORIENT_TOL_RAD

    def _hold_or_orient(self, bot) -> bool:
        """Hold a settled piece, spinning it toward its target heading if needed."""
        if self._aligned(bot):
            return True
        err = _wrap(self._orient[bot.id] - bot.theta)
        turn_mag = min(MAX_WHEEL_SPEED_MMPS, abs(err) * WHEELBASE_MM / (2.0 * DT_S))
        turn = turn_mag if err > 0.0 else -turn_mag
        bot.x, bot.y, bot.theta = step_pose(bot.x, bot.y, bot.theta, -turn, turn, DT_S)
        return False

    def advance(self, bots) -> None:
        """Advance one tick: drive pieces toward their current step's squares."""
        if self._finished:
            return
        if self._plan is None:
            self._drive_to_goals(bots)
            return
        final = self._idx >= len(self._plan) - 1
        targets = self._plan[self._idx]
        positions = {b.id: (b.x, b.y) for b in bots}
        for bot in bots:
            if bot.id in self._home:
                self._hold_or_orient(bot)
                continue
            tx, ty = self._grid.xy(targets[self._index[bot.id]])
            skip = GOAL_TOL_MM if final else REACH_TOL_MM
            if _at(bot.x, bot.y, tx, ty, skip):
                continue
            drive_bot_to(bot, (tx, ty), positions, RADIUS_MM, PIECE_DIAMETER_MM,
                         NEIGHBOR_RANGE_MM, GOAL_TOL_MM, PROJECTION_ITERS, DT_S)
        self._advance_plan(bots, final)

    def _advance_plan(self, bots, final: bool) -> None:
        """Advance to the next configuration, or latch pieces home on the final one."""
        if final:
            for bot in bots:
                cell = self._plan[-1][self._index[bot.id]]
                if _at(bot.x, bot.y, *self._grid.xy(cell), GOAL_TOL_MM):
                    self._home.add(bot.id)
            if len(self._home) == len(bots) and all(self._aligned(b) for b in bots):
                self._finished = True
            return
        reached = all(
            _at(b.x, b.y, *self._grid.xy(self._plan[self._idx][self._index[b.id]]),
                REACH_TOL_MM)
            for b in bots
        )
        if reached:
            self._since = 0
            self._idx += 1
            return
        self._since += 1
        if self._since >= _STALL_LIMIT_STEPS:
            self._replan(bots)

    def _replan(self, bots) -> None:
        """Recompute the plan from actual squares; fall back if it keeps failing."""
        self._since = 0
        starts = self._distinct_cells([(b.x, b.y) for b in bots])
        plan = self._planner(starts, self._goal_cells, self._grid)
        if plan is not None:
            self._fails = 0
            self._home = set()
            self._plan = plan
            self._idx = 1 if len(plan) > 1 else 0
            if len(plan) <= 1:
                self._finished = True
            return
        self._fails += 1
        if self._fails >= _MAX_REPLAN_FAILS:
            self._plan = None

    def _drive_to_goals(self, bots) -> None:
        """Fallback when no plan exists: drive each piece straight at its goal."""
        positions = {b.id: (b.x, b.y) for b in bots}
        all_home = True
        for bot in bots:
            gx, gy = self._goal_xy[bot.id]
            if _at(bot.x, bot.y, gx, gy, REACH_TOL_MM):
                if not self._hold_or_orient(bot):
                    all_home = False
                continue
            all_home = False
            drive_bot_to(bot, (gx, gy), positions, RADIUS_MM, PIECE_DIAMETER_MM,
                         NEIGHBOR_RANGE_MM, GOAL_TOL_MM, PROJECTION_ITERS, DT_S)
        self._finished = all_home

    @property
    def finished(self) -> bool:
        """True when every piece has reached its goal square and faced forward."""
        return self._finished


def run_and_sample(drive: HybridDrive, bots, sample_every: int, step_cap: int) -> list:
    """Run the drive to completion, sampling moved pieces every sample_every ticks.

    Returns a list of waves; each wave is a dict piece_id -> (x, y, theta_rad) for
    the pieces that moved since their last sample. The sampled poses come straight
    from the collision-free executor, so replaying them as waypoints is collision
    free by construction.
    """
    last = {b.id: (b.x, b.y) for b in bots}
    waves: list = []
    step = 0
    while step < step_cap:
        drive.advance(bots)
        step += 1
        if step % sample_every == 0 or drive.finished:
            snap = {}
            for bot in bots:
                if math.hypot(bot.x - last[bot.id][0], bot.y - last[bot.id][1]) >= _MOVED_EPS_MM:
                    snap[bot.id] = (bot.x, bot.y, bot.theta)
                    last[bot.id] = (bot.x, bot.y)
            if snap:
                waves.append(snap)
        if drive.finished:
            break
    return waves
