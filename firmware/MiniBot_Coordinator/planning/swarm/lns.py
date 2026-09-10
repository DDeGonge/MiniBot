"""MAPF-LNS: anytime flowtime refinement of an initial PIBT plan (cell grid).

Starts from PIBT's feasible plan, then repeatedly destroys a random subset of
agents' paths and repairs them with prioritized space-time A* against the rest,
keeping a change only when it lowers flowtime (total travel). This trims the
detours and extra moves greedy PIBT leaves in, so fewer pieces move and they move
less, while staying conflict-free at the cell level. The output matches pibt_plan
(a list of joint configurations), so it drops into the same executor slot.

Reference: Li et al., "Anytime Multi-Agent Path Finding via Large Neighborhood
Search" (IJCAI 2021).
"""
import heapq
import random
import time

from planning.swarm.pibt import _distance_table, pibt_plan

_INF = float("inf")
_MAX_T = 256              # planning horizon in ticks
_GOAL_HOLD = _MAX_T       # a parked goal stays reserved across the whole horizon
_WAIT_COST = 1.0          # a wait delays arrival by a tick, matching flowtime
_DEFAULT_NEIGHBORHOOD = 8
_MAX_ITERS = 100000       # hard cap on refinement iterations (reproducibility)
_STALE_FACTOR = 8         # converged after this many no-improvement rounds per agent


class _Reservation:
    """Space-time reservations: occupied cells per tick, plus swap edges."""

    def __init__(self):
        self._vertex: dict = {}
        self._edge: dict = {}

    def add_path(self, path: list) -> None:
        """Reserve a path's cells and traversals, holding its goal afterward."""
        for t, cell in enumerate(path):
            self._vertex.setdefault(t, set()).add(cell)
            if t + 1 < len(path):
                self._edge.setdefault(t, set()).add((cell, path[t + 1]))
        if path:
            last = path[-1]
            for t in range(len(path), len(path) + _GOAL_HOLD):
                self._vertex.setdefault(t, set()).add(last)

    def vertex_free(self, cell: int, t: int) -> bool:
        """True if cell is unoccupied at tick t."""
        return cell not in self._vertex.get(t, ())

    def edge_free(self, frm: int, to: int, t: int) -> bool:
        """True if moving frm->to at tick t does not swap with a reservation."""
        return (to, frm) not in self._edge.get(t, ())


def _reconstruct(came: dict, key) -> list:
    """Walk parent links from a goal key back to the start; return cells in order."""
    path = []
    while key is not None:
        path.append(key[0])
        key = came[key]
    path.reverse()
    return path


def _astar(start: int, goal: int, grid, hdist: dict, res: _Reservation,
           max_t: int = _MAX_T):
    """Single-agent space-time A* from start to goal avoiding the reservation.

    Move and (non-goal) wait both cost one tick, so the path cost equals arrival
    time; A* therefore minimizes arrival time, matching the flowtime objective.
    """
    open_heap = [(hdist.get(start, _INF), 0.0, start, 0)]
    best = {(start, 0): 0.0}
    came = {(start, 0): None}
    while open_heap:
        _, g, cell, t = heapq.heappop(open_heap)
        if best.get((cell, t), _INF) < g:
            continue
        if cell == goal and all(res.vertex_free(goal, tt) for tt in range(t, t + _GOAL_HOLD)):
            # Accept the goal only when the agent can remain there: a fixed agent
            # may still pass through it later, so require it free for all future
            # ticks (the reservation horizon), or keep searching for a later arrival.
            return _reconstruct(came, (cell, t))
        if t >= max_t:
            continue
        for nxt in [cell] + grid.neighbors(cell):
            nt = t + 1
            if not res.vertex_free(nxt, nt):
                continue
            if nxt != cell and not res.edge_free(cell, nxt, t):
                continue
            ng = g + (_WAIT_COST if nxt == cell else 1.0)
            nkey = (nxt, nt)
            if ng < best.get(nkey, _INF):
                best[nkey] = ng
                came[nkey] = (cell, t)
                heapq.heappush(open_heap, (ng + hdist.get(nxt, _INF), ng, nxt, nt))
    return None


def _path_cost(path: list, goal: int) -> int:
    """Arrival time: timesteps until the agent reaches its goal and stays."""
    cost = 0
    for t, cell in enumerate(path):
        if cell != goal:
            cost = t + 1
    return cost


def _flowtime(paths: list, goals: list) -> int:
    """Sum of per-agent arrival times (total travel to refine)."""
    return sum(_path_cost(paths[i], goals[i]) for i in range(len(paths)))


def _conflict_free(paths: list) -> bool:
    """True if no two agents share a cell or swap at any tick (with goal-holding)."""
    length = max(len(p) for p in paths)
    n = len(paths)

    def at(i, t):
        return paths[i][t] if t < len(paths[i]) else paths[i][-1]

    for t in range(length):
        seen = {}
        for i in range(n):
            cell = at(i, t)
            if cell in seen:
                return False
            seen[cell] = i
        if t + 1 < length:
            for i in range(n):
                for j in range(i + 1, n):
                    if at(i, t) == at(j, t + 1) and at(j, t) == at(i, t + 1) \
                            and at(i, t) != at(i, t + 1):
                        return False
    return True


def _to_configs(paths: list) -> list:
    """Pad agent paths to equal length (holding goals) and transpose to configs."""
    length = max(len(p) for p in paths)
    padded = [p + [p[-1]] * (length - len(p)) for p in paths]
    return [tuple(padded[i][t] for i in range(len(padded))) for t in range(length)]


def _neighborhood(paths, goals, n, size, rng):
    """A randomized neighborhood: a high-cost seed agent plus random others.

    Drawing the seed from the worst-cost half focuses repair where flowtime slack
    lives, while the random seed and random fill let successive rounds explore
    different agent subsets and escape the local minima a fixed sweep gets stuck in.
    """
    size = min(size, n)
    ranked = sorted(range(n), key=lambda i: -_path_cost(paths[i], goals[i]))
    seed_agent = rng.choice(ranked[:max(1, n // 2)])
    pool = [j for j in range(n) if j != seed_agent]
    rng.shuffle(pool)
    return [seed_agent] + pool[:size - 1]


def lns_plan(starts, goals, grid, time_budget_s: float = 0.4,
             neighborhood: int = _DEFAULT_NEIGHBORHOOD, max_t: int = _MAX_T,
             seed: int = 0, priority=None):
    """Refine a PIBT plan to lower flowtime; return a list of joint configurations.

    Each round destroys a randomized neighborhood and repairs it with prioritized
    space-time A*, accepting only conflict-free, lower-flowtime results. Stops at
    the time budget, after _STALE_FACTOR*n no-improvement rounds (converged), or at
    the iteration cap. Falls back to the raw PIBT plan if refinement finds nothing.
    """
    initial = pibt_plan(starts, goals, grid, priority=priority)
    if initial is None:
        return None
    n = len(starts)
    paths = [[cfg[i] for cfg in initial] for i in range(n)]
    hdist = [_distance_table(grid, goals[i]) for i in range(n)]
    rng = random.Random(seed)
    deadline = time.perf_counter() + time_budget_s
    stale_limit = _STALE_FACTOR * n
    stale = 0
    for _ in range(_MAX_ITERS):
        if time.perf_counter() >= deadline or stale >= stale_limit:
            break
        group = _neighborhood(paths, goals, n, neighborhood, rng)
        others = [j for j in range(n) if j not in group]
        res = _Reservation()
        for j in others:
            res.add_path(paths[j])
        repaired = {}
        ok = True
        for i in sorted(group, key=lambda i: -_path_cost(paths[i], goals[i])):
            path = _astar(starts[i], goals[i], grid, hdist[i], res, max_t)
            if path is None:
                ok = False
                break
            repaired[i] = path
            res.add_path(path)
        if not ok:
            stale += 1
            continue
        candidate = list(paths)
        for i, path in repaired.items():
            candidate[i] = path
        if _flowtime(candidate, goals) < _flowtime(paths, goals) and _conflict_free(candidate):
            paths = candidate
            stale = 0
        else:
            stale += 1
    return _to_configs(paths)
