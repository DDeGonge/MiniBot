"""PIBT choreography over the cell grid: who moves where, in priority order.

PIBT (Priority Inheritance with Backtracking) advances every piece one cell per
step toward its goal in priority order; a higher-priority piece can force a
lower-priority occupant to step aside (priority inheritance), and the search
backtracks when a forced move leaves a piece no escape. This is exactly the
"furthest piece gets right of way, others move aside and flow back" behaviour we
want, computed completely rather than hoped for reactively.

Cells sit far enough apart that two distinct cells never conflict, so occupancy
is simply one piece per cell. The output is a sequence of joint configurations (a
cell per piece per step); the BVC layer executes the transitions as smooth,
collision-free, straight center-to-center motion.
"""
from collections import deque

_INF = float("inf")
_FAR = (_INF, _INF)


def _distance_table(grid, goal_cell: int) -> dict:
    """Breadth-first cell-to-goal distances over the 8-connected grid."""
    dist = {goal_cell: 0}
    queue = deque([goal_cell])
    while queue:
        cell = queue.popleft()
        for nbr in grid.neighbors(cell):
            if nbr not in dist:
                dist[nbr] = dist[cell] + 1
                queue.append(nbr)
    return dist


def _pibt_step(config, order, key, occupied_start, succ, n, forced=None):
    """One PIBT timestep; returns the next config tuple, or None if stuck."""
    nxt = [None] * n
    occupied = {}
    if forced:
        for i, cell in forced.items():
            if cell in occupied:
                return None
            nxt[i] = cell
            occupied[cell] = i

    def _candidate_key(i, cell, pushed):
        # Always prefer progress toward the goal, then a straight line. A piece
        # being PUSHED aside additionally prefers a square that starts empty, so
        # it steps into open space and returns rather than shoving a whole line.
        hop, straight = key[i].get(cell, _FAR)
        if pushed:
            return (hop, cell in occupied_start, straight)
        return (hop, straight)

    def assign(i, caller_cell):
        pushed = caller_cell is not None
        candidates = sorted(succ[config[i]], key=lambda v: _candidate_key(i, v, pushed))
        for cell in candidates:
            if caller_cell is not None and cell == caller_cell:
                continue
            if cell in occupied:
                continue
            nxt[i] = cell
            occupied[cell] = i
            blocker = next((j for j in range(n)
                            if nxt[j] is None and config[j] == cell), None)
            if blocker is not None and not assign(blocker, config[i]):
                del occupied[cell]
                nxt[i] = None
                continue
            return True
        return False

    for i in order:
        if nxt[i] is None and not assign(i, None):
            return None
    return tuple(nxt)


def plan_tables(grid, goals):
    """Per-agent BFS distances, sort keys, and successors for a planner.

    Returns (dist, key, succ): dist[i] maps cell to grid-step distance from goal i;
    key[i] maps cell to (hop, squared straight-line distance to the goal) for the
    tie-break that keeps motion straight; succ maps a cell to itself plus its
    neighbors.
    """
    n = len(goals)
    dist = [_distance_table(grid, goals[i]) for i in range(n)]
    goal_xy = [grid.xy(goals[i]) for i in range(n)]
    key = [{cell: (hop, (grid.xy(cell)[0] - goal_xy[i][0]) ** 2
                   + (grid.xy(cell)[1] - goal_xy[i][1]) ** 2)
            for cell, hop in dist[i].items()}
           for i in range(n)]
    succ = {cell: [cell] + grid.neighbors(cell) for cell in range(grid.count)}
    return dist, key, succ


def pibt_plan(starts, goals, grid, max_t: int = 512, priority=None):
    """Plan a sequence of joint cell configurations from starts to goals.

    starts and goals are lists of cell ids indexed by agent. Returns a list of
    configurations (the first is starts, the last is goals), or None if PIBT
    cannot make progress within max_t steps. Without priority, ordering is dynamic
    (the piece farthest from its goal moves first). With priority (a per-agent key,
    lower wins), that key orders conflicts and distance is only the tiebreak.
    """
    n = len(starts)
    dist, key, succ = plan_tables(grid, goals)
    config = tuple(starts)
    goal = tuple(goals)
    configs = [config]
    visited = {config}
    for _ in range(max_t):
        if config == goal:
            return configs
        if priority is None:
            order = sorted(range(n), key=lambda i: -dist[i].get(config[i], _INF))
        else:
            order = sorted(range(n),
                           key=lambda i: (priority[i], -dist[i].get(config[i], _INF)))
        nxt = _pibt_step(config, order, key, set(config), succ, n)
        if nxt is None:
            return None
        if nxt in visited:
            # PIBT+: never repeat a joint configuration. Revisiting one means the
            # greedy step is cycling with no progress; bail so the caller replans.
            return None
        visited.add(nxt)
        config = nxt
        configs.append(config)
    return configs if config == goal else None
