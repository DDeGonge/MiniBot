# Swarm planner (PIBT + LNS + BVC)

A complete, collision-free multi-robot planner for the MiniBot coordinator,
vendored as a self-contained subpackage (standard library + `planning.base_planner`
+ `config` only). Registered as **"Swarm (PIBT+LNS)"**; entry point
`planning.swarm_planner.SwarmPlanner`.

## Why it exists

The existing `EnhancedConflictPlanner` is heuristic and can get stuck. This planner
is complete by construction:

1. **PIBT** (priority inheritance with backtracking) plans the choreography over an
   8-connected cell grid: the piece farthest from its goal gets right of way, others
   step aside and flow back. No two pieces ever share a cell and swaps are forbidden.
2. **MAPF-LNS** refines that plan for lower total travel (destroy-and-repair with
   space-time A*), so fewer pieces move and they move shorter paths.
3. **Buffered Voronoi Cells (BVC)** execute the plan as continuous, collision-free
   motion. Each piece keeps to its own buffered half-space, so pairs stay at least a
   diameter apart at all times.

## How it drives real robots

The robots follow waypoints open-loop (no on-board reciprocal avoidance), so
collision-freedom has to be in the plan. Handing them raw cell steps would risk
diagonal crossings colliding. Instead `plan_moves` forward-simulates the full
PIBT/LNS + BVC pipeline and emits each piece's **actual sampled trajectory** as
wave-ordered `MoveCommand`s. The robots replay the collision-free path; sampled
poses are collision-free by construction. Wave granularity is tunable via
`SwarmPlanner(sample_ms=...)` (fewer, coarser waves = fewer dispatch round-trips).

## Board size: 57.15 mm squares (2.25 in)

This branch sets `config.BOARD.SQUARE_SIZE_MM = 57.15` — the regulation tournament
square — as the board size, for two reasons:

1. **Standard sizing.** 2.25 in is the FIDE/USCF tournament square; real boards and
   pieces are made to it.
2. **Collision-free-motion clearance.** A MiniBot is ~31 mm across. At a 50 mm
   square the gap between piece bodies on neighbouring squares is only ~19 mm, and
   the buffered-Voronoi routing has almost no room to slip a piece past its
   neighbours — it deadlocks and detours far more often. At 57.15 mm that gap grows
   to ~26 mm, which is what keeps dense resets and shuffles collision-free and
   efficient (measurably higher completion and lower total travel on scattered-reset
   benchmarks).

The planner itself reads the square size from `config`, so it stays correct on any
board size; 57.15 mm is simply the size the motion strategy is tuned for. All
piece coordinates in `config.py` derive from `SQUARE_SIZE_MM` (the `_S` shorthand
tracks it), so they scale together and cannot drift. See `RECOMMENDED_SQUARE_MM`
in `planning/swarm_planner.py`.

## Self test

From `firmware/MiniBot_Coordinator/`:

```
python planning/swarm/_selftest.py
```

Scatters all 34 pieces, plans them home, and asserts the waves are wave-ordered,
each piece's last waypoint is on its target, and every wave is collision-free.
