"""Buffered Voronoi Cells: collision-free-by-construction continuous motion.

Each piece moves toward the point nearest its goal that stays at least ``radius``
inside every perpendicular bisector between it and a neighbor (its buffered
Voronoi cell). Because each piece keeps to its own buffered half-space, any two
pieces' targets stay at least 2*radius apart, so the swarm never collides. A
per-tick safety cap on forward distance guarantees no overlap even before the
iterative projection fully converges, so collision-freedom never depends on
projection accuracy.

Operates on lightweight piece state (objects with mutable .x, .y, .theta in mm
and radians) rather than any external robot model, so it is fully self-contained.
"""
import math

# Kinematics of the real MiniBot (from the firmware config.h).
WHEELBASE_MM = 23.4
WHEEL_RADIUS_MM = 5.25
# Planning wheel speed. Real bots reach ~250 mm/s; here the absolute value only
# scales sim ticks, and each emitted MoveCommand's duration_ms is a time budget
# the bot fills, so trajectory shape (not this speed) is what matters downstream.
MAX_WHEEL_SPEED_MMPS = 80.0

# Motion tunables (ported from the source project's tuned values).
PIECE_DIAMETER_MM = 31.0
POSITION_ERROR_MM = 2.0
SAFETY_MARGIN_MM = 2.0
GOAL_TOL_MM = 3.0
NEIGHBOR_RANGE_MM = 200.0
ALIGN_THRESHOLD_RAD = 0.25
PROJECTION_ITERS = 14
DT_S = 0.02

# Buffer radius each piece keeps clear, and the reach tolerance for latching a
# square (wider than the stop tolerance plus the localization noise band).
RADIUS_MM = PIECE_DIAMETER_MM / 2.0 + POSITION_ERROR_MM + SAFETY_MARGIN_MM
REACH_TOL_MM = GOAL_TOL_MM + 2.0 * POSITION_ERROR_MM

_EPS = 1e-9
_SAFETY_EPS_MM = 0.5
_OMEGA_THRESHOLD = 1e-9


def step_pose(x: float, y: float, theta: float, v_l: float, v_r: float,
              dt_s: float) -> tuple:
    """Integrate one differential-drive timestep; returns (x, y, theta).

    Uses the exact arc solution when turning and the straight-line limit when the
    angular velocity is near zero, avoiding a divide-by-zero.
    """
    v_l = max(-MAX_WHEEL_SPEED_MMPS, min(MAX_WHEEL_SPEED_MMPS, v_l))
    v_r = max(-MAX_WHEEL_SPEED_MMPS, min(MAX_WHEEL_SPEED_MMPS, v_r))
    v = 0.5 * (v_l + v_r)
    omega = (v_r - v_l) / WHEELBASE_MM
    if abs(omega) < _OMEGA_THRESHOLD:
        return (x + v * math.cos(theta) * dt_s, y + v * math.sin(theta) * dt_s, theta)
    radius = v / omega
    new_theta = theta + omega * dt_s
    return (x + radius * (math.sin(new_theta) - math.sin(theta)),
            y - radius * (math.cos(new_theta) - math.cos(theta)),
            new_theta)


def buffered_voronoi_target(pos, goal, neighbors, radius: float, iters: int) -> tuple:
    """Project goal into the buffered Voronoi cell of pos; return the target point."""
    tx, ty = goal
    px, py = pos
    for _ in range(iters):
        for nx, ny in neighbors:
            dx, dy = nx - px, ny - py
            dist = math.hypot(dx, dy)
            if dist < _EPS:
                continue
            ux, uy = dx / dist, dy / dist            # unit vector toward the neighbor
            mx, my = (px + nx) / 2.0, (py + ny) / 2.0  # bisector midpoint
            slack = (tx - mx) * ux + (ty - my) * uy + radius
            if slack > 0.0:
                tx -= slack * ux
                ty -= slack * uy
    return (tx, ty)


def _safety_cap(px: float, py: float, neighbors, diameter_mm: float):
    """Largest forward step that cannot overlap the nearest neighbor this tick.

    Half the slack to the nearest neighbor (less a small epsilon): even if that
    neighbor moves toward this piece by the same amount, the pair stays at least
    one diameter apart. Returns None when nothing is near enough to constrain.
    """
    if not neighbors:
        return None
    nearest = min(math.hypot(nx - px, ny - py) for nx, ny in neighbors)
    return max(0.0, (nearest - diameter_mm) / 2.0 - _SAFETY_EPS_MM)


def _drive_to_point(x, y, theta, tx, ty, dt_s, tol_mm, max_step_mm):
    """Wheel speeds moving a diff-drive piece toward (tx, ty) without leaving its cell.

    Rotate in place until roughly aligned, then drive straight, capping speed so a
    step never overshoots the in-cell target or the neighbor safety cap. Rotation
    in place does not translate, so it is always safe and is never capped.
    """
    dx, dy = tx - x, ty - y
    dist = math.hypot(dx, dy)
    if dist < tol_mm:
        return 0.0, 0.0
    error = (math.atan2(dy, dx) - theta + math.pi) % (2.0 * math.pi) - math.pi
    v_max = MAX_WHEEL_SPEED_MMPS
    if abs(error) > ALIGN_THRESHOLD_RAD:
        turn = v_max if error > 0.0 else -v_max
        return -turn, turn
    forward = min(v_max, dist / dt_s)
    if max_step_mm is not None:
        forward = min(forward, max(0.0, max_step_mm) / dt_s)
    return forward, forward


def drive_bot_to(bot, target, positions, radius, diameter_mm, rng_mm,
                 tol_mm, iters, dt_s) -> None:
    """Drive one piece a single tick toward target using the BVC collision-free step.

    Gathers in-range neighbor positions, projects target into this piece's buffered
    Voronoi cell, applies the per-tick safety speed cap, steers, and integrates the
    piece's pose in place.
    """
    px, py = positions[bot.id]
    neighbors = [pos for bid, pos in positions.items()
                 if bid != bot.id
                 and (pos[0] - px) ** 2 + (pos[1] - py) ** 2 <= rng_mm ** 2]
    tx, ty = buffered_voronoi_target((px, py), target, neighbors, radius, iters)
    cap = _safety_cap(px, py, neighbors, diameter_mm)
    v_l, v_r = _drive_to_point(bot.x, bot.y, bot.theta, tx, ty, dt_s, tol_mm, cap)
    bot.x, bot.y, bot.theta = step_pose(bot.x, bot.y, bot.theta, v_l, v_r, dt_s)
