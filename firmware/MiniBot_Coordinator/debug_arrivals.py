import random, os

os.chdir(r"C:\Users\DDeGo\Documents\Drew\Projects\MiniBot\firmware\MiniBot_Coordinator")
random.seed(42)
from config import PIECES
from planning.swarm_planner import (
    _optimise_targets_optimal,
    SwarmPlanner,
    _ARRIVE,
    _dist,
)

home = PIECES.HOME_POSITIONS
positions = {
    pid: (random.uniform(-50, 450), random.uniform(25, 375)) for pid in range(1, 17)
}
targets_orig = {
    pid: (float(home[pid][0]), float(home[pid][1]))
    for pid in range(1, 17)
    if pid in home
}
effective_targets = _optimise_targets_optimal(positions, targets_orig)
planner = SwarmPlanner()
commands = planner.plan_moves(positions, effective_targets, None)

# Track when each piece reached its goal
first_arrival = {}
pos = dict(positions)
for cmd in commands:
    pos[cmd.piece_id] = (cmd.target_x_mm, cmd.target_y_mm)
    pid = cmd.piece_id
    if (
        pid in effective_targets
        and _dist(pos[pid], effective_targets[pid]) < 2.0
        and pid not in first_arrival
    ):
        first_arrival[pid] = cmd.sequence_num

print("First arrival seq for each piece:")
for pid in sorted(first_arrival.keys()):
    t = effective_targets[pid]
    print(f"  0x{pid:02X}: seq={first_arrival[pid]} goal=({t[0]:.0f},{t[1]:.0f})")
print(
    "Never arrived:",
    [f"0x{p:02X}" for p in effective_targets if p not in first_arrival],
)
