"""Self-contained PIBT + LNS + BVC motion pipeline vendored for the MiniBot
coordinator.

This subpackage is a port of an external swarm-motion project's tested planner
and collision-free executor. It has no dependency on that project; only the
Python standard library plus this coordinator's ``planning.base_planner`` and
``config``. The public entry point is ``planning.swarm_planner.SwarmPlanner``.
"""
