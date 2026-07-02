"""Optimal min-cost assignment for small interchangeable-piece groups.

Same-color, same-rank pieces are interchangeable, so a reset should send each to
the nearest free home square overall, not to one fixed square per id. This solves
that as a min-cost perfect matching with a bitmask DP over the target columns -
O(n^2 * 2^n), which is trivial for the small groups here (at most 8 pawns).
"""
_INF = float("inf")


def min_cost_assignment(cost: list) -> list:
    """Return assignment[i] = j minimizing sum(cost[i][j]) over a square matrix.

    cost is an n-by-n list of lists (row = piece, column = target). Returns a list
    mapping each row to a distinct column.
    """
    n = len(cost)
    if n == 0:
        return []
    size = 1 << n
    dp = [_INF] * size
    parent = [(-1, -1)] * size
    dp[0] = 0.0
    for mask in range(size):
        if dp[mask] == _INF:
            continue
        row = bin(mask).count("1")   # next piece to place
        if row >= n:
            continue
        for col in range(n):
            if mask & (1 << col):
                continue
            nxt = mask | (1 << col)
            candidate = dp[mask] + cost[row][col]
            if candidate < dp[nxt]:
                dp[nxt] = candidate
                parent[nxt] = (mask, col)
    assignment = [-1] * n
    mask = size - 1
    for row in range(n - 1, -1, -1):
        prev_mask, col = parent[mask]
        assignment[row] = col
        mask = prev_mask
    return assignment
