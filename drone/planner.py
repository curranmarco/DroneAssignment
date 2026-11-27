"""A* pathfinding ported from Assignment 3 (MATLAB) to Python.

Movement model (unchanged from Assignment 3): 4-connected (up, down, left,
right), cost 1 per move, no diagonals. A* expands the lowest f = g + h with
h = Manhattan distance to G, which never overestimates for unit-cost
4-connected moves, so the route found is the shortest.
"""
import heapq
from dataclasses import dataclass, field

from drone.maps import is_free, in_bounds

MOVES = [(-1, 0), (1, 0), (0, -1), (0, 1)]   # up, down, left, right


@dataclass
class PlanResult:
    found: bool
    route: list = field(default_factory=list)     # [(row, col), ...] S..G
    cost: float = float("inf")                    # number of moves
    searched: list = field(default_factory=list)  # expansion order (for the plot)


def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def _trace(parent, cell):
    route = [cell]
    while cell in parent:
        cell = parent[cell]
        route.append(cell)
    return route[::-1]


def astar(grid, start, goal):
    res = PlanResult(False)
    if not (is_free(grid, start) and is_free(grid, goal)):
        return res
    g = {start: 0}
    parent = {}
    closed = set()
    counter = 0                                  # tie-breaker for the heap
    open_heap = [(manhattan(start, goal), counter, start)]
    while open_heap:
        _, _, cell = heapq.heappop(open_heap)
        if cell in closed:
            continue                             # stale heap entry
        res.searched.append(cell)
        if cell == goal:
            res.found, res.cost, res.route = True, g[cell], _trace(parent, cell)
            return res
        closed.add(cell)
        for dr, dc in MOVES:
            nb = (cell[0] + dr, cell[1] + dc)
            if not is_free(grid, nb) or nb in closed:
                continue
            new_g = g[cell] + 1
            if new_g < g.get(nb, float("inf")):
                g[nb] = new_g
                parent[nb] = cell
                counter += 1
                heapq.heappush(open_heap, (new_g + manhattan(nb, goal), counter, nb))
    return res                                   # open list empty: no route


def validate_route(grid, route, start, goal, cost=None):
    """Same checks as Assignment 3: starts at S, ends at G, every cell inside
    the map and free, consecutive cells exactly one 4-connected move apart,
    and (if given) cost equals the number of moves. Returns (ok, reason)."""
    if not route:
        return False, "route is empty"
    if tuple(route[0]) != tuple(start):
        return False, "route does not begin at S"
    if tuple(route[-1]) != tuple(goal):
        return False, "route does not end at G"
    for i, cell in enumerate(route):
        if not in_bounds(cell):
            return False, f"cell {cell} is outside the map"
        if not is_free(grid, cell):
            return False, f"cell {cell} is blocked"
        if i > 0 and manhattan(cell, route[i - 1]) != 1:
            return False, f"illegal move into {cell}"
    if cost is not None and cost != len(route) - 1:
        return False, "path cost does not match the number of moves"
    return True, ""
