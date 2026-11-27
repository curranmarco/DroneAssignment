"""Grid cell (row, col) <-> world position (x, y, z) in the Loco (LPS) frame.

Grid frame (from the brief): origin at the bottom-left corner of the printed
map, x along the columns, y up the map (towards row 1). For 1-based (r, c):

    xg = (c - 1/2) L        yg = (N - r + 1/2) L

World frame: rotate by theta and translate by the measured grid origin (x0, y0):

    [xw]   [x0]   [cos t  -sin t] [xg]
    [yw] = [y0] + [sin t   cos t] [yg]

z is the flight height, in the same vertical frame as stateEstimate.z.
"""
import math

from drone.maps import N


def cell_to_grid_xy(cell, L):
    r, c = cell
    return (c - 0.5) * L, (N - r + 0.5) * L


def grid_to_world(xg, yg, origin, theta):
    x0, y0 = origin
    ct, st = math.cos(theta), math.sin(theta)
    return x0 + ct * xg - st * yg, y0 + st * xg + ct * yg


def world_to_grid(xw, yw, origin, theta):
    x0, y0 = origin
    dx, dy = xw - x0, yw - y0
    ct, st = math.cos(theta), math.sin(theta)
    return ct * dx + st * dy, -st * dx + ct * dy


def cell_to_world(cell, L, origin, theta, z):
    xg, yg = cell_to_grid_xy(cell, L)
    xw, yw = grid_to_world(xg, yg, origin, theta)
    return xw, yw, z


def world_to_cell(xw, yw, L, origin, theta):
    """Nearest (row, col) to a world point (may be outside 1..N)."""
    xg, yg = world_to_grid(xw, yw, origin, theta)
    c = int(math.floor(xg / L)) + 1
    r = N - int(math.floor(yg / L))
    return r, c


def route_to_waypoints(route, L, origin, theta, z):
    return [cell_to_world(cell, L, origin, theta, z) for cell in route]


def grid_corners_world(L, origin, theta):
    """World coordinates of the 4 outer corners of the 16x16 map (for bounds/plots)."""
    side = N * L
    return [grid_to_world(x, y, origin, theta) for x, y in ((0, 0), (side, 0), (side, side), (0, side))]


def origin_from_two_cells(cell_a, world_a, cell_b, world_b, L=None):
    """Recover grid origin and rotation from two cells whose world positions were
    measured (e.g. S and G). If L is None it is estimated from their distance.
    Returns (origin, theta, L_used, L_estimated)."""
    ga = cell_to_grid_xy(cell_a, 1.0)
    gb = cell_to_grid_xy(cell_b, 1.0)
    grid_dist_cells = math.hypot(gb[0] - ga[0], gb[1] - ga[1])
    wdx, wdy = world_b[0] - world_a[0], world_b[1] - world_a[1]
    L_est = math.hypot(wdx, wdy) / grid_dist_cells
    L_used = L if L else L_est
    theta = math.atan2(wdy, wdx) - math.atan2(gb[1] - ga[1], gb[0] - ga[0])
    theta = math.atan2(math.sin(theta), math.cos(theta))      # wrap to [-pi, pi]
    xg, yg = ga[0] * L_used, ga[1] * L_used
    ct, st = math.cos(theta), math.sin(theta)
    origin = (world_a[0] - (ct * xg - st * yg), world_a[1] - (st * xg + ct * yg))
    return origin, theta, L_used, L_est
