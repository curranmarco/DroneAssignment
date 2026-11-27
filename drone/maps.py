"""16x16 occupancy grid (the original map from Assignment 3, no obstacles).

Convention (matches Assignment3.m and the Assignment 4 brief):
  * cells are 1-based (row, col); row 1 is the TOP of the printed map
  * grid value 0 = free, 1 = blocked
  * S = (15, 2), G = (15, 15)
"""
import numpy as np

N = 16
START = (15, 2)
GOAL = (15, 15)


def original_map():
    """Prescribed map: three horizontal corridors (rows 2, 5, 9) between
    columns 2-15, and two vertical corridors (columns 2 and 15) between rows 2-15."""
    grid = np.ones((N, N), dtype=int)
    for r in (2, 5, 9):
        grid[r - 1, 1:15] = 0          # cols 2..15
    for c in (2, 15):
        grid[1:15, c - 1] = 0          # rows 2..15
    return grid


def in_bounds(cell):
    r, c = cell
    return 1 <= r <= N and 1 <= c <= N


def is_free(grid, cell):
    return in_bounds(cell) and grid[cell[0] - 1, cell[1] - 1] == 0
