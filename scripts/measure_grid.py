"""Measure where the printed 16x16 map sits in the Loco (LPS) frame. No flight.

You place the drone (motors off) on the S cell, then on the G cell. The script
averages the position estimate at each and computes the grid origin and
rotation for config.py. If CELL_SIZE_M is set (tape-measured, preferred) it is
used; the L estimated from the S-G distance is printed as a cross-check.

Usage:  python scripts/measure_grid.py
"""
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

import config
from drone import cf_link, coords, maps

POS = [("stateEstimate.x", "float"), ("stateEstimate.y", "float"), ("stateEstimate.z", "float")]


def measure(scf, label):
    input(f"\nPlace the drone centred on {label}, facing the same way as at take-off, hands off. Press Enter...")
    print("  settling estimate...")
    cf_link.wait_for_position_estimator(scf, verbose=False)
    p = cf_link.read_average(scf, POS, seconds=3.0)
    x, y, z = p["stateEstimate.x"], p["stateEstimate.y"], p["stateEstimate.z"]
    print(f"  {label}: x={x:.3f} y={y:.3f} z={z:.3f}")
    return x, y


def main():
    cflib.crtp.init_drivers()
    with SyncCrazyflie(config.URI, cf=Crazyflie(rw_cache=os.path.join(ROOT, "cache"))) as scf:
        print(f"Connected to {config.URI}")
        s_xy = measure(scf, f"S {maps.START}")
        g_xy = measure(scf, f"G {maps.GOAL}")

    origin, theta, L_used, L_est = coords.origin_from_two_cells(maps.START, s_xy, maps.GOAL, g_xy, config.CELL_SIZE_M)
    print(f"\nS-G distance gives L = {L_est:.3f} m per cell", end="")
    print(f" (config CELL_SIZE_M = {config.CELL_SIZE_M})" if config.CELL_SIZE_M else " (CELL_SIZE_M not set - using this)")
    if config.CELL_SIZE_M and abs(L_est - config.CELL_SIZE_M) / config.CELL_SIZE_M > 0.15:
        print("WARNING: >15% disagreement - check the tape measurement or the LPS accuracy.")
    print(f"Rotation = {math.degrees(theta):.1f} deg")
    print("\nPaste into config.py:")
    print(f"CELL_SIZE_M = {L_used:.4f}")
    print(f"GRID_ORIGIN_XY = ({origin[0]:.4f}, {origin[1]:.4f})")
    print(f"GRID_ROTATION_RAD = {theta:.4f}   # {math.degrees(theta):.1f} deg")
    if abs(math.degrees(theta)) < 3:
        print("(Rotation under 3 deg - you may set GRID_ROTATION_RAD = 0.0 if the map is square to the tent.)")
    for cell in (maps.START, maps.GOAL, (9, 2), (9, 15)):
        x, y, _ = coords.cell_to_world(cell, L_used, origin, theta, 0)
        print(f"   check cell {cell}: expected world ({x:.3f}, {y:.3f})")
    out = os.path.join(ROOT, "data", "grid_measurement.txt")
    with open(out, "w") as f:
        f.write(f"S measured {s_xy}\nG measured {g_xy}\nL from S-G = {L_est:.4f}\n")
        f.write(f"CELL_SIZE_M = {L_used:.4f}\nGRID_ORIGIN_XY = ({origin[0]:.4f}, {origin[1]:.4f})\n")
        f.write(f"GRID_ROTATION_RAD = {theta:.4f}\n")
    print(f"Saved to {os.path.relpath(out, ROOT)}")


if __name__ == "__main__":
    main()
