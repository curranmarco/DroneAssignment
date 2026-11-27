"""Offline test of drone/mission.py against a simulated Crazyflie (no hardware).

The fake drone moves in straight lines towards each go_to target, and its
'shutter' reads DARK when the estimated position is over a route cell and LIGHT
otherwise. Scenarios check normal completion and each abort path.

Usage:  python tests/sim_mission.py
"""
import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from drone import coords, maps, planner
from drone.mission import Mission, MissionSettings
from drone.surface import SurfaceClassifier

L, ORIGIN, THETA, H = 0.10, (0.6, 0.1), math.radians(5), 0.3
DARK_SHUTTER, LIGHT_SHUTTER = 900, 300


class FakeDrone:
    def __init__(self, start, route_cells, speed_scale=1.0):
        self.pos = list(start)
        self.route_cells = set(route_cells)
        self.target, self.v = list(start), 0.0
        self.lock = threading.Lock()
        self.stuck = False            # cannot move (to test waypoint timeout)
        self.drop_telemetry_after = None
        self.offset_marking = None    # (cell) where the marking is missing
        self.t_start = time.monotonic()
        self.armed = False
        self.landed = False
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while True:
            with self.lock:
                d = [t - p for t, p in zip(self.target, self.pos)]
                n = math.sqrt(sum(x * x for x in d))
                step = self.v * 0.01
                if not self.stuck and n > 1e-6:
                    k = min(1.0, step / n)
                    self.pos = [p + k * x for p, x in zip(self.pos, d)]
            time.sleep(0.01)

    # high-level commander API
    def _set(self, x, y, z, dur):
        with self.lock:
            dist = math.dist(self.pos, (x, y, z))
            self.target, self.v = [x, y, z], max(dist / max(dur, 0.05), 0.01)

    def takeoff(self, h, dur):   self._set(self.pos[0], self.pos[1], h, dur)
    def go_to(self, x, y, z, yaw, dur, linear=False): self._set(x, y, z, dur)
    def land(self, h, dur):      self._set(self.pos[0], self.pos[1], h, dur); self.landed = True
    def stop(self):              pass
    def arm(self, on):           self.armed = on

    # telemetry API
    link_lost = False
    extra = {}

    def position(self):
        with self.lock:
            return tuple(self.pos)

    def get(self, name, default=None):
        if name == "pm.vbat":
            return 3.9
        if name == "motion.squal":
            return 100
        if name == "motion.shutter":
            x, y, _ = self.position()
            cell = coords.world_to_cell(x, y, L, ORIGIN, THETA)
            on = cell in self.route_cells and cell != self.offset_marking
            return DARK_SHUTTER if on else LIGHT_SHUTTER
        return default

    def age(self):
        if self.drop_telemetry_after and time.monotonic() - self.t_start > self.drop_telemetry_after:
            return 9.9
        return 0.01


def settings(bounds):
    return MissionSettings(height=H, velocity=1.0, tolerance=0.05, hold_s=0.1,
                           waypoint_timeout_s=1.5, takeoff_s=0.3, land_s=0.2, min_leg_s=0.15,
                           flight_min_battery_v=3.2, telemetry_timeout_s=0.5, bounds=bounds,
                           route_surface="DARK", check_every=1, mark_check_s=0.15,
                           mark_required_fraction=0.6, search_offset_m=0.04)


def run(name, setup=None, expect=True, max_cells=None):
    grid = maps.original_map()
    res = planner.astar(grid, maps.START, maps.GOAL)
    cells = res.route if max_cells is None else res.route[:max_cells + 1]
    wps = coords.route_to_waypoints(cells, L, ORIGIN, THETA, H)
    corners = coords.grid_corners_world(L, ORIGIN, THETA)
    bounds = (min(c[0] for c in corners) - .3, max(c[0] for c in corners) + .3,
              min(c[1] for c in corners) - .3, max(c[1] for c in corners) + .3, 0.8)
    sx, sy, _ = wps[0]
    fake = FakeDrone((sx + 0.02, sy - 0.01, 0.0), res.route)
    if setup:
        setup(fake)
    clf = SurfaceClassifier(700, 500, True, window=3)
    msgs = []
    m = Mission(fake, fake.arm, fake, clf, settings(bounds), wps, cells, log=msgs.append)
    t0 = time.monotonic()
    ok = m.run()
    aborts = [e for e in m.events if e[1] == "ABORT"]
    status = "PASS" if ok == expect and fake.landed and not fake.armed else "FAIL"
    print(f"{status}  {name}: success={ok} reached={m.reached}/{len(wps)} landed={fake.landed} "
          f"disarmed={not fake.armed} {aborts[-1][2] if aborts else ''} ({time.monotonic() - t0:.1f}s)")
    searches = [e for e in m.events if e[1] in ("MARKING_LOST", "SEARCH", "MARKING_FOUND")]
    if searches:
        print("      ", [f"{e[1]} {e[2]}" for e in searches][:6])
    return status == "PASS"


results = [
    run("full route S -> G"),
    run("staged: first 3 cells", max_cells=3),
    run("marking missing at cell (12,2) -> search -> land",
        setup=lambda f: setattr(f, "offset_marking", (12, 2)), expect=False),
    run("drone cannot move -> waypoint timeout -> land",
        setup=lambda f: setattr(f, "stuck", True), expect=False),
    run("telemetry stops -> abort -> land",
        setup=lambda f: setattr(f, "drop_telemetry_after", 2.0), expect=False),
]
print(f"\n{sum(results)}/{len(results)} simulation checks passed")
