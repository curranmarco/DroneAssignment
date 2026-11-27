"""Waypoint execution with position feedback, floor-marking checks and safe stop.

Flight interface: the Crazyflie's onboard high-level commander (takeoff /
go_to / land). Python sends one target at a time and then *watches the
estimated position* - a waypoint only counts as reached when

    e_p = || p_d - p_hat || < tolerance   for `hold_s` seconds,

and a timeout stops the mission if that never happens. A command being sent is
never treated as proof of arrival.

At each checkpoint the Flow Deck classifier must report the route surface.
If it does not, the drone hovers, then tries a small bounded search either
side of the route (perpendicular to the direction of travel, using the known
heading - one shutter sample cannot say left vs right). If the marking is
still not found, route progression stops and the drone lands.

Any of these abort the route and trigger a controlled landing:
  marking lost, waypoint timeout, stale telemetry, radio link lost,
  low battery in flight, position outside the flight volume, Ctrl+C.
"""
import math
import time
from dataclasses import dataclass

from drone.surface import UNKNOWN


class MissionAbort(Exception):
    pass


@dataclass
class MissionSettings:
    height: float
    velocity: float
    tolerance: float
    hold_s: float
    waypoint_timeout_s: float
    takeoff_s: float
    land_s: float
    min_leg_s: float
    flight_min_battery_v: float
    telemetry_timeout_s: float
    bounds: tuple                    # (xmin, xmax, ymin, ymax, zmax)
    route_surface: str               # "DARK" or "LIGHT": which surface is the route
    check_every: int                 # check the marking every N cells (0 = off)
    mark_check_s: float
    mark_required_fraction: float
    search_offset_m: float           # 0 disables the search
    final_hold_s: float = 0.0
    rate_hz: float = 20.0


class Mission:
    def __init__(self, commander, arm, telemetry, classifier, settings,
                 waypoints, cells, log=print):
        """commander: object with takeoff/go_to/land/stop (high-level commander API)
        arm: function arm(bool); telemetry: drone.cf_link.Telemetry (or a fake)
        classifier: SurfaceClassifier or None (sensing disabled)"""
        self.hlc = commander
        self.arm = arm
        self.tel = telemetry
        self.clf = classifier
        self.s = settings
        self.waypoints = waypoints
        self.cells = cells
        self.log = log
        self.events = []                 # (t, event, detail) for the report
        self.t0 = time.monotonic()
        self.flying = False
        self.reached = 0
        self.marking = []                # (cell, state, fraction_on_route)

    # ------------------------------------------------------------- helpers
    def event(self, name, detail=""):
        t = time.monotonic() - self.t0
        self.events.append((round(t, 2), name, detail))
        self.log(f"[{t:6.2f}s] {name} {detail}")

    def _pos(self):
        return self.tel.position()

    def _tick(self):
        """Read the newest shutter value into the classifier and run the safety
        checks. Called at rate_hz while flying."""
        if self.clf is not None:
            state = self.clf.update(self.tel.get("motion.shutter"), self.tel.get("motion.squal"))
            self.tel.extra = {"surface": state}
        self._safety()

    def _safety(self):
        s = self.s
        if getattr(self.tel, "link_lost", False):
            raise MissionAbort("radio link lost")
        if self.tel.age() > s.telemetry_timeout_s:
            raise MissionAbort(f"telemetry stale ({self.tel.age():.2f} s)")
        vbat = self.tel.get("pm.vbat")
        if self.flying and vbat is not None and vbat < s.flight_min_battery_v:
            raise MissionAbort(f"battery low in flight ({vbat:.2f} V)")
        x, y, z = self._pos()
        if None in (x, y, z):
            raise MissionAbort("no position estimate")
        xmin, xmax, ymin, ymax, zmax = s.bounds
        if not (xmin <= x <= xmax and ymin <= y <= ymax and z <= zmax):
            raise MissionAbort(f"outside flight volume ({x:.2f}, {y:.2f}, {z:.2f})")

    def _sleep_tick(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self._tick()
            time.sleep(1.0 / self.s.rate_hz)

    def _wait_reached(self, target, label, timeout=None):
        """Wait until within tolerance of target for hold_s, else MissionAbort."""
        timeout = timeout or self.s.waypoint_timeout_s
        start = time.monotonic()
        inside_since = None
        while True:
            self._tick()
            x, y, z = self._pos()
            err = math.dist((x, y, z), target)
            now = time.monotonic()
            if err < self.s.tolerance:
                inside_since = inside_since or now
                if now - inside_since >= self.s.hold_s:
                    return err, now - start
            else:
                inside_since = None
            if now - start > timeout:
                raise MissionAbort(f"{label} not reached in {timeout:.1f} s (error {err:.3f} m)")
            time.sleep(1.0 / self.s.rate_hz)

    def _go(self, target, label):
        x, y, z = self._pos()
        dist = math.dist((x, y, z), target)
        dur = max(dist / self.s.velocity, self.s.min_leg_s)
        self.hlc.go_to(target[0], target[1], target[2], 0.0, dur, linear=True)
        err, took = self._wait_reached(target, label, timeout=dur + self.s.waypoint_timeout_s)
        return err, took

    def _on_route_fraction(self, seconds):
        """Fraction of classifier states equal to the route surface over a window."""
        states = []
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self._tick()
            states.append(self.clf.state)
            time.sleep(1.0 / self.s.rate_hz)
        valid = [st for st in states if st != UNKNOWN]
        if not valid:
            return 0.0
        return sum(st == self.s.route_surface for st in valid) / len(valid)

    def _check_marking(self, i):
        """Verify the drone is over the route marking at waypoint i."""
        cell = self.cells[i]
        frac = self._on_route_fraction(self.s.mark_check_s)
        if frac >= self.s.mark_required_fraction:
            self.marking.append((cell, "on_route", round(frac, 2)))
            return
        self.event("MARKING_LOST", f"at cell {cell} (on-route fraction {frac:.2f}) - pausing")
        frac = self._on_route_fraction(self.s.mark_check_s)       # pause and re-check
        if frac >= self.s.mark_required_fraction:
            self.event("MARKING_FOUND", "after pause")
            self.marking.append((cell, "found_after_pause", round(frac, 2)))
            return
        if self.s.search_offset_m > 0:
            wp = self.waypoints[i]
            prev = self.waypoints[i - 1] if i > 0 else self.waypoints[min(i + 1, len(self.waypoints) - 1)]
            dx, dy = wp[0] - prev[0], wp[1] - prev[1]
            n = math.hypot(dx, dy) or 1.0
            px, py = -dy / n, dx / n                  # unit vector perpendicular to travel
            for side in (+1, -1):
                tgt = (wp[0] + side * self.s.search_offset_m * px,
                       wp[1] + side * self.s.search_offset_m * py, wp[2])
                self.event("SEARCH", f"offset {side * self.s.search_offset_m:+.2f} m")
                self._go(tgt, "search point")
                frac = self._on_route_fraction(self.s.mark_check_s)
                if frac >= self.s.mark_required_fraction:
                    self.event("MARKING_FOUND", f"at search offset {side:+d}")
                    self.marking.append((cell, f"found_search_{side:+d}", round(frac, 2)))
                    return
            self._go(wp, "return to waypoint")
        self.marking.append((cell, "lost", round(frac, 2)))
        raise MissionAbort(f"route marking not found at cell {cell}")

    def land(self):
        self.event("LANDING")
        try:
            self.hlc.land(0.0, self.s.land_s)
            end = time.monotonic() + self.s.land_s + 1.0
            while time.monotonic() < end:
                time.sleep(0.1)
            self.hlc.stop()
        except Exception as e:
            self.event("LAND_COMMAND_FAILED", repr(e))
        self.flying = False

    # ----------------------------------------------------------------- run
    def run(self):
        """Returns True if G was reached and the drone landed normally."""
        success = False
        try:
            self.arm(True)
            time.sleep(1.0)
            self.event("TAKEOFF", f"to {self.s.height:.2f} m")
            self.flying = True
            x, y, _ = self._pos()
            self.hlc.takeoff(self.s.height, self.s.takeoff_s)
            self._wait_reached((x, y, self.s.height), "take-off height",
                               timeout=self.s.takeoff_s + self.s.waypoint_timeout_s)
            if self.clf is not None:
                self.clf.reset()
            for i, (wp, cell) in enumerate(zip(self.waypoints, self.cells)):
                err, took = self._go(wp, f"waypoint {i} {cell}")
                self.reached = i + 1
                self.event("WAYPOINT", f"{i}/{len(self.waypoints) - 1} {cell} err={err:.3f} m in {took:.1f} s")
                is_check = self.s.check_every and (i % self.s.check_every == 0 or i == len(self.waypoints) - 1)
                if self.clf is not None and is_check:
                    self._check_marking(i)
            if self.s.final_hold_s > 0:
                self.event("HOVER", f"{self.s.final_hold_s:.1f} s at last waypoint")
                self._sleep_tick(self.s.final_hold_s)
            self.event("ROUTE_COMPLETE", f"last cell {self.cells[-1]}")
            success = True
        except MissionAbort as e:
            self.event("ABORT", str(e))
        except KeyboardInterrupt:
            self.event("ABORT", "Ctrl+C")
        except Exception as e:
            self.event("ABORT", f"unexpected error {e!r}")
        finally:
            if self.flying:
                self.land()
            time.sleep(0.5)
            try:
                self.arm(False)
            except Exception:
                pass
        return success
