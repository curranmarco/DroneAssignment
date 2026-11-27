"""Hover test: take off, hover in place, land - and log everything to diagnose drift.

Usage:  python scripts/hover_test.py [--height 0.2] [--time 10] [--label white]

Before running: put a piece of tape on the floor next to the drone so you can
measure how far it ACTUALLY drifted and compare it with the logged estimate.
Output: data/hover_<time>_<label>.csv, data/hover_<time>_<label>.png

Safety: same pre-flight checks as main.py (decks, battery, IMU alive, estimator
settled), type FLY to take off, Ctrl+C = controlled landing. The drone also
lands by itself if telemetry stops, the radio link drops, or the battery
falls below FLIGHT_MIN_BATTERY_V in flight. Motors are never cut in the air.
"""
import argparse
import csv
import os
import sys
import time
from statistics import mean

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

import config
from drone import cf_link

DATA = os.path.join(ROOT, "data")


class DiagLog:
    """Extra log block: estimator variance, raw flow and Flow Deck height (18 bytes, 10 Hz)."""
    FIELDS = [("kalman.varPX", "float"), ("kalman.varPY", "float"), ("kalman.varPZ", "float"),
              ("motion.deltaX", "int16_t"), ("motion.deltaY", "int16_t")]

    def __init__(self, cf):
        self.latest = {}
        self.cfg = LogConfig(name="Diag", period_in_ms=100)
        toc = cf.log.toc
        for name, kind in self.FIELDS:
            if toc.get_element_by_complete_name(name) is not None:
                self.cfg.add_variable(name, kind)
        cf.log.add_config(self.cfg)
        self.cfg.data_received_cb.add_callback(lambda ts, d, lc: self.latest.update(d))

    def start(self):
        self.cfg.start()

    def stop(self):
        try:
            self.cfg.stop()
        except Exception:
            pass


def safety_check(tel):
    """Raise if it is no longer safe to keep hovering."""
    if tel.link_lost:
        raise RuntimeError("radio link lost")
    if tel.age() > config.TELEMETRY_TIMEOUT_S:
        raise RuntimeError(f"telemetry stale ({tel.age():.2f} s)")
    vbat = tel.get("pm.vbat")
    if vbat is not None and vbat < config.FLIGHT_MIN_BATTERY_V:
        raise RuntimeError(f"battery low in flight ({vbat:.2f} V)")
    z = tel.get("stateEstimate.z")
    if z is not None and z > config.MAX_HEIGHT_M:
        raise RuntimeError(f"too high ({z:.2f} m)")


def summarise(rows, x0, y0, height):
    hover = [r for r in rows if r["phase"] == "hover"]
    if not hover:
        print("No hover samples recorded.")
        return
    dxy = [((r["x"] - x0) ** 2 + (r["y"] - y0) ** 2) ** 0.5 for r in hover]
    first, last = hover[0], hover[-1]
    print("\n--- Hover summary (from the onboard ESTIMATE, not ground truth) ---")
    print(f"horizontal distance from start : mean {mean(dxy):.3f} m   max {max(dxy):.3f} m")
    print(f"estimate moved during hover    : dx={last['x'] - first['x']:+.3f} m  dy={last['y'] - first['y']:+.3f} m")
    print(f"height: mean z={mean(r['z'] for r in hover):.3f} m (target {height})  "
          f"Flow Deck range mean={mean(r['zrange'] for r in hover):.0f} mm")
    print(f"motion.squal : mean {mean(r['squal'] for r in hover):.0f}  min {min(r['squal'] for r in hover)}"
          "   (Flow Deck tracking quality: low = little texture/light under the drone)")
    print(f"motion.shutter: mean {mean(r['shutter'] for r in hover):.0f}  (8191 = saturated, too dark)")
    print(f"battery: {first['vbat']:.2f} V -> {last['vbat']:.2f} V")
    if hover[0].get("varPX") is not None:
        print(f"kalman var x/y max: {max(r['varPX'] for r in hover):.4f} / {max(r['varPY'] for r in hover):.4f}")
    print("\nCompare with the tape: if the drone physically drifted but the estimate stayed near the")
    print("start, the POSITION ESTIMATE is wrong (Flow Deck / Loco). If the estimate wandered the same")
    print("way the drone did, the estimate is fine and it is a control/mechanical issue.")


def plot(rows, x0, y0, height, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t = [r["t"] for r in rows]
    fig, ax = plt.subplots(4, 1, figsize=(8, 9), sharex=True)
    ax[0].plot(t, [r["x"] - x0 for r in rows], label="x - x0")
    ax[0].plot(t, [r["y"] - y0 for r in rows], label="y - y0")
    ax[0].set_ylabel("drift (m)")
    ax[1].plot(t, [r["z"] for r in rows], label="stateEstimate.z")
    ax[1].plot(t, [r["zrange"] / 1000 for r in rows], label="range.zrange (m)", alpha=0.7)
    ax[1].axhline(height, ls="--", c="k", lw=0.8)
    ax[1].set_ylabel("height (m)")
    ax[2].plot(t, [r["squal"] for r in rows], label="motion.squal")
    ax[2].set_ylabel("squal")
    if rows[0].get("varPX") is not None:
        ax[3].plot(t, [r["varPX"] for r in rows], label="kalman.varPX")
        ax[3].plot(t, [r["varPY"] for r in rows], label="kalman.varPY")
    ax[3].set_ylabel("variance"); ax[3].set_xlabel("time (s)")
    # shade the phases
    for a in ax:
        for name, colour in (("takeoff", "0.92"), ("land", "0.85")):
            ts = [r["t"] for r in rows if r["phase"] == name]
            if ts:
                a.axvspan(min(ts), max(ts), color=colour, zorder=0)
        a.legend(loc="upper right", fontsize=8); a.grid(alpha=0.3)
    fig.suptitle("Hover test diagnostics (grey = take-off / landing)")
    fig.tight_layout(); fig.savefig(path, dpi=150); plt.close(fig)
    print(f"Saved plot to {path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--height", type=float, default=config.FLIGHT_HEIGHT_M)
    ap.add_argument("--time", type=float, default=10.0, help="hover seconds (logged)")
    ap.add_argument("--label", default="", help="added to the file name, e.g. white / black")
    args = ap.parse_args()

    cflib.crtp.init_drivers()
    rows = []
    with SyncCrazyflie(config.URI, cf=Crazyflie(rw_cache=os.path.join(ROOT, "cache"))) as scf:
        cf = scf.cf
        print(f"Connected to {config.URI}")
        ok, msg = cf_link.preflight(scf, config.MIN_BATTERY_V)
        if not ok:
            sys.exit("PRE-FLIGHT FAILED: " + msg)

        tel = cf_link.Telemetry(cf)
        diag = DiagLog(cf)
        tel.start(); diag.start()
        time.sleep(0.5)
        x0, y0, z0 = tel.position()
        print(f"Start position: x={x0:.3f} y={y0:.3f} z={z0:.3f}  squal on the ground={tel.get('motion.squal')}")
        print(f"Plan: take off to {args.height} m, hover {args.time} s over this point, land.")

        if input("Area clear and demonstrator approved? Type FLY to take off: ").strip() != "FLY":
            tel.stop(); diag.stop()
            sys.exit("Aborted - no take-off.")

        phase = {"name": "ground"}
        t0 = time.monotonic()

        def sample():
            d = tel.latest
            rows.append({"t": round(time.monotonic() - t0, 3), "phase": phase["name"],
                         "x": d.get("stateEstimate.x"), "y": d.get("stateEstimate.y"), "z": d.get("stateEstimate.z"),
                         "vx": d.get("stateEstimate.vx"), "vy": d.get("stateEstimate.vy"),
                         "vbat": d.get("pm.vbat"), "shutter": d.get("motion.shutter"), "squal": d.get("motion.squal"),
                         "zrange": d.get("range.zrange", 0), "supervisor": cf_link.decode_supervisor(d.get("supervisor.info")),
                         "deltaX": diag.latest.get("motion.deltaX"), "deltaY": diag.latest.get("motion.deltaY"),
                         "varPX": diag.latest.get("kalman.varPX"), "varPY": diag.latest.get("kalman.varPY"),
                         "varPZ": diag.latest.get("kalman.varPZ")})

        def record_for(seconds, check=True):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                if check:
                    safety_check(tel)
                sample()
                time.sleep(0.05)

        hlc = cf.high_level_commander
        flying = False
        reason = "completed"
        try:
            cf.supervisor.send_arming_request(True)
            time.sleep(1.0)
            phase["name"] = "takeoff"
            flying = True
            hlc.takeoff(args.height, config.TAKEOFF_S)
            record_for(config.TAKEOFF_S + 1.0)          # climb + let transients die out
            hlc.go_to(x0, y0, args.height, 0.0, 1.0)     # hold over the start point
            phase["name"] = "hover"
            record_for(args.time)
        except KeyboardInterrupt:
            reason = "Ctrl+C"
        except Exception as e:
            reason = f"aborted: {e}"
        finally:
            if flying:
                print(f"Landing ({reason}).")
                phase["name"] = "land"
                try:
                    hlc.land(0.0, config.LAND_S)
                    record_for(config.LAND_S + 1.0, check=False)
                    hlc.stop()
                except Exception as e:
                    print(f"Landing command failed: {e!r}")
            time.sleep(0.5)
            cf.supervisor.send_arming_request(False)
            tel.stop(); diag.stop()
        print("Landed. Wait for motors to stop before touching the drone.")

    if not rows:
        sys.exit("No telemetry recorded.")
    os.makedirs(DATA, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S") + (f"_{args.label}" if args.label else "")
    path = os.path.join(DATA, f"hover_{stamp}.csv")
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader(); w.writerows(rows)
    print(f"Saved {len(rows)} samples to {path}   (result: {reason})")
    summarise(rows, x0, y0, args.height)
    plot(rows, x0, y0, args.height, os.path.join(DATA, f"hover_{stamp}.png"))


if __name__ == "__main__":
    main()
