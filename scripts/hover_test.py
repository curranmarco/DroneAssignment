"""First flight test: take off, hover in place, land - and log everything to diagnose drift.
Usage:  python scripts/hover_test.py

Before running: put a piece of tape on the floor under the drone, so you can
measure how far it *actually* drifted and compare it with the logged estimate.
Output: data/hover_<time>.csv and data/hover_<time>.png
"""
import csv, os, sys, time
from statistics import mean
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from config import URI
from drone import cf_link

# Conservative test values - confirm with the demonstrator before flying.
HOVER_HEIGHT_M = 0.3
HOVER_TIME_S = 10.0
VELOCITY_MPS = 0.2
MIN_BATTERY_V = 3.8
DATA = os.path.join(ROOT, "data")


class DiagLog:
    """Second log block: estimator variance, flow-deck raw data and height (18 bytes, under the 26-byte limit)."""
    FIELDS = [("kalman.varPX", "float"), ("kalman.varPY", "float"), ("kalman.varPZ", "float"),
              ("range.zrange", "uint16_t"), ("motion.deltaX", "int16_t"), ("motion.deltaY", "int16_t")]

    def __init__(self, cf):
        self.latest = {}
        self.cfg = LogConfig(name="Diag", period_in_ms=50)
        for name, kind in self.FIELDS:
            self.cfg.add_variable(name, kind)
        cf.log.add_config(self.cfg)
        self.cfg.data_received_cb.add_callback(lambda ts, d, lc: self.latest.update(d))

    def start(self): self.cfg.start()
    def stop(self):
        try: self.cfg.stop()
        except Exception: pass


def summarise(rows, x0, y0):
    """Print drift statistics for the steady hover part (after the take-off has settled)."""
    hover = [r for r in rows if r["phase"] == "hover"]
    if not hover:
        print("No hover samples recorded.")
        return
    dxy = [((r["x"] - x0) ** 2 + (r["y"] - y0) ** 2) ** 0.5 for r in hover]
    first, last = hover[0], hover[-1]
    print("\n--- Hover summary (from the onboard ESTIMATE, not ground truth) ---")
    print(f"mean horizontal error from start point : {mean(dxy):.3f} m   max: {max(dxy):.3f} m")
    print(f"estimate moved during hover            : dx={last['x'] - first['x']:+.3f} m  dy={last['y'] - first['y']:+.3f} m")
    print(f"height: mean z={mean(r['z'] for r in hover):.3f} m  (target {HOVER_HEIGHT_M})  "
          f"flow-deck range mean={mean(r['zrange'] for r in hover):.0f} mm")
    print(f"motion.squal: mean={mean(r['squal'] for r in hover):.0f}  min={min(r['squal'] for r in hover)}")
    print(f"kalman var x/y: max {max(r['varPX'] for r in hover):.4f} / {max(r['varPY'] for r in hover):.4f}")
    print("\nCompare with the tape: if the drone physically drifted but the estimate above stayed near 0,")
    print("the *position estimate* is wrong (LPS / flow deck). If the estimate also wandered, it is a")
    print("*control / mechanical* problem (props, motors, battery placement, IMU calibration).")


def plot(rows, x0, y0, path):
    import matplotlib.pyplot as plt
    t = [r["t"] for r in rows]
    fig, ax = plt.subplots(4, 1, figsize=(8, 9), sharex=True)
    ax[0].plot(t, [r["x"] - x0 for r in rows], label="x - x0")
    ax[0].plot(t, [r["y"] - y0 for r in rows], label="y - y0")
    ax[0].set_ylabel("drift (m)")
    ax[1].plot(t, [r["z"] for r in rows], label="stateEstimate.z")
    ax[1].plot(t, [r["zrange"] / 1000 for r in rows], label="range.zrange (m)", alpha=0.7)
    ax[1].axhline(HOVER_HEIGHT_M, ls="--", c="k", lw=0.8)
    ax[1].set_ylabel("height (m)")
    ax[2].plot(t, [r["squal"] for r in rows], label="motion.squal")
    ax[2].set_ylabel("squal")
    ax[3].plot(t, [r["varPX"] for r in rows], label="kalman.varPX")
    ax[3].plot(t, [r["varPY"] for r in rows], label="kalman.varPY")
    ax[3].set_ylabel("variance")
    ax[3].set_xlabel("time (s)")
    for a in ax:
        a.legend(loc="upper right"); a.grid(alpha=0.3)
    fig.suptitle("Hover test diagnostics")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    print(f"Saved plot to {path}")
    plt.show()
def wait_for_position_estimator(scf, timeout_s=30.0):
    """Reset the Kalman estimator and wait until the position variance settles.

    Prints the variance spread every sample so you can see whether it is converging.
    Settled = spread of the last 10 samples (5 s) below 0.001 for x, y and z.
    """
    scf.cf.param.set_value("kalman.resetEstimation", "1")
    time.sleep(0.1)
    scf.cf.param.set_value("kalman.resetEstimation", "0")

    names = ("kalman.varPX", "kalman.varPY", "kalman.varPZ")
    cfg = LogConfig(name="Kalman", period_in_ms=500)
    for v in names:
        cfg.add_variable(v, "float")
    hist = {v: [] for v in names}
    start = time.monotonic()
    with SyncLogger(scf, cfg) as logger:
        for _, data, _ in logger:
            for v in names:
                hist[v] = (hist[v] + [data[v]])[-10:]
            spread = {v: max(h) - min(h) for v, h in hist.items()}
            t = time.monotonic() - start
            print(f"  t={t:4.1f}s  var x={data['kalman.varPX']:.4f} y={data['kalman.varPY']:.4f} "
                  f"z={data['kalman.varPZ']:.4f} | spread x={spread[names[0]]:.4f} "
                  f"y={spread[names[1]]:.4f} z={spread[names[2]]:.4f}")
            if all(len(h) == 10 for h in hist.values()) and all(sp < 0.001 for sp in spread.values()):
                return True
            if t > timeout_s:
                print("  -> Not settled. If x/y variance keeps GROWING, the drone is not")
                print("     receiving UWB data (check loco.mode vs anchor mode, Loco tab).")
                print("     If it is small but bouncing, positioning is noisy - ask staff.")
                return False


def safe_land(cf, duration_s=3.0):
    """Controlled landing via the onboard high-level commander (not a motor cut)."""
    try:
        cf.high_level_commander.land(0.0, duration_s)
        time.sleep(duration_s + 0.5)
        cf.high_level_commander.stop()
    except Exception as e:
        print(f"Landing command failed: {e!r}")


def imu_is_alive(scf, seconds=1.0):
    """Return True if accelerometer readings are changing and roughly 1 g on z.

    A frozen IMU sends the exact same numbers every sample; that must never fly.
    """
    cfg = LogConfig(name="Imu", period_in_ms=50)
    for v in ("acc.x", "acc.y", "acc.z"):
        cfg.add_variable(v, "float")
    samples = []
    start = time.monotonic()
    with SyncLogger(scf, cfg) as logger:
        for _, d, _ in logger:
            samples.append((d["acc.x"], d["acc.y"], d["acc.z"]))
            if time.monotonic() - start >= seconds:
                break
    if len(samples) < 5:
        print("IMU check: too few samples received.")
        return False
    changing = len(set(samples)) > 1
    mean_z = sum(s[2] for s in samples) / len(samples)
    print(f"IMU check: {len(samples)} samples, changing={changing}, mean acc.z={mean_z:.2f} g")
    return changing and 0.8 < mean_z < 1.2


def main():
    cflib.crtp.init_drivers()
    rows = []
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache=os.path.join(ROOT, "cache"))) as scf:
        cf = scf.cf
        print(f"Connected to {URI}")

        if not cf_link.check_decks(cf):
            sys.exit("Deck missing - do not fly. Ask staff.")
        print(f"Positioning mode: {cf_link.read_positioning_mode(cf)}")

        vbat = cf_link.read_battery(scf)
        print("Estimator (stabilizer.estimator, 2 = Kalman):", cf.param.get_value("stabilizer.estimator"))
        try:
            print("Loco mode (loco.mode):", cf.param.get_value("loco.mode"))
        except Exception as e:
            print("Could not read loco.mode:", e)

        vbat = read_once(scf, [("pm.vbat", "float")])["pm.vbat"]
        print(f"Battery: {vbat:.2f} V")
        if vbat < MIN_BATTERY_V:
            sys.exit("Battery too low - charge/replace before flying.")

        if not imu_is_alive(scf):
            sys.exit("IMU readings frozen or implausible - power-cycle the drone and check it in cfclient. Do not fly.")

        print("Waiting for position estimate to settle...")
        if not cf_link.wait_for_position_estimator(scf):
            sys.exit("Position estimate did not settle - check Loco anchors. Do not fly.")

        x0, y0, z0 = cf_link.read_position(scf)
        print(f"Start position: x={x0:.2f} y={y0:.2f} z={z0:.2f}")
        print(f"Plan: take off to {HOVER_HEIGHT_M} m, hover {HOVER_TIME_S} s at this x/y, land.")

        if input("Area clear and demonstrator approved? Type FLY to take off: ").strip() != "FLY":
            sys.exit("Aborted - no take-off.")

        monitor = cf_link.FlightMonitor(cf)
        diag = DiagLog(cf)
        monitor.start(); diag.start()
        phase = {"name": "ground"}
        t0 = time.monotonic()

        def sample():
            d, g = monitor.latest(), diag.latest
            if d and g:
                rows.append({"t": round(time.monotonic() - t0, 3), "phase": phase["name"],
                             "x": d["stateEstimate.x"], "y": d["stateEstimate.y"], "z": d["stateEstimate.z"],
                             "vbat": d["pm.vbat"], "shutter": d["motion.shutter"], "squal": d["motion.squal"],
                             "zrange": g["range.zrange"], "deltaX": g["motion.deltaX"], "deltaY": g["motion.deltaY"],
                             "varPX": g["kalman.varPX"], "varPY": g["kalman.varPY"], "varPZ": g["kalman.varPZ"]})

        def record_for(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                sample(); time.sleep(0.05)

        # Newer firmware requires arming before flight.
        cf.supervisor.send_arming_request(True)
        time.sleep(1.0)
        try:
            with PositionHlCommander(scf, x=x0, y=y0, z=z0,
                                     default_height=HOVER_HEIGHT_M,
                                     default_velocity=VELOCITY_MPS) as pc:
                # Take-off happens on entering this block.
                phase["name"] = "settle"
                pc.go_to(x0, y0, HOVER_HEIGHT_M)
                record_for(2.0)               # let take-off transients die out
                phase["name"] = "hover"
                record_for(HOVER_TIME_S)
                phase["name"] = "land"
                # Landing happens on leaving this block (also on Ctrl+C / errors).
                time.sleep(HOVER_TIME_S)
                # Landing happens on leaving this block (also on Ctrl+C).
        except KeyboardInterrupt:
            print("Ctrl+C - landing.")
            safe_land(cf)
        except Exception as e:
            # Never cut motors in the air: land first, then report.
            print(f"ERROR during flight: {e!r} - landing.")
            safe_land(cf)
            raise
        finally:
            time.sleep(0.5)
            cf.supervisor.send_arming_request(False)
            monitor.stop(); diag.stop()
        print("Landed. Wait for motors to stop before touching the drone.")

    if not rows:
        sys.exit("No telemetry recorded.")
    os.makedirs(DATA, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    path = os.path.join(DATA, f"hover_{stamp}.csv")
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader(); w.writerows(rows)
    print(f"Saved {len(rows)} samples to {path}")
    summarise(rows, x0, y0)
    plot(rows, x0, y0, os.path.join(DATA, f"hover_{stamp}.png"))


if __name__ == "__main__":
    main()
