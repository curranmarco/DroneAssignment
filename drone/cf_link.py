"""Crazyflie connection helpers: pre-flight checks and background telemetry.

Telemetry load is deliberately light: two log blocks at 20 Hz / 10 Hz
(about 35 bytes per 100 ms in total), far less than cfclient itself streams.
"""
import csv
import threading
import time

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

# Variables streamed during flight. Each block must stay under ~26 bytes.
FAST_VARS = [  # 20 Hz: position, shutter, battery, supervisor state
    ("stateEstimate.x", "float"), ("stateEstimate.y", "float"),
    ("stateEstimate.z", "float"), ("pm.vbat", "float"),
    ("motion.shutter", "uint16_t"), ("motion.squal", "uint8_t"),
    ("supervisor.info", "uint16_t"),
]
SLOW_VARS = [  # 10 Hz: velocity and Flow Deck height
    ("stateEstimate.vx", "float"), ("stateEstimate.vy", "float"),
    ("stateEstimate.vz", "float"), ("range.zrange", "uint16_t"),
]


# ------------------------------------------------------------------ checks
def check_decks(cf):
    flow = int(cf.param.get_value("deck.bcFlow2"))
    loco = int(cf.param.get_value("deck.bcLoco"))
    print(f"Flow Deck V2: {'OK' if flow else 'MISSING'} | Loco Deck: {'OK' if loco else 'MISSING'}")
    return bool(flow and loco)


def read_once(scf, variables):
    cfg = LogConfig(name="Once", period_in_ms=100)
    for name, kind in variables:
        cfg.add_variable(name, kind)
    with SyncLogger(scf, cfg) as logger:
        for _, data, _ in logger:
            return data


def read_average(scf, variables, seconds=2.0):
    """Average numeric log variables over a few seconds (drone stationary)."""
    cfg = LogConfig(name="Avg", period_in_ms=100)
    for name, kind in variables:
        cfg.add_variable(name, kind)
    sums, n = {name: 0.0 for name, _ in variables}, 0
    start = time.monotonic()
    with SyncLogger(scf, cfg) as logger:
        for _, data, _ in logger:
            for k in sums:
                sums[k] += data[k]
            n += 1
            if time.monotonic() - start >= seconds:
                break
    return {k: v / max(n, 1) for k, v in sums.items()}


def imu_is_alive(scf, seconds=1.0):
    """True if accelerometer readings change and show ~1 g on z (not frozen)."""
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


def wait_for_position_estimator(scf, timeout_s=30.0, verbose=True):
    """Reset the Kalman filter and wait until x/y/z variance stops changing
    (spread of last 10 samples at 2 Hz < 0.001), as in Bitcraze's LPS examples."""
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
            spread = [max(h) - min(h) for h in hist.values()]
            t = time.monotonic() - start
            if verbose:
                print(f"  t={t:4.1f}s var x={data[names[0]]:.4f} y={data[names[1]]:.4f} "
                      f"z={data[names[2]]:.4f} | spread max={max(spread):.4f}")
            if all(len(h) == 10 for h in hist.values()) and max(spread) < 0.001:
                return True
            if t > timeout_s:
                return False


def preflight(scf, min_battery_v):
    """All checks that must pass before arming. Returns (ok, message)."""
    cf = scf.cf
    if not check_decks(cf):
        return False, "Deck missing - do not fly, ask staff."
    print("Estimator (2 = Kalman):", cf.param.get_value("stabilizer.estimator"))
    try:
        print("Loco mode (loco.mode):", cf.param.get_value("loco.mode"))
    except Exception:
        pass
    vbat = read_average(scf, [("pm.vbat", "float")], seconds=1.0)["pm.vbat"]
    print(f"Battery (at rest): {vbat:.2f} V")
    if vbat < min_battery_v:
        return False, (f"Battery {vbat:.2f} V < {min_battery_v} V. Charge it: a low battery "
                       "collapses under motor load and the drone drops after a few seconds.")
    if not imu_is_alive(scf):
        return False, "IMU frozen/implausible - power-cycle the drone."
    print("Waiting for position estimate to settle...")
    if not wait_for_position_estimator(scf):
        return False, "Position estimate did not settle - check Loco anchors / mode."
    return True, "ok"


# ------------------------------------------------------------- telemetry
class Telemetry:
    """Streams FAST_VARS/SLOW_VARS in the background, keeps the latest values,
    and records every sample for the CSV log."""

    def __init__(self, cf):
        self.cf = cf
        self.latest = {}
        self.last_rx = None
        self.rows = []
        self.t0 = time.monotonic()
        self.link_lost = False
        self._lock = threading.Lock()
        self._configs = []
        self.extra = {}            # e.g. classifier state, set by the mission
        cf.connection_lost.add_callback(self._on_lost)
        cf.disconnected.add_callback(self._on_lost)

    def _on_lost(self, *args):
        self.link_lost = True

    def _available(self, variables):
        toc = self.cf.log.toc
        ok = [(n, k) for n, k in variables if toc.get_element_by_complete_name(n) is not None]
        missing = [n for n, _ in variables if (n, _) not in ok]
        if missing:
            print("Telemetry: not in firmware, skipped:", ", ".join(missing))
        return ok

    def start(self):
        for name, variables, period in (("Fast", FAST_VARS, 50), ("Slow", SLOW_VARS, 100)):
            cfg = LogConfig(name=name, period_in_ms=period)
            for n, k in self._available(variables):
                cfg.add_variable(n, k)
            self.cf.log.add_config(cfg)
            cfg.data_received_cb.add_callback(self._on_data)
            cfg.start()
            self._configs.append(cfg)
        # wait for first packet
        deadline = time.monotonic() + 3.0
        while self.last_rx is None and time.monotonic() < deadline:
            time.sleep(0.05)
        if self.last_rx is None:
            raise RuntimeError("No telemetry received")

    def _on_data(self, timestamp, data, logconf):
        now = time.monotonic()
        with self._lock:
            self.latest.update(data)
            self.last_rx = now
            row = {"t": round(now - self.t0, 3), "block": logconf.name}
            row.update(data)
            row.update(self.extra)
            self.rows.append(row)

    def get(self, name, default=None):
        with self._lock:
            return self.latest.get(name, default)

    def position(self):
        with self._lock:
            return (self.latest.get("stateEstimate.x"), self.latest.get("stateEstimate.y"),
                    self.latest.get("stateEstimate.z"))

    def age(self):
        return float("inf") if self.last_rx is None else time.monotonic() - self.last_rx

    def stop(self):
        for cfg in self._configs:
            try:
                cfg.stop()
            except Exception:
                pass

    def save_csv(self, path):
        keys = []
        for row in self.rows:
            for k in row:
                if k not in keys:
                    keys.append(k)
        with open(path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(self.rows)
        return path


def decode_supervisor(info):
    """supervisor.info bitfield (Crazyflie firmware): useful after an unexpected landing."""
    if info is None:
        return "n/a"
    bits = ["canBeArmed", "isArmed", "autoArm", "canFly", "isFlying",
            "isTumbled", "isLocked", "isCrashed", "hlControlActive",
            "hlTrajFinished", "hlControlDisabled"]
    return ",".join(b for i, b in enumerate(bits) if int(info) >> i & 1) or "none"
