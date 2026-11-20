"""Log motion.shutter over one surface and save it to data/. Does NOT start the motors.

Usage (hold the drone at the approved height over the surface, still):
    python scripts/shutter_calibration.py light
    python scripts/shutter_calibration.py dark
Then run with 'summary' to get medians and a suggested threshold.
"""
import csv, os, sys, time
from statistics import median, pstdev
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
DATA = os.path.join(ROOT, "data")

def load(label):
    with open(os.path.join(DATA, f"shutter_{label}.csv")) as f:
        return [int(r["shutter"]) for r in csv.DictReader(f)]

if len(sys.argv) < 2 or sys.argv[1] not in ("light", "dark", "summary"):
    sys.exit(__doc__)

if sys.argv[1] == "summary":
    light, dark = load("light"), load("dark")
    ml, md = median(light), median(dark)
    print(f"light: median={ml:.1f} sd={pstdev(light):.1f} range={min(light)}-{max(light)} n={len(light)}")
    print(f"dark:  median={md:.1f} sd={pstdev(dark):.1f} range={min(dark)}-{max(dark)} n={len(dark)}")
    print(f"preliminary threshold T = {(ml + md) / 2:.1f}; dark has higher shutter: {md > ml}")
    overlap = max(light) >= min(dark) if md > ml else max(dark) >= min(light)
    print("WARNING: distributions overlap - check with demonstrator" if overlap else "No overlap.")
    sys.exit()

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from config import URI

label, duration = sys.argv[1], 15.0
cflib.crtp.init_drivers()
cfg = LogConfig(name="Surface", period_in_ms=100)
cfg.add_variable("motion.shutter", "uint16_t")
cfg.add_variable("motion.squal", "uint8_t")
cfg.add_variable("range.zrange", "uint16_t")

rows = []
with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
    with SyncLogger(scf, cfg) as logger:
        start = time.monotonic()
        for ts, d, _ in logger:
            t = time.monotonic() - start
            if t >= duration:
                break
            rows.append([round(t, 2), d["motion.shutter"], d["motion.squal"], d["range.zrange"]])
            print(f"t={t:5.1f}s shutter={d['motion.shutter']} squal={d['motion.squal']} height={d['range.zrange']}mm")

os.makedirs(DATA, exist_ok=True)
path = os.path.join(DATA, f"shutter_{label}.csv")
with open(path, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t", "shutter", "squal", "zrange_mm"])
    w.writerows(rows)
print(f"Saved {len(rows)} samples to {path}")
