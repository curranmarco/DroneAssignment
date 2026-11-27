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
    from drone.surface import thresholds_from_calibration
    light, dark = load("light"), load("dark")
    r = thresholds_from_calibration(light, dark)
    for lab, v in (("light", light), ("dark", dark)):
        print(f"{lab:5s}: median={median(v):.1f} sd={pstdev(v):.1f} range={min(v)}-{max(v)} n={len(v)}"
              f"  ({sum(x >= 8191 for x in v)} at the sensor maximum 8191)")
    print(f"T = (median_light + median_dark) / 2 = {r['T']:.1f}; dark has higher shutter: {r['dark_has_higher_shutter']}")
    if r["overlap"]:
        print("WARNING: the light and dark distributions OVERLAP - shutter-only classification is")
        print("unreliable. Report this and ask the demonstrator before autonomous route following.")
    else:
        print("No overlap between the two distributions.")
        print("\nPaste into config.py:")
        print(f"SHUTTER_ENTER_DARK = {r['enter_dark']:.1f}")
        print(f"SHUTTER_EXIT_DARK = {r['exit_dark']:.1f}")
        print(f"DARK_HAS_HIGHER_SHUTTER = {r['dark_has_higher_shutter']}")
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(7, 4))
        ax.hist(light, bins=30, alpha=0.6, label="light surface")
        ax.hist(dark, bins=30, alpha=0.6, label="dark surface")
        for v, lab, st in ((r["T"], "T", "-"), (r["enter_dark"], "enter dark", "--"), (r["exit_dark"], "exit dark", ":")):
            ax.axvline(v, color="k", linestyle=st, label=f"{lab} = {v:.0f}")
        ax.set_xlabel("motion.shutter"); ax.set_ylabel("samples"); ax.legend(fontsize=8)
        ax.set_title("Shutter calibration: light vs dark")
        fig.tight_layout(); fig.savefig(os.path.join(DATA, "shutter_calibration.png"), dpi=130)
        print("Histogram saved to data/shutter_calibration.png")
    except Exception as e:
        print("Plot skipped:", e)
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
