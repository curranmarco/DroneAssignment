"""First flight test: take off, hover in place, land. Nothing else.
Usage:  python scripts/hover_test.py
"""
import os, sys, time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.position_hl_commander import PositionHlCommander
from config import URI

# Conservative test values - confirm with the demonstrator before flying.
HOVER_HEIGHT_M = 0.3
HOVER_TIME_S = 10.0
VELOCITY_MPS = 0.2
MIN_BATTERY_V = 3.0


def check_decks(cf):
    flow = int(cf.param.get_value("deck.bcFlow2"))
    loco = int(cf.param.get_value("deck.bcLoco"))
    print(f"Flow Deck V2: {'OK' if flow else 'MISSING'} | Loco Deck: {'OK' if loco else 'MISSING'}")
    return flow and loco


def read_once(scf, variables):
    cfg = LogConfig(name="Once", period_in_ms=100)
    for name, kind in variables:
        cfg.add_variable(name, kind)
    with SyncLogger(scf, cfg) as logger:
        for _, data, _ in logger:
            return data


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
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
        cf = scf.cf
        print(f"Connected to {URI}")

        if not check_decks(cf):
            sys.exit("Deck missing - do not fly. Ask staff.")

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
        if not wait_for_position_estimator(scf):
            sys.exit("Position estimate did not settle - check Loco anchors. Do not fly.")

        pos = read_once(scf, [("stateEstimate.x", "float"),
                              ("stateEstimate.y", "float"),
                              ("stateEstimate.z", "float")])
        x0, y0, z0 = pos["stateEstimate.x"], pos["stateEstimate.y"], pos["stateEstimate.z"]
        print(f"Start position: x={x0:.2f} y={y0:.2f} z={z0:.2f}")
        print(f"Plan: take off to {HOVER_HEIGHT_M} m, hover {HOVER_TIME_S} s at this x/y, land.")

        if input("Area clear and demonstrator approved? Type FLY to take off: ").strip() != "FLY":
            sys.exit("Aborted - no take-off.")

        # Newer firmware requires arming before flight.
        cf.supervisor.send_arming_request(True)
        time.sleep(1.0)

        try:
            with PositionHlCommander(scf, x=x0, y=y0, z=z0,
                                     default_height=HOVER_HEIGHT_M,
                                     default_velocity=VELOCITY_MPS) as pc:
                # Take-off happens on entering this block.
                pc.go_to(x0, y0, HOVER_HEIGHT_M)
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
        print("Landed. Wait for motors to stop before touching the drone.")


if __name__ == "__main__":
    main()
