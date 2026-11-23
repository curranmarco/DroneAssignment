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
HOVER_TIME_S = 5.0
VELOCITY_MPS = 0.2
MIN_BATTERY_V = 3.8


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


def wait_for_position_estimator(scf, timeout_s=15.0):
    """Reset the Kalman estimator and wait until the position variance settles."""
    scf.cf.param.set_value("kalman.resetEstimation", "1")
    time.sleep(0.1)
    scf.cf.param.set_value("kalman.resetEstimation", "0")

    cfg = LogConfig(name="Kalman", period_in_ms=500)
    for v in ("kalman.varPX", "kalman.varPY", "kalman.varPZ"):
        cfg.add_variable(v, "float")
    hist = {v: [] for v in ("kalman.varPX", "kalman.varPY", "kalman.varPZ")}
    start = time.monotonic()
    with SyncLogger(scf, cfg) as logger:
        for _, data, _ in logger:
            for v in hist:
                hist[v] = (hist[v] + [data[v]])[-10:]
            if all(len(h) == 10 and max(h) - min(h) < 0.001 for h in hist.values()):
                return True
            if time.monotonic() - start > timeout_s:
                return False


def main():
    cflib.crtp.init_drivers()
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
        cf = scf.cf
        print(f"Connected to {URI}")

        if not check_decks(cf):
            sys.exit("Deck missing - do not fly. Ask staff.")

        vbat = read_once(scf, [("pm.vbat", "float")])["pm.vbat"]
        print(f"Battery: {vbat:.2f} V")
        if vbat < MIN_BATTERY_V:
            sys.exit("Battery too low - charge/replace before flying.")

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
                # Landing happens on leaving this block (also on Ctrl+C / errors).
        except KeyboardInterrupt:
            print("Ctrl+C - landing triggered.")
        finally:
            cf.supervisor.send_arming_request(False)
        print("Landed. Wait for motors to stop before touching the drone.")


if __name__ == "__main__":
    main()
