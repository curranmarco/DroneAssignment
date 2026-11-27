"""EEP55C21 Assignment 4 - Pathfinding on Crazyflie (entry point).

Examples
  python main.py --plan-only                        # plan + plot, no drone
  python main.py --dry-run                          # connect + all checks, no take-off
  python main.py --max-cells 3                      # staged test: first 3 moves, then land
  python main.py                                    # full route S -> G

Sequence: original map -> A* from S to G -> validate -> world waypoints
-> pre-flight checks -> confirm -> fly with position + floor-marking feedback
-> land -> save logs and plots.
"""
import argparse
import math
import os
import sys
import time

import config
from drone import coords, maps, planner, plots

ROOT = os.path.dirname(os.path.abspath(__file__))


def plan_route():
    grid = maps.original_map()
    t0 = time.monotonic()
    result = planner.astar(grid, maps.START, maps.GOAL)
    planning_s = time.monotonic() - t0
    if not result.found:
        print("NO ROUTE EXISTS from S to G. Nothing will be flown.")
        return grid, result, False
    ok, reason = planner.validate_route(grid, result.route, maps.START, maps.GOAL, result.cost)
    print(f"A*: {result.cost} moves, {len(result.searched)} cells searched, "
          f"{planning_s * 1000:.2f} ms, valid={ok} {reason}")
    return grid, result, ok


def check_physical(L):
    """Physical feasibility notes: the route corridor is one cell wide, so the
    position tolerance should keep the drone over the marking (< L/2)."""
    warnings = []
    if config.WAYPOINT_TOLERANCE_M >= L / 2:
        warnings.append(f"tolerance {config.WAYPOINT_TOLERANCE_M} m >= L/2 = {L / 2:.3f} m: "
                        "the drone could sit off the one-cell-wide marking while 'reached'")
    if config.SEARCH_OFFSET_M > L:
        warnings.append(f"search offset {config.SEARCH_OFFSET_M} m > one cell: search could leave the corridor")
    return warnings


def require(names):
    missing = [n for n in names if getattr(config, n) is None]
    if missing:
        sys.exit("Set these in config.py first (see README): " + ", ".join(missing))


def run_flight(args, result, outdir):
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    from drone import cf_link
    from drone.mission import Mission, MissionSettings
    from drone.surface import SurfaceClassifier

    require(["CELL_SIZE_M", "GRID_ORIGIN_XY"])
    sensing = not args.no_sensing
    if sensing:
        require(["SHUTTER_ENTER_DARK", "SHUTTER_EXIT_DARK"])
    L, origin, theta, H = config.CELL_SIZE_M, config.GRID_ORIGIN_XY, config.GRID_ROTATION_RAD, config.FLIGHT_HEIGHT_M

    cells = result.route if args.max_cells is None else result.route[: args.max_cells + 1]
    waypoints = coords.route_to_waypoints(cells, L, origin, theta, H)
    for w in check_physical(L):
        print("WARNING:", w)

    corners = coords.grid_corners_world(L, origin, theta)
    m = config.BOUNDS_MARGIN_M
    bounds = (min(c[0] for c in corners) - m, max(c[0] for c in corners) + m,
              min(c[1] for c in corners) - m, max(c[1] for c in corners) + m, config.MAX_HEIGHT_M)

    settings = MissionSettings(
        height=H, velocity=config.VELOCITY_MPS, tolerance=config.WAYPOINT_TOLERANCE_M,
        hold_s=config.WAYPOINT_HOLD_S, waypoint_timeout_s=config.WAYPOINT_TIMEOUT_S,
        takeoff_s=config.TAKEOFF_S, land_s=config.LAND_S, min_leg_s=config.MIN_LEG_S,
        flight_min_battery_v=config.FLIGHT_MIN_BATTERY_V, telemetry_timeout_s=config.TELEMETRY_TIMEOUT_S,
        bounds=bounds, route_surface=config.ROUTE_SURFACE, check_every=config.MARK_CHECK_EVERY,
        mark_check_s=config.MARK_CHECK_S, mark_required_fraction=config.MARK_REQUIRED_FRACTION,
        search_offset_m=config.SEARCH_OFFSET_M, final_hold_s=args.hover)
    classifier = (SurfaceClassifier(config.SHUTTER_ENTER_DARK, config.SHUTTER_EXIT_DARK,
                                    config.DARK_HAS_HIGHER_SHUTTER, config.SHUTTER_FILTER_WINDOW,
                                    config.SQUAL_MIN) if sensing else None)

    cflib.crtp.init_drivers()
    with SyncCrazyflie(config.URI, cf=Crazyflie(rw_cache=os.path.join(ROOT, "cache"))) as scf:
        cf = scf.cf
        print(f"Connected to {config.URI}")
        ok, msg = cf_link.preflight(scf, config.MIN_BATTERY_V)
        if not ok:
            sys.exit("PRE-FLIGHT FAILED: " + msg)

        tel = cf_link.Telemetry(cf)
        tel.start()
        try:
            x, y, z = tel.position()
            sx, sy, _ = waypoints[0]
            d = math.hypot(x - sx, y - sy)
            print(f"Estimated position ({x:.3f}, {y:.3f}, {z:.3f}); S cell centre ({sx:.3f}, {sy:.3f}); distance {d:.3f} m")
            if d > config.START_TOLERANCE_M:
                sys.exit(f"Drone is {d:.2f} m from S (> {config.START_TOLERANCE_M} m). Place it on S or "
                         "re-check GRID_ORIGIN_XY / rotation (scripts/measure_grid.py).")
            print(f"Plan: {len(waypoints) - 1} moves at {H} m, {config.VELOCITY_MPS} m/s, "
                  f"tolerance {config.WAYPOINT_TOLERANCE_M} m, sensing={'ON' if sensing else 'OFF'}, "
                  f"route surface={config.ROUTE_SURFACE}")
            if args.dry_run:
                print("Dry run: all checks passed. No take-off.")
                return
            if input("Area clear and demonstrator approved? Type FLY to take off: ").strip() != "FLY":
                print("Aborted - no take-off.")
                return

            mission = Mission(cf.high_level_commander, cf.supervisor.send_arming_request, tel,
                              classifier, settings, waypoints, cells)
            success = mission.run()
        finally:
            tel.stop()

    # ----------------------------------------------------------- save evidence
    tel.save_csv(os.path.join(outdir, "telemetry.csv"))
    with open(os.path.join(outdir, "events.csv"), "w") as f:
        f.write("t,event,detail\n")
        for t, e, dtl in mission.events:
            f.write(f'{t},{e},"{dtl}"\n')
    with open(os.path.join(outdir, "marking_checks.csv"), "w") as f:
        f.write("row,col,result,on_route_fraction\n")
        for (r, c), res, frac in mission.marking:
            f.write(f"{r},{c},{res},{frac}\n")
    with open(os.path.join(outdir, "waypoints.csv"), "w") as f:
        f.write("index,row,col,x,y,z\n")
        for i, ((r, c), (wx, wy, wz)) in enumerate(zip(cells, waypoints)):
            f.write(f"{i},{r},{c},{wx:.4f},{wy:.4f},{wz:.4f}\n")
    plots.plot_trajectory(waypoints, tel.rows, L, origin, theta,
                          "Planned vs estimated trajectory", os.path.join(outdir, "trajectory.png"))
    plots.plot_shutter(tel.rows, config.SHUTTER_ENTER_DARK, config.SHUTTER_EXIT_DARK,
                       "Shutter and classification", os.path.join(outdir, "shutter.png"))
    reached = f"{mission.reached}/{len(waypoints)} waypoints"
    print(f"\nRESULT: {'SUCCESS' if success else 'ABORTED'} ({reached}). Logs and plots in {outdir}")
    if not success:
        last = [e for e in mission.events if e[1] == "ABORT"]
        print("Reason:", last[-1][2] if last else "see events.csv")
        print("Supervisor state at end:", cf_link.decode_supervisor(tel.get("supervisor.info")))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--plan-only", action="store_true", help="plan and plot only, no drone")
    ap.add_argument("--dry-run", action="store_true", help="connect and run all checks, no take-off")
    ap.add_argument("--max-cells", type=int, default=None, help="staged test: fly only the first N moves (0 = take off and hover over S)")
    ap.add_argument("--hover", type=float, default=0.0, help="seconds to hover at the last waypoint before landing")
    ap.add_argument("--no-sensing", action="store_true", help="fly without floor-marking checks (testing only)")
    args = ap.parse_args()

    outdir = os.path.join(ROOT, "data", time.strftime("%Y%m%d_%H%M%S") + "_route")
    os.makedirs(outdir, exist_ok=True)
    grid, result, ok = plan_route()
    plots.plot_plan(grid, result, "Original map - A*", os.path.join(outdir, "plan.png"))
    if config.CELL_SIZE_M and config.GRID_ORIGIN_XY and result.found:
        wps = coords.route_to_waypoints(result.route, config.CELL_SIZE_M, config.GRID_ORIGIN_XY,
                                        config.GRID_ROTATION_RAD, config.FLIGHT_HEIGHT_M)
        print(f"world: S=({wps[0][0]:.3f}, {wps[0][1]:.3f})  G=({wps[-1][0]:.3f}, {wps[-1][1]:.3f})")
    if args.plan_only:
        print(f"plot: {os.path.relpath(outdir, ROOT)}/plan.png")
        return
    if not ok:
        sys.exit("Not flying: no valid route.")
    run_flight(args, result, outdir)


if __name__ == "__main__":
    main()
