# DroneAssignment – EEP55C21 Assignment 4: Pathfinding on Crazyflie

Plans the shortest route on the original 16×16 occupancy grid (A*, ported from Assignment 3), converts it to Loco Positioning world waypoints, and flies it with the Crazyflie. The Flow Deck `motion.shutter` reading is used to verify the floor marking at every cell; if the marking is lost the drone pauses, makes a small bounded search either side of the route, and lands if it still cannot find it.

**Entry point:** `main.py` · **All lab values:** `config.py` · **Dependencies:** `requirements.txt` (cflib, cfclient, numpy, matplotlib)

## Setup
- **macOS:** `bash setup_mac.sh`
- **Windows (PowerShell):** `python -m venv .venv` → `.\.venv\Scripts\Activate.ps1` → `python -m pip install -r requirements.txt`, then install the Crazyradio driver with Zadig (libusb-win32).
- Set the radio URI in `config.py` (or `CFLIB_URI`). Close cfclient before running any script.

## Lab workflow (in order)
| # | Step | Command | Flies? |
|---|---|---|---|
| 1 | Check the planned route | `python main.py --plan-only` | No |
| 2 | Shutter calibration at flight height over the white route and the black background → paste output into config | `python scripts/shutter_calibration.py light` / `dark` / `summary` | No |
| 3 | Tape-measure one grid cell → `CELL_SIZE_M` in config | – | No |
| 4 | Measure map position in the LPS frame (drone on S, then on G) → paste output into config | `python scripts/measure_grid.py` | No |
| 5 | Hover test with logging (start on the white strip) | `python scripts/hover_test.py --label white` | Yes |
| 6 | Dry run: all pre-flight checks + start position vs S | `python main.py --dry-run` | No |
| 7 | Staged route test (first 3 moves) | `python main.py --max-cells 3` | Yes |
| 8 | Full route S → G | `python main.py` | Yes |

Every flight asks you to type `FLY`. Ctrl+C at any time = controlled landing.

## Outputs (evidence for the submission)
Each run creates `data/<time>_route/` with:
`plan.png` (grid, cells searched by A*, route), `waypoints.csv`, `telemetry.csv` (position, battery, shutter, classification), `events.csv` (take-off, each waypoint and its error, marking checks, aborts, landing), `marking_checks.csv`, `trajectory.png` (planned vs estimated), `shutter.png` (raw shutter, thresholds, DARK/LIGHT state).
Calibration: `data/shutter_light.csv`, `data/shutter_dark.csv`, `data/shutter_calibration.png`. Hover tests: `data/hover_<time>_<label>.csv/.png`. Grid measurement: `data/grid_measurement.txt`.

## Design
| File | Responsibility |
|---|---|
| `drone/maps.py` | Original 16×16 map, S=(15,2), G=(15,15) |
| `drone/planner.py` | A* (Manhattan heuristic), 4-connected unit cost; route validation |
| `drone/coords.py` | Grid ↔ world conversion; origin/rotation from two measured cells |
| `drone/surface.py` | Median filter + hysteresis dark/light classifier; thresholds from calibration |
| `drone/cf_link.py` | Pre-flight checks (decks, battery, IMU alive, estimator settled), background telemetry, CSV logging |
| `drone/mission.py` | Take-off, waypoint following with tolerance + timeout, marking checks and search, aborts, landing |
| `drone/plots.py` | Plan, trajectory and shutter figures |
| `scripts/hover_test.py` | Logged hover test for diagnosing position hold |
| `tests/sim_mission.py` | Offline test of the mission logic against a simulated drone (normal runs + every abort path) |

**Movement model:** 4-connected (up/down/left/right), cost 1 per move, as in Assignment 3.

**Coordinate convention:** 1-based cells (row r, column c), row 1 at the top of the printed map. Grid frame origin at the map's bottom-left corner: x_g = (c − ½)L, y_g = (16 − r + ½)L. World (LPS) frame: [x, y] = (x0, y0) + R(θ)·[x_g, y_g], with L, (x0, y0) and θ in `config.py`.

**Waypoint reached:** ‖p_d − p̂‖ < `WAYPOINT_TOLERANCE_M` continuously for `WAYPOINT_HOLD_S`, using the onboard estimate; otherwise abort after the leg duration + `WAYPOINT_TIMEOUT_S`.

**Safe stop (controlled landing) on:** marking lost after pause + search, waypoint timeout, telemetry older than 0.5 s, radio link lost, battery below `FLIGHT_MIN_BATTERY_V` in flight, estimate outside the map area + margin or above `MAX_HEIGHT_M`, Ctrl+C.

**Floor marking:** white paper route on black foam (`ROUTE_SURFACE = "LIGHT"`), detected with `motion.shutter` thresholds from calibration.
