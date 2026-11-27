"""Lab configuration - the ONLY file you should need to edit in the lab.

Values marked MEASURE come from the lab (scripts/measure_grid.py,
scripts/shutter_calibration.py). Values marked CONFIRM are conservative
defaults: confirm them with the demonstrator before flying.
The radio URI can also be set with the CFLIB_URI environment variable.
"""
import math

try:
    from cflib.utils import uri_helper
    URI = uri_helper.uri_from_env(default="radio://0/80/2M/EE5C21CF04")
except ImportError:                      # allows plan-only use without cflib
    URI = "radio://0/80/2M/EE5C21CF04"

# ---------------------------------------------------------------- map -> world
CELL_SIZE_M = 0.0816  # from LPS S-G distance (tape: tile 0.35 m / 4 = 0.0875); LPS value keeps S and G exact in the frame the drone flies in
GRID_ORIGIN_XY = (0.7766, 0.5092)  # from measure_grid S/G readings 27/11
GRID_ROTATION_RAD = -0.0117  # -0.7 deg, from measure_grid 27/11
START_TOLERANCE_M = 0.20    # drone must sit within this of the S cell before take-off

# ------------------------------------------------------------- flight envelope
FLIGHT_HEIGHT_M = 0.20      # CONFIRM: matches calibration height (~0.21 m)
VELOCITY_MPS = 0.15         # CONFIRM: cruise speed between cells
WAYPOINT_TOLERANCE_M = 0.08  # carpet hover stayed within ~5 cm mean / 8 cm max
WAYPOINT_HOLD_S = 0.3       # must stay inside the tolerance this long
WAYPOINT_TIMEOUT_S = 6.0    # extra time allowed beyond the planned leg duration
MIN_LEG_S = 1.0             # shortest go_to duration (smooth, slow legs)
TAKEOFF_S = 2.0
LAND_S = 2.5
BOUNDS_MARGIN_M = 0.30      # abort if the estimate leaves the map area + margin
MAX_HEIGHT_M = 0.8          # abort above this
MIN_BATTERY_V = 3.8  # at rest before take-off; old battery (3.68 V) aborted at take-off
FLIGHT_MIN_BATTERY_V = 3.05  # new battery sat at ~3.25-3.3 V while hovering
TELEMETRY_TIMEOUT_S = 0.5   # abort if no telemetry for this long

# ---------------------------------------------------- Flow Deck floor marking
ROUTE_SURFACE = "LIGHT"         # the lab map: WHITE paper route on BLACK foam
SHUTTER_ENTER_DARK = 5602.8  # calibration 27/11 16:51 light / 16:38 dark
SHUTTER_EXIT_DARK = 3384.2  # calibration 27/11
DARK_HAS_HIGHER_SHUTTER = True  # black foam 8191 (max), white paper ~660-1000
SHUTTER_FILTER_WINDOW = 5       # median filter length (samples at 20 Hz)
SQUAL_MIN = None                # optional: ignore shutter readings below this surface quality
MARK_CHECK_EVERY = 1            # check the marking at every Nth cell (1 = every cell)
MARK_CHECK_S = 0.5              # time spent sampling at each checkpoint
MARK_REQUIRED_FRACTION = 0.6    # fraction of samples that must be the route surface
SEARCH_OFFSET_M = 0.05          # CONFIRM: bounded search either side (0 = no search)

# ------------------------------------------------------------------- anchors
# Staff-measured anchor coordinates from the Assignment 4 brief (for reference).
ANCHORS = {
    0: (0.0, 0.0, 0.0),
    1: (3.0, 0.0, 0.0),
    2: (1.5, 1.5, 0.0),
    3: (0.0, 1.5, 1.5),
    4: (1.5, 0.0, 1.5),
    5: (3.0, 1.5, 1.5),
}
