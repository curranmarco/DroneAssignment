"""Lab configuration. Fill in the staff-approved values before any flight.

The radio URI can also be set with the CFLIB_URI environment variable.
"""
from cflib.utils import uri_helper

# Replace XX with the group number and 80 with your channel if different.
URI = uri_helper.uri_from_env(default="radio://0/80/2M/EE5C21CF04")

# --- Grid / world mapping (staff-approved; placeholders) ---
CELL_SIZE_M = None        # L, metres per grid cell
GRID_ORIGIN_XY = None     # (x0, y0) of grid origin in the LPS frame, metres
GRID_ROTATION_RAD = 0.0   # theta, rotation of grid axes vs LPS axes

# --- Flight envelope (staff-approved; placeholders) ---
FLIGHT_HEIGHT_M = None
VELOCITY_MPS = None
WAYPOINT_TOLERANCE_M = None
WAYPOINT_TIMEOUT_S = None

# --- Flow Deck shutter thresholds (from calibration) ---
SHUTTER_ENTER_DARK = None
SHUTTER_EXIT_DARK = None
DARK_HAS_HIGHER_SHUTTER = True

# Anchor coordinates from the Assignment 4 brief (for checking the Loco tab).
ANCHORS = {
    0: (0.0, 0.0, 0.0),
    1: (3.0, 0.0, 0.0),
    2: (1.5, 1.5, 0.0),
    3: (0.0, 1.5, 1.5),
    4: (1.5, 0.0, 1.5),
    5: (3.0, 1.5, 1.5),
}
