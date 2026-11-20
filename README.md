# DroneAssignment – EEP55C21 Assignment 4: Pathfinding on Crazyflie

Python programme that plans a route on the 16×16 occupancy grid (ported from Assignment 3), converts it to Loco Positioning world waypoints, and flies it with a Crazyflie using Flow Deck `motion.shutter` dark/light sensing to verify the floor marking.

## Setup

**macOS:** `bash setup_mac.sh` (installs Python 3.12 + libusb via Homebrew, creates `.venv`, installs `requirements.txt`).

**Windows (PowerShell):**
```
python -m venv .venv
.\.venv\Scripts\Activate.ps1      # if blocked: Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```
Then install the Crazyradio driver with Zadig (Options → List All Devices → Crazyradio → libusb-win32 → Install).

Every new terminal: activate `.venv` first (`source .venv/bin/activate` on Mac).

## Radio address
Set your group's URI in `config.py`, or per terminal:
- Mac: `export CFLIB_URI="radio://0/80/2M/EE5C21CFXX"`
- Windows: `$env:CFLIB_URI = "radio://0/80/2M/EE5C21CFXX"`

Close cfclient before running any script – only one program can hold the radio.

## Calibration (no flight)
`python scripts/shutter_calibration.py light`, then `dark`, then `summary` → CSVs in `data/` and a suggested threshold (required submission item).

## Layout
| Path | Responsibility |
|---|---|
| `config.py` | URI, cell size, grid origin/rotation, flight limits, thresholds (staff-approved values) |
| `drone/maps.py` | Original grid + 3 obstacle scenarios |
| `drone/planner.py` | Pathfinding + route validation |
| `drone/coords.py` | Grid → world conversion |
| `drone/cf_link.py` | Connection, logging, health checks |
| `drone/surface.py` | Shutter filtering and dark/light classification |
| `drone/mission.py` | Waypoint execution, timeouts, path-loss handling, landing |
| `main.py` | Entry point |

## Coordinate convention
1-based row r (top = 1), column c: x = (c − ½)·L, y = (16 − r + ½)·L, then offset/rotate into the LPS frame using `config.py`.
