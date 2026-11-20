#!/usr/bin/env bash
# One-time macOS setup for the Crazyflie Assignment 4 project.
# Run from this folder:  bash setup_mac.sh
set -e
cd "$(dirname "$0")"

if ! command -v brew >/dev/null; then
  echo "Homebrew not found. Install it from https://brew.sh, then rerun this script."; exit 1
fi

echo "==> Installing Python 3.12 and libusb (Crazyradio needs libusb on macOS)"
brew install python@3.12 libusb

PY="$(brew --prefix)/bin/python3.12"
echo "==> Creating virtual environment .venv with $($PY --version)"
[ -d .venv ] || "$PY" -m venv .venv
source .venv/bin/activate

echo "==> Installing Python packages"
python -m pip install --upgrade pip
python -m pip install -r requirements.txt

# Make sure pyusb can find Homebrew's libusb (needed on Apple Silicon)
LINE="export DYLD_LIBRARY_PATH=\"$(brew --prefix)/lib:\$DYLD_LIBRARY_PATH\""
grep -qxF "$LINE" .venv/bin/activate || echo "$LINE" >> .venv/bin/activate

echo "==> Linking to GitHub"
if [ ! -d .git ]; then
  git init -b main
  git remote add origin https://github.com/curranmarco/DroneAssignment.git
fi

echo
echo "Done. Next:"
echo "  source .venv/bin/activate"
echo "  python scripts/shutter_calibration.py light   # with drone connected"
echo "  cfclient                            # GUI"
