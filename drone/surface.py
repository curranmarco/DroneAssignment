"""Flow Deck surface-contrast sensing using motion.shutter.

motion.shutter is the optical-flow sensor's exposure time in clock cycles
(uint16). Darker surfaces usually need longer exposure (larger values), but
the ordering must come from calibration (scripts/shutter_calibration.py).

Classifier = short median filter + hysteresis (two thresholds), as the brief
recommends, so the state does not flicker at the edge of the marking.
"""
from collections import deque
from statistics import median

DARK, LIGHT, UNKNOWN = "DARK", "LIGHT", "UNKNOWN"


class SurfaceClassifier:
    def __init__(self, enter_dark, exit_dark, dark_has_higher_shutter=True,
                 window=5, min_squal=None):
        if enter_dark is None or exit_dark is None:
            raise ValueError("Shutter thresholds not set - run scripts/shutter_calibration.py")
        if dark_has_higher_shutter and not enter_dark > exit_dark:
            raise ValueError("Need enter_dark > exit_dark when dark has higher shutter")
        if not dark_has_higher_shutter and not enter_dark < exit_dark:
            raise ValueError("Need enter_dark < exit_dark when dark has lower shutter")
        self.enter_dark = enter_dark
        self.exit_dark = exit_dark
        self.dark_higher = dark_has_higher_shutter
        self.min_squal = min_squal
        self.samples = deque(maxlen=window)
        self.state = UNKNOWN
        self.filtered = None

    def reset(self):
        self.samples.clear()
        self.state = UNKNOWN
        self.filtered = None

    def update(self, shutter, squal=None):
        """Feed one shutter reading, return the current state (DARK / LIGHT / UNKNOWN)."""
        if shutter is None or shutter <= 0 or shutter >= 65535:
            return self.state                     # invalid / saturated: ignore
        if self.min_squal is not None and squal is not None and squal < self.min_squal:
            return self.state                     # poor surface quality: ignore
        self.samples.append(shutter)
        if len(self.samples) < self.samples.maxlen:
            return self.state                     # filter not full yet
        s = median(self.samples)
        self.filtered = s
        if self.dark_higher:
            if self.state == DARK:
                self.state = LIGHT if s < self.exit_dark else DARK
            else:
                self.state = DARK if s > self.enter_dark else LIGHT
        else:
            if self.state == DARK:
                self.state = LIGHT if s > self.exit_dark else DARK
            else:
                self.state = DARK if s < self.enter_dark else LIGHT
        return self.state


def thresholds_from_calibration(light, dark, hysteresis_fraction=0.15):
    """Preliminary threshold T = (median_light + median_dark) / 2 (from the brief),
    plus hysteresis of +/- hysteresis_fraction * |gap| around T.
    Returns dict with medians, T, enter/exit thresholds, ordering and overlap flag."""
    ml, md = median(light), median(dark)
    dark_higher = md > ml
    T = (ml + md) / 2
    h = hysteresis_fraction * abs(md - ml)
    if dark_higher:
        enter, exit_ = T + h, T - h
        overlap = max(light) >= min(dark)
    else:
        enter, exit_ = T - h, T + h
        overlap = max(dark) >= min(light)
    return {"median_light": ml, "median_dark": md, "T": T, "enter_dark": enter,
            "exit_dark": exit_, "dark_has_higher_shutter": dark_higher, "overlap": overlap}
