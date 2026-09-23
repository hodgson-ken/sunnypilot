"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import json
import os

from openpilot.common.swaglog import cloudlog

# Personal-fork lateral tuning knobs for the V0 torque controller.
#
# Values are read from a JSON file on the device so they can be changed over
# SSH between drives without a rebuild or a new Params key. Every knob has a
# stock default and a hard clamp; a missing, unreadable or malformed file
# leaves the controller exactly as upstream ships it.

STEER_TUNE_PATH = "/data/steer_tune.json"
REFRESH_FRAMES = 300  # ~3 s at 100 Hz, matches PARAMS_UPDATE_PERIOD

# name: (stock default, min, max)
KNOBS: dict[str, tuple[float, float, float]] = {
  "kd": (0.0, 0.0, 0.5),                  # derivative gain on filtered lat accel rate, m/s^2 per m/s^3
  "kp_scale": (1.0, 0.3, 1.5),            # multiplier on the whole speed-scheduled KP curve
  "ki": (0.3, 0.0, 0.5),                  # integral gain
  # Lat accel error band over which the friction term ramps in. Speed-scheduled like the
  # slew limit: a narrow band at low speed makes the friction feedforward reach full strength
  # as soon as there is any error, which is what breaks the steering rack's static friction
  # and stops the stick-slip stepping in slow corners. A wide band at highway speed keeps the
  # same term from acting as a relay and chattering. friction_threshold sets the high-speed
  # value; friction_threshold_lo the low-speed one (both interpolated on SLEW_SPEEDS).
  "friction_threshold": (0.3, 0.1, 1.0),
  "friction_threshold_lo": (0.3, 0.1, 1.0),
  "rate_filter_hz": (1.2, 0.3, 5.0),      # low-pass cutoff on the measurement rate that feeds KD
  # max rate of change of the final torque command, in steer-max units per second,
  # interpolated on speed between slew_lo (at/below 7 m/s) and slew_hi (at/above 16 m/s).
  # 100 is effectively unlimited (stock); Toyota's own EPS rate limit is ~1.0/s.
  "slew_lo": (100.0, 0.5, 100.0),
  "slew_hi": (100.0, 0.1, 100.0),
}

# The highway ping-pong is present from ~35 mph up, so the tight limit must be fully
# in by 16 m/s; the loose limit only needs to cover parking-lot and tight-corner speeds.
SLEW_SPEEDS = [7.0, 16.0]  # m/s


class SteerTune:
  def __init__(self, path: str = STEER_TUNE_PATH):
    self.path = path
    self.frame = -1
    self.mtime = None
    self.values: dict[str, float] = {k: v[0] for k, v in KNOBS.items()}

  def __getattr__(self, name: str) -> float:
    try:
      return self.__dict__["values"][name]
    except KeyError:
      raise AttributeError(name) from None

  def update(self) -> bool:
    """Re-read the file every REFRESH_FRAMES. Returns True when any value changed."""
    self.frame += 1
    if self.frame % REFRESH_FRAMES != 0:
      return False

    try:
      mtime = os.path.getmtime(self.path)
    except OSError:
      mtime = None

    if mtime == self.mtime:
      return False
    self.mtime = mtime

    new = {k: v[0] for k, v in KNOBS.items()}
    if mtime is not None:
      try:
        with open(self.path) as f:
          raw = json.load(f)
        for k, (_, lo, hi) in KNOBS.items():
          if k in raw:
            new[k] = min(max(float(raw[k]), lo), hi)
      except (OSError, ValueError, TypeError) as e:
        cloudlog.warning(f"steer_tune: ignoring {self.path}: {e}")
        new = {k: v[0] for k, v in KNOBS.items()}

    changed = new != self.values
    self.values = new
    if changed:
      cloudlog.info(f"steer_tune: {self.values}")
    return changed
