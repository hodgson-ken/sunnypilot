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

# A/B switch for back-to-back road testing. If this file exists and its first
# character is "0", every knob reverts to its stock default no matter what
# steer_tune.json says; "1" (or a missing file) uses the tune normally. It is a
# plain file rather than a Param because unregistered Param keys raise, and
# registering one means editing params_keys.h and rebuilding on the device.
STEER_TUNE_ENABLE_PATH = "/data/steer_tune_enabled"

# name: (stock default, min, max)
KNOBS: dict[str, tuple[float, float, float]] = {
  "kd": (0.0, 0.0, 0.5),                  # derivative gain on filtered lat accel rate, m/s^2 per m/s^3
  "kp_scale": (1.0, 0.3, 1.5),            # multiplier on the whole speed-scheduled KP curve
  # Multiplier applied to the KP curve only at the low-speed end, blending back to kp_scale
  # by KP_LO_SPEED. The stock schedule reaches KP 250 at 1 m/s, so a near-zero error can
  # still command full torque at parking speeds; this trims that end without touching
  # highway gain.
  "kp_scale_lo": (1.0, 0.05, 1.5),
  "ki": (0.3, 0.0, 0.5),                  # integral gain
  # Lat accel error band over which the friction term ramps in. Speed-scheduled like the
  # slew limit: a narrow band at low speed makes the friction feedforward reach full strength
  # as soon as there is any error, which is what breaks the steering rack's static friction
  # and stops the stick-slip stepping in slow corners. A wide band at highway speed keeps the
  # same term from acting as a relay and chattering. friction_threshold sets the high-speed
  # value; friction_threshold_lo the low-speed one (interpolated on FRICTION_SPEEDS).
  "friction_threshold": (0.3, 0.1, 1.0),
  "friction_threshold_lo": (0.3, 0.1, 1.0),
  # Multiplier on the friction feedforward's magnitude at low speed, ramping back to 1.0
  # by FRICTION_SPEEDS[1]. The learned friction coefficient is fitted over a whole drive
  # and lands well below what the rack needs to break away from a standstill, which shows
  # up as the wheel stalling then stepping in slow corners. Only scales the friction term.
  "friction_gain_lo": (1.0, 1.0, 3.0),
  "rate_filter_hz": (1.2, 0.3, 5.0),      # low-pass cutoff on the measurement rate that feeds KD
  # max rate of change of the final torque command, in steer-max units per second,
  # interpolated on speed between slew_lo (at/below 7 m/s) and slew_hi (at/above 16 m/s).
  # 100 is effectively unlimited (stock); Toyota's own EPS rate limit is ~1.0/s.
  "slew_lo": (100.0, 0.5, 100.0),
  "slew_hi": (100.0, 0.1, 100.0),
}

# The highway ping-pong is present from ~35 mph up, so the tight limit must be fully
# in by 16 m/s; the loose limit only needs to cover parking-lot and tight-corner speeds.
SLEW_SPEEDS = [7.0, 16.0]  # m/s  (~16 and ~36 mph)

# Stiction is a low-steering-effort problem, not a low-speed one: it still shows up in
# 30 mph corners. Hold the narrow (stiction-breaking) band all the way to 18 m/s (~40 mph)
# and only widen it over the range where the relay chatter actually appears.
FRICTION_SPEEDS = [18.0, 27.0]  # m/s  (~40 and ~60 mph)

# kp_scale_lo applies fully at or below the first speed and blends to kp_scale by the second.
KP_LO_SPEEDS = [7.0, 14.0]  # m/s  (~16 and ~31 mph)


class SteerTune:
  def __init__(self, path: str = STEER_TUNE_PATH, enable_path: str = STEER_TUNE_ENABLE_PATH):
    self.path = path
    self.enable_path = enable_path
    self.frame = -1
    self.mtime = None
    self.enabled = True
    self.values: dict[str, float] = {k: v[0] for k, v in KNOBS.items()}

  def _read_enabled(self) -> bool:
    try:
      with open(self.enable_path) as f:
        return f.read(1) != "0"
    except OSError:
      return True

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

    enabled = self._read_enabled()
    if mtime == self.mtime and enabled == self.enabled:
      return False
    self.mtime = mtime
    self.enabled = enabled

    new = {k: v[0] for k, v in KNOBS.items()}
    if mtime is not None and enabled:
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
      cloudlog.info(f"steer_tune: {'ON' if enabled else 'OFF (stock)'} {self.values}")
    return changed
