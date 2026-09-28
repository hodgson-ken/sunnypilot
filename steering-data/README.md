# RAV4 TSS2 steering tuning data

Raw analysis windows and scripts from the 2026-09 lateral-control tuning effort on a
2021 RAV4 Hybrid (TOYOTA_RAV4_TSS2, comma four, sunnypilot release-mici).

## Contents

- `bookmark_windows/bm_dump_r<route>/seg<n>/*.json` — 63 windows, each the 8 s of
  carState + controlsState immediately before a driver bookmark. Times are relative
  to the bookmark press.
- `analyze_bookmarks.py` — per-bookmark report (wheel motion, torque controller,
  detrended spectrum). Run it on the device:
  `PYTHONPATH=/data/openpilot /usr/local/venv/bin/python3 analyze_bookmarks.py <segment_dir> --dump <out_dir>`
- `scan_route.py` — finds preserved segments in a route and runs the above on each.
- `peakscan.py` — whole-segment scan: peak wheel-rate per second, hands-on
  transitions, max torque slew. Catches events that fall outside the bookmark window.
- `compare.py` — tabulates spectral peak, torque swing and saturation across every
  window set, grouped by which tune was active.
- `best_known_tune.json` — the tune that produced the best results; write it to
  `/data/steer_tune.json` on the device to restore (re-read within ~3 s, no reboot).

## What the data shows

Above ~30 mph the combination of NNLC, the DTRV6 driving model and a torque slew
limit of 0.5/s produced the calmest windows recorded here: 80 mph with a peak wheel
rate of 7 deg/s, 45 mph with 0.

Below ~20 mph a limit cycle persists: the wheel swings roughly +/-35 deg at about
2 Hz while the planner requests a steady, near-zero lateral acceleration. Trimming
the low-speed end of the KP curve (250 -> 87 -> 37 at 1 m/s) reduced but never
eliminated it.

## Result (2026-09-28, confirmed by A/B on a road trip)

Toggled back and forth at highway speed on the same roads, using RainbowMode as
the switch (on = stock, off = the tune):

- **Stock: wobbled at high speed.**
- **Tune: did not.**

The driver's summary: "tried rainbow and it wobbled at high speed, reverted to our
build and we are in a better spot at high speed."

The tune in place for that comparison:

    {"kp_scale_lo": 0.08, "slew_lo": 1.2, "slew_hi": 0.5}

with NNLC enabled and the DTRV6 driving model. `slew_hi` is the knob that matters
above ~36 mph; `kp_scale_lo` and `slew_lo` blend out to no effect by ~31 mph.
