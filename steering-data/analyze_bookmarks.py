#!/usr/bin/env python3
"""
Analyze steering behavior around userBookmark events.

Runs OFFLINE on a PC against rlogs copied from the device. It does not run on
the comma, does not publish messages, and does not touch device state.

Usage:
  python analyze_bookmarks.py /path/to/segment_or_route_dir [--window 8]

For each bookmark found, prints a window of steering diagnostics ending at the
bookmark (you press it AFTER the event, so the interesting data is before it).
"""

import argparse
import sys
from pathlib import Path

import numpy as np

from openpilot.tools.lib.logreader import LogReader

# services we care about for lateral oscillation diagnosis
CARSTATE = "carState"
CONTROLS = "controlsState"


def collect(lr):
    """Single pass over the log: bookmarks + steering time series."""
    bookmarks = []
    cs_t, angle, rate, torque, pressed, vego = [], [], [], [], [], []
    ct_t, err, p_term, i_term, f_term, out, des_la, act_la, sat, active = [], [], [], [], [], [], [], [], [], []
    d_term = []

    t0 = None
    for msg in lr:
        typ = msg.which()
        t = msg.logMonoTime / 1e9
        if t0 is None:
            t0 = t
        t -= t0

        if typ == "userBookmark":
            bookmarks.append(t)
        elif typ == CARSTATE:
            m = msg.carState
            cs_t.append(t)
            angle.append(m.steeringAngleDeg)
            rate.append(m.steeringRateDeg)
            torque.append(m.steeringTorque)
            pressed.append(m.steeringPressed)
            vego.append(m.vEgo)
        elif typ == CONTROLS:
            m = msg.controlsState
            lat = m.lateralControlState
            which = lat.which()
            if which != "torqueState":
                continue
            ts = lat.torqueState
            ct_t.append(t)
            err.append(ts.error)
            p_term.append(ts.p)
            i_term.append(ts.i)
            d_term.append(ts.d)
            f_term.append(ts.f)
            out.append(ts.output)
            des_la.append(ts.desiredLateralAccel)
            act_la.append(ts.actualLateralAccel)
            sat.append(ts.saturated)
            active.append(ts.active)

    car = {
        "t": np.array(cs_t), "angle": np.array(angle), "rate": np.array(rate),
        "torque": np.array(torque), "pressed": np.array(pressed), "vego": np.array(vego),
    }
    ctrl = {
        "t": np.array(ct_t), "error": np.array(err), "p": np.array(p_term),
        "i": np.array(i_term), "d": np.array(d_term), "f": np.array(f_term), "output": np.array(out),
        "desired": np.array(des_la), "actual": np.array(act_la), "saturated": np.array(sat),
        "active": np.array(active),
    }
    return bookmarks, car, ctrl


def zero_crossings(x):
    """Count sign changes - the direct signature of oscillation."""
    s = np.sign(x)
    s = s[s != 0]
    if len(s) < 2:
        return 0
    return int(np.sum(s[:-1] != s[1:]))


FREQ_FLOOR_HZ = 0.3  # ignore the slow trend of the road curve itself


def dominant_freq(x, t):
    """Peak frequency above FREQ_FLOOR_HZ of the linearly-detrended signal."""
    if len(x) < 8:
        return float("nan")
    dt = np.median(np.diff(t))
    if not np.isfinite(dt) or dt <= 0:
        return float("nan")
    tt = t - t[0]
    slope, intercept = np.polyfit(tt, x, 1)
    xd = x - (slope * tt + intercept)
    spec = np.abs(np.fft.rfft(xd))
    freqs = np.fft.rfftfreq(len(xd), dt)
    band = freqs >= FREQ_FLOOR_HZ
    if not np.any(band):
        return float("nan")
    return float(freqs[band][int(np.argmax(spec[band]))])


def window(d, lo, hi):
    m = (d["t"] >= lo) & (d["t"] <= hi)
    return {k: v[m] for k, v in d.items()}


def dump_window(path, name, bt, c, k):
    """Write the raw window as JSON so it can be plotted elsewhere."""
    import json
    out = {
        "name": name, "bookmark_t": float(bt),
        "car": {kk: [float(v) for v in vv] for kk, vv in c.items()},
        "ctrl": {kk: [float(v) for v in vv] for kk, vv in k.items()},
    }
    with open(path, "w") as f:
        json.dump(out, f)


def describe(name, bt, car, ctrl, width, dump_dir=None):
    lo, hi = bt - width, bt
    c = window(car, lo, hi)
    k = window(ctrl, lo, hi)
    if dump_dir:
        import os
        os.makedirs(dump_dir, exist_ok=True)
        safe = name.replace(" ", "_").replace("/", "of")
        dump_window(os.path.join(dump_dir, f"{safe}_{bt:.0f}s.json"), name, bt, c, k)

    print(f"\n{'=' * 70}")
    print(f"{name}  @ t={bt:.1f}s   (window {lo:.1f} -> {hi:.1f}s, {width:.0f}s before press)")
    print("=" * 70)

    if len(c["t"]) == 0:
        print("  no carState samples in window")
        return

    print(f"  speed            {np.mean(c['vego']) * 2.237:6.1f} mph "
          f"(min {np.min(c['vego']) * 2.237:.0f}, max {np.max(c['vego']) * 2.237:.0f})")
    print(f"  hands on wheel   {100.0 * np.mean(c['pressed']):6.1f}% of window")

    print("\n  -- wheel motion --")
    print(f"  steer angle      mean {np.mean(c['angle']):7.2f}  range {np.ptp(c['angle']):6.2f} deg")
    print(f"  steer rate       max |{np.max(np.abs(c['rate'])):6.1f}| deg/s   "
          f"rms {np.sqrt(np.mean(c['rate'] ** 2)):.1f}")
    print(f"  rate reversals   {zero_crossings(c['rate']):4d}   "
          f"({zero_crossings(c['rate']) / max(width, 1e-9):.1f} /s)")
    print(f"  dominant freq    {dominant_freq(c['rate'], c['t']):6.2f} Hz  (steer rate, >= {FREQ_FLOOR_HZ} Hz)")

    if len(k["t"]) == 0:
        print("\n  no torque controlsState in window")
        return

    print("\n  -- torque controller --")
    print(f"  lateral active   {100.0 * np.mean(k['active']):6.1f}% of window")
    if np.mean(k["active"]) < 0.5:
        print("  (mostly disengaged - controller stats below are not meaningful)")
    print(f"  lat accel err    mean {np.mean(k['error']):7.3f}  max |{np.max(np.abs(k['error'])):.3f}|")
    print(f"  err reversals    {zero_crossings(k['error']):4d}   "
          f"({zero_crossings(k['error']) / max(width, 1e-9):.1f} /s)")
    print(f"  output torque    max |{np.max(np.abs(k['output'])):.3f}|  "
          f"reversals {zero_crossings(k['output'])}  "
          f"dominant {dominant_freq(k['output'], k['t']):.2f} Hz")
    print(f"  P / I / D / FF   {np.mean(np.abs(k['p'])):.3f} / {np.mean(np.abs(k['i'])):.3f} / "
          f"{np.mean(np.abs(k['d'])):.3f} / {np.mean(np.abs(k['f'])):.3f}  (mean |magnitude|)")
    print(f"  desired lat acc  mean {np.mean(k['desired']):7.3f}  max |{np.max(np.abs(k['desired'])):.3f}|")
    print(f"  saturated        {100.0 * np.mean(k['saturated']):6.1f}% of window")

    # interpretation hints
    rev_rate = zero_crossings(k["error"]) / max(width, 1e-9)
    notes = []
    if rev_rate > 2.0:
        notes.append(f"error sign flips {rev_rate:.1f}x/s -> closed-loop oscillation")
    if np.mean(np.abs(k["p"])) > 2 * np.mean(np.abs(k["f"])) and np.mean(np.abs(k["f"])) > 0:
        notes.append("P dominates feedforward -> gain too high for this disturbance")
    if np.max(np.abs(c["rate"])) > 100:
        notes.append("steer rate >100 deg/s -> near Toyota MAX_STEER_RATE fault cutoff")
    if np.mean(k["saturated"]) > 0.1:
        notes.append("controller saturating -> commanded more torque than allowed")
    if notes:
        print("\n  -- notes --")
        for n in notes:
            print(f"  * {n}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path", help="segment dir, rlog file, or route identifier")
    ap.add_argument("--window", type=float, default=8.0,
                    help="seconds before each bookmark to analyze (default 8)")
    ap.add_argument("--dump", default=None, metavar="DIR",
                    help="also write each window's raw time series as JSON into DIR")
    args = ap.parse_args()

    p = Path(args.path)
    if p.exists() and p.is_dir():
        logs = sorted(str(f) for f in p.rglob("rlog*") if f.is_file())
        if not logs:
            print(f"no rlog files under {p}", file=sys.stderr)
            return 1
    else:
        logs = [args.path]

    print(f"reading {len(logs)} log file(s)...")
    lr = LogReader(logs if len(logs) > 1 else logs[0])
    bookmarks, car, ctrl = collect(lr)

    print(f"carState samples: {len(car['t'])}   torque controlsState: {len(ctrl['t'])}")
    if not bookmarks:
        print("\nNo bookmarks found in these logs.")
        print("Baseline over the whole log instead:")
        if len(car["t"]):
            describe("FULL LOG (no bookmark)", car["t"][-1], car, ctrl,
                     float(car["t"][-1] - car["t"][0]))
        return 0

    print(f"\nFound {len(bookmarks)} bookmark(s).")
    for i, bt in enumerate(bookmarks, 1):
        describe(f"BOOKMARK {i}/{len(bookmarks)}", bt, car, ctrl, args.window, args.dump)

    return 0


if __name__ == "__main__":
    sys.exit(main())
