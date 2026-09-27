import json, glob, os, numpy as np
SP = os.path.dirname(os.path.abspath(__file__))
def detrend(y, t):
    b, a = np.polyfit(t, y, 1); return y - (b * t + a)
def spec(y, t, fmax=5.0, step=0.05):
    yd = detrend(np.array(y), np.array(t)); n = len(yd)
    fs = np.arange(0.3, fmax + 1e-9, step)
    mags = [abs(np.sum(yd * np.exp(-2j * np.pi * f * np.array(t)))) / n for f in fs]
    i = int(np.argmax(mags)); return fs[i], mags[i], np.sqrt(np.mean(yd ** 2))
def flips(a):
    s = np.sign(a); s = s[s != 0]; return int(np.sum(s[:-1] != s[1:]))
rows = []
for label, pat in (("BASE", "bm_dump/*/*.json"), ("KD05", "bm_dump_r16/*/*.json"), ("KD05", "bm_dump_r17/*/*.json"), ("KD15", "bm_dump_r1a/*/*.json"), ("KD15", "bm_dump_r30/*/*.json"), ("KD15", "bm_dump_r31/*/*.json"), ("KP70", "bm_dump_r32/*/*.json"), ("KP50", "bm_dump_r33/*/*.json"), ("FR60", "bm_dump_r37/*/*.json"), ("FR60", "bm_dump_r39/*/*.json")):
    for f in sorted(glob.glob(os.path.join(SP, pat))):
        d = json.load(open(f)); c, k = d["car"], d["ctrl"]
        t = np.array(k["t"]); t = t - t[0]
        pf, pm, rms = spec(k["output"], t)
        mph = np.mean(c["vego"]) * 2.237 if "vego" in c else float("nan")
        rows.append((label, os.path.basename(os.path.dirname(f)) + "/" + os.path.basename(f)[:13], mph, pf, pm, rms,
                     flips(np.array(k["error"])) / 8, np.mean(np.abs(k["p"])), np.mean(np.abs(k.get("d", [0.0]))),
                     np.mean(np.abs(k["f"])), np.mean(np.abs(np.array(k["output"])) >= 0.99), np.mean(c["pressed"]),
                     np.max(np.abs(c["rate"]))))
print(f"{'set':5} {'window':28} {'mph':>5} {'pkHz':>5} {'pkMag':>6} {'rmsAC':>6} {'errF/s':>6} {'|P|':>5} {'|D|':>5} {'|FF|':>5} {'sat%':>5} {'hand%':>5} {'rateMx':>6}")
for r in rows:
    print(f"{r[0]:5} {r[1]:28} {r[2]:5.1f} {r[3]:5.2f} {r[4]:6.3f} {r[5]:6.3f} {r[6]:6.1f} {r[7]:5.3f} {r[8]:5.3f} {r[9]:5.3f} {100*r[10]:5.1f} {100*r[11]:5.1f} {r[12]:6.0f}")
print("\n(pkMag = torque-output spectral peak magnitude after detrend; rmsAC = rms of detrended torque output)")
for f in (os.path.join(SP, "bm_dump_r16/seg6/BOOKMARK_1of1_412s.json"), os.path.join(SP, "bm_dump_r17/seg17/BOOKMARK_1of1_1063s.json")):
    d = json.load(open(f)); c, k = d["car"], d["ctrl"]
    print(f"\n=== timeline {f.split(os.sep)[-2]} (every 0.5 s; t rel. to swipe) ===")
    print(f"{'t':>5} {'mph':>5} {'angle':>7} {'rate':>6} {'desired':>8} {'actual':>7} {'err':>6} {'out':>6} {'P':>6} {'D':>6} {'FF':>6} hand")
    kt = np.array(k["t"]); ct = np.array(c["t"])
    for tt in np.arange(kt[0], kt[-1] + 1e-6, 0.5):
        i = int(np.argmin(np.abs(kt - tt))); j = int(np.argmin(np.abs(ct - tt)))
        print(f"{kt[i]-kt[-1]:5.1f} {c['vego'][j]*2.237:5.1f} {c['angle'][j]:7.1f} {c['rate'][j]:6.0f} {k['desired'][i]:8.2f} {k['actual'][i]:7.2f} {k['error'][i]:6.2f} {k['output'][i]:6.2f} {k['p'][i]:6.2f} {k.get('d', [0.0]*len(kt))[i]:6.2f} {k['f'][i]:6.2f} {'Y' if c['pressed'][j] else ''}")
