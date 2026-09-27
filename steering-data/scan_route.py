import glob, os, subprocess, sys
from openpilot.tools.lib.logreader import LogReader
R = sys.argv[1]; OUT = sys.argv[2]
segs = sorted(glob.glob(f'/data/media/0/realdata/{R}--*/'), key=lambda p: int(p.rstrip('/').split('--')[-1]))
pres = [s for s in segs if 'user.preserve' in os.listxattr(s)]
segno = lambda s: s.rstrip('/').split('--')[-1]
print('route', R, 'segments', len(segs), 'preserved', [segno(s) for s in pres])
for s in pres:
    n = nz = 0; dmax = 0.0; ver = set(); bm = []; t0 = None
    for m in LogReader(s + 'qlog.zst'):
        w = m.which(); t = m.logMonoTime / 1e9
        if t0 is None: t0 = t
        if w == 'controlsState':
            l = m.controlsState.lateralControlState
            if l.which() == 'torqueState':
                ts = l.torqueState; ver.add(ts.version)
                if ts.active:
                    n += 1; nz += int(abs(ts.d) > 1e-6); dmax = max(dmax, abs(ts.d))
        elif w == 'userBookmark':
            bm.append(round(t - t0, 1))
    print(f'  seg {segno(s)}: version {ver} active {n} d!=0 {nz} max|d| {dmax:.3f} bookmarks@ {bm}')
keys = ('BOOKMARK', 'speed', 'hands', 'dominant', 'reversals', 'output torque', 'P / I', 'lat accel err', 'desired lat', 'steer rate', 'steer angle', 'notes', '*')
for s in pres:
    print(f'##### SEG {segno(s)} #####')
    out = subprocess.run(['/usr/local/venv/bin/python3', '/data/analyze_bookmarks.py', s, '--dump', f'{OUT}/seg{segno(s)}'],
                         capture_output=True, text=True, env={**os.environ, 'PYTHONPATH': '/data/openpilot'})
    for line in out.stdout.splitlines():
        if any(k in line for k in keys): print(line)
    if out.returncode: print(out.stderr[-800:])
