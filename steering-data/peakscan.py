import glob, os, sys
import numpy as np
from openpilot.tools.lib.logreader import LogReader
R = sys.argv[1]
segs=[s for s in sorted(glob.glob(f'/data/media/0/realdata/{R}--*/'), key=lambda p:int(p.rstrip('/').split('--')[-1])) if 'user.preserve' in os.listxattr(s)]
for s in segs:
    t0=None; T=[];R_=[];A=[];P=[];V=[];O=[];bm=[]
    for m in LogReader(s+'rlog.zst'):
        t=m.logMonoTime/1e9
        if t0 is None: t0=t
        w=m.which()
        if w=='carState':
            c=m.carState; T.append(t-t0); R_.append(c.steeringRateDeg); A.append(c.steeringAngleDeg); P.append(c.steeringPressed); V.append(c.vEgo)
        elif w=='controlsState':
            l=m.controlsState.lateralControlState
            if l.which()=='torqueState': O.append((t-t0,l.torqueState.output,l.torqueState.active))
        elif w=='userBookmark': bm.append(round(t-t0,1))
    T=np.array(T);R_=np.array(R_);A=np.array(A);P=np.array(P);V=np.array(V)
    print(f'=== seg {s.rstrip("/").split("--")[-1]}  bookmarks@ {bm}  mph {V.mean()*2.237:.0f}')
    bins=np.arange(T.min(),T.max(),1.0)
    b=[]
    for x in bins:
        m_=(T>=x)&(T<x+1)
        if m_.any(): b.append((x,np.max(np.abs(R_[m_])),np.ptp(A[m_]),P[m_].mean()))
    b.sort(key=lambda z:-z[1])
    for x,r,pa,h in b[:3]: print(f'   t={x:6.1f}  max|rate| {r:4.0f} deg/s  swing {pa:5.1f} deg  hands {100*h:3.0f}%')
    ch=np.where(np.diff(P.astype(int))!=0)[0]
    print('   hands-on transitions:', ' '.join(f'{T[i]:.1f}{"+" if P[i+1] else "-"}' for i in ch[:10]) or 'none')
    OT=np.array([o[0] for o in O]);OO=np.array([o[1] for o in O])
    if len(OO)>10:
        d=np.abs(OO[10:]-OO[:-10]); i=int(np.argmax(d)); print(f'   max torque change/0.1s: {d[i]:.3f} at t={OT[i]:.1f}')
