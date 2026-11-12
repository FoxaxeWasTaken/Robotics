"""Token-center reach (pad point, gripper frame) in the arm plane with roll +90: brute force over lift, elbow, flex, pan kept at 0.
Prints for each x the highest and lowest z reachable, and the tilt there."""
import sys; sys.path.insert(0,'/w')
import numpy as np, mujoco, tempfile, time
from exp_connect4_scene import *
path,p,tz=build(tempfile.mkdtemp(prefix='so101_'))
rig=Rig(path,p,tz); m,d=rig.m,rig.d
g=rig.gripper
pad=np.array([-0.0079+0.00325,0,-0.0981])
L=np.linspace(-1.74,1.74,45); E=np.linspace(-1.68,1.68,45); F=np.linspace(-1.65,1.65,33)
d.qpos[rig.jq['wrist_roll']]=np.pi/2
pts=[];t0=time.time()
for l in L:
    for e in E:
        for f in F:
            d.qpos[rig.jq['shoulder_lift']]=l; d.qpos[rig.jq['elbow_flex']]=e; d.qpos[rig.jq['wrist_flex']]=f
            mujoco.mj_kinematics(m,d)
            R=d.xmat[g].reshape(3,3); P=d.xpos[g]+R@pad
            # approach axis tilt from straight down
            a=-R[:,2]; tilt=np.degrees(np.arccos(np.clip(-a[2],-1,1)))
            # lowest arm point height over wrist body and lower arm body: clearance is checked later
            pts.append((P[0],P[1],P[2],tilt,l,e,f))
pts=np.array(pts); print('samples',len(pts),time.time()-t0)
np.save('/w/reach.npy',pts)
for x in (0.14,0.18,0.217,0.254,0.291,0.328,0.365,0.402):
    s=pts[np.abs(pts[:,0]-x)<0.004]
    print('x',x,'zmax',s[:,2].max().round(3),end=' | ')
    for z in (0.17,0.19,0.20,0.21,0.22,0.23,0.24,0.25,0.26):
        q=s[np.abs(s[:,2]-z)<0.004]
        print(f'{z:.2f}:{q[:,3].min():.0f}' if len(q) else f'{z:.2f}:--',end=' ')
    print()
