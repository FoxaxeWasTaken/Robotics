"""E10: the design as documented vs this one; across window by sign; offsets; pre-drop + descent; guide."""
import numpy as np, json, sys
from e7_cycle import *
import e7_cycle
O2 = np.array([-0.00315, 0.00025, -0.00245])   # token in the TCP frame, measured at release (E7)
def ok(r, i):
    return r["land"] == i and abs(r["z_final"] - r["bottom"]) < 0.01
def run(gap, N, dy, dz, h, pre=None, seed=5, guide=False, across=0.0):
    rng = np.random.default_rng(seed)
    b = Bench(gap=gap, offset=OFFSET, tokens=1, guide=guide, **CONTACT)
    n = held = 0; miss = []
    for k in range(N):
        i = k % 7
        a, c = rng.uniform(-dy, dy), rng.uniform(-dz, dz)
        q = rel_pose(i, h, o=O2, across=across)
        if pre is not None:
            r = cycle2(b, i, h, a, c, q, rel_pose(i, pre, o=O2, across=across))
        else:
            r = cycle(b, i, h, dy=a, dz=c, q_rel=q)
        park(b)
        n += ok(r, i); held += r["held"]
        if not ok(r, i): miss.append(r["land"])
    return n, held, miss
def cycle2(b, i, h, dy, dz, q, qpre):
    """Pre-drop, straight joint descent, open, lift, HANDOVER."""
    b.on_tick = None
    b.teleport_arm(HANDOVER); b.wait(0.2)
    bid = give(b, dy, dz)
    b.goal["gripper"] = 0.0; b.wait(0.8)
    b.d.xfrc_applied[bid, 2] = 0.0
    held = b.effort() > 0.06
    b.move(qpre); b.move(q)
    b.goal["gripper"] = 100.0; b.wait(0.5)
    b.move(qpre); b.move(HANDOVER); b.wait(0.5)
    p2, R2 = b.token_pose()
    lands = int(round((p2[0] - PAN_AXIS_X - R0) / PITCH)) if p2[2] < TOP - 0.05 else None
    return dict(land=lands, z_final=p2[2], bottom=TOP - ROWS * PITCH + TOKEN_R, held=held)
mode = sys.argv[1]
if mode == "doc":   # grid plane through the pan axis; roll exactly +-pi/2, TCP aimed at the slot centre, tilt free (ik_jaw_normal of #83)
    for sign in (1, -1):
        for gap in (0.0095, 0.0105):
            b = Bench(gap=gap, offset=0.0, tokens=1, **CONTACT)
            e7_cycle.OFFSET = 0.0
            n = 0; rows = []
            for i in range(7):
                roll = sign * np.pi / 2
                p = np.array([PAN_AXIS_X + R0 + i * PITCH, 0.0, Z_SLOT + 0.008 + TOKEN_R])
                q = kin.ik_pos(p, roll=roll)
                e7_cycle.HANDOVER = np.array([0.9, -0.7, 0.1, 0.9, roll])
                r = cycle(b, i, 0.008, q_rel=q); park(b)
                rows.append((r["land"], r["err_mm"][1], r["tilt"]))
                n += ok(r, i)
            print(f"documented design, roll {sign*90:+d}, gap {gap*1e3:.1f}: right {n}/7; (land, across err mm, token tilt deg): {rows}", flush=True)
if mode == "window":
    for gap in (0.0085, 0.0095, 0.0105):
        for s in (+1, -1):
            line = []
            for e in (0.5, 1, 1.5, 2, 2.5, 3):
                n, _, _ = run(gap, 7, 0, 0, 0.008, across=s * e * 1e-3)
                line.append(f"{e:g}:{n}/7")
            print(f"across window gap {gap*1e3:.1f} side {s:+d} (h 8, no offsets, O2): " + " ".join(line), flush=True)
if mode == "off":
    k = int(sys.argv[2])
    cases = [(0.0085, 3e-3, 5e-3, 0.008, None), (0.0085, 1.5e-3, 2e-3, 0.008, None), (0.0085, 3e-3, 5e-3, -0.003, 0.020), (0.0085, 1.5e-3, 2e-3, -0.003, 0.020),
             (0.0095, 3e-3, 5e-3, 0.008, None), (0.0095, 1.5e-3, 2e-3, 0.008, None), (0.0095, 3e-3, 5e-3, -0.003, 0.020), (0.0085, 3e-3, 5e-3, 0.002, 0.020)]
    for c in cases[k::4]:
        n, held, miss = run(c[0], 70, c[1], c[2], c[3], c[4])
        print(f"gap {c[0]*1e3:.1f} offsets +-{c[1]*1e3:.1f}/+-{c[2]*1e3:.1f} mm, release h {c[3]*1e3:+.0f} mm, pre-drop {c[4]}: right {n}/70, held {held}/70, misses land {miss}", flush=True)
if mode == "guide":
    for gap in (0.0085,):
        n, held, miss = run(gap, 70, 3e-3, 5e-3, 0.023, guide=True)
        print(f"guide, gap {gap*1e3:.1f}, offsets +-3/+-5, release 8 mm above the guide: right {n}/70 held {held}/70 misses {miss}", flush=True)
if mode == "bias":
    k = int(sys.argv[2])
    cases = [(0.0085, 3e-3, 5e-3, 0.008, -0.001, False), (0.0085, 1.5e-3, 2e-3, 0.008, -0.001, False), (0.0095, 3e-3, 5e-3, 0.008, 0.0, False), (0.0095, 3e-3, 5e-3, 0.008, -0.002, False),
             (0.0085, 3e-3, 5e-3, 0.023, 0.0, True), (0.0085, 3e-3, 5e-3, 0.023, 0.003, True), (0.0085, 3e-3, 5e-3, 0.023, -0.003, True), (0.0085, 3e-3, 5e-3, 0.023, -0.005, True)]
    for c in cases[k::4]:
        n, held, miss = run(c[0], 70, c[1], c[2], c[3], None, across=c[4], guide=c[5])
        print(f"gap {c[0]*1e3:.1f} offsets +-{c[1]*1e3:.1f}/+-{c[2]*1e3:.1f}, h {c[3]*1e3:+.0f} mm, across {c[4]*1e3:+.1f} mm, guide {c[5]}: right {n}/70, held {held}/70, misses land {miss}", flush=True)
