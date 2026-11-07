"""E8: capture: how far off can the token be, across and along, without a guide? Random handover offsets."""
import numpy as np, json, sys
from e7_cycle import *
def ok(r, i):
    return r["land"] == i and abs(r["z_final"] - r["bottom"]) < 0.01
mode = sys.argv[1]
if mode == "across":
    for gap in (0.0085, 0.0095, 0.0105):
        b = Bench(gap=gap, offset=OFFSET, tokens=1, **CONTACT)
        for h in (0.005, 0.010):
            line = []
            for e in (0, 1, 1.5, 2, 2.5, 3, 4, 5, 6):
                n = 0; tot = 0
                for s in (+1, -1):
                    for i in (0, 3, 6):
                        r = cycle(b, i, h, across=s * e * 1e-3); park(b)
                        tot += 1; n += ok(r, i)
                line.append(f"{e:g}mm:{n}/{tot}")
            print(f"across error, gap {gap*1e3:.1f} h {h*1e3:.0f}: " + " ".join(line), flush=True)
if mode == "along":
    b = Bench(gap=0.0095, offset=OFFSET, tokens=1, **CONTACT)
    for h in (0.005, 0.010):
        line = []
        for e in (0, 2, 4, 6, 8, 10, 12, 14, 16, 18):
            n = 0; tot = 0; wrong = 0; stuck = 0
            for s in (+1, -1):
                for i in (1, 3, 5):
                    r = cycle(b, i, h, along=s * e * 1e-3); park(b)
                    tot += 1; n += ok(r, i); wrong += (r["land"] is not None and r["land"] != i); stuck += r["land"] is None
            line.append(f"{e:g}mm:{n}/{tot}(wrong {wrong},stuck {stuck})")
        print(f"along error, gap 9.5 h {h*1e3:.0f}: " + " ".join(line), flush=True)
if mode == "hand":
    rng = np.random.default_rng(3)
    for gap, N in ((0.0095, 70), (0.0085, 70)):
        b = Bench(gap=gap, offset=OFFSET, tokens=1, **CONTACT)
        fails = 0; rows = []
        for k in range(N):
            i = k % 7
            dy, dz = rng.uniform(-3e-3, 3e-3), rng.uniform(-5e-3, 5e-3)
            r = cycle(b, i, 0.008, dy=dy, dz=dz); park(b)
            rows.append((r["err_mm"][0], r["err_mm"][1], r["err_mm"][2], r["tilt"], r["held"], ok(r, i)))
            if not ok(r, i):
                fails += 1
                print("  miss", i, round(dy*1e3,1), round(dz*1e3,1), json.dumps(r)[:300], flush=True)
        x = np.array(rows, dtype=float)
        print(f"handover offsets +-3 mm width, +-5 mm depth, gap {gap*1e3:.1f}, h 8: right {N-fails}/{N}; token error at release along {x[:,0].min():.1f}..{x[:,0].max():.1f}, across {x[:,1].min():.2f}..{x[:,1].max():.2f}, z {x[:,2].min():.1f}..{x[:,2].max():.1f} mm, tilt {x[:,3].min():.2f}..{x[:,3].max():.2f} deg, held {int(x[:,4].sum())}/{N}", flush=True)
