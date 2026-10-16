import time, cv2

def fourcc_str(v):
    v = int(v)
    return "".join(chr((v >> 8 * i) & 0xFF) for i in range(4))

for dev in ["/dev/video0", "/dev/video2"]:
    for fmt, (w, h), fps in [(None, (640, 480), None), ("MJPG", (640, 480), 30), ("MJPG", (320, 240), 15), ("YUYV", (320, 240), 15)]:
        c = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if not c.isOpened():
            print(dev, "cannot open"); break
        if fmt:
            c.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fmt))
        c.set(cv2.CAP_PROP_FRAME_WIDTH, w); c.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        if fps:
            c.set(cv2.CAP_PROP_FPS, fps)
        got = fourcc_str(c.get(cv2.CAP_PROP_FOURCC)), int(c.get(cv2.CAP_PROP_FRAME_WIDTH)), int(c.get(cv2.CAP_PROP_FRAME_HEIGHT))
        t = time.time(); n = 0; shape = None
        while time.time() - t < 4:
            ok, f = c.read()
            if ok:
                n += 1; shape = f.shape
        dt = time.time() - t
        print(f"{dev} asked={fmt or 'default'} {w}x{h} -> got={got} frames={n} ({n/dt:.1f} fps) shape={shape}", flush=True)
        c.release()
