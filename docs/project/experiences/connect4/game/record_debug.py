"""The README's game GIF, the phase-2 video and the picture of a circled four (#246, #234, #97), from
/connect4/debug_image.

    python3 record_debug.py record <dir> [every]   # in a container, a game running; SIGTERM or Ctrl-C ends it
    python3 record_debug.py gif <dir> <out.gif> [step]   # anywhere with Pillow (the images have none): the host
    python3 record_debug.py mp4 <dir> <out.mp4> [first] [speed]   # in a container (OpenCV)

`record` keeps one image in `every` (5) as <dir>/0001.png, ...: the reader publishes at 10 Hz. `gif` writes one
of them in `step` (2) at half size in 64 colors, 8 images/s: the game at 8x; the last image stays 3 s. It is
the board at the end, the winning four circled. The README's GIF is a game of 9 tokens with the robot taking
its tokens at the dispenser, two minutes: `step` 4, the game at 16x, to stay under 1 MB (docs/project/15 §7).
`mp4` writes every image from the `first` (1) at `speed` (4) times the reader's 10 Hz, the speed written in a
corner, the last image held 3 s: record with `every` 1. It goes to a GitHub Release, never in Git.
"""

import signal
import sys
from pathlib import Path


def record(folder, every):
    import cv2
    import rclpy
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image

    bridge, count = CvBridge(), [0, 0]  # seen, kept

    def on_image(msg):
        count[0] += 1
        if count[0] % every == 0:
            count[1] += 1
            bgr = cv2.cvtColor(bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8"), cv2.COLOR_RGB2BGR)
            cv2.imwrite(str(folder / f"{count[1]:04d}.png"), bgr)

    def stop(*_):
        raise KeyboardInterrupt

    folder.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node("record_debug")
    node.create_subscription(Image, "/connect4/debug_image", on_image, 1)  # Reliable: every image [MES-67]
    signal.signal(signal.SIGTERM, stop)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f"{count[1]} images of {count[0]} kept")


def gif(folder, out, step=2):
    from PIL import Image

    paths = sorted(folder.glob("*.png"))
    frames = [Image.open(path) for path in [*paths[:-1:step], paths[-1]]]
    small = [f.resize((f.width // 2, f.height // 2)).quantize(64) for f in frames]
    pauses = [125] * (len(small) - 1) + [3000]
    small[0].save(out, save_all=True, append_images=small[1:], duration=pauses, loop=0, optimize=True)
    print(f"{len(small)} images, {Path(out).stat().st_size / 1e6:.2f} MB")


def mp4(folder, out, first=1, speed=4):
    import cv2

    paths = sorted(folder.glob("*.png"))[first - 1 :]
    fps = 10 * speed  # the reader publishes at 10 Hz
    height, width = cv2.imread(str(paths[0])).shape[:2]
    video = cv2.VideoWriter(str(out), cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height))  # avc1: not in the image
    assert video.isOpened(), f"cannot write {out}"
    for path in [*paths, *[paths[-1]] * (3 * fps)]:
        frame = cv2.imread(str(path))
        for color, thickness in (((0, 0, 0), 4), ((255, 255, 255), 2)):  # white on a black edge
            cv2.putText(frame, f"{speed}x", (width - 60, height - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, thickness)
        video.write(frame)
    video.release()
    print(f"{len(paths)} images, {(len(paths) / fps + 3):.1f} s, {Path(out).stat().st_size / 1e6:.2f} MB")


if __name__ == "__main__":
    if sys.argv[1] == "record":
        record(Path(sys.argv[2]), int(sys.argv[3]) if len(sys.argv) > 3 else 5)
    elif sys.argv[1] == "mp4":
        mp4(Path(sys.argv[2]), sys.argv[3], *(int(a) for a in sys.argv[4:6]))
    else:
        gif(Path(sys.argv[2]), sys.argv[3], *(int(a) for a in sys.argv[4:5]))
