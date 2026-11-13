"""The README's game GIF and the picture of a circled four (#246, #234), from /connect4/debug_image.

    python3 record_debug.py record <dir> [every]   # in a container, a game running; SIGTERM or Ctrl-C ends it
    python3 record_debug.py gif <dir> <out.gif> [step]   # anywhere with Pillow (the images have none): the host

`record` keeps one image in `every` (5) as <dir>/0001.png, ...: the reader publishes at 10 Hz. `gif` writes one
of them in `step` (2) at half size in 64 colors, 8 images/s: the game at 8x; the last image stays 3 s. It is
the board at the end, the winning four circled. The README's GIF is a game of 9 tokens with the robot taking
its tokens at the dispenser, two minutes: `step` 4, the game at 16x, to stay under 1 MB (docs/project/15 §7).
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


if __name__ == "__main__":
    if sys.argv[1] == "record":
        record(Path(sys.argv[2]), int(sys.argv[3]) if len(sys.argv) > 3 else 5)
    else:
        gif(Path(sys.argv[2]), sys.argv[3], *(int(a) for a in sys.argv[4:5]))
