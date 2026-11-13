"""Plays one game through the console of `tools/so101 play` on a pipe (#246): the PR's console output and,
with record_debug.py next to it, the README's GIF. In a container where the game's stack runs:

    python3 play_keys.py "e 1,m 7,h ? 1,7,1,7,1"

One group of keys at each "your turn", the last one a column: here the easy level then column 1, the medium
level then column 7, the hard level, a hint and column 1, then 7, 1, 7, 1; column 4 once the list is empty.
It starts a game as soon as the console says the stack is ready, and quits at the end of the game.
"""

import subprocess
import sys
import time

script = sys.argv[1].split(",")
console = subprocess.Popen(
    ["ros2", "run", "so101_bringup", "connect4_play"],
    stdin=subprocess.PIPE,
    stdout=subprocess.PIPE,
    text=True,
    bufsize=1,
)


def send(key):
    console.stdin.write(key)
    console.stdin.flush()
    time.sleep(1.5)  # the console draws what the key changed before the next one


started = sent = False
for line in console.stdout:
    print(line, end="", flush=True)
    text = line.strip()
    if not started and text.startswith("n: new game"):
        started = True
        send("n")
    elif text.startswith(("the robot plays", "the robot thinks", "the grid is read")):
        sent = False  # the turn went on: the next "your turn" is a new one
    elif text == "your turn (R): 1 to 7" and not sent:
        sent = True  # a level key draws "your turn" again: the same turn
        for key in (script.pop(0) if script else "4").split():
            send(key)
    elif started and text.startswith(("the robot wins", "you win", "a draw")):
        time.sleep(3)
        send("q")
        break
console.wait(timeout=10)
