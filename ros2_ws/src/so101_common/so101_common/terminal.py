"""terminal.py - what the demo consoles share (#284): a screen redrawn in place, a key read without Enter, and
a terminal left as it was found. Standard library only, plain ANSI sequences.
"""

import os
import select
import sys
import termios
import tty
from contextlib import contextmanager

HIDE = "\x1b[?25l\x1b[?7l"  # no cursor, and no wrap: a line too long is cut, the frame keeps its height
RESTORE = "\x1b[?7h\x1b[?25h\x1b[0m"


WIDTH = 78  # every line of a console: it never wraps in an 80-column terminal
GLYPHS = {  # name: on a terminal, then its ASCII twin; one cell each, so both layouts are the same
    "rule": "─-",
    "side": "│|",
    "done": "●+",
    "run": "►>",
    "go": "►>",  # the person acts
    "todo": "○.",
    # The multiplication sign, on purpose: one cell in every console font
    "fail": "×x",  # noqa: RUF001
    "full": "█#",
    "part": "▓=",
    "empty": "░-",
}
COLORS = {
    "head": "1;7",
    "chrome": "36",
    "key": "1",
    "done": "32",
    "go": "32",
    "run": "33",
    "fail": "31",
    "todo": "0",
}


def pretty(out=None) -> bool:
    """Whether `out` takes box characters and colors: a terminal that speaks UTF-8, and no NO_COLOR."""
    out = out or sys.stdout
    return (
        out.isatty()
        and "utf" in (getattr(out, "encoding", "") or "").lower()
        and not os.environ.get("NO_COLOR")
    )


class Pen:
    """The look the consoles share: a header, titled rules, a status line, a key bar, a progress bar. `pretty`
    on a terminal; plain ASCII with no escape code through a pipe and in the tests."""

    def __init__(self, pretty=False):
        self.pretty = pretty

    def glyph(self, name) -> str:
        return GLYPHS[name][0 if self.pretty else 1]

    def paint(self, role, text) -> str:
        """`text` in the color of `role` (COLORS), or an SGR code as it is."""
        return f"\x1b[{COLORS.get(role, role)}m{text}\x1b[0m" if self.pretty else text

    def header(self, title, tag="") -> str:
        """The screen's name on the left, the node's own state on the right."""
        left = f" SO-101 {self.glyph('side')} {title}"
        return self.paint("head", left + f"{tag} ".rjust(WIDTH - len(left)))

    def rule(self, title="") -> str:
        head = self.glyph("rule") * 2 + f" {title} " if title else ""
        return self.paint("chrome", head + self.glyph("rule") * (WIDTH - len(head)))

    def status(self, kind, text) -> str:
        """One sentence with its mark; `kind`: done, run or fail."""
        return self.paint(kind, f" {self.glyph(kind)} {text}")

    def keys(self, pairs) -> str:
        return " " + "  ".join(f"{self.paint('key', key)} {label}" for key, label in pairs)

    def bar(self, done, total, cells, current=None) -> str:
        """`done` steps of `total`, `cells` characters each; `current`: "run" or "fail", the step in hand."""
        text = self.paint("done", self.glyph("full") * done * cells) if done else ""
        if current and done < total:
            text += self.paint(current, self.glyph("part") * cells)
            done += 1
        return text + self.glyph("empty") * (total - done) * cells


def frame(text) -> str:
    """`text` painted over the frame before it: home, each line erased to its end as it is written, then what
    is left below. Clearing the screen first shows it blank between two frames."""
    return "\x1b[H" + text.replace("\n", "\x1b[K\n") + "\x1b[K\x1b[J"


def show(text):
    print(text, flush=True)  # through a pipe stdout is block-buffered: the drawing would wait


@contextmanager
def session(out=None):
    """A console's time on the terminal: a key at a time with no echo, the cursor hidden; everything is given
    back at the end, a Ctrl-C or a failure included. Through a pipe it changes nothing."""
    out = out or sys.stdout
    saved = termios.tcgetattr(sys.stdin) if sys.stdin.isatty() else None
    screen = out.isatty()
    try:
        if saved:
            tty.setcbreak(sys.stdin)
            attrs = termios.tcgetattr(sys.stdin)
            attrs[0] &= ~termios.IXON  # Ctrl-S froze the output until a Ctrl-Q
            attrs[6][termios.VSUSP] = attrs[6][termios.VQUIT] = b"\x00"  # Ctrl-Z, Ctrl-\: the console gone
            termios.tcsetattr(sys.stdin, termios.TCSANOW, attrs)
        if screen:
            out.write(HIDE)
            out.flush()
        yield
    finally:
        if screen:
            out.write(RESTORE)
            out.flush()
        if saved:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, saved)


def read_key():
    """A key typed since the last call, else None; "q" at the end of the input. Read from the descriptor, a
    byte at a time: Python's buffered input took every pending byte and gave one, so a key typed ahead
    stayed there, unseen by select, and came out at the next key press. A special key is nothing: its
    sequence holds digits (Delete is ESC [ 3 ~), which are columns."""
    stdin = sys.stdin.fileno()
    if not select.select([stdin], [], [], 0)[0]:
        return None
    byte = os.read(stdin, 1)
    if not byte:
        return "q"
    if byte == b"\x1b":
        while select.select([stdin], [], [], 0)[0]:
            if not os.read(stdin, 32):
                break
        return None
    return byte.decode(errors="ignore") or None
