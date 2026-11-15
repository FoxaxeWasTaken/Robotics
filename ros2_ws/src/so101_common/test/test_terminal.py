"""terminal.py (#284): what the two demo consoles share to draw a screen in place and read a key, and to leave
the terminal as they found it."""

import os
import pty
import sys
import termios

import pytest

from so101_common import terminal


def test_a_frame_is_painted_over_the_one_before_with_no_blank_screen_between():
    """Clearing the screen then writing blinks at 10 frames a second: home, each line erased to its end as it
    is written, then what is left under the frame."""
    assert terminal.frame("a\nbc") == "\x1b[Ha\x1b[K\nbc\x1b[K\x1b[J"


class Tty:
    def __init__(self):
        self.written = ""

    def isatty(self):
        return True

    def write(self, text):
        self.written += text

    def flush(self):
        pass


@pytest.fixture
def keyboard(monkeypatch):
    master, slave = pty.openpty()
    stdin = os.fdopen(slave, "r")
    monkeypatch.setattr(sys, "stdin", stdin)
    yield stdin
    stdin.close()
    os.close(master)


def test_a_session_takes_a_key_at_a_time_hides_the_cursor_and_gives_everything_back(keyboard):
    before, out = termios.tcgetattr(keyboard), Tty()
    with terminal.session(out):
        inside = termios.tcgetattr(keyboard)
        assert not inside[3] & termios.ICANON and not inside[3] & termios.ECHO  # a key at a time, not shown
        assert not inside[0] & termios.IXON  # Ctrl-S froze the screen
        assert inside[6][termios.VSUSP] == b"\x00"  # Ctrl-Z stopped the console for good
        assert "\x1b[?25l" in out.written and "\x1b[?25h" not in out.written
    assert termios.tcgetattr(keyboard) == before
    assert out.written.endswith("\x1b[?25h\x1b[0m")  # the cursor back, no color left


def test_a_session_gives_the_terminal_back_when_the_console_fails(keyboard):
    before, out = termios.tcgetattr(keyboard), Tty()
    with pytest.raises(RuntimeError), terminal.session(out):
        raise RuntimeError("the console failed")
    assert termios.tcgetattr(keyboard) == before and "\x1b[?25h" in out.written


def test_through_a_pipe_a_session_changes_nothing(monkeypatch, capsys):
    with open(os.devnull) as nothing:
        monkeypatch.setattr(sys, "stdin", nothing)
        with terminal.session():
            pass
    assert capsys.readouterr().out == ""


# The pen: the consoles' shared look, box characters and colors on a terminal, their ASCII twins elsewhere

PLAIN, PRETTY = terminal.Pen(), terminal.Pen(pretty=True)


def strip(text):
    import re

    return re.sub(r"\x1b\[[0-9;]*m", "", text)


def test_the_header_names_the_screen_on_the_left_and_the_state_on_the_right_over_the_whole_width():
    line = PLAIN.header("Pick & place, simulation", "DESCEND")
    assert line.startswith(" SO-101 | Pick & place, simulation") and line.endswith("DESCEND ")
    assert len(line) == terminal.WIDTH
    pretty = PRETTY.header("Pick & place, simulation", "DESCEND")
    assert pretty.startswith("\x1b[1;7m") and len(strip(pretty)) == terminal.WIDTH and "│" in pretty


def test_a_rule_carries_its_title_to_the_full_width():
    assert PLAIN.rule() == "-" * terminal.WIDTH
    assert PLAIN.rule("cycle 3").startswith("-- cycle 3 ---") and len(PLAIN.rule("cycle 3")) == terminal.WIDTH
    assert strip(PRETTY.rule("cycle 3")).startswith("── cycle 3 ───")


def test_the_plain_pen_writes_no_escape_code_and_only_ascii():
    lines = [
        PLAIN.header("Connect Four", "MOVE"),
        PLAIN.rule("game"),
        PLAIN.status("run", "the robot plays"),
        PLAIN.keys([("n", "new cycle"), ("q", "quit")]),
        PLAIN.bar(3, 10, 5, "run"),
    ]
    assert all(line.isascii() and "\x1b" not in line for line in lines)


def test_a_status_has_its_mark_and_the_key_bar_its_keys():
    assert PLAIN.status("done", "cube in the box") == " + cube in the box"
    assert PLAIN.status("fail", "the camera lost the cube") == " x the camera lost the cube"
    assert strip(PRETTY.status("run", "cycle 3")) == " ► cycle 3"
    assert PLAIN.keys([("n", "new cycle"), ("s", "stop"), ("q", "quit")]) == " n new cycle  s stop  q quit"


def test_the_bar_shows_the_steps_done_the_one_in_hand_and_the_ones_to_come():
    assert PLAIN.bar(3, 10, 2, "run") == "######==" + "-" * 12
    assert PLAIN.bar(10, 10, 2) == "#" * 20
    assert PLAIN.bar(0, 10, 2) == "-" * 20
    assert strip(PRETTY.bar(1, 3, 2, "fail")) == "██▓▓░░"
    assert "\x1b[31m▓▓" in PRETTY.bar(1, 3, 2, "fail")  # the step that failed, in red


def test_pretty_needs_a_terminal_that_speaks_utf8_and_no_no_color(monkeypatch):
    class Out(Tty):
        encoding = "utf-8"

    monkeypatch.delenv("NO_COLOR", raising=False)
    assert terminal.pretty(Out())
    ascii_only = Out()
    ascii_only.encoding = "ANSI_X3.4-1968"
    assert not terminal.pretty(ascii_only)
    monkeypatch.setenv("NO_COLOR", "1")
    assert not terminal.pretty(Out())
    with open(os.devnull, "w") as file:  # a pipe, a file
        assert not terminal.pretty(file)
