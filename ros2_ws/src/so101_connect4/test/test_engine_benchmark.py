"""The engine's time for a move at the hard level, a nightly trend next to the IK timings (08 §2): measured,
no threshold (CI runners are too noisy for a gate; test_game.py holds the 2 s limit of P2-04)."""

import pytest

from so101_connect4 import game


def test_best_move_hard(request):
    """The first move of a game: the widest search, no proven line to cut it short.

    pytest-benchmark is in the control image only. The skip is here, not at the module's level: launch_testing
    imports each test module while pytest collects the folder, and a skip raised then skips every test of it.
    """
    pytest.importorskip("pytest_benchmark")
    choice = request.getfixturevalue("benchmark")(game.best_move, 0, 0, None, game.LEVELS["hard"])
    assert choice.depth == game.LEVELS["hard"]
