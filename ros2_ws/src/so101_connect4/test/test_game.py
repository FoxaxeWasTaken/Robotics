"""The Connect Four engine (phase2/P2-02 §4), target P2-04: only legal moves, 100/100 against a random
player, < 2 s per move. Ported from the prototype's self-check: each bit trick against brute force."""

import random
import time

import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

from so101_connect4 import game
from so101_connect4.game import CELLS, COLUMNS, EMPTY, HUMAN, ROBOT, ROWS


def at(cells, column, row):
    return cells[row * COLUMNS + column] if 0 <= column < COLUMNS and 0 <= row < ROWS else None


def four_through(cells, i, player):
    """Brute force: a line of four of `player` through cell i."""
    c0, r0 = i % COLUMNS, i // COLUMNS
    for dc, dr in ((1, 0), (0, 1), (1, 1), (1, -1)):
        for back in range(4):
            if all(at(cells, c0 + (k - back) * dc, r0 + (k - back) * dr) == player for k in range(4)):
                return True
    return False


def any_four(cells, player):
    return any(v == player and four_through(cells, i, player) for i, v in enumerate(cells))


def brute_threats(cells, player):
    """The empty cells where `player` would make four, gravity ignored, as bits."""
    bits = 0
    for i, v in enumerate(cells):
        if v == EMPTY and four_through([*cells[:i], player, *cells[i + 1 :]], i, player):
            bits |= game._bit(i % COLUMNS, i // COLUMNS)
    return bits


def random_cells(rng, stones):
    """A board reached by alternate drops, the human first; a four may be on it."""
    cells, heights, player = [EMPTY] * CELLS, [0] * COLUMNS, HUMAN
    while stones:
        column = rng.randrange(COLUMNS)
        if heights[column] < ROWS:
            cells[heights[column] * COLUMNS + column] = player
            heights[column] += 1
            stones -= 1
            player = HUMAN + ROBOT - player
    return cells


def cells_of(columns, first=HUMAN):
    """The cells after these drops from the empty board, `first` dropping first."""
    cells, heights, player = [EMPTY] * CELLS, [0] * COLUMNS, first
    for column in columns:
        cells[heights[column] * COLUMNS + column] = player
        heights[column] += 1
        player = HUMAN + ROBOT - player
    return cells


def board(stones):
    """The cells from {(column, row): player}."""
    cells = [EMPTY] * CELLS
    for (column, row), player in stones.items():
        cells[row * COLUMNS + column] = player
    return cells


def mirrored(cells):
    return [cells[r * COLUMNS + COLUMNS - 1 - c] for r in range(ROWS) for c in range(COLUMNS)]


# The bit tricks


def test_the_bit_tricks_agree_with_brute_force_on_random_boards():
    rng = random.Random(1)
    for _ in range(3000):
        cells = random_cells(rng, rng.randrange(CELLS + 1))
        me, mask = game.from_cells(cells, ROBOT)
        assert game.has_four(me) == any_four(cells, ROBOT)
        assert game.has_four(me ^ mask) == any_four(cells, HUMAN)
        assert game._threats(me, mask) == brute_threats(cells, ROBOT)
        assert game.legal_columns(mask) == [
            c for c in range(COLUMNS) if cells[(ROWS - 1) * COLUMNS + c] == EMPTY
        ]
        for column in game.legal_columns(mask):
            row = game.row_of(mask, column)
            assert cells[row * COLUMNS + column] == EMPTY
            assert row == 0 or cells[(row - 1) * COLUMNS + column] != EMPTY
            assert game.play(me, mask, column) == (me ^ mask, mask | game._bit(column, row))


# The rules


@pytest.mark.parametrize(
    "moves",
    [
        pytest.param([0, 1, 0, 1, 0, 1, 0], id="vertical"),
        pytest.param([0, 0, 1, 1, 2, 2, 3], id="horizontal"),
        pytest.param([0, 1, 1, 2, 2, 3, 2, 3, 3, 6, 3], id="diagonal-up"),
        pytest.param([6, 5, 5, 4, 4, 3, 4, 3, 3, 0, 3], id="diagonal-down"),
    ],
)
def test_four_in_a_row_wins_in_the_four_directions(moves):
    assert game.winner(cells_of(moves[:-1])) == EMPTY
    assert game.winner(cells_of(moves)) == HUMAN
    assert game.winner(cells_of(moves, first=ROBOT)) == ROBOT


DRAW = [int(c) for row in ("2121212", "2121212", "1212121", "1212121", "2121212", "2121212") for c in row]


def test_a_full_board_without_four_in_a_row_is_a_draw():
    assert game.winner(DRAW) == EMPTY
    me, mask = game.from_cells(DRAW, ROBOT)
    assert game.legal_columns(mask) == []
    with pytest.raises(ValueError, match="no legal move"):
        game.best_move(me, mask, None, 4)


def test_a_full_column_takes_no_more_token():
    _, mask = game.from_cells(cells_of([0] * ROWS), HUMAN)
    assert 0 not in game.legal_columns(mask)
    assert game.row_of(mask, 0) == ROWS


def test_the_last_two_cells_of_a_drawn_game_score_0():
    cells = list(DRAW)
    cells[(ROWS - 1) * COLUMNS : (ROWS - 1) * COLUMNS + 2] = [EMPTY, EMPTY]  # top-left two
    assert game.best_move(*game.from_cells(cells, HUMAN), None, 4).score == 0


def test_the_last_empty_cell_is_played_at_once():
    cells = list(DRAW)
    cells[(ROWS - 1) * COLUMNS] = EMPTY  # top-left
    assert game.best_move(*game.from_cells(cells, HUMAN), None, 4).column == 0


# Known positions


def test_it_takes_a_win_at_once():
    cells = [EMPTY] * CELLS
    cells[0:3] = [ROBOT] * 3
    cells[7:10] = [HUMAN] * 3
    assert game.best_move(*game.from_cells(cells, ROBOT), None, 6).column == 3


def test_it_blocks_a_win_of_the_opponent():
    cells = [EMPTY] * CELLS
    cells[0:3] = [HUMAN] * 3
    cells[5:7] = [ROBOT] * 2
    assert game.best_move(*game.from_cells(cells, ROBOT), None, 6).column == 3


def test_it_never_plays_just_below_a_winning_cell_of_the_opponent():
    # the human holds columns 1 to 3 of row 1: a robot token in column 0 or 4 lets the human complete the row
    cells = board(
        {(1, 0): ROBOT, (2, 0): HUMAN, (3, 0): ROBOT, (5, 0): ROBOT, (5, 1): ROBOT}
        | {(c, 1): HUMAN for c in (1, 2, 3)}
    )
    assert game.best_move(*game.from_cells(cells, ROBOT), None, 6).column not in (0, 4)


def test_an_open_three_on_the_bottom_row_wins_whatever_the_reply():
    me, mask = game.from_cells(cells_of([2, 2, 3, 3], first=ROBOT), ROBOT)
    me, mask = game.play(me, mask, game.best_move(me, mask, None, 4).column)  # the human to move
    for reply in game.legal_columns(mask):
        robot, after = game.play(me, mask, reply)
        assert game.best_move(robot, after, None, 2).score > game._IS_MATE, f"reply {reply}"


def test_it_opens_in_the_center_column():
    assert game.best_move(0, 0, None, 6).column == COLUMNS // 2


def test_equal_moves_go_to_the_most_central_column_whatever_the_seed():
    """#244. At depth 1 every first move scores the same: the centre, not one of the 7 drawn by the seed."""
    assert {game.best_move(0, 0, None, 1, seed=seed).column for seed in range(30)} == {COLUMNS // 2}


def test_the_seed_only_decides_between_columns_as_central():
    me, mask = game.from_cells(cells_of([3] * ROWS), HUMAN)  # symmetric, center full: mirrored moves tie
    chosen = {game.best_move(me, mask, None, 2, seed=seed).column for seed in range(30)}
    assert chosen == {2, 4}


@pytest.mark.parametrize(
    ("stones", "plies"),
    [
        pytest.param(
            {(1, 0): ROBOT, (2, 0): ROBOT, (3, 0): ROBOT, (1, 1): HUMAN, (2, 1): HUMAN}, 1, id="win now"
        ),
        pytest.param({(2, 0): ROBOT, (3, 0): ROBOT, (2, 1): HUMAN, (3, 1): HUMAN}, 3, id="open three next"),
        pytest.param(
            {(2, 0): HUMAN, (3, 0): HUMAN, (4, 0): HUMAN, (6, 0): ROBOT, (6, 1): ROBOT}, -2, id="lost"
        ),
        pytest.param({(3, 0): HUMAN}, 0, id="not proven"),
    ],
)
def test_a_proven_score_gives_the_plies_to_the_end(stones, plies):
    """#244: > 0 the side to move wins in that many plies, its own included; < 0 it loses; 0 not proven."""
    cells = board(stones)
    me, mask = game.from_cells(cells, ROBOT)
    choice = game.best_move(me, mask, None, 4)
    assert game.ends_in(choice.score, len(stones)) == plies


def test_the_hard_level_beats_the_easy_one():
    """#244, [MES-70]: 20 games from random two-token openings, each side first in half of them."""
    wins = {"easy": 0, "hard": 0}
    for seed in range(20):
        rng = random.Random(seed)
        me = mask = 0
        for _ in range(2):
            me, mask = game.play(me, mask, rng.randrange(COLUMNS))
        sides = ("easy", "hard") if seed % 2 == 0 else ("hard", "easy")
        for ply in range(CELLS - 2):
            me, mask = game.play(
                me, mask, game.best_move(me, mask, None, game.LEVELS[sides[ply % 2]], seed).column
            )
            if game.has_four(me ^ mask):
                wins[sides[ply % 2]] += 1
                break
    assert wins["hard"] >= 14
    assert wins["easy"] <= 2


# The search


def test_a_seed_replays_the_same_move_and_breaks_the_ties():
    me, mask = game.from_cells(cells_of([3] * ROWS), HUMAN)  # symmetric, center full: mirrored moves tie
    chosen = {game.best_move(me, mask, None, 2, seed=seed).column for seed in range(20)}
    assert len(chosen) >= 2
    assert {COLUMNS - 1 - c for c in chosen} == chosen
    assert len({game.best_move(me, mask, None, 2, seed=7)[:3] for _ in range(3)}) == 1


def test_the_time_budget_stops_the_search_and_keeps_the_last_complete_depth():
    start = time.monotonic()
    choice = game.best_move(0, 0, budget_s=0.2)
    assert time.monotonic() - start < 0.5
    assert 1 <= choice.depth < CELLS
    assert choice.column in range(COLUMNS)


@pytest.mark.parametrize("moves", [(), (3, 3, 2, 4), (3, 2, 3, 3, 4, 4, 2, 5, 1)])
def test_a_move_with_the_1_s_budget_takes_less_than_2_s(moves):
    me, mask = game.from_cells(cells_of(moves), HUMAN if len(moves) % 2 == 0 else ROBOT)
    start = time.monotonic()
    game.best_move(me, mask, budget_s=1.0)
    assert time.monotonic() - start < 2.0


def test_it_wins_100_games_out_of_100_against_a_seeded_random_player_at_depth_4():
    not_won = []
    for seed in range(100):
        rng = random.Random(seed)
        me, mask, engine_turn = 0, 0, seed % 2 == 0  # the engine first in half of the games
        while True:
            legal = game.legal_columns(mask)
            column = game.best_move(me, mask, None, 4, seed=seed).column if engine_turn else rng.choice(legal)
            assert column in legal
            me, mask = game.play(me, mask, column)
            if game.has_four(me ^ mask):  # the side that just played
                break
            if mask.bit_count() == CELLS:
                engine_turn = False  # a draw is not a win
                break
            engine_turn = not engine_turn
        if not engine_turn:
            not_won.append(seed)
    assert not_won == []


# Properties (Hypothesis): any reachable position


@st.composite
def reachable(draw):
    """(cells, player to move) reached by legal drops from the empty board, nobody has won, not full."""
    cells, heights, player = [EMPTY] * CELLS, [0] * COLUMNS, draw(st.sampled_from([HUMAN, ROBOT]))
    for column in draw(st.lists(st.integers(0, COLUMNS - 1), max_size=CELLS - 1)):
        if heights[column] == ROWS:
            continue
        nxt = list(cells)
        nxt[heights[column] * COLUMNS + column] = player
        if game.winner(nxt) != EMPTY:
            break
        cells, heights[column], player = nxt, heights[column] + 1, HUMAN + ROBOT - player
    return cells, player


@settings(max_examples=60, deadline=None)
@given(reachable())
def test_it_only_plays_legal_moves(position):
    cells, player = position
    me, mask = game.from_cells(cells, player)
    assert game.best_move(me, mask, None, 3).column in game.legal_columns(mask)


@settings(max_examples=60, deadline=None)
@given(reachable())
def test_the_mirrored_position_gets_the_same_score(position):
    cells, player = position
    assert (
        game.best_move(*game.from_cells(cells, player), None, 4).score
        == game.best_move(*game.from_cells(mirrored(cells), player), None, 4).score
    )
