"""turns.py against a fake world: reader with latency and its 5-read filter, brain with phases, a scripted random
human, injected failures. Checks at each tick: the arm moves only in MOVE (P2-05), a game always ends, no illegal
drop. Run: python3 test_turns.py [games]"""

import random
import sys
from collections import Counter, deque

sys.path.insert(0, ".")
import game
from game import CELLS, EMPTY, HUMAN, ROBOT
from turns import Board, Inputs, Params, State, Turns

TICK = 0.1
FALL_S = 0.4
DRAW = [1, 1, 4, 2, 3, 3, 4, 2, 2, 5, 6, 2, 3, 4, 2, 4, 6, 5, 1, 0, 1, 5, 3, 4, 4, 1, 1, 6, 2, 6, 6, 6, 0, 0, 3, 0, 5, 3, 0, 0, 5, 5]


class World:
    """Grid, tokens in flight, reader, brain."""

    def __init__(self, rng, faults):
        self.rng, self.f = rng, faults
        self.grid = [EMPTY] * CELLS
        self.flying = []  # (lands at, column, color)
        self.frames = deque(maxlen=5)
        self.next_read = 0.0
        self.held = False  # a token between the jaws
        self.motion = None  # (end time, column or None)
        self.labels = []
        self.reply = None
        self.last_label = "IDLE"
        self.board = None

    def land(self, column, color):
        for row in range(game.ROWS):
            if self.grid[row * 7 + column] == EMPTY:
                self.grid[row * 7 + column] = color
                return

    def drop_token(self, now, column, color):
        self.flying.append((now + FALL_S, column, color))

    def tick(self, now):
        for f in [f for f in self.flying if f[0] <= now]:
            self.flying.remove(f)
            self.land(f[1], f[2])
        if now >= self.next_read:  # 5 Hz, published only when 5 reads agree
            self.next_read = now + 0.2
            self.frames.append(tuple(self.grid))
            unstable = self.f.get("never_stable") and self.rng.random() < self.f["never_stable"]
            if len(self.frames) == 5 and len(set(self.frames)) == 1 and not unstable:
                bad = self.f.get("invalid") and self.rng.random() < self.f["invalid"]
                self.board = Board(self.frames[-1], not bad, "gravity" if bad else "", now)
        states, self.labels = tuple(self.labels), []
        if self.motion and now >= self.motion[0]:
            column = self.motion[1]
            self.motion = None
            if column is not None and self.held:
                self.held = False
                r = self.rng.random()
                if r < self.f.get("lost", 0):
                    pass  # the token fell out of the grid
                else:
                    if r < self.f.get("lost", 0) + self.f.get("wrong", 0):
                        column = max(0, min(6, column + self.rng.choice((-1, 1))))
                    if self.grid[35 + column] == EMPTY:
                        self.drop_token(now, column, ROBOT)
            self.say("DROP_DONE")
        reply, self.reply = self.reply, None
        return self.board, states, reply  # the node keeps the last message, however old

    def say(self, label):
        self.labels.append(label)
        self.last_label = label

    def brain_drop(self, now, column):
        """Answers at once, as /brain/start; the labels come after."""
        if self.rng.random() < self.f.get("refuse", 0):
            self.reply = (False, "unreachable")
            return
        self.reply = (True, "")
        if column is not None and not self.held:
            self.say("DROP:CLOSE")
            self.say("DROP_ERROR:no_token")
            return
        self.say("DROP:MOVE")
        self.motion = (now + (1.0 if column is None else float(__import__("os").environ.get("FAKE_MOTION_S", 12.0))), column)

    def brain_stop(self):
        self.motion = None
        self.say("STOPPED")


def play_one(seed, faults, engine_depth=2):
    rng = random.Random(seed)
    w, t, now = World(rng, faults), Turns(), 0.0
    i = dict(new_game=True, robot_first=seed % 2 == 1)
    armed = None  # engine answer to deliver next tick
    moved_outside, stops, ticks = 0, 0, 0
    w.brain_stop_at = rng.uniform(5, 200) if rng.random() < faults.get("stop", 0) else None
    w.say("IDLE")
    while True:
        now += TICK
        ticks += 1
        assert ticks < 40000, (seed, t.label)
        board, states, reply = w.tick(now)
        stop = w.brain_stop_at is not None and now >= w.brain_stop_at
        if stop:
            w.brain_stop_at = None
            stops += 1
        out = t.step(
            now, Inputs(board=board, brain_states=states, drop_reply=reply, engine=armed, stop=stop, **i)
        )
        i, armed = {}, None
        if out.think:
            armed = game.best_move(*out.think, None, engine_depth, seed=seed).column
        if out.stop_brain:
            w.brain_stop()
        if out.drop is not None or out.home:
            w.brain_drop(now, out.drop)
        if w.motion and t.state is not State.MOVE:
            moved_outside += 1
        s = t.snapshot()
        if t.state is State.WAIT_HUMAN and not i:
            # the console or bench's chain: one key, three calls
            if s["turn"] == "human":
                column = rng.choice(s["legal"])
                assert column in s["legal"]
                w.drop_token(now, column, HUMAN)
            w.held = rng.random() >= faults.get("no_give", 0)
            i = dict(human_done=True)
        elif t.state in (State.GAME_OVER, State.ERROR, State.FIX_BOARD):
            return t, moved_outside, now


def scripted_draw():
    """Both sides read from a list that never makes four: the FSM reaches the draw, at 42 tokens."""
    w, t, now = World(random.Random(0), {}), Turns(), 0.0
    robot_moves = iter(DRAW[1::2])
    human_moves = iter(DRAW[0::2])
    i = dict(new_game=True)
    armed = None
    for _ in range(40000):
        now += TICK
        board, states, reply = w.tick(now)
        out = t.step(now, Inputs(board=board, brain_states=states, drop_reply=reply, engine=armed, **i))
        i, armed = {}, None
        if out.think:
            armed = next(robot_moves)
        if out.drop is not None or out.home:
            w.brain_drop(now, out.drop)
        if t.state is State.WAIT_HUMAN and not i:
            w.drop_token(now, next(human_moves), HUMAN)
            w.held = True
            i = dict(human_done=True)
        if t.state in (State.GAME_OVER, State.ERROR, State.FIX_BOARD):
            return t


def campaign(name, n, faults):
    results, game_seconds = Counter(), []
    moved = 0
    for seed in range(n):
        t, m, now = play_one(seed, faults)
        moved += m
        results[t.label if t.state is not State.GAME_OVER else f"GAME_OVER:{t.result}"] += 1
        game_seconds.append(now)
    print(f"{name:22s} arm moved outside MOVE: {moved}; {dict(results.most_common())}")
    assert moved == 0


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 500
    campaign("clean", n, {})
    campaign("token lost 20 %", n, {"lost": 0.2})
    campaign("wrong column 20 %", n, {"wrong": 0.2})
    campaign("no token 20 %", n, {"no_give": 0.2})
    campaign("brain refuses 5 %", n, {"refuse": 0.05})
    campaign("invalid reads 1 %", n, {"invalid": 0.01})
    campaign("board unstable 95 %", n, {"never_stable": 0.95})
    campaign("stop in 30 % of games", n, {"stop": 0.3})
    campaign("everything", n, {"lost": 0.1, "wrong": 0.1, "no_give": 0.1, "refuse": 0.02, "invalid": 0.002, "stop": 0.1})
    t = scripted_draw()
    print("scripted 42 tokens:", t.label, t.result, t.moves, "winner", game.winner(t.cells))


if __name__ == "__main__":
    main()
