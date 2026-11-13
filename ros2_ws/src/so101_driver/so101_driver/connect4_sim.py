"""connect4_sim.py - the simulated person of the Connect Four game (docs/project/phase2/P2-02 §7, #80): drops
a token into a column, puts one between the open jaws, clears the board, reads the true board. No ROS.

The scene is so101_bringup.sim_scene's: a `grid` body whose frame is the grid frame (so101_connect4.layout),
tokens named token_red_<i> and token_yellow_<i>, each on a free joint, parked where the model starts them.
The layout's token.robot_color says which color the robot plays; the person plays the other (#255).

The caller holds the driver's lock. Only MjData is written, never the model (the camera reads it unlocked),
and nothing steps the simulation: mj_forward only refreshes the poses. One exception, move_grid: the grid's
pose in the model and, for one forward pass, the sleep flag; the camera draws the poses of a copy of MjData
and reads neither.
"""

import mujoco
import numpy as np

HUMAN, ROBOT = 1, 2  # so101_interfaces: DropToken's color, Connect4Board's cells
DROP_HEIGHT = 0.02  # m: a dropped token starts that far above a full column, inside the depth guide
MIN_OPEN_PCT = 30.0  # give_token needs the jaws that open
OFF_FIXED_JAW = 0.0005  # m between the given token's face and the fixed jaw
GIVE_ALONG, GIVE_DEPTH = 0.003, 0.005  # m: how far a hand misses the TCP, along the grid and in depth
PARKED = 0.01  # m: a token that close to its start pose is a spare one
IN_JAWS = 0.03  # m from the TCP: a token that close is between the jaws
NO_DISPENSER = "no dispenser in this scene"
OFF_RAIL = 0.005  # m: a robot's token further than that from its rail's line has left the dispenser
AT_STATION = 0.001  # m: a queue whose first token is further from its station is on its way down
WAKE_FORCE = 1e-6  # N, downwards, for one tick: it wakes a sleeping token, it moves nothing [MES-75]
AT_REST = 0.05  # m/s: slower, a token in a cell is on the board
IN_CELL = 0.25  # of a pitch, around a cell's centre
DISC_IN_SLOT = np.array([[1.0, 0, 0], [0, 0, 1], [0, -1, 0]])  # a token's axes in the grid frame: its z on y
DISC_ON_JAW = np.array([[0.0, 0, 1], [1, 0, 0], [0, 1, 0]])  # in the gripper body: its z on x, the jaws' axis


class Connect4Sim:
    def __init__(self, sim, layout, rng):
        """sim: the connected SO101Sim (its model and data); layout: connect4_grid.yaml; rng: the hand's."""
        self._sim, self._rng = sim, rng
        grid = layout["grid"]
        self._columns, self._rows = grid["columns"], grid["rows"]
        self._pitch, self._row_pitch = grid["column_pitch"], grid["row_pitch"]
        self._slot = grid["slot_gap"] / 2 + grid["plate"]  # half the grid's thickness
        self._on_jaw = layout["token"]["thickness"] / 2 + OFF_FIXED_JAW
        self._diameter = layout["token"]["diameter"]
        model = sim.model
        self._grid = model.body("grid").id
        self._grid_start = model.body_pos[self._grid].copy(), model.body_quat[self._grid].copy()
        robot = layout["token"]["robot_color"]  # red or yellow, the person plays the other (#255)
        self._color = {HUMAN: "yellow" if robot == "red" else "red", ROBOT: robot}
        self._tokens = {color: [] for color in self._color.values()}
        for body in range(model.nbody):
            name = model.body(body).name
            if name.startswith("token_"):
                self._tokens[name.split("_")[1]].append(name)
        for names in self._tokens.values():
            names.sort(key=lambda name: int(name.rsplit("_", 1)[1]))
        self._jaws = [self._contact_geoms("gripper"), self._contact_geoms("moving_jaw_so101_v1")]
        self.floating = None  # the token given and not yet pinched: its gravity is cancelled
        # The token dispenser (ADR-041), when the scene has one: the robot's tokens start in its rails
        self._dispenser = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "dispenser")
        if self._dispenser >= 0:
            slope, self._per_rail = layout["dispenser"]["slope"], layout["dispenser"]["tokens_per_rail"]
            self._up_rail = np.array([np.cos(slope), 0.0, np.sin(slope)])  # in the dispenser's frame
        self._nudged = []  # the tokens the last tick woke

    def _contact_geoms(self, body) -> set:
        model = self._sim.model
        body = model.body(body).id
        return {g for g in range(model.ngeom) if model.geom_bodyid[g] == body and model.geom_contype[g]}

    def _addresses(self, name):
        joint = self._sim.model.joint(f"{name}_free")
        return joint.qposadr[0], joint.dofadr[0]

    def _place(self, name, position, axes):
        """A token at rest at `position`, its axes the columns of `axes` (world)."""
        data, (q, v) = self._sim.data, self._addresses(name)
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, np.ascontiguousarray(axes).ravel())
        data.qpos[q : q + 3], data.qpos[q + 3 : q + 7] = position, quat
        data.qvel[v : v + 6] = 0
        data.qfrc_applied[v : v + 6] = 0

    def _in_rail(self, name):
        """How far down its rail a robot's token is from where it started (m, 0 or more), or None when it
        left the rail."""
        model, data = self._sim.model, self._sim.data
        q, _ = self._addresses(name)
        up = data.xmat[self._dispenser].reshape(3, 3) @ self._up_rail
        moved = data.qpos[q : q + 3] - model.qpos0[q : q + 3]
        down = -(moved @ up)
        aside = np.linalg.norm(moved + down * up)
        in_rail = name != self.floating and aside < OFF_RAIL and down > -OFF_RAIL
        return down if in_rail else None

    def _rails(self) -> list:
        """For each rail, its tokens still in it, the nearest to the station first: (name, how far down)."""
        names = self._tokens[self._color[ROBOT]]
        rails = []
        for first in range(0, len(names), self._per_rail):
            queue = [(name, self._in_rail(name)) for name in names[first : first + self._per_rail]]
            rails.append([(name, down) for name, down in queue if down is not None])
        return rails

    def _spare(self, color):
        """A token of that color nobody plays: the first still parked; from a dispenser, the top one of the
        fullest queue, so that the stations stay served. None when there is none."""
        if self._dispenser >= 0 and color == self._color[ROBOT]:
            queue = max(self._rails(), key=len)
            return queue[-1][0] if queue else None
        model, data = self._sim.model, self._sim.data
        for name in self._tokens[color]:
            q, _ = self._addresses(name)
            if (
                name != self.floating
                and np.linalg.norm(data.qpos[q : q + 3] - model.qpos0[q : q + 3]) < PARKED
            ):
                return name
        return None

    def _cell(self, name):
        """The index of the grid's cell a token is in, row-major from the bottom row, or None."""
        data = self._sim.data
        R, origin = data.xmat[self._grid].reshape(3, 3), data.xpos[self._grid]
        x, y, z = (data.body(name).xpos - origin) @ R
        column, row = round(x / self._pitch), round(z / self._row_pitch)
        in_cell = (
            abs(x / self._pitch - column) < IN_CELL
            and abs(z / self._row_pitch - row) < IN_CELL
            and abs(y) < self._slot
            and 0 <= column < self._columns
            and 0 <= row < self._rows
        )
        return row * self._columns + column if in_cell else None

    def board(self) -> list:
        """The true board: 42 cells, row-major from the bottom row, 0 empty, HUMAN or ROBOT. A token counts
        once at rest in a cell."""
        data = self._sim.data
        cells = [0] * (self._columns * self._rows)
        for value, color in self._color.items():
            for name in self._tokens[color]:
                _, v = self._addresses(name)
                cell = self._cell(name)
                if cell is not None and np.linalg.norm(data.qvel[v : v + 3]) < AT_REST:
                    cells[cell] = value
        return cells

    def drop_token(self, column, color) -> tuple:
        """A spare token of `color` above `column`, left to fall. (False, why) for an unknown column or
        color, a full column, no spare token."""
        if not 0 <= column < self._columns:
            return False, f"column {column}: not 0 to {self._columns - 1}"
        if color not in self._color:
            return False, f"color {color}: not {HUMAN} (human) or {ROBOT} (robot)"
        if self.board()[(self._rows - 1) * self._columns + column]:
            return False, f"column {column} is full"
        name = self._spare(self._color[color])
        if name is None:
            return False, f"no spare {self._color[color]} token"
        data = self._sim.data
        R, origin = data.xmat[self._grid].reshape(3, 3), data.xpos[self._grid]
        above = [column * self._pitch, 0.0, self._rows * self._row_pitch + DROP_HEIGHT]
        self._place(name, origin + R @ above, R @ DISC_IN_SLOT)
        mujoco.mj_forward(self._sim.model, data)
        return True, f"{name} above column {column}"

    def give_token(self, gripper_pct) -> tuple:
        """A spare robot token between the jaws, on the fixed one, where a hand would hold it: off the TCP by
        a seeded offset. It floats (gravity cancelled) until release_if_pinched() sees both jaws on it: a
        free token falls 30 mm in 0.1 s, the jaws need 0.46 s. (False, why) if the jaws are not open."""
        if gripper_pct < MIN_OPEN_PCT:
            return False, f"jaws not open: {gripper_pct:.0f} %, {MIN_OPEN_PCT:.0f} % needed"
        name = self.floating or self._spare(self._color[ROBOT])
        if name is None:
            return False, f"no spare {self._color[ROBOT]} token"
        model, data = self._sim.model, self._sim.data
        R = data.body("gripper").xmat.reshape(3, 3)
        offset = [
            self._on_jaw,
            self._rng.uniform(-GIVE_ALONG, GIVE_ALONG),
            self._rng.uniform(-GIVE_DEPTH, GIVE_DEPTH),
        ]
        self._place(name, data.site("gripperframe").xpos + R @ offset, R @ DISC_ON_JAW)
        _, v = self._addresses(name)
        data.qfrc_applied[v + 2] = -model.opt.gravity[2] * model.body(name).mass[0]
        self.floating = name
        mujoco.mj_forward(model, data)
        return True, f"{name} between the jaws"

    def empty_dispenser(self) -> tuple:
        """The person takes every token out of the rails and lays it flat on the table, at its `spare_` site
        of the scene: a dispenser found empty in the middle of a game."""
        if self._dispenser < 0:
            return False, NO_DISPENSER
        names = [name for queue in self._rails() for name, _ in queue]
        for name in names:
            self._place(name, self._sim.data.site(f"spare_{name}").xpos, np.eye(3))
        mujoco.mj_forward(self._sim.model, self._sim.data)
        return True, f"{len(names)} tokens taken out of the dispenser"

    def fill_dispenser(self) -> tuple:
        """The person puts back in the rails every robot's token that is neither in the grid nor between the
        jaws: each rail gets its own, from its station up, where the scene started them."""
        if self._dispenser < 0:
            return False, NO_DISPENSER
        model, data = self._sim.model, self._sim.data
        tcp, names, count = data.site("gripperframe").xpos, self._tokens[self._color[ROBOT]], 0
        for first in range(0, len(names), self._per_rail):
            back = [
                name
                for name in names[first : first + self._per_rail]
                if self._cell(name) is None
                and name != self.floating
                and np.linalg.norm(data.body(name).xpos - tcp) > IN_JAWS
            ]
            for place, name in enumerate(back):
                (q, v), start = self._addresses(name), self._addresses(names[first + place])[0]
                data.qpos[q : q + 7] = model.qpos0[start : start + 7]
                data.qvel[v : v + 6] = 0
                data.qfrc_applied[v : v + 6] = 0
            count += len(back)
        mujoco.mj_forward(model, data)
        return True, f"{count} tokens in the dispenser"

    def move_grid(self, nearer, around) -> tuple:
        """The person sets the empty grid down elsewhere: `nearer` the arm along its own line (m) and carried
        `around` the arm's pan axis (rad), both from where the scene starts it; 0 and 0 put it back. Where it
        stands is the camera's to find (#260). (False, why) with a token in it: it would stay behind."""
        if any(self.board()):
            return False, "the grid is not empty"
        model, data = self._sim.model, self._sim.data
        position, quat = self._grid_start
        start = np.zeros(9)
        mujoco.mju_quat2Mat(start, quat)
        start = start.reshape(3, 3)
        c, s = np.cos(around), np.sin(around)
        turn = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        pan = data.xanchor[model.joint("shoulder_pan").id]
        mujoco.mju_mat2Quat(model.body_quat[self._grid], (turn @ start).ravel())
        model.body_pos[self._grid] = pan + turn @ (position - nearer * start[:, 0] - pan)
        # With sleep enabled MuJoCo computes a fixed body's pose once: one forward pass without it, so that
        # MjData, the camera and the contacts follow the model [MES-78]
        enabled, sleep = model.opt.enableflags, int(mujoco.mjtEnableBit.mjENBL_SLEEP)
        model.opt.enableflags = enabled & ~sleep
        mujoco.mj_forward(model, data)
        model.opt.enableflags = enabled
        return True, f"the grid {nearer * 1e3:.0f} mm nearer, {np.degrees(around):.1f} deg around the arm"

    def tick(self):
        """Each control tick of the driver."""
        self.release_if_pinched()
        if self._dispenser >= 0:
            self._wake_queues()

    def _wake_queues(self):
        """A queue whose front token left must slide down. While the arm lifts that token, the one behind
        follows its rim slower than MuJoCo's sleep tolerance: it falls asleep 3 mm further, and nothing wakes
        it [MES-75]. A force of 1 uN does, applied only while a station is empty: a full dispenser sleeps."""
        data = self._sim.data
        for name in self._nudged:
            data.qfrc_applied[self._addresses(name)[1] + 2] = 0
        self._nudged = []
        for index, queue in enumerate(self._rails()):
            if not queue:
                continue
            start = index * self._per_rail  # the rail's first token started at its station
            at_station = min(
                abs(down - (int(name.rsplit("_", 1)[1]) - start) * self._diameter) for name, down in queue
            )
            if at_station > AT_STATION:
                for name, _ in queue:
                    data.qfrc_applied[self._addresses(name)[1] + 2] = -WAKE_FORCE
                    self._nudged.append(name)

    def release_if_pinched(self):
        """Once both jaws touch the floating token, its weight is its own again."""
        if self.floating is None:
            return
        model, data = self._sim.model, self._sim.data
        token = model.geom(self.floating).id
        touched = {
            contact.geom2 if contact.geom1 == token else contact.geom1
            for contact in data.contact[: data.ncon]
            if token in (contact.geom1, contact.geom2)
        }
        if all(touched & jaw for jaw in self._jaws):
            _, v = self._addresses(self.floating)
            data.qfrc_applied[v : v + 6] = 0
            self.floating = None

    def reset_board(self):
        """Every token back where the model parks it, at rest."""
        model, data = self._sim.model, self._sim.data
        for names in self._tokens.values():
            for name in names:
                q, v = self._addresses(name)
                data.qpos[q : q + 7] = model.qpos0[q : q + 7]
                data.qvel[v : v + 6] = 0
                data.qfrc_applied[v : v + 6] = 0
        self.floating = None
        mujoco.mj_forward(model, data)
