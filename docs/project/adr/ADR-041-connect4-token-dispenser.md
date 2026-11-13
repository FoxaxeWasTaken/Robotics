# ADR-041 — Connect Four: the robot takes its tokens from a dispenser, three stacked rails with open pick stations

| Status | Date | Experts (author → reviewers) | Related requirements |
| --- | --- | --- | --- |
| DECIDED for the sim (spike measured, #257); the printed part is `v2.1.0` | 2026-11-13 | KIN → CTRL, VIS, SAFE | C-41, C-54 |

## Context

ADR-038 chose a handover: a person puts each token between the robot's open jaws, and a dispenser stayed a stretch feature. Two things changed on 2026-11-13. The staff's day-4 slide describes our project as "Connect 4, with a way to grab the pieces easily" [J4]. And the team asks that the arm takes its tokens by itself, from a part that holds the 21 of a game or is quick to refill, that it could print in 3D, and whose place the camera measures: nothing about where things stand is given to the arm.

What bounds the shape, measured on the sim [MES-75]:

- The arm has 5 axes: its jaws close across its own vertical plane, as at a drop (ADR-035). A token must wait on edge, in a plane through the pan axis, both faces free.
- The pick pose (the drop's, `release_pose`) reaches a token centre 25 to 41 cm from the pan axis up to 5 cm above the base, nearer when higher.
- Around the token the open jaws go 5 mm under its centre and 7 mm beyond it, outwards; everything else of the gripper is on the arm's side and above. So tokens must come from the outside, and whatever holds the token to take stays under 9 mm.

## Options considered

| Option | For | Against |
| --- | --- | --- |
| Handover (ADR-038) | Nothing to build | A person in each of the robot's turns; a hand next to the gripper |
| A rack of open slots, as a Scrabble holder | No moving token, the simplest print | One pick pose per slot, and which slots are full must be read; 21 slots are 60 cm, out of reach: 7 or 8 at most, refilled one by one |
| One gravity rail with an open pick station | One pose; the token stops within 0.05 mm; refilled from its top | 21 tokens are 72 cm of rail |
| One rail folded in three levels | 21 tokens in 25 cm | At each turn a token lies on the one below, which must roll out from under it: a force estimate says it jams; one jam stops the whole supply |
| **Three straight rails stacked in one vertical plane, each with its own station, staggered like stairs** | 21 tokens, no turn, three independent queues; one azimuth for the three stations; each station is the single rail's, measured | Three pick poses; a tall thin part (about 36 cm above the table at a 25° slope) |
| A flat stack in a tube with a drawer | 21 tokens in 14 cm | The arm must work a drawer and turn the token on edge |
| A flat tray, tokens taken by their rim | Simple part | Another grip than the one the 50 drops of 50 were measured with [MES-69] |
| A motorized carousel | Compact | An actuator, its electronics and its driver |

## Decision

- **Three straight rails, stacked in the arm's vertical plane.** Tokens stand on edge between two walls and slide down to a stop; the lowest place of each rail is its pick station. The stations are staggered outwards and upwards, so that the gripper at one station is clear of the rail above.
- **The pick station is open.** Its side walls are 7 mm high, the walls behind are 30 mm. The stop is a fin 4 mm wide in the token's own plane, as high as the token: the jaws close on either side of it, and a fast token cannot hop over it.
- **The slot leaves 3 mm across**, as the grid's slots do. The arm comes straight down with open jaws, closes, checks the grip as `/brain/drop` does, and lifts straight up. The token then sits in the jaws as a hand-given one does [MES-75]: the drop is unchanged.
- **The dispenser carries its own ArUco markers**, four, flat and facing up at the corners of its marker plate, before the lowest station. Its pose is measured as the grid's is (#258): its stations within 1.4 mm [MES-76]; the stations are that pose plus the part's own dimensions, from `connect4_grid.yaml`. Its placement in that file builds the sim scene and nothing else.
- **In the sim the rails slope at 25°.** The tokens' joint damping (0.005, on rotations too, [MES-63]) holds a rolling token at 0.4 mm/s on a gentle slope; at 25° it slides, and the contact set of the 50 drops stays untouched. The printed part may be gentler: real tokens roll. The slope is a setting of the layout.
- **The sim wakes a queue whose front token left.** MuJoCo puts slow bodies to sleep. While the arm lifts a token, at 0.03 m/s, the one behind follows the leaving rim slower than the sleep tolerance: it falls asleep 3 mm further and nothing wakes it. Without a wake 2 tokens of 7 came down, with it 7 of 7 [MES-75]. A force of 1 µN while a station is empty does it, applied by the driver's simulated scene, not by the game; a full dispenser at rest sleeps, as the tokens in the grid do.
- **The handover stays** behind a setting (`token_source: hand`) until the dispenser passes the drops' and the games' benches (#259).

Supersedes in ADR-038: "Token supply, first version: handover", for the sim. The rest of ADR-038 stands.

## Evidence

[MES-75] (`experiences/connect4/dispenser/`): the reach of the pick pose; the jaws' envelope; on one rail at 25° with the scene's contacts, 7 tokens of 7 fed to within 0.03 mm and 7 of 7 taken, the arm 2.5 to 3.7 mm from the rail, the token held 1.5 to 2.3 mm across and tilted 3.5°, as a hand-given one (1.4 to 1.9 mm, 3.5°); the same on a 10° rail without angular damping.

## Reopen if

The three rails' feed or picks fall under the handover's rates on the benches of #259. The camera cannot place the stations within 2 mm (#258): larger markers, or a marker on each rail. The printed part jams or lets tokens hop: slope, stop and walls are settings to tune on the part (`v2.1.0`).

## Update (2026-11-13, #259): the arm takes its tokens

The game runs on the dispenser: `token_source: dispenser` in `so101_connect4/config/game.yaml`; `hand` keeps the handover. What the pick changed or settled [MES-77]:

- **The pick is `/brain/pick`**, the drop's pose and the drop's check of the grip (`phase2/P2-02` §2.3): 40 mm above the station with the jaws at 40 %, straight down through waypoints 10 mm apart, close, straight up, back to `HANDOVER` with the token. Closed on nothing, the jaws open and the arm comes back up before it says `no_token`.
- **The way down is 2.5 mm aside, then the fixed jaw is seated sideways.** The slot leaves the token 1.5 mm of play on each side, as decided, and the camera places a station within 1.5 mm. Aimed at the slot's middle, the fixed jaw came down 0.8 mm from the token's face: it landed on a token standing 1 mm nearer and the pick stopped, one game of the first ten.
- **The station's low walls run over the 33 mm a lifted token sweeps**, not 3 mm behind the token: lifted straight up, a token moves up the sloped rail, and one held 1.5 mm off the slot's middle rubbed the 30 mm wall there and stopped the lift.
- **The stop is 2 mm wide, 0.45 mm off the tokens' plane towards the fixed jaw**, not 4 mm in the plane. The jaws do not close parallel: where the stop stands, the moving jaw passed 0.6 mm from a 4 mm fin, and touched it as it shut at each pick. It now stands half-way between the two jaws where they shut, 2.7 mm from each, shut on nothing included.
- **A handover after a pick cut short rises straight up first**, 40 mm: a stop may leave the arm down in a station, and one joint motion from there to `HANDOVER` swept 9 mm into the stop.
- **The game takes the highest station that holds a token**, on a read of the stations newer than its move: from `HANDOVER` the wrist turns the least there, 8.2 s a pick against 12.3 s at the lowest.
- **An empty dispenser is a state, not an error**: the game waits for the person, who fills it and says done (`WAIT_HUMAN:dispenser_empty`); a dispenser the camera does not read waits the same way. Nothing moves meanwhile.

The benches the decision asked for: 50 tokens of 50 taken and dropped in their column, and 20 games of 20 with the poses the camera measures [MES-77]. The first version of the pick lost one game of ten: the measurement that found why is in the same row.
