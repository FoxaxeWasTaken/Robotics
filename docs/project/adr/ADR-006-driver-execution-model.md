# ADR-006 — Driver execution model (loop, threads, rendering)

| Status | Date | Experts (author → reviewers) | Related requirements |
| --- | --- | --- | --- |
| DECIDED (implemented) | 2026-10-08 | CTRL → ARC, QA | C-11, C-12, C-15 |

## Context

`SO101Sim` physics only advances on each `get_observation`/`send_action` call (10 ms per call). OSMesa camera rendering costs 50–140 ms. The early code did everything in a single thread: `/joint_states` dropped to 9 Hz.

## Options considered

| Option | Pros | Cons |
| --- | --- | --- |
| Single thread (early code) | Simple | Control p99 = 52 ms; `/joint_states` at 9 Hz under load [MES-20], [MES-22] |
| Camera thread rendering **under the lock** | Simple | Same jitter: the lock is held for the whole render |
| **Dedicated camera thread rendering a state copy** (`mj_copyData` under the lock, render outside) | p99 = 25 ms; 30 Hz measured; MuJoCo releases the GIL; one OpenGL context bound to one thread | Uses `sim.model/data`, confined to the driver |
| `MultiThreadedExecutor` + callback groups | ROS-native | The thread pool may use the OpenGL context from different threads |
| Separate rendering process | Full isolation | Complexity (shared memory) |

## Decision

Single-threaded executor for the 50 Hz control timer (`send_action` + `get_observation`: the sim runs **in real time**), the 30 Hz `/joint_states` timer and the subscription; **one dedicated Python thread** renders a **state copy** for the sim camera at 10 Hz; **one backend lock** held only for the backend calls and the copy.

## Consequences

- The copy reads the sim internals: acceptable, confined to the driver's sim branch; the rest of the stack still ignores the backend.
- Subscription callbacks never touch the backend.
- Shutdown: stop event + join before `disconnect()`; the thread exits when `rclpy.ok()` turns false.

## Evidence / validation

[MES-02], [MES-03], [MES-20], [MES-22]; merged driver: 30.0 Hz, 6/6 tests, clean shutdown [MES-24].

## Reconsider if

GPU rendering (EGL) becomes available: rendering cost vanishes and a single thread is acceptable again.

## Update (2026-10-16, step 1.2)

Implemented as decided. Measured on the team driver [MES-34]: control period p99 20.4 ms with the camera at 10 Hz (20.2 ms without it), once OSMesa's render threads are limited to half the cores (`LP_NUM_THREADS`); with one render thread per core, p99 rose to 23.9–25.4 ms. Shutdown: 100/100 clean SIGINT/SIGTERM stops.
