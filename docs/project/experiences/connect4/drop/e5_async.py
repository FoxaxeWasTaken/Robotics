"""E5: does an async service callback wait on rclpy's single-threaded spin without holding the 50 Hz timer?"""
import time, rclpy, threading
import rclpy.node
from rclpy.task import Future
from rclpy.clock import Clock, ClockType
from std_srvs.srv import Trigger
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
class N(rclpy.node.Node):
    def __init__(self):
        super().__init__("n")
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.ticks = []; self.pending = None
        self.create_timer(0.02, self.tick, clock=self.steady)
        self.create_service(Trigger, "/t", self.srv, callback_group=MutuallyExclusiveCallbackGroup())
    async def srv(self, req, res):
        self.pending = (Future(), time.monotonic() + 2.0)
        ok, msg = await self.pending[0]
        res.success, res.message = ok, msg
        return res
    def tick(self):
        self.ticks.append(time.monotonic())
        if self.pending and time.monotonic() > self.pending[1]:
            self.pending[0].set_result((True, "moved")); self.pending = None
rclpy.init()
n = N()
c = rclpy.node.Node("c")
cl = c.create_client(Trigger, "/t")
def client():
    time.sleep(0.5)
    cl.wait_for_service(5)
    t0 = time.monotonic()
    f = cl.call_async(Trigger.Request())
    while not f.done():
        time.sleep(0.01)
    print("reply after", round(time.monotonic() - t0, 3), f.result().success, f.result().message)
    c2.append(time.monotonic())
c2 = []
from rclpy.executors import SingleThreadedExecutor
e = SingleThreadedExecutor(); e.add_node(n); e.add_node(c)
th = threading.Thread(target=client, daemon=True); th.start()
t_end = time.monotonic() + 4
while time.monotonic() < t_end:
    e.spin_once(timeout_sec=0.05)
import numpy as np
d = np.diff(n.ticks)
print("ticks", len(n.ticks), "max period ms", round(d.max() * 1e3, 1), "p99", round(np.percentile(d, 99) * 1e3, 1))
