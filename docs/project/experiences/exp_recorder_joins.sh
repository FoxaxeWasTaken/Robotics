#!/usr/bin/env bash
# exp_recorder_joins.sh: does a DDS participant that joins hold the driver's executor? (#274, MES-80)
#
# The sim stack idle, then rosbag recorders started one after the other, as the bench does at each episode.
# The driver warns when a timer runs 0.25 s late ("... s late: the driver's executor was held", #176): the
# script counts these lines after 90 s with nothing joining, after 25 recorders on the bench's topics, and
# after 25 more on the topics the driver does not publish.
#
# Usage, on 4 cores as the nightly's runner (a throwaway container, the workspace built in it):
#   docker run --rm --network none --cpuset-cpus 0-3 -e ROS_DOMAIN_ID=79 -e MUJOCO_GL=osmesa \
#     -v "$PWD:/repo:ro" tek5/control bash -lc 'mkdir -p /tmp/ws && cp -r /repo/ros2_ws/src /tmp/ws/ &&
#     cd /tmp/ws && source /opt/ros/jazzy/setup.bash && colcon build --packages-up-to so101_bringup > /dev/null &&
#     source install/setup.bash && bash /repo/docs/project/experiences/exp_recorder_joins.sh'
set -u
log=$(mktemp)
held() { grep -c "executor was held" "$log"; }
recorders() {  # recorders <topics...>: 25 recorders of 3.5 s each
  for i in $(seq 1 25); do
    timeout -s INT 3.5 ros2 bag record -o "$(mktemp -d)/bag$i" "$@" > /dev/null 2>&1
  done
}

(ros2 launch so101_bringup sim.launch.py expose_ground_truth:=true perception:=true > "$log" 2>&1 &)
timeout 120 ros2 topic echo --once /end_effector_pose geometry_msgs/msg/PoseStamped > /dev/null || exit 1
sleep 10
echo "after the start: $(held) lines"
sleep 90
echo "90 s idle: $(held) lines"
recorders /joint_states /joint_command /object_position /drop_box_position /brain_state \
  /sim/ground_truth/object /sim/ground_truth/drop_box
echo "25 recorders on the bench's topics: $(held) lines"
recorders /joint_command /object_position /drop_box_position /brain_state
echo "25 more on no topic of the driver: $(held) lines"
grep "executor was held" "$log" | sed 's/.*so101_driver\]: //' | sort | uniq -c
