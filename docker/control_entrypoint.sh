#!/usr/bin/env bash
# Builds the workspace, launches the ROS2 nodes, and shuts them down cleanly
# on SIGTERM so destroy_node() runs (the real arm gets a proper disconnect).
set -e
set -m  # each background job gets its own process group

source /opt/ros/jazzy/setup.bash
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash

pids=()
launch() {
  local pkg=$1 file=$2
  shift 2
  if [ -f "install/$pkg/share/$pkg/launch/$file" ]; then
    # --noninteractive: compose gives the container a TTY, so launch would take our SIGINT for a terminal
    # Ctrl-C already sent to its nodes and not forward it; they were killed at 5 s, destroy_node() never ran.
    ros2 launch --noninteractive "$pkg" "$file" "$@" &
    pids+=($!)
  fi
}

# Ensure serial device nodes exist in /dev for real hardware access
for i in 0 1 2 3; do
  [ -e /dev/ttyACM$i ] || mknod /dev/ttyACM$i c 166 $i 2>/dev/null && chmod 666 /dev/ttyACM$i 2>/dev/null || true
  [ -e /dev/ttyUSB$i ] || mknod /dev/ttyUSB$i c 188 $i 2>/dev/null && chmod 666 /dev/ttyUSB$i 2>/dev/null || true
done

launch so101_bringup sim.launch.py "use_sim:=${USE_SIM:-true}" "port:=${REAL_ROBOT_PORT:-/dev/ttyACM0}" \
  "expose_ground_truth:=${EXPOSE_GROUND_TRUTH:-false}"  # evaluation bench only (C-06): the E2E jobs set it

shutdown() {
  trap - TERM INT
  set +e  # wait returns the child status; errexit would abort the shutdown
  [ ${#pids[@]} -eq 0 ] && exit 0
  # SIGINT to ros2 launch only: it forwards one to each node. Signalling the whole group gave each
  # node a second SIGINT that interrupted destroy_node() (KeyboardInterrupt traceback).
  for p in "${pids[@]}"; do kill -INT "$p" 2>/dev/null; done
  ( sleep 5; for p in "${pids[@]}"; do kill -KILL -"$p" 2>/dev/null; done ) &
  for p in "${pids[@]}"; do wait "$p"; done
  exit 0
}

trap shutdown TERM INT

sleep infinity &
wait $!
