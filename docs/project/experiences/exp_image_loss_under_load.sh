#!/usr/bin/env bash
# exp_image_loss_under_load.sh: does a machine short of CPU lose perception's best-effort images? (#288, MES-82)
#
# Perception's launch test N times (3; MES-82: 160 over 8 containers), six busy loops beside it. After each
# run: the test's verdict and the kernel's count of UDP datagrams dropped on a full socket buffer
# (RcvbufErrors, cumulated). A 320x240 image is 230 kB, the buffer 208 kB (net.core.rmem_default).
#
# Usage, held to one core (a throwaway container, the workspace built in it); without --cpuset-cpus and with
# N loops of 0, the idle reference:
#   docker run --rm --network none --cpuset-cpus 7 -e ROS_DOMAIN_ID=77 -e USE_SIM=true \
#     -v "$PWD:/repo:ro" tek5/perception bash -lc 'mkdir -p /tmp/ws/src && cd /repo/ros2_ws/src &&
#     cp -r so101_common so101_interfaces so101_perception /tmp/ws/src/ && cd /tmp/ws &&
#     source /opt/ros/jazzy/setup.bash && colcon build --packages-up-to so101_perception > /dev/null &&
#     source install/setup.bash && bash /repo/docs/project/experiences/exp_image_loss_under_load.sh'
set -u
runs=${1:-3}
loops=${2:-6}
for _ in $(seq 1 "$loops"); do
  (while :; do :; done) &
done
trap 'kill $(jobs -p) 2>/dev/null' EXIT
cd /tmp/ws/src/so101_perception || exit 1
for run in $(seq 1 "$runs"); do
  verdict=$(python3 -m pytest -q -p no:cacheprovider test/test_perception_launch.py 2>&1 | tail -n 1)
  # the second "Udp:" line of /proc/net/snmp holds the values; RcvbufErrors is its 6th field
  dropped=$(awk '/^Udp:/ && ++n == 2 {print $6}' /proc/net/snmp)
  echo "run $run: $verdict; RcvbufErrors $dropped"
done
