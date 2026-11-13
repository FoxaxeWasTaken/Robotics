#!/usr/bin/env bash
# Project-rule guards (docs/project/00: C-05, C-06, C-23; ADR-037 for so101_connect4).
# Fails (exit 1) if a rule is violated. Used in CI and locally: bash tools/ci/guards.sh
set -uo pipefail
cd "$(git rev-parse --show-toplevel)" || exit 1
fail=0
NO_TESTS=':!**/test/**'  # a pathspec: the rule holds outside the tests
check() {  # check "<description>" <ERE pattern> -- <pathspecs...>
  local desc=$1 pattern=$2; shift 2
  local out
  if out=$(git grep --untracked -nE "$pattern" "$@" 2>/dev/null); then
    echo "❌ $desc"; echo "$out" | sed 's/^/   /'; fail=1
  else
    echo "✅ $desc"
  fi
}
check "C-05  brain/perception/common/connect4 never access the backend" \
  "SO101Sim|SO101Follower|so101_sim|lerobot" -- ros2_ws/src/so101_brain ros2_ws/src/so101_perception ros2_ws/src/so101_common \
  ros2_ws/src/so101_connect4
check "C-06  no ground truth in brain/perception/connect4 (outside tests)" \
  "get_object_position|get_drop_box_position|ground_truth" -- ros2_ws/src/so101_brain ros2_ws/src/so101_perception \
  ros2_ws/src/so101_connect4 "$NO_TESTS"
check "C-06  no pipeline launch file enables ground truth" \
  "gt_perception_stub|expose_ground_truth[^,)]*[Tt]rue" -- '*.launch.py' "$NO_TESTS"
check "C-23  perception/connect4 do not use the provided extrinsics (outside tests)" \
  "get_camera_extrinsics" -- ros2_ws/src/so101_perception ros2_ws/src/so101_connect4 "$NO_TESTS"
check "C-05  connect4 never commands the motors: the brain moves the arm (ADR-037, outside tests)" \
  "joint_command" -- ros2_ws/src/so101_connect4 "$NO_TESTS"
check "C-06  no sim-only service in brain/perception/connect4 (outside tests)" \
  "[/]sim/" -- ros2_ws/src/so101_brain ros2_ws/src/so101_perception ros2_ws/src/so101_connect4 "$NO_TESTS"
exit $fail
