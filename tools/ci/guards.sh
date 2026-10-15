#!/usr/bin/env bash
# Project-rule guards (docs/project/00: C-05, C-06, C-23).
# Fails (exit 1) if a rule is violated. Used in CI and locally: bash tools/ci/guards.sh
set -uo pipefail
cd "$(git rev-parse --show-toplevel)"
fail=0
check() {  # check "<description>" <ERE pattern> -- <pathspecs...>
  local desc=$1 pattern=$2; shift 2
  local out
  if out=$(git grep --untracked -nE "$pattern" "$@" 2>/dev/null); then
    echo "❌ $desc"; echo "$out" | sed 's/^/   /'; fail=1
  else
    echo "✅ $desc"
  fi
}
check "C-05  brain/perception never access the backend" \
  "SO101Sim|SO101Follower|so101_sim|lerobot" -- ros2_ws/src/so101_brain ros2_ws/src/so101_perception
check "C-06  no ground truth in brain/perception (outside tests)" \
  "get_object_position|get_drop_box_position|ground_truth" -- ros2_ws/src/so101_brain ros2_ws/src/so101_perception ':!**/test/**'
check "C-06  no pipeline launch file enables ground truth" \
  "gt_perception_stub|expose_ground_truth[^,)]*[Tt]rue" -- '*.launch.py' ':!**/test/**'
check "C-23  perception does not use the provided extrinsics (outside tests)" \
  "get_camera_extrinsics" -- ros2_ws/src/so101_perception ':!**/test/**'
exit $fail
