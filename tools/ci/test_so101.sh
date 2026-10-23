#!/usr/bin/env bash
# Test of tools/so101 (#133) without Docker: the --dry-run output of each command, the bench on its own ROS domain,
# and the refused arguments. A fake `docker` on PATH fails the test if a dry run calls it. pre-commit runs it when
# either script changes. Usage, from the repository root: bash tools/ci/test_so101.sh
set -euo pipefail
so101=$(pwd)/tools/so101
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
fail() { echo "FAIL: $*"; exit 1; }
mkdir "$tmp/bin"
printf '#!/usr/bin/env bash\necho "docker called: $*" >&2\nexit 99\n' > "$tmp/bin/docker"
chmod +x "$tmp/bin/docker"
export PATH="$tmp/bin:$PATH"

dry() {  # dry <expected substring>... -- <so101 arguments>: one printed command holding each substring
  local expected=() out
  while [ "$1" != -- ]; do expected+=("$1"); shift; done
  shift
  out=$("$so101" --dry-run "$@") || fail "so101 $*: exit $?"
  [ "$(grep -c '^+ docker compose -f .*/docker/docker-compose.yml ' <<< "$out")" = 1 ] || fail "so101 $*: $out"
  for e in "${expected[@]}"; do grep -qF -- "$e" <<< "$out" || fail "so101 $*: no '$e' in: $out"; done
}

dry "up -d control perception viz" -- up
dry " down" -- down
dry "logs -f control" -- logs
dry "logs -f viz" -- logs viz
dry "exec control bash -lc 'source /ros2_ws/install/setup.bash && exec bash'" -- shell
dry "rviz2 -d /ros2_ws/install/so101_bringup/share/so101_bringup/rviz/so101.rviz" -- rviz
echo "ok: up, down, logs, shell and rviz print their docker compose command"
dry "exec -T control" "/sim/randomize" "/brain/start" "/brain_state" "(DONE|ERROR|STOPPED)" -- cycle
echo "ok: cycle randomizes, starts and follows /brain_state to the end of the cycle"
dry "-e ROS_DOMAIN_ID=77 control" "&& set -m;" "pgrep -f \"[/]lib/so101_\"" "grep -qxz ROS_DOMAIN_ID=77 /proc/" "a stack already runs on ROS domain 77" \
  "expose_ground_truth:=true perception:=true" \
  "evaluate --episodes 10 --out /ros2_ws/eval_bench.csv'" -- bench --perception
dry "-e ROS_DOMAIN_ID=77 control" "test/gt_stub.launch.py" "--episodes 3 " "--bags /ros2_ws/bags" \
  -- bench --stub -n 3 --bags /ros2_ws/bags
echo "ok: bench runs on ROS domain 77 only, refuses a busy domain, real pipeline or stub, N episodes, bags"
dry "colcon build --symlink-install --packages-select so101_brain" "colcon test --packages-select so101_brain" \
  -- test so101_brain
echo "ok: test builds and tests one package"

refused() { ! "$so101" --dry-run "$@" > /dev/null 2>&1 || fail "so101 $* accepted"; }
refused bench
refused bench --perception -n 0
refused bench --perception -n "3; rm -rf /"
refused bench --stub --bags "/tmp/x; reboot"
refused test
refused test "so101_brain && reboot"
refused logs control-real
refused deploy
"$so101" --help | grep -q "bench --stub|--perception" || fail "--help"
echo "ok: a bench without its mode, a bad count or path, a bad package or service, an unknown command: refused"
