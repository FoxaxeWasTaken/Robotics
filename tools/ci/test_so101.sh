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
  local expected=() out arg
  for arg; do shift; [[ $arg == -- ]] && break; expected+=("$arg"); done
  out=$("$so101" --dry-run "$@") || fail "so101 $*: exit $?"
  [[ "$(grep -c '^+ docker compose -f .*/docker/docker-compose.yml ' <<< "$out")" = 1 ]] || fail "so101 $*: $out"
  for e in "${expected[@]}"; do grep -qF -- "$e" <<< "$out" || fail "so101 $*: no '$e' in: $out"; done
}

dry "up -d control perception viz" -- up
dry " down" -- down
dry "logs -f control" -- logs
dry "logs -f viz" -- logs viz
dry "exec control bash -lc 'source /ros2_ws/install/setup.bash && exec bash'" -- shell
dry "rviz2 -d /ros2_ws/install/so101_bringup/share/so101_bringup/rviz/so101.rviz" -- rviz
dry "yml build" -- build
dry "yml build perception" -- build perception
dry "restart control" -- restart
dry "restart viz" -- restart viz
echo "ok: up, down, logs, shell, rviz, build and restart print their docker compose command"
dry "exec -T control" "/sim/randomize" "/brain/start" "/brain_state" "DONE*|ERROR*|STOPPED*) break" -- cycle
echo "ok: cycle randomizes, starts and follows /brain_state to the end of the cycle"

# cycle for real against fakes (#164): docker runs the command without its `source`, ros2 answers the services and
# publishes /brain_state like the brain, its states then IDLE forever. cycle must return at DONE, not at its timeout.
mkdir "$tmp/run"
printf '#!/usr/bin/env bash\ncmd=${*: -1}\nexec bash -c "${cmd#*&& }"\n' > "$tmp/run/docker"
cat > "$tmp/run/ros2" <<'FAKE'
#!/usr/bin/env bash
case "$*" in
  *"service call"*) echo "success: true" ;;
  *--once*) echo IDLE ;;
  *"topic echo"*)
    for s in WAIT_TARGET PLAN PLAN "$FINAL"; do echo "$s"; echo ---; sleep 0.1; done
    while :; do echo IDLE; echo ---; sleep 0.2; done ;;
esac
FAKE
# The control image's awk is mawk, which reads a pipe in blocks: it saw DONE only when the echo ended, at the
# timeout (#169). This awk reads all its input first, whatever the host's awk is.
printf '#!/usr/bin/env bash\nin=$(cat)\nexec %s "$@" <<< "$in"\n' "$(command -v awk)" > "$tmp/run/awk"
chmod +x "$tmp/run/docker" "$tmp/run/ros2" "$tmp/run/awk"
for final in DONE ERROR:grasp_failed STOPPED; do
  start=$SECONDS
  out=$(FINAL=$final PATH="$tmp/run:$PATH" timeout 10 "$so101" cycle 2> /dev/null) || fail "cycle did not return after $final (exit $?)"
  [[ $((SECONDS - start)) -le 3 ]] || fail "cycle returned $((SECONDS - start)) s after it started ($final)"
  [[ "$(grep -v '^+ ' <<< "$out")" = "$(printf 'WAIT_TARGET\nPLAN\n%s' "$final")" ]] || fail "cycle printed: $out"
done
echo "ok: cycle returns at DONE, ERROR or STOPPED, each state printed once"
dry "-e ROS_DOMAIN_ID=77 control" "&& set -m;" "pgrep -f \"[/]lib/so101_\"" "grep -qxz ROS_DOMAIN_ID=77 /proc/" "a stack already runs on ROS domain 77" \
  "expose_ground_truth:=true perception:=true" \
  "evaluate --episodes 10 --out /ros2_ws/eval_bench.csv'" -- bench --perception
dry "-e ROS_DOMAIN_ID=77 control" "test/gt_stub.launch.py" "--episodes 3 " "--bags /ros2_ws/bags" \
  -- bench --stub -n 3 --bags /ros2_ws/bags
echo "ok: bench runs on ROS domain 77 only, refuses a busy domain, real pipeline or stub, N episodes, bags"

# From a terminal, bench gets one in the container (#58): with exec -T, Ctrl+C stopped only the docker client and
# the bench went on inside; a pipe or CI keeps -T (a TTY there fails: "the input device is not a TTY").
out=$("$so101" --dry-run bench --perception < /dev/null)
grep -qF "exec -T -e ROS_DOMAIN_ID=77" <<< "$out" || fail "bench without a terminal: no exec -T in: $out"
if command -v script > /dev/null; then  # util-linux: a pseudo-terminal for stdin (none in Git Bash on Windows)
  out=$(script -qec "$so101 --dry-run bench --perception" /dev/null)
  grep -qF "exec -e ROS_DOMAIN_ID=77" <<< "$out" || fail "bench from a terminal: still exec -T in: $out"
  echo "ok: bench gets a terminal from a terminal (Ctrl+C reaches it), keeps exec -T otherwise"
else
  echo "skip: no script(1) here, the terminal case of bench is checked where it exists (CI)"
fi
dry "colcon build --symlink-install --packages-select so101_brain" "colcon test --packages-select so101_brain" \
  "colcon test-result --test-result-base build/so101_brain --verbose" -- test so101_brain
dry "colcon test --executor sequential --packages-select so101_common so101_driver so101_brain so101_perception so101_bringup " -- test
dry "colcon test --packages-select so101_brain --event-handlers console_direct+ --pytest-args -k fsm\\ and\\ lock" \
  -- test so101_brain -k "fsm and lock"
dry "run --rm --no-deps -T control bash -lc 'source /opt/ros/jazzy/setup.bash && cd /ros2_ws" -- test
dry "run --rm --no-deps -T control bash -lc 'source /opt/ros/jazzy/setup.bash && cd /ros2_ws" -- test so101_driver
echo "ok: test builds and tests one package, or all of ours, in a fresh control container (no stack needed), the rest to pytest, quoted"

for case in 1 2 3a 3b 4 5; do
  dry "run --rm --no-deps -e ROS_DOMAIN_ID=78 control bash -lc 'source /ros2_ws/install/setup.bash && ros2 launch so101_bringup robustness_demo.launch.py case:=$case'" \
    -- robustness $case
done
dry "robustness_demo.launch.py case:=5 lighting:=dim episodes:=1'" -- robustness 5 --lighting dim -n 1
echo "ok: robustness runs each case of the demo in a new control container on ROS domain 78, lighting and episodes"

# Next to the everyday stack the demo saw "2 drivers on this ROS domain" (13 §2, #191): refused, nothing started.
mkdir "$tmp/up"
printf '#!/usr/bin/env bash\ncase "$*" in *" ps -q control") echo 3f2a ;; *) echo "docker called: $*" >&2; exit 99 ;; esac\n' \
  > "$tmp/up/docker"
chmod +x "$tmp/up/docker"
out=$(PATH="$tmp/up:$PATH" "$so101" robustness 1 2>&1) && fail "robustness next to the stack: exit 0"
grep -qF "the everyday stack runs" <<< "$out" || fail "robustness next to the stack: $out"
echo "ok: robustness refuses to start while the everyday stack runs"

refused() { ! "$so101" --dry-run "$@" > /dev/null 2>&1 || fail "so101 $* accepted"; }
refused robustness
refused robustness 6
refused robustness 3c
refused robustness "1; reboot"
refused robustness 5 -n 0
refused robustness 5 -n x
refused robustness 5 --lighting "dim; reboot"
refused bench
refused bench --perception -n 0
refused bench --perception -n "3; rm -rf /"
refused bench --stub --bags "/tmp/x; reboot"
refused build viz
refused restart deploy
refused test "so101_brain && reboot"
refused logs control-real
refused deploy
"$so101" --help | grep -q "bench --stub|--perception" || fail "--help"
echo "ok: a bench without its mode, a bad count or path, a bad package or service, an unknown command: refused"
