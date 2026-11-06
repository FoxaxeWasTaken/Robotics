#!/usr/bin/env bash
# Test of tools/so101-arm (#199) without Docker or an arm: the --dry-run output of each command and its refusals.
# The script runs from a copy whose docker/.env the test writes; a fake `docker` answers `compose ps` (the stack up or
# not) and fails the test on any other call. pre-commit runs it when either file changes.
# Usage, from the repository root: bash tools/ci/test_so101_arm.sh
set -euo pipefail
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
fail() { echo "FAIL: $*"; exit 1; }
mkdir -p "$tmp/repo/tools" "$tmp/repo/docker" "$tmp/bin" "$tmp/cal"
cp tools/so101-arm tools/so101_arm.py "$tmp/repo/tools/"
arm=$tmp/repo/tools/so101-arm
cat > "$tmp/bin/docker" <<'FAKE'
#!/usr/bin/env bash
case "$*" in
  *" ps -q control") echo "${STACK:-}" ;;
  *) echo "docker called: $*" >&2; exit 99 ;;
esac
FAKE
chmod +x "$tmp/bin/docker"
export PATH="$tmp/bin:$PATH"
env_file() { printf '%s\n' "USE_SIM=true" "$@" > "$tmp/repo/docker/.env"; }
env_file "CALIBRATION_FILE=$tmp/cal/arm.json" "REAL_ROBOT_PORT=/dev/ttyACM1"

out=$("$arm" --dry-run calibrate) || fail "calibrate without a file yet: exit $?"
grep -q "^+ cp" <<< "$out" && fail "calibrate backed up a file that does not exist: $out"
grep -qF "+ docker run --rm -i --network none --device /dev/ttyACM1 -v $tmp/cal:/calibration -v $tmp/repo/tools:/tools:ro tek5/control python3 /tools/so101_arm.py calibrate /dev/ttyACM1 arm" <<< "$out" \
  || fail "calibrate: $out"
echo '{}' > "$tmp/cal/arm.json"
out=$("$arm" --dry-run calibrate) || fail "calibrate: exit $?"
grep -qE "^\+ cp -p $tmp/cal/arm.json $tmp/cal/arm\.[0-9]{8}-[0-9]{6}\.json$" <<< "$out" || fail "calibrate backup: $out"
[[ $(grep -c "^+ " <<< "$out") = 2 && $(head -n 1 <<< "$out") == "+ cp"* ]] || fail "calibrate: the backup must come first: $out"
[[ $(ls "$tmp/cal") = arm.json ]] || fail "a dry run copied the file: $(ls "$tmp/cal")"
echo "ok: calibrate backs the file up first, then runs calibrate() in a container with only the port, no network"

out=$("$arm" --dry-run zero-offsets) || fail "zero-offsets: exit $?"
grep -qF "+ docker run --rm -i --network none --device /dev/ttyACM1 -v $tmp/cal:/calibration:ro -v $tmp/repo/tools:/tools:ro tek5/control python3 /tools/so101_arm.py zero-offsets /dev/ttyACM1 arm 1" <<< "$out" \
  || fail "zero-offsets: $out"
"$arm" --dry-run zero-offsets -n 3 | grep -qF "zero-offsets /dev/ttyACM1 arm 3" || fail "zero-offsets -n 3"
echo "ok: zero-offsets reads in a container with the calibration read-only, N readings"

out=$("$arm" --dry-run touch 18) || fail "touch: exit $?"
grep -qF -- "-v $tmp/cal:/calibration:ro -v $tmp/repo/ros2_ws/src:/src:ro" <<< "$out" || fail "touch mounts: $out"
grep -qF "touch /dev/ttyACM1 arm 18" <<< "$out" || fail "touch: $out"
echo "ok: touch reads in a container with the calibration and the sources read-only"

refused() {  # refused <expected message> -- <so101-arm arguments>: exit 2, the message, no docker run
  local expected=$1 out code=0
  shift 2
  out=$("$arm" --dry-run "$@" 2>&1) || code=$?
  [[ $code = 2 ]] || fail "so101-arm $*: exit $code: $out"
  grep -qF -- "$expected" <<< "$out" || fail "so101-arm $*: no '$expected' in: $out"
  grep -q "^+ " <<< "$out" && fail "so101-arm $*: ran something: $out"
  return 0
}
STACK=3f2a refused "the stack runs" -- calibrate
STACK=3f2a refused "the stack runs" -- zero-offsets
STACK=3f2a refused "the stack runs" -- touch 0
refused "the board corner" -- touch
refused "unknown option" -- touch -1
refused "-n takes a number" -- zero-offsets -n 0
refused "-n takes a number" -- zero-offsets -n "3; reboot"
refused "unknown command" -- deploy
refused "unknown option" -- calibrate --force
env_file "REAL_ROBOT_PORT=/dev/ttyACM1"
refused "CALIBRATION_FILE" -- calibrate
env_file "CALIBRATION_FILE=~/.cache/arm.json"
refused "CALIBRATION_FILE" -- calibrate
env_file "CALIBRATION_FILE=cal/arm.json"
refused "CALIBRATION_FILE" -- zero-offsets
env_file "CALIBRATION_FILE=$tmp/none/arm.json"
refused "no folder" -- calibrate
mkdir "$tmp/cal/dir.json"
env_file "CALIBRATION_FILE=$tmp/cal/dir.json"
refused "is a directory" -- calibrate
env_file "CALIBRATION_FILE=$tmp/cal/missing.json"
refused "no calibration file" -- zero-offsets
env_file "CALIBRATION_FILE=$tmp/cal/arm.json" "REAL_ROBOT_PORT=/dev/ttyACM0; reboot"
refused "REAL_ROBOT_PORT" -- calibrate
"$arm" --help | grep -q "zero-offsets \[-n N\]" || fail "--help"
echo "ok: the stack up, CALIBRATION_FILE unset, relative, a directory or missing, a bad port, count or command: refused"
