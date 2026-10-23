#!/usr/bin/env bash
# Waits until the compose stack runs: the control container built the workspace and started the driver and the
# brain, the perception container its node. The E2E jobs call it between `up` and `evaluate`.
# Usage, from the repository root: bash tools/ci/wait_for_stack.sh [timeout_s]   (default 900)
set -uo pipefail
timeout_s=${1:-900}
compose=(docker compose -f docker/docker-compose.yml)
ready() {  # ready <service> <log line>
  "${compose[@]}" logs "$1" 2>&1 | grep -q "$2"
}
until ready control "Driver node ready" && ready control "Brain node ready" && ready perception "Perception node ready"; do
  if [ "$SECONDS" -ge "$timeout_s" ]; then
    echo "The stack is not ready after ${timeout_s} s:"
    "${compose[@]}" logs --tail 40 control perception
    exit 1
  fi
  sleep 5
done
echo "Stack ready in ${SECONDS} s"
