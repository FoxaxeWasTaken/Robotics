#!/usr/bin/env bash
# Waits until the compose stack runs: the control container built the workspace and started the driver and the
# brain, the perception container its node. The E2E jobs call it between `up` and `evaluate`.
# With CONNECT4=true in the environment (the game of phase 2): the game node too, and the board reader as the
# perception container's node.
# Usage, from the repository root: bash tools/ci/wait_for_stack.sh [timeout_s]   (default 900)
set -uo pipefail
timeout_s=${1:-900}
compose=(docker compose -f docker/docker-compose.yml)
ready() {  # ready <service> <log line>
  local service=$1 line=$2
  "${compose[@]}" logs "$service" 2>&1 | grep -q "$line"
}
if [[ "${CONNECT4:-false}" = true ]]; then
  nodes_ready() { ready control "Game node ready" && ready perception "Board reader ready"; }
else
  nodes_ready() { ready perception "Perception node ready"; }
fi
until ready control "Driver node ready" && ready control "Brain node ready" && nodes_ready; do
  if [[ "$SECONDS" -ge "$timeout_s" ]]; then
    echo "The stack is not ready after ${timeout_s} s:"
    "${compose[@]}" logs --tail 40 control perception
    exit 1
  fi
  sleep 5
done
echo "Stack ready in ${SECONDS} s"
