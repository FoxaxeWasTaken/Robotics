#!/usr/bin/env bash
# run.sh <args>: the prototype in a throwaway container, its output next to it
here=$(cd "$(dirname "$0")" && pwd)
exec docker run --rm --network none -e MUJOCO_GL=osmesa -u "$(id -u):$(id -g)" -e HOME=/tmp -v "$here:/w" tek5/control \
  bash -lc "source /opt/ros/jazzy/setup.bash; cd /w && python3 -u /w/exp_connect4_scene.py $*"
