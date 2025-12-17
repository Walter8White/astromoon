#!/usr/bin/env bash
set -e

docker exec -it astromoon bash -lc "ros2 launch astromoon_core fortress_spawn.launch.py"
