#!/usr/bin/env bash
set -e

# Launch the simulation inside the running Docker container
docker exec -it astromoon bash -lc "ros2 launch astromoon_core fortress_spawn.launch.py"
