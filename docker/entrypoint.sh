#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/humble/setup.bash

# If user already built the workspace, make it available automatically
if [ -f /astromoon_ws/install/setup.bash ]; then
  source /astromoon_ws/install/setup.bash
fi

exec "$@"
