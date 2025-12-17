#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash

# Source ton workspace s’il est build
if [ -f /astromoon_ws/install/setup.bash ]; then
  source /astromoon_ws/install/setup.bash
fi

exec "$@"
