# setup.sh
#!/usr/bin/env bash
set -euo pipefail

REPO_URL="https://github.com/Walter8White/astromoon.git"
WS_NAME="astromoon"
COMPOSE_FILE="docker/docker-compose.yml"
CONTAINER_NAME="astromoon"

echo ""
echo "== AstroMoon setup (Linux) =="

# 1) Prereqs
if ! command -v git >/dev/null 2>&1; then
  echo "ERROR: git is not installed. Install it first: sudo apt-get update && sudo apt-get install -y git"
  exit 1
fi

if ! command -v docker >/dev/null 2>&1; then
  echo "ERROR: docker is not installed. Install Docker Engine + Docker Compose plugin."
  exit 1
fi

if ! docker info >/dev/null 2>&1; then
  echo "ERROR: Docker daemon not reachable."
  echo "Fix: start docker service, or add your user to the docker group, or run with sudo."
  exit 1
fi

if ! docker compose version >/dev/null 2>&1; then
  echo "ERROR: docker compose is not available (Compose V2 plugin)."
  echo "Fix: install the docker compose plugin."
  exit 1
fi

# 2) Clone repo
if [ ! -d "${WS_NAME}" ]; then
  echo "Cloning repo into ./${WS_NAME} ..."
  git clone "${REPO_URL}" "${WS_NAME}"
else
  echo "Repo folder ./${WS_NAME} already exists. Skipping clone."
fi

cd "${WS_NAME}"

# 3) X11 access (Gazebo GUI on host)
if command -v xhost >/dev/null 2>&1; then
  # Allow local root (container runs as root by default)
  xhost +local:root >/dev/null 2>&1 || true
else
  echo "WARNING: xhost not found. If Gazebo GUI does not appear, install it: sudo apt-get install -y x11-xserver-utils"
fi

# 4) Build + start container
echo "Building and starting container..."
docker compose -f "${COMPOSE_FILE}" up -d --build

echo ""
echo "== Ready =="
echo "Container name: ${CONTAINER_NAME}"
echo ""
echo "Next:"
echo "  docker exec -it ${CONTAINER_NAME} bash"
echo ""
echo "Inside the container (first time):"
echo "  cd /astromoon_ws"
echo "  rosdep install --from-paths src --ignore-src -r -y"
echo "  colcon build --symlink-install"
echo "  source install/setup.bash"
echo "  ros2 launch astromoon_core lunar_world.launch.py"
echo ""
echo "Inside the container (next times):"
echo "  cd /astromoon_ws"
echo "  colcon build --symlink-install"
echo "  source install/setup.bash"
echo "  ros2 launch astromoon_core lunar_world.launch.py"
echo ""
echo "Stop everything (host):"
echo "  docker compose -f ${WS_NAME}/${COMPOSE_FILE} down"
echo ""
