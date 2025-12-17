#!/usr/bin/env bash
set -e

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO_DIR"

echo "AstroMoon setup (Docker MVP)"

# Check Docker presence
if ! command -v docker >/dev/null 2>&1; then
  echo "Docker is not installed."
  echo "Ubuntu: sudo apt update && sudo apt install -y docker.io docker-compose-plugin"
  exit 1
fi

# Check Docker daemon access
if ! docker info >/dev/null 2>&1; then
  echo "Docker daemon is not reachable."
  echo "Try: sudo systemctl enable --now docker"
  echo "Then (recommended): sudo usermod -aG docker \$USER  (logout/login required)"
  exit 1
fi

# Optional GPU enablement (Intel/AMD DRM). Prevents llvmpipe software rendering.
if [ -d /dev/dri ]; then
  NEED_GROUPS=0
  id -nG "$USER" | grep -qw video  || NEED_GROUPS=1
  id -nG "$USER" | grep -qw render || NEED_GROUPS=1

  if [ "$NEED_GROUPS" -eq 1 ]; then
    echo ""
    echo "Optional (recommended): enable GPU acceleration for Docker (prevents llvmpipe / major lag)."
    read -r -p "Add $USER to 'video,render' groups? (y/N) " ans
    if [[ "$ans" =~ ^[Yy]$ ]]; then
      sudo usermod -aG video,render "$USER"
      echo ""
      echo "Done. Logout/login (or reboot) is required. Then re-run: ./setup.sh"
      exit 0
    else
      echo "Continuing without GPU acceleration (it will work but may be slow)."
    fi
  fi
fi

# Allow Docker containers to use X11 (Gazebo GUI)
xhost +local:docker >/dev/null 2>&1 || true

echo ""
echo "[1/2] Building Docker image..."
docker compose -f docker/docker-compose.yml build

echo ""
echo "[2/2] Starting container..."
docker compose -f docker/docker-compose.yml up -d

echo ""
echo "Setup complete."
echo "Next:"
echo "  ./launch.sh"
