#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."

xhost +local:docker >/dev/null 2>&1 || true

docker compose -f docker/docker-compose.yml up -d
docker exec -it astromoon bash

