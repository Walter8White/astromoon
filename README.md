# AstroMoon

AstroMoon is a ROS 2 + Gazebo simulation environment packaged with Docker for a robust and reproducible setup on Linux.

The goal is simple:
- one command to set up the environment
- then build and run the simulation inside a container

This project is **Linux-only** (tested on Ubuntu).

---

## Prerequisites (host machine)

You must have:

- Git
- Docker Engine
- Docker Compose (v2 plugin)

Verify:
```bash
git --version
docker --version
docker compose version
```

Your user should be allowed to run Docker without sudo (recommended).

---

## Setup (one command)

From any directory on your machine, run:

```bash
curl -fsSL https://raw.githubusercontent.com/Walter8White/astromoon/main/setup.sh -o setup.sh
chmod +x setup.sh
./setup.sh

```

What this does:
- clones the AstroMoon repository
- builds a Docker image with ROS 2 + Gazebo
- starts a container in the background
- configures X11 so Gazebo can open a GUI

At the end, the container is **running**, but you are still on your host machine.

---

## Enter the container

```bash
docker exec -it astromoon bash
```

You are now **inside the Docker container**.

---

## First build (inside the container, once)

```bash
cd /astromoon_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

This:
- installs ROS dependencies declared in `package.xml`
- builds the workspace

---

## Run the simulation

```bash
ros2 launch astromoon_core lunar_world.launch.py
```

Gazebo should open on your Linux desktop.

---

## Next runs

On later runs, you only need:

```bash
docker exec -it astromoon bash
cd /astromoon_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch astromoon_core lunar_world.launch.py
```

---

## Stop the environment (host machine)

```bash
docker compose -f astromoon/docker/docker-compose.yml down
```

---

## Notes

- This setup is intended for **Linux desktop** systems.
- GPU acceleration is enabled for Intel/AMD via `/dev/dri`.
- NVIDIA GPUs require additional configuration (not included).
- The Docker image contains only the environment; your code is mounted live.

---

## Troubleshooting

If Gazebo does not appear:
```bash
xhost +local:root
```

If dependencies change:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

---

## License

MIT