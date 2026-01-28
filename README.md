# AstroMoon

AstroMoon is a modular ROS 2 simulation platform for autonomous rover missions,
built on Gazebo and Nav2, with a strong focus on mission design and evaluation.

The goal is simple:
- one command to set up the environment
- one command to launch autonomous rover missions

This project is **Linux-only** (tested on Ubuntu).

---

## Project structure

AstroMoon is organized as a ROS 2 workspace with multiple packages:

- astromoon_core  
  Robot description, Gazebo worlds, spawning and simulation bringup

- astromoon_nav  
  Navigation stack (Nav2) configuration and launch

- astromoon_missions  
  Mission orchestration, mission logic, and evaluation

Each package contains its own README with usage and design details.

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

---

## Setup (one command)

```bash
curl -fsSL https://raw.githubusercontent.com/Walter8White/astromoon/main/setup.sh -o setup.sh
chmod +x setup.sh
./setup.sh
```

---

## Enter the container

```bash
docker exec -it astromoon bash
```

---

## First build (inside the container)

```bash
cd /astromoon_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

---


## License

MIT