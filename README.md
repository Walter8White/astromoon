# AstroMoon (ROS 2 + Gazebo)

Moon world in Gazebo (Ignition/Fortress) with a rover, packaged as a Docker-first ROS 2 workspace.

## Quickstart (Linux)

### 0) Requirements

- Linux desktop (Ubuntu recommended)
- Docker + Docker Compose plugin


### 1) Install Docker (Ubuntu)

Please follow the official Docker installation guide for Ubuntu:

https://docs.docker.com/engine/install/ubuntu/

Make sure that after installation:
- Docker is running
- Your user is added to the `docker` group

```bash
sudo usermod -aG docker $USER
```

Log out / log in (required for the docker group change).

Verify Docker :

```bash
docker run hello-world
```

### 2) Clone the repository

```bash
git clone git@github.com:Walter8White/astromoon.git
cd astromoon
```

### 3) Run setup (build + run the container)

```bash
chmod +x setup.sh launch.sh
./setup.sh
```

What setup.sh does:
- checks Docker availability
- optionally enables GPU acceleration (video/render groups)
- builds the Docker image
- starts the container

If you accepted the GPU option, you must log out / log in (or reboot) and then run:

```bash
./setup.sh
```

### 4) Launch the simulation

```bash
./launch.sh
```

## Troubleshooting

### Gazebo window does not show up (X11 permissions)

```bash
xhost +local:docker
```

Then retry:

```bash
./launch.sh
```

### Very slow rendering / massive lag (llvmpipe)

Enable GPU access for Docker (Intel / AMD):

```bash
sudo usermod -aG video,render $USER
```

Log out / log in (or reboot), then:

```bash
./setup.sh
./launch.sh
```

## Container management

Stop the container:

```bash
docker compose -f docker/docker-compose.yml down
```

Enter an interactive shell in the running container:

```bash
docker exec -it astromoon bash
```

## Notes

- This repository is a ROS 2 workspace.
- Large mesh assets are managed with Git LFS.# AstroMoon (ROS2 + Gazebo)
