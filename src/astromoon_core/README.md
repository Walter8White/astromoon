# astromoon_core

This package contains the simulation backbone of AstroMoon.

## Responsibilities

- Gazebo world definitions
- Rover URDF / Xacro description
- Spawning the rover in simulation
- Providing simulation time, TF, and odometry

Other packages **assume astromoon_core is running**.

## Usage

Launch the simulation world and rover:

```bash
ros2 launch astromoon_core moon_world.launch.py
```

This provides:
- /clock (simulation time)
- TF frames (base_link, wheels, etc.)
- /odom topic

## Notes

This package does not contain mission logic or navigation behavior.