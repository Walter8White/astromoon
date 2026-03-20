# astromoon_core

Simulation backbone of AstroMoon.

## Responsibilities

- Gazebo / Ignition Fortress world definition
- Rover URDF / Xacro description
- Rover spawn in simulation
- ROS <-> Gazebo bridges for command and odometry
- TF publication for navigation
- Manual / auto command mux bootstrap

Other packages assume `astromoon_core` is running.

## Main launch file

```bash
ros2 launch astromoon_core moon_world.launch.py
```

### Arguments

- `mode:=auto|manual`
  Sets the default command mode of the rover command mux.

Examples:

```bash
ros2 launch astromoon_core moon_world.launch.py mode:=manual
ros2 launch astromoon_core moon_world.launch.py mode:=auto
```

## What it launches

- Ignition Gazebo with the lunar world
- Rover spawn from Xacro-generated URDF
- `/odom` bridge from Gazebo
- `/cmd_vel_manual` bridge for Gazebo teleop
- `/cmd_vel_muxed` bridge toward the rover model
- `robot_state_publisher`
- `odom_tf_broadcaster`
- `map_to_odom_broadcaster`
- `cmd_vel_mux`

## Topics and frames

Important topics:

- `/cmd_vel`
  Automatic velocity commands, typically from Nav2.

- `/cmd_vel_manual`
  Manual velocity commands, currently used by the Gazebo teleop widget.

- `/cmd_vel_muxed`
  Effective velocity command forwarded to the rover.

- `/odom`
  Odometry bridged from Gazebo.

- `/gz/pose_tf`
  Gazebo world poses bridged as `TFMessage`.

Important frames:

- `world`
- `map`
- `rover/odom`
- `rover/base_link`

## Gazebo GUI

The world includes:

- a 3D view
- world stats and world control
- a teleop panel
- entity selection / context menu plugins
- camera tracking support for `Follow`

The teleop panel publishes to `/cmd_vel_manual`.

## Notes

- This package does not launch Nav2 or mission logic.
- The `world -> map` transform is currently an identity static transform.
- The main entry point for the full project is `astromoon_missions`.
