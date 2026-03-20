# astromoon_nav

Navigation package for AstroMoon.

It configures and launches ROS 2 Nav2 for the rover in simulation.

## Responsibilities

- Nav2 launch file
- Nav2 parameter set
- controller / planner / behavior configuration
- odom-based navigation setup

## Assumptions

This package assumes:

- the rover is already spawned
- `/odom` is available
- TF is available
- simulation time is enabled

In practice, it is usually launched through `astromoon_missions` in `mode:=auto`.

## Launch

Standalone debug usage:

```bash
ros2 launch astromoon_nav nav2.launch.py
```

### Arguments

- `use_sim_time:=true|false`
  Defaults to `true`.

- `params_file:=<path>`
  Overrides the default Nav2 parameter file.

## What it launches

- `planner_server`
- `controller_server`
- `smoother_server`
- `behavior_server`
- `bt_navigator`
- `waypoint_follower`
- `lifecycle_manager_navigation`

## Configuration

Default parameter file:

- `config/nav2_params.yaml`

Current setup is intentionally simple:

- no map server
- no AMCL
- odom-driven navigation
- `FollowWaypoints` mission execution path

## RViz

The package contains the RViz config path used by `astromoon_missions`:

- `rviz/nav2.rviz`

## Notes

- This package does not spawn the world or rover.
- This package does not contain mission logic.
- For normal project usage, prefer:

```bash
ros2 launch astromoon_missions mission.launch.py mode:=auto
```
