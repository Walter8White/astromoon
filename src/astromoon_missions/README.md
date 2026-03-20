# astromoon_missions

Main entry point of AstroMoon.

This package orchestrates:

- simulation bringup
- optional operator visualization with RViz
- automatic navigation with Nav2
- mission execution
- mission evaluation

## Main launch file

```bash
ros2 launch astromoon_missions mission.launch.py
```

### Arguments

- `mode:=manual|auto`
  `manual`: simulation only, no Nav2, no mission.
  `auto`: simulation + Nav2 + mission nodes.

- `mission:=<mission_name>`
  Mission YAML name without `.yaml`.
  Used only in `auto` mode.
  Default: `m1_waypoint_traverse`.

- `use_rviz:=true|false`
  Launches RViz2 in either mode.

## Typical usage

Manual mode:

```bash
ros2 launch astromoon_missions mission.launch.py mode:=manual
ros2 launch astromoon_missions mission.launch.py mode:=manual use_rviz:=true
```

Auto mode:

```bash
ros2 launch astromoon_missions mission.launch.py mode:=auto
ros2 launch astromoon_missions mission.launch.py mode:=auto mission:=m1_waypoint_traverse
ros2 launch astromoon_missions mission.launch.py mode:=auto use_rviz:=true
```

## What each mode does

### Manual mode

Launches:

- `astromoon_core`
- Gazebo world and rover
- manual command path through `/cmd_vel_manual`
- optional RViz

Does not launch:

- Nav2
- `mission_manager`
- `mission_referee`

### Auto mode

Launches:

- `astromoon_core`
- `astromoon_nav`
- `mission_manager`
- `mission_referee`
- optional RViz

Mission execution currently relies on Nav2 `FollowWaypoints`.

## Mission model

Mission behavior is described in YAML files under `missions/`.

Current default mission:

- `m1_waypoint_traverse`

The mission manager:

- loads a mission YAML file
- waits for Nav2 `waypoint_follower` to become active
- sends a `FollowWaypoints` goal
- optionally loops the mission
- publishes mission events on `/mission/events`

The mission referee:

- monitors `/odom`
- checks waypoint completion
- enforces a timeout
- publishes the result on `/mission/result`

## Notes

- `mode` is validated at launch time and must be `manual` or `auto`.
- In `manual` mode, the mission argument is ignored.
- This package is the recommended entry point for demos and full-stack runs.

## Related packages

- `astromoon_core`
  Simulation, Gazebo GUI, bridges, TF.

- `astromoon_nav`
  Nav2 configuration and bringup.
