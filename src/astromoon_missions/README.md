# astromoon_missions

This package is the **main entry point** of AstroMoon.

It orchestrates:
- simulation bringup
- navigation (Nav2)
- mission execution
- mission evaluation

## Quick start

```bash
ros2 launch astromoon_missions mission.launch.py mission:=m1_waypoint_traverse use_nav2:=true
```

## Launch file

`mission.launch.py` can launch the full stack (simulation + navigation + mission nodes) depending on options.

### Arguments

- `mission` (string)  
  Mission id / YAML name (example: `m1_waypoint_traverse`)

- `use_nav2` (bool)  
  If `true`, launches Nav2 bringup from `astromoon_nav`

- `use_rviz` (bool)  
  If `true`, launches RViz (TODO)

- `use_sim_time` (bool)  
  Use simulation clock (`/clock`)

### What it launches (high level)

When you run:

```bash
ros2 launch astromoon_missions mission.launch.py mission:=m1_waypoint_traverse use_nav2:=true
```

it typically launches:

1. Simulation bringup (from `astromoon_core`)
   - Gazebo world + rover spawn
   - TF publishing (robot_state_publisher)
   - Odometry bridge + odom->base_link TF broadcaster (for Nav2)

2. Navigation bringup (from `astromoon_nav`) if `use_nav2:=true`
   - planner_server, controller_server, bt_navigator, waypoint_follower
   - lifecycle_manager_navigation
   - costmaps + behaviors (depending on params)

3. Mission nodes (from this package)
   - `mission_manager` (execution)
   - `mission_referee` (validation / scoring)

## Responsibilities

- Mission launch logic
- Mission manager (execution)
- Mission referee (validation & scoring)
- Mission YAML definitions

## Mission flow

1. Launch simulation (`astromoon_core`)
2. Launch navigation (`astromoon_nav`) (optional)
3. Execute mission objectives (manager)
4. Evaluate success / failure (referee)

## Design

- Mission behavior is defined in YAML
- The mission_manager executes objectives (e.g. Nav2 actions like FollowWaypoints)
- The mission_referee evaluates outcomes (timeout, criteria, scoring)

## Related package READMEs

- `astromoon_core`: simulation bringup details  
  [README](../astromoon_core/README.md)

- `astromoon_nav`: Nav2 parameters and bringup details  
  [README](../astromoon_nav/README.md)