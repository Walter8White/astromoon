# astromoon_missions

This package is the **main entry point** of AstroMoon.

It orchestrates:
- simulation bringup
- navigation (Nav2)
- mission execution
- mission evaluation

## Responsibilities

- Mission launch logic
- Mission manager (execution)
- Mission referee (validation & scoring)
- Mission YAML definitions

## Run a mission

```bash
ros2 launch astromoon_missions mission.launch.py mission:=m1_waypoint_traverse use_nav2:=true
```

## Mission flow

1. Launch simulation (astromoon_core)
2. Launch navigation (astromoon_nav)
3. Execute mission objectives
4. Evaluate success / failure

## Design

- Mission behavior is defined in YAML
- The mission_manager executes objectives
- The mission_referee evaluates outcomes

This package may launch other packages as part of mission orchestration.