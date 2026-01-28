# astromoon_nav

This package configures and launches the ROS 2 Navigation stack (Nav2)
for the AstroMoon rover.

## Responsibilities

- Nav2 parameters (planners, controllers, costmaps)
- Navigation launch files
- Frame conventions for navigation

## Assumptions

- The rover is already spawned (via astromoon_core)
- TF and /odom are available
- use_sim_time is enabled

## Usage (debug)

Launch Nav2 alone (after simulation is running):

```bash
ros2 launch astromoon_nav nav2.launch.py
```

## Notes

This package does not spawn the robot or run missions.