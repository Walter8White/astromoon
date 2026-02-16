Nav2 – Architecture and Parameters Overview
===========================================

Nav2 (Navigation2) is a modular ROS 2 navigation stack that converts high-level
navigation goals into safe and continuous velocity commands (/cmd_vel).

It is composed of multiple independent nodes, each configured via nav2_params.yaml.
Each node has a single responsibility and communicates via actions, topics, and TF.


1. Global Architecture
----------------------

High-level goal (pose or waypoints)
        |
        v
BT Navigator
        |
        +--> Planner Server  --> Global Costmap
        |
        +--> Controller Server --> Local Costmap --> /cmd_vel


2. bt_navigator
---------------

Role:
- High-level coordinator of the navigation process
- Executes a Behavior Tree (BT)
- Orchestrates planning, control, and recovery behaviors

Interfaces:
- Action servers:
  - NavigateToPose
  - FollowWaypoints

Configuration:
- BT XML file (defines the navigation logic)
- Timeouts and recovery behavior settings

This is the only Nav2 component that user code typically interacts with.


3. waypoint_follower
--------------------

Role:
- Sequentially executes multiple navigation goals
- Sends waypoints one by one to the BT Navigator
- Handles success / failure between waypoints

Key parameters:
- loop_rate (Hz):
  Frequency at which the node checks the current waypoint status.
  This does NOT affect robot speed, only internal checking cadence.

Example:
- loop_rate: 1  -> checks once per second
- loop_rate: 5  -> checks five times per second

Note:
- "Processing waypoint X..." logs are emitted at each loop iteration.
- Log verbosity should be controlled via log level, not loop_rate.


4. planner_server
-----------------

Role:
- Computes a global path from the current robot pose to the goal pose
- Outputs a nav_msgs/Path

Inputs:
- Current robot pose (via TF)
- Goal pose

Outputs:
- Global path (list of poses)

Configuration:
- planner_plugins (e.g. NavFn, SmacPlanner)
- Tolerances and resolution parameters

This node does NOT move the robot.


5. controller_server
--------------------

Role:
- Converts the global path into real-time velocity commands (/cmd_vel)
- Ensures smooth and safe path tracking

Inputs:
- Global path
- Local costmap
- Current robot pose

Outputs:
- /cmd_vel

Configuration:
- controller_plugins (e.g. DWB, Regulated Pure Pursuit)
- Velocity limits
- Acceleration limits
- Goal tolerances

This is where the equivalent of PID / control logic lives.


6. Costmaps
-----------

Two costmaps are used:

- global_costmap:
  Used by the planner to compute long-range paths

- local_costmap:
  Used by the controller for short-range obstacle avoidance

Each costmap is composed of layers:
- obstacle_layer
- inflation_layer
- static_layer (optional)

Inputs:
- Sensor data (e.g. scan, point clouds)
- TF
- Robot footprint

Costmaps represent the environment as occupancy grids.


7. lifecycle_manager
--------------------

Role:
- Manages the lifecycle states of all Nav2 nodes
- Ensures nodes are configured and activated in the correct order

Without the lifecycle manager:
- Nav2 nodes remain inactive
- Navigation actions are unavailable


8. TF and Frames
----------------

Nav2 relies heavily on TF.

Required frames:
- odom
- base_link (or base_footprint)
- map (optional, depending on setup)

TF is used to:
- Localize the robot
- Transform goals and paths
- Compute control commands

If TF is missing or incorrect, Nav2 will not function.


9. Summary
----------

Nav2 is a pipeline:
- bt_navigator decides WHAT to do
- planner_server decides WHERE to go
- controller_server decides HOW to move
- costmaps decide WHERE it is safe
- waypoint_follower sequences multiple goals

All behavior is controlled via parameters.
User code typically interacts only through Nav2 actions.
