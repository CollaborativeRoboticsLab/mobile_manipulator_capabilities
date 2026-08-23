# mobile_manipulator_capabilities

Capability-layer adapters for the generic mobile-manipulator ROS APIs.

## Purpose

This package maps capability inputs and outputs onto the runtime server in `mobile_manipulator_ros`.

## Runners

- `CheckTaskFeasibilityRunner`: calls `mobile_manipulator_msgs/srv/CheckTaskFeasibility`
- `FindFeasiblePoseRunner`: calls `mobile_manipulator_msgs/srv/FindFeasiblePose`
- `ExecuteTaskPoseRunner`: calls `mobile_manipulator_msgs/action/ExecuteTaskPose`

## Design intent

- Feasibility checking is separate from candidate search.
- Execution is separate from feasibility/search and requires a validated solution.
- Fallback policy lives behind the ROS service/action contracts and pluginized runtime, not in the capability runners.
