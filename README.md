# mobile_manipulator_capabilities

Capability-layer adapters for the generic mobile-manipulator ROS APIs.

## Purpose

This package maps capability inputs and outputs onto the runtime server in `mobile_manipulator_ros`.

## Runners

- `CheckTaskFeasibilityRunner`: calls `mobile_manipulator_msgs/srv/CheckTaskFeasibility`
- `FindFeasiblePoseRunner`: calls `mobile_manipulator_msgs/srv/FindFeasiblePose`
- `ExecuteTaskPoseRunner`: calls `mobile_manipulator_msgs/action/ExecuteTaskPose`


## Example Plans

- [plans/manipulation_1.xml](./plans/manipulation_1.xml): direct task-feasibility check for a target pose with no supplied base pose.
- [plans/manipulation_2.xml](./plans/manipulation_2.xml): task-feasibility check with an explicit mobile base pose.
- [plans/manipulation_3.xml](./plans/manipulation_3.xml): feasible-pose search that asks the mobile-manipulator stack to generate and validate candidate base poses.
- [plans/manipulation_4.xml](./plans/manipulation_4.xml): execute an already validated task pose with an approved base pose.