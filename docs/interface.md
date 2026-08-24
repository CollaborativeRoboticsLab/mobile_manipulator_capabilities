# Interface Information

The mobile_manipulator_capabilities package exposes three capability runners that wrap the public ROS APIs from `mobile_manipulator_ros`. These runners cover direct feasibility checking, feasible-pose search, and execution of an already approved task pose.

## CheckTaskFeasibilityRunner

This runner sends a task pose through `mobile_manipulator_msgs/srv/CheckTaskFeasibility`.

Input Parameters:

| Parameter Name | Type | Description |
| --- | --- | --- |
| x | double | Target position X in the supplied frame. |
| y | double | Target position Y in the supplied frame. |
| z | double | Target position Z in the supplied frame. |
| qx | double | Target orientation quaternion X. Defaults to `0.0`. |
| qy | double | Target orientation quaternion Y. Defaults to `0.0`. |
| qz | double | Target orientation quaternion Z. Defaults to `0.0`. |
| qw | double | Target orientation quaternion W. Defaults to `1.0`. |
| frame_id | string | Frame of the requested task pose. Defaults to `map`. |
| mode | string | Validation mode such as `arm_only_ik`, `arm_only_plan`, or `mobile_manipulator`. |
| base_pose_valid | bool | Whether a concrete base pose is supplied for validation. Defaults to `false`. |
| base_x | double | Supplied base pose X. |
| base_y | double | Supplied base pose Y. |
| base_z | double | Supplied base pose Z. |
| base_qx | double | Supplied base pose quaternion X. |
| base_qy | double | Supplied base pose quaternion Y. |
| base_qz | double | Supplied base pose quaternion Z. |
| base_qw | double | Supplied base pose quaternion W. |
| base_frame_id | string | Frame of the supplied base pose. |

Output Parameters:

| Parameter Name | Type | Description |
| --- | --- | --- |
| feasible | bool | Whether the requested task is feasible. |
| failure_reason | string | Machine-readable infeasibility reason. |
| suggested_fallback | string | Recommended fallback action when not feasible. |
| message | string | Human-readable result summary. |

## FindFeasiblePoseRunner

This runner sends a target pose through `mobile_manipulator_msgs/srv/FindFeasiblePose` and asks the runtime server to generate and validate candidate base poses.

Input Parameters:

| Parameter Name | Type | Description |
| --- | --- | --- |
| x | double | Target position X in the supplied frame. |
| y | double | Target position Y in the supplied frame. |
| z | double | Target position Z in the supplied frame. |
| qx | double | Target orientation quaternion X. Defaults to `0.0`. |
| qy | double | Target orientation quaternion Y. Defaults to `0.0`. |
| qz | double | Target orientation quaternion Z. Defaults to `0.0`. |
| qw | double | Target orientation quaternion W. Defaults to `1.0`. |
| frame_id | string | Frame of the requested task pose. Defaults to `map`. |
| mode | string | Search mode for candidate generation. Defaults to `mobile_manipulator`. |
| base_pose_valid | bool | Whether a reference base pose is supplied for ranking. Defaults to `false`. |
| base_x | double | Optional reference base pose X. |
| base_y | double | Optional reference base pose Y. |
| base_z | double | Optional reference base pose Z. |
| base_qx | double | Optional reference base pose quaternion X. |
| base_qy | double | Optional reference base pose quaternion Y. |
| base_qz | double | Optional reference base pose quaternion Z. |
| base_qw | double | Optional reference base pose quaternion W. |
| base_frame_id | string | Frame of the optional reference base pose. |

Output Parameters:

| Parameter Name | Type | Description |
| --- | --- | --- |
| feasible | bool | Whether a feasible candidate was found. |
| failure_reason | string | Machine-readable reason when search fails. |
| message | string | Human-readable search result summary. |
| base_target_pose_valid | bool | Whether the returned base target pose is valid. |

## ExecuteTaskPoseRunner

This runner sends an approved solution through `mobile_manipulator_msgs/action/ExecuteTaskPose`.

Input Parameters:

| Parameter Name | Type | Description |
| --- | --- | --- |
| x | double | Approved target position X. |
| y | double | Approved target position Y. |
| z | double | Approved target position Z. |
| qx | double | Approved target orientation quaternion X. Defaults to `0.0`. |
| qy | double | Approved target orientation quaternion Y. Defaults to `0.0`. |
| qz | double | Approved target orientation quaternion Z. Defaults to `0.0`. |
| qw | double | Approved target orientation quaternion W. Defaults to `1.0`. |
| frame_id | string | Frame of the approved target pose. Defaults to `map`. |
| mode | string | Execution mode associated with the approved solution. Defaults to `mobile_manipulator`. |
| validated | bool | Whether the target pose was validated before execution. Defaults to `false`. |
| approved_base_pose_valid | bool | Whether an approved base pose accompanies the target pose. Defaults to `false`. |
| approved_base_x | double | Approved base pose X. |
| approved_base_y | double | Approved base pose Y. |
| approved_base_z | double | Approved base pose Z. |
| approved_base_qx | double | Approved base pose quaternion X. |
| approved_base_qy | double | Approved base pose quaternion Y. |
| approved_base_qz | double | Approved base pose quaternion Z. |
| approved_base_qw | double | Approved base pose quaternion W. |
| approved_base_frame | string | Frame of the approved base pose. |

Output Parameters:

| Parameter Name | Type | Description |
| --- | --- | --- |
| message | string | Human-readable execution result message. |
| failure_reason | string | Machine-readable execution failure reason. |
| stage | string | Terminal or latest coordinator stage. |