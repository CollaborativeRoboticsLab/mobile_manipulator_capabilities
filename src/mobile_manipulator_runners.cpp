#include <pluginlib/class_list_macros.hpp>

#include <capabilities2_runner/runner_base.hpp>
#include <mobile_manipulator_capabilities/check_task_feasibility_runner.hpp>
#include <mobile_manipulator_capabilities/execute_task_pose_runner.hpp>

PLUGINLIB_EXPORT_CLASS(capabilities2_runner::CheckTaskFeasibilityRunner, capabilities2_runner::RunnerBase)
PLUGINLIB_EXPORT_CLASS(capabilities2_runner::ExecuteTaskPoseRunner, capabilities2_runner::RunnerBase)