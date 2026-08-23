#pragma once

#include <capabilities2_events/event_parameters.hpp>
#include <capabilities2_runner/action_runner.hpp>
#include <mobile_manipulator_msgs/action/execute_task_pose.hpp>
#include <mobile_manipulator_msgs/srv/check_task_feasibility.hpp>
#include <string>

namespace capabilities2_runner
{
/**
 * @brief Capability runner that sends an approved execution request to the runtime action server.
 *
 * This runner assumes the upstream planner has already selected or approved
 * the execution solution and simply adapts capability parameters into an
 * `ExecuteTaskPose` action goal.
 */
class ExecuteTaskPoseRunner : public ActionRunner<mobile_manipulator_msgs::action::ExecuteTaskPose>
{
public:
  /**
   * @brief Construct the runner with the default action-runner base setup.
   */
  ExecuteTaskPoseRunner() : ActionRunner()
  {
  }

  /**
   * @brief Initialize the action client and emit the started event.
   *
   * @param node ROS node that owns the action client.
   * @param run_config Runner configuration from the capability provider.
   * @param bond_id Lifecycle bond identifier for startup notification.
   */
  void start(rclcpp::Node::SharedPtr node, const runner_opts& run_config, const std::string& bond_id) override
  {
    init_action(node, run_config, "/execute_task_pose", "mobile_manipulator_msgs::action::ExecuteTaskPose");
    emit_started(bond_id, "", param_on_started());
  }

protected:
  /**
   * @brief Build an execution goal from capability event parameters.
   *
   * @param parameters Event input parameters provided by the capability system.
   * @return Populated action goal.
   */
  mobile_manipulator_msgs::action::ExecuteTaskPose::Goal
  generate_goal(capabilities2_events::EventParameters& parameters) override
  {
    mobile_manipulator_msgs::action::ExecuteTaskPose::Goal goal;
    goal.pose.position.x = std::any_cast<double>(parameters.get_value("x", 0.0));
    goal.pose.position.y = std::any_cast<double>(parameters.get_value("y", 0.0));
    goal.pose.position.z = std::any_cast<double>(parameters.get_value("z", 0.0));
    goal.pose.orientation.x = std::any_cast<double>(parameters.get_value("qx", 0.0));
    goal.pose.orientation.y = std::any_cast<double>(parameters.get_value("qy", 0.0));
    goal.pose.orientation.z = std::any_cast<double>(parameters.get_value("qz", 0.0));
    goal.pose.orientation.w = std::any_cast<double>(parameters.get_value("qw", 1.0));
    goal.frame_id = std::any_cast<std::string>(parameters.get_value("frame_id", std::string("map")));
    goal.mode = std::any_cast<std::string>(parameters.get_value("mode", std::string("mobile_manipulator")));
    goal.validated = std::any_cast<bool>(parameters.get_value("validated", false));
    goal.approved_base_pose_valid = std::any_cast<bool>(parameters.get_value("approved_base_pose_valid", false));
    goal.approved_base_pose.position.x = std::any_cast<double>(parameters.get_value("approved_base_x", 0.0));
    goal.approved_base_pose.position.y = std::any_cast<double>(parameters.get_value("approved_base_y", 0.0));
    goal.approved_base_pose.position.z = std::any_cast<double>(parameters.get_value("approved_base_z", 0.0));
    goal.approved_base_pose.orientation.x = std::any_cast<double>(parameters.get_value("approved_base_qx", 0.0));
    goal.approved_base_pose.orientation.y = std::any_cast<double>(parameters.get_value("approved_base_qy", 0.0));
    goal.approved_base_pose.orientation.z = std::any_cast<double>(parameters.get_value("approved_base_qz", 0.0));
    goal.approved_base_pose.orientation.w = std::any_cast<double>(parameters.get_value("approved_base_qw", 1.0));
    goal.approved_base_frame = std::any_cast<std::string>(parameters.get_value("approved_base_frame", std::string("")));
    return goal;
  }

  /**
   * @brief Convert action feedback into a compact capability feedback string.
   *
   * @param msg Feedback message from the action server.
   * @return Human-readable stage and status summary.
   */
  std::string
  generate_feedback(const mobile_manipulator_msgs::action::ExecuteTaskPose::Feedback::ConstSharedPtr msg) override
  {
    return msg->stage + ": " + msg->message;
  }

  /**
   * @brief Cache the latest action result for later event emission.
   *
   * @param result Result returned by the runtime action server.
   */
  void process_result(mobile_manipulator_msgs::action::ExecuteTaskPose::Result::SharedPtr result) override
  {
    result_ = result;
  }

  /**
   * @brief Convert an action result into capability output parameters.
   *
   * @param response Result returned by the runtime action server.
   * @return Event parameters containing execution outcome fields.
   */
  capabilities2_events::EventParameters
  response_parameters(const mobile_manipulator_msgs::action::ExecuteTaskPose::Result& response) const
  {
    capabilities2_events::EventParameters parameters;
    parameters.set_value("success", response.success, capabilities2_events::OptionType::BOOL);
    parameters.set_value("feasible", response.feasible, capabilities2_events::OptionType::BOOL);
    parameters.set_value("mode_used", response.mode_used, capabilities2_events::OptionType::STRING);
    parameters.set_value("failure_reason", response.failure_reason, capabilities2_events::OptionType::STRING);
    parameters.set_value("suggested_fallback", response.suggested_fallback, capabilities2_events::OptionType::STRING);
    parameters.set_value("message", response.message, capabilities2_events::OptionType::STRING);
    parameters.set_value("stage", response.stage, capabilities2_events::OptionType::STRING);
    parameters.set_value("base_target_pose_valid", response.base_target_pose_valid,
                         capabilities2_events::OptionType::BOOL);
    return parameters;
  }

  /**
   * @brief Return output parameters for a successful action completion.
   *
   * @return Event parameters derived from the cached action result.
   */
  capabilities2_events::EventParameters param_on_success() override
  {
    return response_parameters(*result_);
  }

  /**
   * @brief Return output parameters for a failed or aborted action completion.
   *
   * @return Event parameters derived from the cached action result.
   */
  capabilities2_events::EventParameters param_on_failure() override
  {
    return response_parameters(*result_);
  }
};
}  // namespace capabilities2_runner