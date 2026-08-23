#pragma once

#include <capabilities2_events/event_parameters.hpp>
#include <capabilities2_runner/service_runner.hpp>
#include <mobile_manipulator_msgs/srv/check_task_feasibility.hpp>
#include <string>

namespace capabilities2_runner
{
/**
 * @brief Capability runner that calls the generic task-feasibility service.
 *
 * This runner adapts flat capability parameters into a
 * `mobile_manipulator_msgs::srv::CheckTaskFeasibility` request and maps the
 * structured response back into capability event parameters.
 */
class CheckTaskFeasibilityRunner : public ServiceRunner<mobile_manipulator_msgs::srv::CheckTaskFeasibility>
{
public:
  /**
   * @brief Construct the runner with the default service-runner base setup.
   */
  CheckTaskFeasibilityRunner() : ServiceRunner()
  {
  }

  /**
   * @brief Initialize the service client and emit the started event.
   *
   * @param node ROS node that owns the client.
   * @param run_config Runner configuration from the capability provider.
   * @param bond_id Lifecycle bond identifier for startup notification.
   */
  void start(rclcpp::Node::SharedPtr node, const runner_opts& run_config, const std::string& bond_id) override
  {
    init_service(node, run_config, "/check_task_feasibility", "mobile_manipulator_msgs::srv::CheckTaskFeasibility");
    emit_started(bond_id, "", param_on_started());
  }

protected:
  /**
   * @brief Build a task-feasibility request from capability event parameters.
   *
   * @param parameters Event input parameters provided by the capability system.
   * @return Populated service request.
   */
  mobile_manipulator_msgs::srv::CheckTaskFeasibility::Request
  generate_request(capabilities2_events::EventParameters& parameters) override
  {
    mobile_manipulator_msgs::srv::CheckTaskFeasibility::Request request;
    request.pose.position.x = std::any_cast<double>(parameters.get_value("x", 0.0));
    request.pose.position.y = std::any_cast<double>(parameters.get_value("y", 0.0));
    request.pose.position.z = std::any_cast<double>(parameters.get_value("z", 0.0));
    request.pose.orientation.x = std::any_cast<double>(parameters.get_value("qx", 0.0));
    request.pose.orientation.y = std::any_cast<double>(parameters.get_value("qy", 0.0));
    request.pose.orientation.z = std::any_cast<double>(parameters.get_value("qz", 0.0));
    request.pose.orientation.w = std::any_cast<double>(parameters.get_value("qw", 1.0));
    request.frame_id = std::any_cast<std::string>(parameters.get_value("frame_id", std::string("map")));
    request.mode = std::any_cast<std::string>(parameters.get_value("mode", std::string("mobile_manipulator")));
    request.base_pose_valid = std::any_cast<bool>(parameters.get_value("base_pose_valid", false));
    request.base_pose.position.x = std::any_cast<double>(parameters.get_value("base_x", 0.0));
    request.base_pose.position.y = std::any_cast<double>(parameters.get_value("base_y", 0.0));
    request.base_pose.position.z = std::any_cast<double>(parameters.get_value("base_z", 0.0));
    request.base_pose.orientation.x = std::any_cast<double>(parameters.get_value("base_qx", 0.0));
    request.base_pose.orientation.y = std::any_cast<double>(parameters.get_value("base_qy", 0.0));
    request.base_pose.orientation.z = std::any_cast<double>(parameters.get_value("base_qz", 0.0));
    request.base_pose.orientation.w = std::any_cast<double>(parameters.get_value("base_qw", 1.0));
    request.base_frame_id = std::any_cast<std::string>(parameters.get_value("base_frame_id", std::string("")));
    return request;
  }

  /**
   * @brief Cache the latest service response for later event emission.
   *
   * @param response Service response returned by the runtime server.
   */
  void process_response(mobile_manipulator_msgs::srv::CheckTaskFeasibility::Response::SharedPtr response) override
  {
    feasible_ = response->feasible;
    mode_used_ = response->mode_used;
    failure_reason_ = response->failure_reason;
    suggested_fallback_ = response->suggested_fallback;
    message_ = response->message;
    planning_frame_ = response->planning_frame;
    planning_pose_valid_ = response->planning_pose_valid;
    planning_pose_ = response->planning_pose;
    base_target_frame_ = response->base_target_frame;
    base_target_pose_valid_ = response->base_target_pose_valid;
    base_target_pose_ = response->base_target_pose;
    candidate_score_ = response->candidate_score;
  }

  /**
   * @brief Convert a successful service response into capability output parameters.
   *
   * @return Event parameters containing feasibility and pose details.
   */
  capabilities2_events::EventParameters param_on_success() override
  {
    capabilities2_events::EventParameters parameters;
    parameters.set_value("feasible", feasible_, capabilities2_events::OptionType::BOOL);
    parameters.set_value("mode_used", mode_used_, capabilities2_events::OptionType::STRING);
    parameters.set_value("failure_reason", failure_reason_, capabilities2_events::OptionType::STRING);
    parameters.set_value("suggested_fallback", suggested_fallback_, capabilities2_events::OptionType::STRING);
    parameters.set_value("message", message_, capabilities2_events::OptionType::STRING);
    parameters.set_value("planning_frame", planning_frame_, capabilities2_events::OptionType::STRING);
    parameters.set_value("planning_pose_valid", planning_pose_valid_, capabilities2_events::OptionType::BOOL);
    parameters.set_value("planning_x", planning_pose_.position.x, capabilities2_events::OptionType::DOUBLE);
    parameters.set_value("planning_y", planning_pose_.position.y, capabilities2_events::OptionType::DOUBLE);
    parameters.set_value("planning_z", planning_pose_.position.z, capabilities2_events::OptionType::DOUBLE);
    parameters.set_value("planning_qx", planning_pose_.orientation.x, capabilities2_events::OptionType::DOUBLE);
    parameters.set_value("planning_qy", planning_pose_.orientation.y, capabilities2_events::OptionType::DOUBLE);
    parameters.set_value("planning_qz", planning_pose_.orientation.z, capabilities2_events::OptionType::DOUBLE);
    parameters.set_value("planning_qw", planning_pose_.orientation.w, capabilities2_events::OptionType::DOUBLE);
    parameters.set_value("base_target_frame", base_target_frame_, capabilities2_events::OptionType::STRING);
    parameters.set_value("base_target_pose_valid", base_target_pose_valid_, capabilities2_events::OptionType::BOOL);
    parameters.set_value("base_target_x", base_target_pose_.position.x, capabilities2_events::OptionType::DOUBLE);
    parameters.set_value("base_target_y", base_target_pose_.position.y, capabilities2_events::OptionType::DOUBLE);
    parameters.set_value("base_target_z", base_target_pose_.position.z, capabilities2_events::OptionType::DOUBLE);
    parameters.set_value("base_target_qx", base_target_pose_.orientation.x, capabilities2_events::OptionType::DOUBLE);
    parameters.set_value("base_target_qy", base_target_pose_.orientation.y, capabilities2_events::OptionType::DOUBLE);
    parameters.set_value("base_target_qz", base_target_pose_.orientation.z, capabilities2_events::OptionType::DOUBLE);
    parameters.set_value("base_target_qw", base_target_pose_.orientation.w, capabilities2_events::OptionType::DOUBLE);
    parameters.set_value("candidate_score", candidate_score_, capabilities2_events::OptionType::DOUBLE);
    return parameters;
  }

  /**
   * @brief Return failure output when the service call itself fails.
   *
   * @return Event parameters containing a generic transport-level failure message.
   */
  capabilities2_events::EventParameters param_on_failure() override
  {
    capabilities2_events::EventParameters parameters;
    parameters.set_value("message", std::string("Task feasibility service call failed."),
                         capabilities2_events::OptionType::STRING);
    return parameters;
  }

private:
  bool feasible_{ false };
  std::string mode_used_;
  std::string failure_reason_;
  std::string suggested_fallback_;
  std::string message_;
  std::string planning_frame_;
  bool planning_pose_valid_{ false };
  geometry_msgs::msg::Pose planning_pose_;
  std::string base_target_frame_;
  bool base_target_pose_valid_{ false };
  geometry_msgs::msg::Pose base_target_pose_;
  double candidate_score_{ 0.0 };
};
}  // namespace capabilities2_runner