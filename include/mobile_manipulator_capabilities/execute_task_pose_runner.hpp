#pragma once

#include <chrono>
#include <future>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <grasping_msgs/action/move_to_pose.hpp>
#include <mobile_manipulator_msgs/srv/check_task_feasibility.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <capabilities2_events/event_parameters.hpp>
#include <capabilities2_runner/threadtrigger_runner.hpp>

namespace capabilities2_runner
{
class ExecuteTaskPoseRunner : public ThreadTriggerRunner
{
public:
  ExecuteTaskPoseRunner()
  : ThreadTriggerRunner()
  {
  }

  void start(rclcpp::Node::SharedPtr node, const runner_opts & run_config, const std::string & bond_id) override
  {
    init_base(node, run_config);
    task_client_ = node_->create_client<mobile_manipulator_msgs::srv::CheckTaskFeasibility>("/check_task_feasibility");
    navigation_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node_, "/navigate_to_pose");
    arm_client_ = rclcpp_action::create_client<grasping_msgs::action::MoveToPose>(node_, "/move_arm_to_pose");
    emit_started(bond_id, "", param_on_started());
  }

  void stop(const std::string & bond_id, const std::string & instance_id = "") override
  {
    if (!node_ || !task_client_ || !navigation_client_ || !arm_client_)
    {
      throw runner_exception("cannot stop runner that was not started");
    }
    emit_stopped(bond_id, instance_id, param_on_stopped());
  }

protected:
  void execution(capabilities2_events::EventParameters parameters, const std::string & thread_id) override
  {
    const std::string bond_id = ThreadTriggerRunner::bond_from_thread_id(thread_id);
    const std::string instance_id = ThreadTriggerRunner::instance_from_thread_id(thread_id);

    const auto request = build_task_request(parameters);
    auto task_response = call_task_service(request);
    if (!task_response)
    {
      emit_failed(bond_id, instance_id, failure_parameters("service_unavailable", "task_feasibility", "Task feasibility service call failed."));
      return;
    }

    if (!task_response->feasible)
    {
      emit_failed(bond_id, instance_id, response_parameters(*task_response, "task_feasibility"));
      return;
    }

    if (task_response->base_target_pose_valid)
    {
      std::string navigation_message;
      if (!navigate_to_base_target(*task_response, navigation_message))
      {
        emit_failed(
          bond_id, instance_id,
          failure_parameters(task_response->failure_reason, "navigation", navigation_message, task_response->suggested_fallback));
        return;
      }

      auto recheck_request = request;
      recheck_request.mode = "arm_only_plan";
      auto recheck_response = call_task_service(recheck_request);
      if (!recheck_response)
      {
        emit_failed(bond_id, instance_id, failure_parameters("service_unavailable", "recheck", "Task feasibility re-check failed after navigation."));
        return;
      }
      if (!recheck_response->feasible)
      {
        emit_failed(bond_id, instance_id, response_parameters(*recheck_response, "recheck"));
        return;
      }
    }

    std::string arm_message;
    if (!move_arm_to_target(request, arm_message))
    {
      emit_failed(bond_id, instance_id, failure_parameters("arm_execution_failed", "arm_execution", arm_message, task_response->suggested_fallback));
      return;
    }

    auto success_parameters = response_parameters(*task_response, "arm_execution");
    success_parameters.set_value("message", arm_message, capabilities2_events::OptionType::STRING);
    emit_succeeded(bond_id, instance_id, success_parameters);
  }

private:
  mobile_manipulator_msgs::srv::CheckTaskFeasibility::Request build_task_request(
    capabilities2_events::EventParameters & parameters) const
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
    return request;
  }

  std::shared_ptr<mobile_manipulator_msgs::srv::CheckTaskFeasibility::Response> call_task_service(
    const mobile_manipulator_msgs::srv::CheckTaskFeasibility::Request & request)
  {
    if (!task_client_->wait_for_service(std::chrono::seconds(3)))
    {
      return nullptr;
    }

    auto request_ptr = std::make_shared<mobile_manipulator_msgs::srv::CheckTaskFeasibility::Request>(request);
    auto future = task_client_->async_send_request(request_ptr);
    if (future.wait_for(std::chrono::seconds(10)) != std::future_status::ready)
    {
      return nullptr;
    }
    return future.get();
  }

  bool navigate_to_base_target(
    const mobile_manipulator_msgs::srv::CheckTaskFeasibility::Response & task_response,
    std::string & message)
  {
    if (!navigation_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
      message = "NavigateToPose action server '/navigate_to_pose' is not available.";
      return false;
    }

    nav2_msgs::action::NavigateToPose::Goal goal;
    goal.pose.header.frame_id = task_response.base_target_frame.empty() ? "map" : task_response.base_target_frame;
    goal.pose.header.stamp = node_->now();
    goal.pose.pose = task_response.base_target_pose;

    auto send_future = navigation_client_->async_send_goal(goal);
    if (send_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready)
    {
      message = "Failed to send NavigateToPose goal.";
      return false;
    }

    auto goal_handle = send_future.get();
    if (!goal_handle)
    {
      message = "NavigateToPose goal was rejected.";
      return false;
    }

    auto result_future = navigation_client_->async_get_result(goal_handle);
    if (result_future.wait_for(std::chrono::seconds(120)) != std::future_status::ready)
    {
      message = "NavigateToPose result not received before timeout.";
      return false;
    }

    const auto result = result_future.get();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
    {
      message = "NavigateToPose action failed.";
      return false;
    }

    message = "Base reposition completed successfully.";
    return true;
  }

  bool move_arm_to_target(
    const mobile_manipulator_msgs::srv::CheckTaskFeasibility::Request & request,
    std::string & message)
  {
    if (!arm_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
      message = "MoveToPose action server '/move_arm_to_pose' is not available.";
      return false;
    }

    grasping_msgs::action::MoveToPose::Goal goal;
    goal.target_pose.header.frame_id = request.frame_id;
    goal.target_pose.header.stamp = node_->now();
    goal.target_pose.pose = request.pose;

    auto send_future = arm_client_->async_send_goal(goal);
    if (send_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready)
    {
      message = "Failed to send MoveToPose goal.";
      return false;
    }

    auto goal_handle = send_future.get();
    if (!goal_handle)
    {
      message = "MoveToPose goal was rejected.";
      return false;
    }

    auto result_future = arm_client_->async_get_result(goal_handle);
    if (result_future.wait_for(std::chrono::seconds(120)) != std::future_status::ready)
    {
      message = "MoveToPose result not received before timeout.";
      return false;
    }

    const auto result = result_future.get();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result || !result.result->success)
    {
      message = result.result ? result.result->message : "MoveToPose action failed.";
      return false;
    }

    message = result.result->message;
    return true;
  }

  capabilities2_events::EventParameters response_parameters(
    const mobile_manipulator_msgs::srv::CheckTaskFeasibility::Response & response,
    const std::string & stage) const
  {
    capabilities2_events::EventParameters parameters;
    parameters.set_value("feasible", response.feasible, capabilities2_events::OptionType::BOOL);
    parameters.set_value("mode_used", response.mode_used, capabilities2_events::OptionType::STRING);
    parameters.set_value("failure_reason", response.failure_reason, capabilities2_events::OptionType::STRING);
    parameters.set_value("suggested_fallback", response.suggested_fallback, capabilities2_events::OptionType::STRING);
    parameters.set_value("message", response.message, capabilities2_events::OptionType::STRING);
    parameters.set_value("stage", stage, capabilities2_events::OptionType::STRING);
    parameters.set_value("base_target_pose_valid", response.base_target_pose_valid, capabilities2_events::OptionType::BOOL);
    return parameters;
  }

  capabilities2_events::EventParameters failure_parameters(
    const std::string & failure_reason,
    const std::string & stage,
    const std::string & message,
    const std::string & suggested_fallback = "") const
  {
    capabilities2_events::EventParameters parameters;
    parameters.set_value("feasible", false, capabilities2_events::OptionType::BOOL);
    parameters.set_value("failure_reason", failure_reason, capabilities2_events::OptionType::STRING);
    parameters.set_value("suggested_fallback", suggested_fallback, capabilities2_events::OptionType::STRING);
    parameters.set_value("message", message, capabilities2_events::OptionType::STRING);
    parameters.set_value("stage", stage, capabilities2_events::OptionType::STRING);
    return parameters;
  }

  rclcpp::Client<mobile_manipulator_msgs::srv::CheckTaskFeasibility>::SharedPtr task_client_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_client_;
  rclcpp_action::Client<grasping_msgs::action::MoveToPose>::SharedPtr arm_client_;
};
}  // namespace capabilities2_runner