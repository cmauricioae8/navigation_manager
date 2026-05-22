/*
@description:
Waypoint manager node:
- Service server to receive waypoint missions.
- Action client for NavigateToPose to execute missions sequentially.
*/

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "navigation_manager/msg/waypoint.hpp"
#include "navigation_manager/srv/set_waypoint_list.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class WaypointManagerNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using NavigateToPoseGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using Waypoint = navigation_manager::msg::Waypoint;
  using SetWaypointList = navigation_manager::srv::SetWaypointList;

  WaypointManagerNode()
  : Node("nav2_waypoint_manager")
  {
    this->declare_parameter<double>("time_to_recover", 3.0);
    this->declare_parameter<std::string>("skip_turn_behavior_tree_token", "__skip_turn__");
    this->get_parameter("time_to_recover", time_to_recover_);
    this->get_parameter("skip_turn_behavior_tree_token", skip_turn_behavior_tree_token_);

    on_set_params_handle_ = this->add_on_set_parameters_callback(
      std::bind(&WaypointManagerNode::parameters_cb, this, std::placeholders::_1));

    nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");
    set_waypoint_list_srv_ = this->create_service<SetWaypointList>(
      "set_waypoint_list",
      std::bind(
        &WaypointManagerNode::set_waypoint_list_cb, this,
        std::placeholders::_1, std::placeholders::_2));

    manager_timer_ = this->create_wall_timer(100ms, std::bind(&WaypointManagerNode::manager_tick, this));

    RCLCPP_INFO(this->get_logger(), "Waypoint manager ready");
  }

private:
  enum class ManagerState
  {
    IDLE,
    WAITING_FOR_SERVER,
    SENDING_GOAL,
    WAITING_RESULT,
    WAITING_WAYPOINT_DELAY,
    WAITING_RECOVERY
  };

  rcl_interfaces::msg::SetParametersResult parameters_cb(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    double new_time_to_recover = time_to_recover_;
    std::string new_skip_token = skip_turn_behavior_tree_token_;

    for (const auto & param : parameters) {
      if (param.get_name() == "time_to_recover") {
        if (param.get_type() == rclcpp::PARAMETER_DOUBLE) {
          new_time_to_recover = param.as_double();
        } else if (param.get_type() == rclcpp::PARAMETER_INTEGER) {
          new_time_to_recover = static_cast<double>(param.as_int());
        } else {
          return failed_param_result("time_to_recover must be numeric");
        }
      } else if (param.get_name() == "skip_turn_behavior_tree_token") {
        if (param.get_type() != rclcpp::PARAMETER_STRING) {
          return failed_param_result("skip_turn_behavior_tree_token must be a string");
        }
        new_skip_token = param.as_string();
      }
    }

    if (new_time_to_recover < 0.0) {
      return failed_param_result("time_to_recover must be >= 0.0");
    }
    if (new_skip_token.empty()) {
      return failed_param_result("skip_turn_behavior_tree_token must be non-empty");
    }

    time_to_recover_ = new_time_to_recover;
    skip_turn_behavior_tree_token_ = std::move(new_skip_token);

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  rcl_interfaces::msg::SetParametersResult failed_param_result(const std::string & reason) const
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;
    result.reason = reason;
    return result;
  }

  void set_waypoint_list_cb(
    const std::shared_ptr<SetWaypointList::Request> request,
    std::shared_ptr<SetWaypointList::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (request->waypoints.empty()) {
      response->accepted = false;
      response->mission_id = mission_id_;
      response->message = "Received empty waypoint list";
      return;
    }

    if (request->loops_number == 0) {
      response->accepted = false;
      response->mission_id = mission_id_;
      response->message = "loops_number cannot be 0";
      return;
    }

    if (mission_active_ && !request->replace_active_mission) {
      response->accepted = false;
      response->mission_id = mission_id_;
      response->message = "Mission already active. Set replace_active_mission=true to override";
      return;
    }

    if (mission_active_ && request->replace_active_mission) {
      cancel_active_goal_locked();
    }

    mission_waypoints_ = request->waypoints;
    for (auto & waypoint : mission_waypoints_) {
      if (waypoint.wait_time < 0) {
        waypoint.wait_time = 0;
      }
      if (waypoint.pose.header.frame_id.empty()) {
        waypoint.pose.header.frame_id = "map";
      }
    }

    loops_number_ = request->loops_number;
    loops_completed_ = 0;
    is_path_ = request->is_path;
    current_waypoint_idx_ = 0;
    latest_feedback_distance_ = 0.0;
    latest_feedback_stamp_ = this->now();
    mission_active_ = true;
    state_ = ManagerState::SENDING_GOAL;
    ++mission_id_;

    response->accepted = true;
    response->mission_id = mission_id_;
    response->message = "Mission accepted with " + std::to_string(mission_waypoints_.size()) +
      " waypoints and loops_number=" + std::to_string(loops_number_);

    RCLCPP_INFO(
      this->get_logger(),
      "Accepted mission_id=%u, waypoints=%zu, loops_number=%d, is_path=%s",
      mission_id_, mission_waypoints_.size(), loops_number_, is_path_ ? "true" : "false");
  }

  void manager_tick()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!mission_active_) {
      state_ = ManagerState::IDLE;
      return;
    }

    switch (state_) {
      case ManagerState::IDLE:
        return;
      case ManagerState::WAITING_FOR_SERVER:
      case ManagerState::SENDING_GOAL:
        send_current_waypoint_goal_locked();
        return;
      case ManagerState::WAITING_RESULT:
        // Feedback is updated by action feedback callback while waiting for result.
        return;
      case ManagerState::WAITING_WAYPOINT_DELAY:
        if (this->now() >= waiting_until_) {
          advance_after_success_locked();
        }
        return;
      case ManagerState::WAITING_RECOVERY:
        if (this->now() >= waiting_until_) {
          state_ = ManagerState::SENDING_GOAL;
        }
        return;
    }
  }

  void send_current_waypoint_goal_locked()
  {
    if (current_waypoint_idx_ >= mission_waypoints_.size()) {
      complete_or_repeat_mission_locked();
      return;
    }

    if (!nav_action_client_->action_server_is_ready()) {
      state_ = ManagerState::WAITING_FOR_SERVER;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "Waiting for /navigate_to_pose action server");
      return;
    }

    const auto mission_id_at_send = mission_id_;
    const auto & waypoint = mission_waypoints_[current_waypoint_idx_];

    NavigateToPose::Goal goal;
    goal.pose = waypoint.pose;
    goal.behavior_tree = waypoint.with_orientation ? "" : skip_turn_behavior_tree_token_;

    RCLCPP_INFO(
      this->get_logger(),
      "Sending mission_id=%u waypoint=%zu/%zu with_orientation=%s wait_time=%d mandatory=%s",
      mission_id_,
      current_waypoint_idx_ + 1,
      mission_waypoints_.size(),
      waypoint.with_orientation ? "true" : "false",
      waypoint.wait_time,
      waypoint.is_mandatory ? "true" : "false");

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback =
      [this, mission_id_at_send](const NavigateToPoseGoalHandle::SharedPtr & goal_handle) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (mission_id_at_send != mission_id_ || !mission_active_) {
          return;
        }

        if (!goal_handle) {
          RCLCPP_WARN(this->get_logger(), "Goal rejected for mission_id=%u", mission_id_);
          schedule_recovery_locked("goal rejected by action server");
          return;
        }

        active_goal_handle_ = goal_handle;
        state_ = ManagerState::WAITING_RESULT;
      };

    options.feedback_callback =
      [this, mission_id_at_send](
      NavigateToPoseGoalHandle::SharedPtr,
      const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (mission_id_at_send != mission_id_ || !mission_active_) {
          return;
        }

        latest_feedback_distance_ = feedback->distance_remaining;
        latest_feedback_stamp_ = this->now();
      };

    options.result_callback =
      [this, mission_id_at_send](const NavigateToPoseGoalHandle::WrappedResult & result) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (mission_id_at_send != mission_id_ || !mission_active_) {
          return;
        }

        active_goal_handle_.reset();

        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          const auto wait_time_seconds =
            std::max<int32_t>(0, mission_waypoints_[current_waypoint_idx_].wait_time);
          waiting_until_ = this->now() + rclcpp::Duration::from_seconds(wait_time_seconds);
          state_ = ManagerState::WAITING_WAYPOINT_DELAY;

          RCLCPP_INFO(
            this->get_logger(),
            "Waypoint %zu succeeded. Waiting %d seconds before next goal",
            current_waypoint_idx_ + 1, wait_time_seconds);
          return;
        }

        if (result.code == rclcpp_action::ResultCode::ABORTED) {
          schedule_recovery_locked("goal aborted");
          return;
        }

        if (result.code == rclcpp_action::ResultCode::CANCELED) {
          schedule_recovery_locked("goal canceled");
          return;
        }

        schedule_recovery_locked("unknown goal result code");
      };

    (void)nav_action_client_->async_send_goal(goal, options);
    state_ = ManagerState::WAITING_RESULT;
  }

  void advance_after_success_locked()
  {
    ++current_waypoint_idx_;
    complete_or_repeat_mission_locked();
  }

  void complete_or_repeat_mission_locked()
  {
    if (current_waypoint_idx_ < mission_waypoints_.size()) {
      state_ = ManagerState::SENDING_GOAL;
      return;
    }

    ++loops_completed_;
    if (loops_number_ == -1 || loops_completed_ < loops_number_) {
      current_waypoint_idx_ = 0;
      state_ = ManagerState::SENDING_GOAL;
      if (loops_number_ == -1) {
        RCLCPP_INFO(this->get_logger(), "Completed loop. Restarting mission (infinite mode)");
      } else {
        RCLCPP_INFO(
          this->get_logger(), "Completed loop %d/%d. Restarting mission",
          loops_completed_, loops_number_);
      }
      return;
    }

    mission_active_ = false;
    state_ = ManagerState::IDLE;
    RCLCPP_INFO(this->get_logger(), "Mission_id=%u completed", mission_id_);
  }

  void schedule_recovery_locked(const std::string & reason)
  {
    waiting_until_ = this->now() + rclcpp::Duration::from_seconds(time_to_recover_);
    state_ = ManagerState::WAITING_RECOVERY;
    RCLCPP_WARN(
      this->get_logger(),
      "Waypoint %zu failed (%s). Retrying in %.2f seconds",
      current_waypoint_idx_ + 1, reason.c_str(), time_to_recover_);
  }

  void cancel_active_goal_locked()
  {
    if (active_goal_handle_) {
      (void)nav_action_client_->async_cancel_goal(active_goal_handle_);
      active_goal_handle_.reset();
    }
    state_ = ManagerState::IDLE;
    mission_active_ = false;
  }

  std::mutex mutex_;

  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
  rclcpp::Service<SetWaypointList>::SharedPtr set_waypoint_list_srv_;
  rclcpp::TimerBase::SharedPtr manager_timer_;
  OnSetParametersCallbackHandle::SharedPtr on_set_params_handle_;

  std::vector<Waypoint> mission_waypoints_;
  std::shared_ptr<NavigateToPoseGoalHandle> active_goal_handle_;

  ManagerState state_{ManagerState::IDLE};
  rclcpp::Time waiting_until_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_feedback_stamp_{0, 0, RCL_ROS_TIME};
  double latest_feedback_distance_{0.0};

  double time_to_recover_{3.0};
  std::string skip_turn_behavior_tree_token_{"__skip_turn__"};

  size_t current_waypoint_idx_{0};
  int32_t loops_number_{1};
  int32_t loops_completed_{0};
  bool is_path_{false};
  bool mission_active_{false};
  uint32_t mission_id_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointManagerNode>());
  rclcpp::shutdown();
  return 0;
}
