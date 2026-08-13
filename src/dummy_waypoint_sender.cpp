/*
@description:
Dummy node that sends a waypoint list mission to the waypoint manager service.
*/

#include <chrono>
#include <functional>
#include <memory>

#include "manager_lane_follower/msg/waypoint.hpp"
#include "manager_lane_follower/srv/set_waypoint_list.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class DummyWaypointSenderNode : public rclcpp::Node
{
public:
  using Waypoint = manager_lane_follower::msg::Waypoint;
  using SetWaypointList = manager_lane_follower::srv::SetWaypointList;

  DummyWaypointSenderNode()
  : Node("dummy_waypoint_sender")
  {
    set_waypoint_list_client_ = this->create_client<SetWaypointList>("set_waypoint_list");
    send_timer_ = this->create_wall_timer(1s, std::bind(&DummyWaypointSenderNode::send_mission_once, this));
  }

private:
  static Waypoint make_waypoint(
    const std::string & name,
    const double x,
    const double y,
    const double yaw_quaternion_z,
    const double yaw_quaternion_w,
    const bool is_mandatory,
    const int32_t wait_time,
    const std::string & action_on_site,
    const bool with_orientation)
  {
    Waypoint waypoint;
    waypoint.name = name;
    waypoint.pose.header.frame_id = "map";
    waypoint.pose.pose.position.x = x;
    waypoint.pose.pose.position.y = y;
    waypoint.pose.pose.position.z = 0.0;
    waypoint.pose.pose.orientation.x = 0.0;
    waypoint.pose.pose.orientation.y = 0.0;
    waypoint.pose.pose.orientation.z = yaw_quaternion_z;
    waypoint.pose.pose.orientation.w = yaw_quaternion_w;
    waypoint.is_mandatory = is_mandatory;
    waypoint.wait_time = wait_time;
    waypoint.action_on_site = action_on_site;
    waypoint.with_orientation = with_orientation;
    return waypoint;
  }

  void send_mission_once()
  {
    if (request_sent_) {
      return;
    }

    if (!set_waypoint_list_client_->wait_for_service(100ms)) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "Waiting for set_waypoint_list service...");
      return;
    }

    auto request = std::make_shared<SetWaypointList::Request>();
    request->replace_active_mission = true;
    request->loops_number = 2;
    request->is_path = false;
    request->waypoints = {
      make_waypoint("Punto1", 1.0, 0.0, 0.0, 1.0, true, 2, 
        "ffplay /home/mau2/Documents/MY_RESEARCH/CPP_tutorials/miscellaneous_examples/birds.mp3", true),
      make_waypoint("Punto2", 2.0, 0.0, 0.0, 1.0, false, 10, 
        "ffplay /home/mau2/Documents/MY_RESEARCH/CPP_tutorials/miscellaneous_examples/endgame.mp3", false),
      make_waypoint("Punto3", 2.0, 1.0, 0.0, 1.0, true, 1, "", true)
    };

    request_sent_ = true;
    auto future = set_waypoint_list_client_->async_send_request(
      request,
      [this](rclcpp::Client<SetWaypointList>::SharedFuture response_future) {
        const auto response = response_future.get();
        RCLCPP_INFO(
          this->get_logger(),
          "Service response -> accepted=%s, mission_id=%u, message='%s'",
          response->accepted ? "true" : "false",
          response->mission_id,
          response->message.c_str());
      });
    (void)future;

    RCLCPP_INFO(this->get_logger(), "Dummy mission sent");
  }

  rclcpp::Client<SetWaypointList>::SharedPtr set_waypoint_list_client_;
  rclcpp::TimerBase::SharedPtr send_timer_;
  bool request_sent_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummyWaypointSenderNode>());
  rclcpp::shutdown();
  return 0;
}
