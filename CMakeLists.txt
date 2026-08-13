cmake_minimum_required(VERSION 3.5)
project(manager_lane_follower)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

set(dependencies
  "rclcpp"
  "rclcpp_action"
  "nav2_msgs"
  "geometry_msgs"
  "rcl_interfaces"
)

set(msg_files
  "msg/Waypoint.msg"
)

set(srv_files
  "srv/SetWaypointList.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES
    geometry_msgs
)

add_executable(manager_follower src/manager_lane_follower_node.cpp)
ament_target_dependencies(manager_follower ${dependencies})

add_executable(dummy_waypoint_sender src/dummy_waypoint_sender.cpp)
ament_target_dependencies(dummy_waypoint_sender ${dependencies})

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(manager_follower "${cpp_typesupport_target}")
target_link_libraries(dummy_waypoint_sender "${cpp_typesupport_target}")

ament_export_dependencies(rosidl_default_runtime)

install(TARGETS
  manager_follower
  dummy_waypoint_sender
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
