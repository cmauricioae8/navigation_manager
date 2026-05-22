# navigation_manager

`navigation_manager` is a ROS 2 package that manages waypoint missions using:

- A **service server** to receive waypoint lists.
- An **action client** to send sequential goals to `/navigate_to_pose` (`nav2_msgs/action/NavigateToPose`).

This package is designed to work with a `NavigateToPose` server such as your `nav2_lane_follower` node.

## Nodes

- **`nav2_manager`**
  - Node name: `nav2_waypoint_manager`
  - Service server: `set_waypoint_list`
  - Action client: `/navigate_to_pose`

- **`dummy_waypoint_sender`**
  - Node name: `dummy_waypoint_sender`
  - Sends a hardcoded mission to `set_waypoint_list` for quick testing.

## Interfaces

### Message: `Waypoint.msg`

```text
geometry_msgs/PoseStamped pose
bool is_mandatory
int32 wait_time
bool with_orientation
```

### Service: `SetWaypointList.srv`

```text
Waypoint[] waypoints
bool replace_active_mission
int32 loops_number
bool is_path
---
bool accepted
uint32 mission_id
string message
```

## Mission Execution Logic

For each waypoint:

1. Send `NavigateToPose` goal.
2. If goal **succeeds**, wait `wait_time` seconds, then continue.
3. If goal is **aborted/canceled/rejected**, wait `time_to_recover` and retry same waypoint.

### `loops_number` semantics

- `-1`: infinite loop over the full waypoint list.
- `> 0`: execute the full list exactly `loops_number` times total.
- `0`: request is rejected as invalid.

### `with_orientation` mapping

When creating each `NavigateToPose` goal:

- `with_orientation = true` -> `behavior_tree = ""` (empty string)
- `with_orientation = false` -> `behavior_tree = "__skip_turn__"` (or configured token)

This enables/skip final orientation alignment depending on server behavior.

### `is_path`

`is_path` is accepted and stored in manager mission state for future behavior extensions.
Current workflow does not branch on `is_path`.

## Parameters (`nav2_manager`)

- `time_to_recover` (double, default: `3.0`)
  - Retry delay (seconds) after abort/cancel/reject.
  - Dynamic at runtime.

- `skip_turn_behavior_tree_token` (string, default: `__skip_turn__`)
  - Non-empty token used in `NavigateToPose.goal.behavior_tree` when `with_orientation=false`.
  - Dynamic at runtime.

## Build

If this package is still nested under:

- `src/nav2_lane_follower/navigation_manager`

build with:

```bash
cd ~/colcon_ws
colcon build --base-paths src/nav2_lane_follower/navigation_manager --symlink-install
source install/setup.bash
```

After moving it to standard workspace level (`src/navigation_manager`), use:

```bash
cd ~/colcon_ws
colcon build --packages-select navigation_manager --symlink-install
source install/setup.bash
```

## Run

Terminal 1:

```bash
ros2 run navigation_manager nav2_manager
```

Terminal 2 (optional quick test):

```bash
ros2 run navigation_manager dummy_waypoint_sender
```

## Service Call Example

```bash
ros2 service call /set_waypoint_list navigation_manager/srv/SetWaypointList "{
  waypoints: [
    {
      pose: {
        header: {frame_id: map},
        pose: {
          position: {x: 1.0, y: 0.0, z: 0.0},
          orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
        }
      },
      is_mandatory: true,
      wait_time: 2,
      with_orientation: true
    },
    {
      pose: {
        header: {frame_id: map},
        pose: {
          position: {x: 2.0, y: 0.0, z: 0.0},
          orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
        }
      },
      is_mandatory: false,
      wait_time: 0,
      with_orientation: false
    }
  ],
  replace_active_mission: true,
  loops_number: 1,
  is_path: false
}"
```
