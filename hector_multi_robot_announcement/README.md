# hector_multi_robot_announcement

Announcement nodes for multi robot use.

- [SimpleMultiRobotAnnouncer](#SimpleMultiRobotAnnouncer)

## `SimpleMultiRobotAnnouncer`

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
|       |      |             |

### Published Topics

| Topic               | Type                                         | Description                                                                             |
|---------------------|----------------------------------------------|-----------------------------------------------------------------------------------------|
| robot_announcement  | hector_multi_robot_msgs/msg/RobotAnnouncement | The announcement message for the robot.                                                 |
| /robot_announcement | hector_multi_robot_msgs/msg/RobotAnnouncement | The announcement message for the robot in the global namespace (if node is namespaced). |
| robot_status        | hector_multi_robot_msgs/msg/RobotStatus       | The status of the robot.                                                                |

### Services

| Service | Type | Description |
|---------|------|-------------|
|         |      |             |

### Actions

| Action | Type | Description |
|--------|------|-------------|
|        |      |             |

### Parameters

| Parameter       | Type   | Description                           |
|-----------------|--------|---------------------------------------|
| robot_id        | string | The unique id of the robot.           |
| robot_name      | string | The human-readable name of the robot. |
| robot_namespace | string | The ROS namespace of the robot.       |
