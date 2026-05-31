# hector_multi_robot_announcement

Announcement nodes for multi robot use.

- [SimpleMultiRobotAnnouncer](#SimpleMultiRobotAnnouncer)

## `SimpleMultiRobotAnnouncer`

### Subscribed Topics

Only subscribed when `enable_tf_forwarding` is true and `robot_namespace` is set (non-root).

| Topic                         | Type                   | Description                                |
|-------------------------------|------------------------|--------------------------------------------|
| `<robot_namespace>/tf`        | tf2_msgs/msg/TFMessage | Robot-local dynamic transforms to forward. |
| `<robot_namespace>/tf_static` | tf2_msgs/msg/TFMessage | Robot-local static transforms to forward.  |

### Published Topics

| Topic               | Type                                          | Description                                                                             |
|---------------------|-----------------------------------------------|-----------------------------------------------------------------------------------------|
| robot_announcement  | hector_multi_robot_msgs/msg/RobotAnnouncement | The announcement message for the robot.                                                 |
| /robot_announcement | hector_multi_robot_msgs/msg/RobotAnnouncement | The announcement message for the robot in the global namespace (if node is namespaced). |
| robot_status        | hector_multi_robot_msgs/msg/RobotStatus       | The status of the robot.                                                                |
| /tf                 | tf2_msgs/msg/TFMessage                        | Forwarded dynamic transforms, frame ids prefixed, rate-limited per child frame.         |
| /tf_static          | tf2_msgs/msg/TFMessage                        | Forwarded static transforms, frame ids prefixed (no rate limit).                        |

`/tf` and `/tf_static` are only published when forwarding is active (see Subscribed Topics). They are
published on the global (un-namespaced) topics even when the node runs under the common multi-robot
`/tf:=tf` remap: the node injects identity remaps (`/tf:=/tf`, `/tf_static:=/tf_static`) that out-rank it.

### Services

| Service | Type | Description |
|---------|------|-------------|
|         |      |             |

### Actions

| Action | Type | Description |
|--------|------|-------------|
|        |      |             |

### Parameters

All parameters are read-only.

| Parameter                                  | Type     | Default | Description                                                                                              |
|--------------------------------------------|----------|---------|----------------------------------------------------------------------------------------------------------|
| robot_id                                   | string   |         | The unique id of the robot.                                                                              |
| robot_name                                 | string   |         | The human-readable name of the robot.                                                                    |
| robot_namespace                            | string   |         | The ROS namespace of the robot.                                                                          |
| enable_tf_forwarding                       | bool     | false   | Forward the robot's tf tree to the global tf tree, prefixing frame ids with the robot namespace.         |
| tf_config.global_frames                    | string[] | []      | Frame ids shared across robots that must not be prefixed when forwarded (e.g. `map`, `world`).           |
| tf_config.max_rate                         | double   | 30.0    | Max rate (Hz) at which frames without an explicit per-frame rate are forwarded. 0 disables the limit.    |
| tf_config.frame_configs.\<frame>.rate       | double   |         | Per-frame max forwarding rate (Hz) for child frame `<frame>`. 0 forwards every message.                 |

To forward every frame (no shared frames), omit `tf_config.global_frames` entirely. Do **not**
set it to an empty list (`global_frames: []`): an empty YAML sequence has no element type, so the
node aborts on startup with `parameter_value_from failed ... No parameter value set`. The same
applies to any other array parameter.

See [config/simple_announcer.example.yaml](config/simple_announcer.example.yaml) for a full example.
