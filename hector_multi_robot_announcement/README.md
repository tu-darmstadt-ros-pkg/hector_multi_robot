# hector_multi_robot_announcement

Announcement nodes for multi robot use.

- [SimpleMultiRobotAnnouncer](#SimpleMultiRobotAnnouncer)

## `SimpleMultiRobotAnnouncer`

### Subscribed Topics

| Topic                         | Type                   | Description                                |
| ----------------------------- | ---------------------- | ------------------------------------------ |
| `<robot_namespace>/tf`        | tf2_msgs/msg/TFMessage | Robot-local dynamic transforms to forward. |
| `<robot_namespace>/tf_static` | tf2_msgs/msg/TFMessage | Robot-local static transforms to forward.  |

The tf topics are only subscribed when `enable_tf_forwarding` is true and `robot_namespace` is set
(non-root). Additionally, when `status.status_frequency` is > 0, each topic configured under
`status.battery.state_topics.<label>` is subscribed as a `sensor_msgs/msg/BatteryState` and aggregated into
`robot_status`.

### Published Topics

| Topic               | Type                                          | Description                                                                                                            |
| ------------------- | --------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------- |
| robot_announcement  | hector_multi_robot_msgs/msg/RobotAnnouncement | The announcement message for the robot.                                                                                |
| /robot_announcement | hector_multi_robot_msgs/msg/RobotAnnouncement | The announcement message for the robot in the global namespace (if node is namespaced).                                |
| robot_status        | hector_multi_robot_msgs/msg/RobotStatus       | Robot status: code/message and aggregated battery; latched (transient_local). Only when `status.status_frequency` > 0. |
| robot_heartbeat     | hector_multi_robot_msgs/msg/Heartbeat         | Periodic heartbeat (stamp + sequence number) for liveness. Only when `status.heartbeat_frequency` > 0.                 |
| /tf                 | tf2_msgs/msg/TFMessage                        | Forwarded dynamic transforms, frame ids prefixed, rate-limited per child frame.                                        |
| /tf_static          | tf2_msgs/msg/TFMessage                        | Forwarded static transforms, frame ids prefixed (no rate limit).                                                       |

`/tf` and `/tf_static` are only published when forwarding is active (see Subscribed Topics). They are
published on the global (un-namespaced) topics even when the node runs under the common multi-robot
`/tf:=tf` remap: the node injects identity remaps (`/tf:=/tf`, `/tf_static:=/tf_static`) that out-rank it.

### Services

| Service    | Type                                  | Description                                                                                           |
| ---------- | ------------------------------------- | ----------------------------------------------------------------------------------------------------- |
| set_status | hector_multi_robot_msgs/srv/SetStatus | Set the status code/message reported in `robot_status`. Available when `status.status_frequency` > 0. |

### Parameters

All parameters are read-only.

| Parameter                               | Type     | Default   | Description                                                                                                              |
| --------------------------------------- | -------- | --------- | ------------------------------------------------------------------------------------------------------------------------ |
| robot_id                                | string   |           | The unique id of the robot.                                                                                              |
| robot_name                              | string   |           | The human-readable name of the robot.                                                                                    |
| robot_namespace                         | string   |           | The ROS namespace of the robot.                                                                                          |
| type                                    | string   | ""        | Robot type (e.g. `wheeled`, `tracked`, `legged`, `quadcopter`, `humanoid`; custom values allowed).                       |
| configuration.\<key>                    | string   |           | Robot configuration key/value pairs copied into the announcement's `keys`/`values`. Values stringified.                  |
| visualizations.\<id>.topic              | string   |           | Topic a UI can visualize (required; entry skipped if missing/empty). Relative names resolve against the namespace.       |
| visualizations.\<id>.name               | string   | \<id>     | Human-readable display name. Defaults to `<id>` when absent or empty.                                                    |
| visualizations.\<id>.message_type       | string   | ""        | Message type, e.g. `sensor_msgs/msg/PointCloud2`. Empty lets the consumer resolve it from the ROS graph.                 |
| visualizations.\<id>.kind               | string   | ""        | Semantic hint (`point_cloud`, `map`, `path`, ...; custom allowed).                                                       |
| visualizations.\<id>.group              | string   | ""        | Optional UI grouping label, e.g. `Mapping`, `Navigation`.                                                                |
| visualizations.\<id>.default_visibility | string   | "hidden"  | Default visibility: `hidden` (off until enabled), `when_active` (shown while this robot is active/selected) or `always`. |
| visualizations.\<id>.hints.\<key>       | string   |           | Display hints copied into the visualization's `keys`/`values`, e.g. `colormap: turbo`. Values stringified.               |
| status.status_frequency                 | double   | 0.0       | Publish `RobotStatus` on `robot_status` at this rate (Hz). 0 disables status, battery monitoring and `set_status`.       |
| status.heartbeat_frequency              | double   | 0.0       | Publish `Heartbeat` on `robot_heartbeat` at this rate (Hz). 0 disables the heartbeat.                                    |
| status.battery.aggregation              | string   | "minimum" | Aggregation for `battery_level`: `minimum` (worst pack) or `combined` (capacity-weighted pooled level).                  |
| status.battery.state_topics.\<label>    | string   |           | Topic publishing `sensor_msgs/msg/BatteryState` to aggregate; one entry per battery (`<label>` is free-form).            |
| enable_tf_forwarding                    | bool     | false     | Forward the robot's tf tree to the global tf tree, prefixing frame ids with the robot namespace.                         |
| tf_config.global_frames                 | string[] | []        | Frame ids shared across robots that must not be prefixed when forwarded (e.g. `map`, `world`).                           |
| tf_config.max_rate                      | double   | 30.0      | Max rate (Hz) at which frames without an explicit per-frame rate are forwarded. 0 disables the limit.                    |
| tf_config.frame_configs.\<frame>.rate   | double   |           | Per-frame max forwarding rate (Hz) for child frame `<frame>`. 0 forwards every message.                                  |

`configuration.<key>` holds an open-ended map, so its keys are read straight from the parameter
overrides rather than declared. They therefore do not appear in `ros2 param list` and are read once at
startup. Provide them as a nested YAML map (`configuration: { main_track: "true", ... }`).

`visualizations` is an open-ended map read the same way (not declared, read once at startup). Each
`<id>` describes one topic a UI (e.g. rviz) can auto-add as a display and is copied into the
announcement's `visualizations` array. `topic` is required; an entry without a non-empty `topic` is
skipped with a warning. `name` falls back to `<id>`. Per-`<id>` `hints.<key>` become the
visualization message's parallel `keys`/`values`.

With tf forwarding, forwarded frames are prefixed on the global `/tf` (e.g. `robot1/odom`), but
visualization messages keep their original, un-prefixed `frame_id`s. Visualization topics consumed
globally should therefore publish in frames listed in `tf_config.global_frames` (forwarded
un-prefixed), or the consumer must rewrite the frame ids itself to match the prefixed global tf tree.

To forward every frame (no shared frames), omit `tf_config.global_frames` entirely. Do **not**
set it to an empty list (`global_frames: []`): an empty YAML sequence has no element type, so the
node aborts on startup with `parameter_value_from failed ... No parameter value set`. The same
applies to any other array parameter.

See [config/simple_announcer.example.yaml](config/simple_announcer.example.yaml) for a full example.
