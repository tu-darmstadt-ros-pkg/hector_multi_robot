# hector_multi_robot_announcement

Announcement nodes for multi robot use.

- [SimpleMultiRobotAnnouncer](#SimpleMultiRobotAnnouncer)

## `SimpleMultiRobotAnnouncer`

### Subscribed Topics

| Topic                         | Type                   | Description                                                                      |
| ----------------------------- | ---------------------- | -------------------------------------------------------------------------------- |
| `<robot_namespace>/tf`        | tf2_msgs/msg/TFMessage | Robot-local dynamic transforms to forward.                                       |
| `<robot_namespace>/tf_static` | tf2_msgs/msg/TFMessage | Robot-local static transforms to forward.                                        |
| `<robot_namespace>/<topic>`   | (auto-discovered)      | Each `topic_forwarding.topics.<id>` source topic (type resolved from the graph). |

The tf topics are only subscribed when `enable_tf_forwarding` is true and `robot_namespace` is set
(non-root). Each `topic_forwarding.topics.<id>` topic is subscribed when `robot_namespace` is set and
`topic_forwarding.subnamespace` is non-empty (see Topic forwarding). Additionally, when
`status.status_frequency` is > 0, each topic configured under `status.battery.state_topics.<label>` is
subscribed as a `sensor_msgs/msg/BatteryState` and aggregated into `robot_status`.

### Published Topics

| Topic                                      | Type                                          | Description                                                                                                            |
| ------------------------------------------ | --------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------- |
| robot_announcement                         | hector_multi_robot_msgs/msg/RobotAnnouncement | The announcement message for the robot.                                                                                |
| /robot_announcement                        | hector_multi_robot_msgs/msg/RobotAnnouncement | The announcement message for the robot in the global namespace (if node is namespaced).                                |
| robot_status                               | hector_multi_robot_msgs/msg/RobotStatus       | Robot status: code/message and aggregated battery; latched (transient_local). Only when `status.status_frequency` > 0. |
| robot_heartbeat                            | hector_multi_robot_msgs/msg/Heartbeat         | Periodic heartbeat (stamp + sequence number) for liveness. Only when `status.heartbeat_frequency` > 0.                 |
| /tf                                        | tf2_msgs/msg/TFMessage                        | Forwarded dynamic transforms, frame ids prefixed, rate-limited per child frame.                                        |
| /tf_static                                 | tf2_msgs/msg/TFMessage                        | Forwarded static transforms, frame ids prefixed (no rate limit).                                                       |
| `<robot_namespace>/<subnamespace>/<topic>` | (same as source)                              | Forwarded topic with every `frame_id`/`child_frame_id` prefixed. One per `topic_forwarding.topics.<id>`.               |

`/tf` and `/tf_static` are only published when forwarding is active (see Subscribed Topics). They are
published on the global (un-namespaced) topics even when the node runs under the common multi-robot
`/tf:=tf` remap: the node injects identity remaps (`/tf:=/tf`, `/tf_static:=/tf_static`) that out-rank it.

### Services

| Service    | Type                                  | Description                                                                                           |
| ---------- | ------------------------------------- | ----------------------------------------------------------------------------------------------------- |
| set_status | hector_multi_robot_msgs/srv/SetStatus | Set the status code/message reported in `robot_status`. Available when `status.status_frequency` > 0. |

### Parameters

All parameters are read-only.

| Parameter                                  | Type     | Default   | Description                                                                                                                |
| ------------------------------------------ | -------- | --------- | -------------------------------------------------------------------------------------------------------------------------- |
| robot_id                                   | string   |           | The unique id of the robot.                                                                                                |
| robot_name                                 | string   |           | The human-readable name of the robot.                                                                                      |
| robot_namespace                            | string   |           | The ROS namespace of the robot.                                                                                            |
| type                                       | string   | ""        | Robot type (e.g. `wheeled`, `tracked`, `legged`, `quadcopter`, `humanoid`; custom values allowed).                         |
| configuration.\<key>                       | string   |           | Robot configuration key/value pairs copied into the announcement's `keys`/`values`. Values stringified.                    |
| visualizations.\<id>.topic                 | string   |           | Topic a UI can visualize (required; entry skipped if missing/empty). Relative names resolve against the namespace.         |
| visualizations.\<id>.name                  | string   | \<id>     | Human-readable display name. Defaults to `<id>` when absent or empty.                                                      |
| visualizations.\<id>.message_type          | string   | ""        | Message type, e.g. `sensor_msgs/msg/PointCloud2`. Empty lets the consumer resolve it from the ROS graph.                   |
| visualizations.\<id>.kind                  | string   | ""        | Semantic hint (`point_cloud`, `map`, `path`, ...; custom allowed).                                                         |
| visualizations.\<id>.group                 | string   | ""        | Optional UI grouping label, e.g. `Mapping`, `Navigation`.                                                                  |
| visualizations.\<id>.default_visibility    | string   | "hidden"  | Default visibility: `hidden` (off until enabled), `when_active` (shown while this robot is active/selected) or `always`.   |
| visualizations.\<id>.hints.\<key>          | string   |           | Display hints copied into the visualization's `keys`/`values`, e.g. `colormap: turbo`. Values stringified.                 |
| sensors.\<id>.topic                        | string   |           | Topic the sensor value is read from (required; entry skipped if missing/empty). Resolves against the namespace.            |
| sensors.\<id>.name                         | string   | \<id>     | Human-readable display name. Defaults to `<id>` when absent or empty.                                                      |
| sensors.\<id>.message_type                 | string   | ""        | Message type, e.g. `std_msgs/msg/Float64`. Empty lets the consumer resolve it from the ROS graph.                          |
| sensors.\<id>.field                        | string   | ""        | Message member to display, dotted for nested access (e.g. `data`, `status.mode.name`).                                     |
| sensors.\<id>.unit                         | string   | ""        | Display unit, e.g. `ppm`, `μSv/h`.                                                                                         |
| sensors.\<id>.icon                         | string   | ""        | Symbolic icon name a UI maps to its icon set, e.g. `co2`.                                                                  |
| sensors.\<id>.hints.\<key>                 | string   |           | Display/threshold hints copied into the sensor's `keys`/`values` (e.g. `warn_above`, `decimals`, `timeout`). Stringified.  |
| status.status_frequency                    | double   | 0.0       | Publish `RobotStatus` on `robot_status` at this rate (Hz). 0 disables status, battery monitoring and `set_status`.         |
| status.heartbeat_frequency                 | double   | 0.0       | Publish `Heartbeat` on `robot_heartbeat` at this rate (Hz). 0 disables the heartbeat.                                      |
| status.battery.aggregation                 | string   | "minimum" | Aggregation for `battery_level`: `minimum` (worst pack) or `combined` (capacity-weighted pooled level).                    |
| status.battery.state_topics.\<label>       | string   |           | Topic publishing `sensor_msgs/msg/BatteryState` to aggregate; one entry per battery (`<label>` is free-form).              |
| enable_tf_forwarding                       | bool     | false     | Forward the robot's tf tree to the global tf tree, prefixing frame ids with the robot namespace.                           |
| tf_config.global_frames                    | string[] | []        | Frame ids shared across robots that must not be prefixed when forwarded (e.g. `map`, `world`). Reused by topic forwarding. |
| tf_config.max_rate                         | double   | 30.0      | Max rate (Hz) at which frames without an explicit per-frame rate are forwarded. 0 disables the limit.                      |
| tf_config.frame_configs.\<frame>.rate      | double   |           | Per-frame max forwarding rate (Hz) for child frame `<frame>`. 0 forwards every message.                                    |
| topic_forwarding.subnamespace              | string   | "world"   | Subnamespace forwarded topics are republished under (`<ns>/<subnamespace>/<topic>`). Empty disables topic forwarding.      |
| topic_forwarding.topics.\<id>.topic        | string   |           | Source topic to forward, relative to `<robot_namespace>` (required; entry skipped if missing/empty).                       |
| topic_forwarding.topics.\<id>.message_type | string   | ""        | Message type, e.g. `nav_msgs/msg/Path`. Empty auto-discovers it from the ROS graph; a set value pins it.                   |

`configuration.<key>` holds an open-ended map, so its keys are read straight from the parameter
overrides rather than declared. They therefore do not appear in `ros2 param list` and are read once at
startup. Provide them as a nested YAML map (`configuration: { main_track: "true", ... }`).

`visualizations` is an open-ended map read the same way (not declared, read once at startup). Each
`<id>` describes one topic a UI (e.g. rviz) can auto-add as a display and is copied into the
announcement's `visualizations` array. `topic` is required; an entry without a non-empty `topic` is
skipped with a warning. `name` falls back to `<id>`. Per-`<id>` `hints.<key>` become the
visualization message's parallel `keys`/`values`.

`sensors` is an open-ended map read the same way (not declared, read once at startup). Each `<id>`
describes one sensor value a UI can display and is copied into the announcement's `sensors` array;
the `<id>` is used verbatim as the message `id`. `topic` is required; an entry without a non-empty
`topic` is skipped with a warning. `name` falls back to `<id>`, and `field` selects the message member
to display (dotted for nested access). Per-`<id>` `hints.<key>` become the sensor message's parallel
`keys`/`values` (display/threshold tuning the consumer interprets). The announcer only carries these
declarations; it does not subscribe to the sensor topics or read their values.

With tf forwarding, forwarded frames are prefixed on the global `/tf` (e.g. `robot1/odom`), but
visualization messages keep their original, un-prefixed `frame_id`s. Visualization topics consumed
globally should therefore publish in frames listed in `tf_config.global_frames` (forwarded
un-prefixed), or the consumer must rewrite the frame ids itself to match the prefixed global tf tree.

To forward every frame (no shared frames), omit `tf_config.global_frames` entirely. Do **not**
set it to an empty list (`global_frames: []`): an empty YAML sequence has no element type, so the
node aborts on startup with `parameter_value_from failed ... No parameter value set`. The same
applies to any other array parameter.

### Topic forwarding

`topic_forwarding` re-publishes a configured set of the robot's onboard topics into a subnamespace,
rewriting every `frame_id`/`child_frame_id` so it matches the prefixed global tf tree produced by tf
forwarding. Example: `/athena/move_base/path` is forwarded to `/athena/world/move_base/path` with
`header.frame_id` `odom` rewritten to `athena/odom`.

`topics` is an open-ended map read straight from the overrides (like `visualizations`). Each `<id>`
names one forwarded topic; only `topic` is required. The message type is auto-discovered from the ROS
graph, so a topic is only subscribed once a publisher appears (late publishers are picked up
automatically); set `message_type` to pin it explicitly. Frames listed in `tf_config.global_frames`
(shared with tf forwarding) are forwarded un-prefixed; everything else is prefixed with
`<robot_namespace>/`.

Forwarding is presence-driven and stays inert unless the node is namespaced, at least one topic is configured, and
`topic_forwarding.subnamespace` is non-empty. An empty subnamespace is rejected because the output
topic would equal the input topic. Republished topics live under the robot namespace, so unlike tf
forwarding they do not need the global `/tf` remap bypass.

Messages are forwarded via [`ros_babel_fish`](https://github.com/LOEWE-emergenCITY/ros_babel_fish)
which provides full message introspection, so the types do not need to be known in advance.
This targets low-rate topics (paths, maps, plans); there is no per-topic rate limiting, so a
high-rate sensor topic would be forwarded unthrottled at full rate.

See [config/simple_announcer.example.yaml](config/simple_announcer.example.yaml) for a full example.
