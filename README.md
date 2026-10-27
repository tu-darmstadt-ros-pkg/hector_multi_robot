# Hector Multi Robot

This package contains messages and a publisher to announce robots for multi robot setups
to enable dynamic inclusion of robots for example in the user interface.

## Quick Start

1. **Create a config** (`my_robot_announcer.yaml`):

   ```yaml
   /**:
     ros__parameters:
       robot_id: "athena"
       robot_name: "Athena"
       robot_namespace: "/athena"
       type: "tracked"

       enable_tf_forwarding: true   # robot tf -> global /tf, frames prefixed with "athena/"
       tf_config:
         global_frames: [world]     # shared frames, not prefixed (omit key instead of [])

       visualizations:              # what a UI can show for this robot
         path:
           topic: "move_base/plan"
           message_type: "nav_msgs/msg/Path"
           kind: "path"

       topic_forwarding:            # copies with prefixed frames for UIs in the global namespace
         topics:
           path:
             topic: "move_base/plan"  # -> /athena/world/move_base/plan
   ```

2. **Launch it in the robot namespace**:

   ```bash
   ros2 run hector_multi_robot_announcement simple_multi_robot_announcer \
     --ros-args -r __ns:=/athena -r /tf:=tf -r /tf_static:=tf_static \
     --params-file my_robot_announcer.yaml
   ```

   The node still publishes the forwarded tf on the global `/tf` despite the `/tf:=tf` remap.

3. **Check it**:

   ```bash
   ros2 topic echo /robot_announcement
   ```

### Which topic should a UI use?

| UI runs in        | tf                            | Use topic                         |
| ----------------- | ----------------------------- | --------------------------------- |
| Robot namespace   | `/athena/tf`, frames as-is    | `/athena/move_base/plan`          |
| Global namespace  | `/tf`, frames `athena/...`    | `/athena/world/move_base/plan`    |

Only forward topics that a global UI needs and whose frames are not in `global_frames`. Forwarding
is not rate-limited, so use it for low-rate data (paths, maps, plans).

Visualizations and sensors can also be added at runtime via the `add_visualization`/`add_sensor`
services (not persisted across restarts). See
[hector_multi_robot_announcement/README.md](hector_multi_robot_announcement/README.md) for all
parameters, battery/status reporting and the
[full example config](hector_multi_robot_announcement/config/simple_announcer.example.yaml).

### UI Integration

Robots announced using this package, can be discovered by the [hector_user_interface](https://github.com/tu-darmstadt-ros-pkg/hector_user_interface).
