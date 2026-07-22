# Describes a topic a UI (e.g. rviz) can visualize for this robot.

string name           # Human-readable, e.g. "Elevation Map"
string topic          # Relative names resolve against ros_namespace, absolute allowed
string message_type   # e.g. "sensor_msgs/msg/PointCloud2". May be empty; then the
                      # consumer must resolve it from the ROS graph once the topic exists.

# Semantic hint. Custom allowed, but prefer one of the following for interoperability:
# point_cloud | laser_scan | map | path | pose | markers | image
string kind

string group          # Optional UI grouping, e.g. "Mapping", "Navigation"

# Default visibility of this display in a consuming UI:
#   HIDDEN      - off until the user enables it
#   WHEN_ACTIVE - shown while this robot is active/selected in the UI
#   ALWAYS      - shown regardless of which robot is active
uint8 DEFAULT_VISIBILITY_HIDDEN = 0
uint8 DEFAULT_VISIBILITY_WHEN_ACTIVE = 1
uint8 DEFAULT_VISIBILITY_ALWAYS = 2
uint8 default_visibility

# Optional display hints as key value store.
# Generic keys: ("color", "#e91e63"), ("colormap", "turbo"), ("alpha", "0.6"), ("line_width", "0.05")
# Consumer-specific keys namespaced: ("rviz.style", "Points")
# rviz keys name a display property, "/" separates nested properties:
#   ("rviz.Topic/Reliability Policy", "Best Effort"), ("rviz.Filter/Z Min Value", "-2.0")
string[] keys
string[] values
