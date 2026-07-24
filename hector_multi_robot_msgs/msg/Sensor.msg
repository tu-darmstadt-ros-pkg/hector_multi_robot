# Describes a sensor value a UI can display for this robot.

string id             # Stable unique identifier of the sensor (the config map key). Kept
                      # constant across re-announcements so consumers can track a sensor.
string name           # Human-readable, e.g. "CO2". Falls back to id when empty.
string topic          # Relative names resolve against ros_namespace, absolute allowed
string message_type   # e.g. "std_msgs/msg/Float64". May be empty; then the consumer must
                      # resolve it from the ROS graph once the topic exists.
string field          # Message member to display, dotted for nested access, e.g. "data" or
                      # "deduction_status.mode.name". May be empty; the consumer decides.
string unit           # Display unit, e.g. "ppm", "μSv/h". May be empty.
string icon           # Symbolic icon name a UI maps to its icon set, e.g. "co2". May be empty.

# Optional display/threshold hints as a parallel key/value store. Custom keys allowed, but prefer
# these for interoperability:
#   ("decimals", "1")             number of decimals to display
#   ("timeout", "5.0")            seconds before a stale value is shown as "?"
#   ("warn_above", "10")          warn threshold; also warn_below / danger_above / danger_below
#   ("warn_topic", "co2detected") separate boolean topic that drives the warn state
string[] keys
string[] values
