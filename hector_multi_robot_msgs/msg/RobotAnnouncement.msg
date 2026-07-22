std_msgs/Header header
string id
string name
string ros_namespace
# Type can be custom but if possible stick to one of the following types for better interoperability:
# wheeled | tracked | legged | quadcopter | humanoid
string type

# For each type you can add additional information in key value store such as
# E.g. for a tracked robot:
# ("main_track", "true") if the robot has a main track (may be used for visualization)
# ("flipper_links", '["flipper_fr_link", "flipper_fl_link", "flipper_bl_link", "flipper_br_link"]') as a JSON array
string[] keys
string[] values

# Topics a UI (e.g. rviz) can visualize for this robot.
Visualization[] visualizations
