# hector_multi_robot_msgs

Messages to announce and control multi robot setups.

## Messages

| Type              | Description                                                                                                            |
| ----------------- | ---------------------------------------------------------------------------------------------------------------------- |
| RobotAnnouncement | Announces the robot with its id, name, ros namespace and configuration information in form of key value pairs.         |
| RobotStatus       | Reports the status for a given robot with battery level, status code and a human readable status message if necessary. |
| Sensor            | Describes a sensor value a UI can display for a robot (topic, field, unit, icon and display/threshold hints).          |
| Visualization     | Describes a topic a UI (e.g. rviz) can visualize for a robot, so displays can be auto-added on discovery.              |
