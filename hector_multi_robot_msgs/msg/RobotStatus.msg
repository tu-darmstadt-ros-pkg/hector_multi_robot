builtin_interfaces/Time timestamp
string robot_id
# Battery level in percent from 0 to 100, use -1 to indicate unknown
int8 battery_level
# Battery voltage in 10mV, e.g. 2540 for 25.4V
int16 battery_voltage
# Robot status
uint8 status_code
# Additional status message for human-readable information
string status_message


uint8 STATUS_OK = 0
uint8 STATUS_INFO = 40
uint8 STATUS_WARNING = 128
uint8 STATUS_ERROR = 200
uint8 STATUS_ESTOP = 220
