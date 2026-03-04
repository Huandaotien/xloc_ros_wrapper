uint8 PENDING     = 0
uint8 ACTIVE      = 1
uint8 PREEMPTED   = 2
uint8 SUCCEEDED   = 3
uint8 ABORTED     = 4
uint8 REJECTED    = 5
uint8 PREEMPTING  = 6
uint8 RECALLING   = 7
uint8 RECALLED    = 8
uint8 LOST        = 9
uint8 PLANNING    = 10	
uint8 CONTROLLING = 11
uint8 CLEARING    = 12 
uint8 PAUSED      = 13

std_msgs/Header header
bool initialized_ok
bool command_call_ok
string latest_command
uint8 status
string message