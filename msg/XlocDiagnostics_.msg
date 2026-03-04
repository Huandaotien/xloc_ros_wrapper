uint8 MAPPING = 0
uint8 LOCALIZATION = 1
uint8 CALIBRATION = 2
uint8 READY = 3
uint8 ERROR = 4

std_msgs/Header header
uint8 SlamState
string SlamStateDetail
string CurrentActiveMap
uint8 LocalizationQuality