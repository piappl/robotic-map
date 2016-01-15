# RobotInformation.msg
# Message format for informing the OCU
# about state of a particular robot

# robot type
# e.g. "melex"
string robot_type

# robot id
# e.g. "12345"
uint32 robot_id

# name of the robot
# e.g. "frankie"
string robot_name

# short description
# e.g. "An autonomous melex"
string description

# robot localization types
uint8 TYPE_NO_POSITION=0    # I have no idea where I am! Won't display on the map 
uint8 TYPE_LOCAL_RELATIVE=1 # odometry, relative to previous robot location
uint8 TYPE_LOCAL_ABSOLUTE=2 # robot location on its local map either static or created through SLAM
uint8 TYPE_GLOBAL=3         # GPS

# type of robot localization
# contained in this message
uint8 localization_type

# robot position info
float64 x
float64 y
float64 theta
