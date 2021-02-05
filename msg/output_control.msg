uint64 timestamp		 # time since system start (microseconds)
uint64 timestamp_sample	 # the timestamp the data this control response is based on was sampled

uint8 NUM_OUTPUT_CONTROL_GROUPS = 4 # Max number of instances of this topic
uint8 MAX_ACTUATORS = 16  # Max actuators available in a single topic publication

uint8 n_outputs
uint16[16] function       # Actuator function being controlled
float32[16] value         # Value being applied to each function

# Actuator Function List
uint16 FUNCTION_NONE = 0
uint16 FUNCTION_MIXER = 1

# Misc. functions for different vehicle types
uint16 FUNCTION_PARACHUTE         = 10
uint16 FUNCTION_LANDING_GEAR      = 11
uint16 FUNCTION_LANDING_GEAR_DOOR = 12
uint16 FUNCTION_FLAPS             = 13
uint16 FUNCTION_SPOILER           = 14
uint16 FUNCTION_AIRBRAKS          = 15
uint16 FUNCTION_PAYLOAD_RELEASE   = 16

# Up to 8 individual actuators controlled by DO_SET_SERVO
uint16 FUNCTION_MAVLINK_SERVO0 = 100
uint16 FUNCTION_MAVLINK_SERVO1 = 101
uint16 FUNCTION_MAVLINK_SERVO2 = 102
uint16 FUNCTION_MAVLINK_SERVO3 = 103
uint16 FUNCTION_MAVLINK_SERVO4 = 104
uint16 FUNCTION_MAVLINK_SERVO5 = 105
uint16 FUNCTION_MAVLINK_SERVO6 = 106
uint16 FUNCTION_MAVLINK_SERVO7 = 107

# Specific multicopter motors (Map to MultirotorMixer outputs)
#uint16 FUNCTION_MR_MOTOR1 = 111
#uint16 FUNCTION_MR_MOTOR2 = 112
#uint16 FUNCTION_MR_MOTOR3 = 113
#uint16 FUNCTION_MR_MOTOR4 = 114
#uint16 FUNCTION_MR_MOTOR5 = 115
#uint16 FUNCTION_MR_MOTOR6 = 116
#uint16 FUNCTION_MR_MOTOR7 = 117
#uint16 FUNCTION_MR_MOTOR8 = 118

# Specific fixed-wing functions
uint16 FUNCTION_AILERON1  = 200
uint16 FUNCTION_AILERON2  = 201
uint16 FUNCTION_ELEVATOR1 = 202
uint16 FUNCTION_ELEVATOR2 = 203
uint16 FUNCTION_RUDDER1   = 204
uint16 FUNCTION_RUDDER2   = 205
uint16 FUNCTION_ENGINE1   = 206
uint16 FUNCTION_ENGINE2   = 207

uint16 FUNCTION_GIMBAL_ROLL    = 300
uint16 FUNCTION_GIMBAL_PITCH   = 301
uint16 FUNCTION_GIMBAL_YAW     = 302
uint16 FUNCTION_GIMBAL_SHUTTER = 303
uint16 FUNCTION_CAMERA_ZOOM    = 304

# Functions mapping to outputs of the Control Allocation module
uint16 FUNCTION_CA0  = 1000
uint16 FUNCTION_CA1  = 1001
uint16 FUNCTION_CA2  = 1002
uint16 FUNCTION_CA3  = 1003
uint16 FUNCTION_CA4  = 1004
uint16 FUNCTION_CA5  = 1005
uint16 FUNCTION_CA6  = 1006
uint16 FUNCTION_CA7  = 1007
uint16 FUNCTION_CA8  = 1008
uint16 FUNCTION_CA9  = 1009
uint16 FUNCTION_CA10 = 1010
uint16 FUNCTION_CA11 = 1011
uint16 FUNCTION_CA12 = 1012
uint16 FUNCTION_CA13 = 1013
uint16 FUNCTION_CA14 = 1014
uint16 FUNCTION_CA15 = 1015
# Placeholders for future expansion:
# uint16 FUNCTION_CA16 = 1016
# uint16 FUNCTION_CA17 = 1017
# uint16 FUNCTION_CA18 = 1018
# uint16 FUNCTION_CA19 = 1019
# uint16 FUNCTION_CA20 = 1020
# uint16 FUNCTION_CA21 = 1021
# uint16 FUNCTION_CA22 = 1022
# uint16 FUNCTION_CA23 = 1023
# uint16 FUNCTION_CA24 = 1024
# uint16 FUNCTION_CA25 = 1025
# uint16 FUNCTION_CA26 = 1026
# uint16 FUNCTION_CA27 = 1027
# uint16 FUNCTION_CA28 = 1028
# uint16 FUNCTION_CA29 = 1029
# uint16 FUNCTION_CA30 = 1030
# uint16 FUNCTION_CA31 = 1031

# TOPICS output_control output_control_ca output_control_mixer output_control_mavlink output_control_internal
