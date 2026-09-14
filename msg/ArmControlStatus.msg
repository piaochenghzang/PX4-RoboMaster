uint64 timestamp

float32[6] joint_target
float32[6] joint_command

float32[3] ee_target
float32[3] ee_command
float32[3] ee_measured

uint8 IK_SUCCESS = 0
uint8 IK_MAX_ITERATIONS = 1
uint8 IK_STAGNATED = 2
uint8 IK_NUMERICAL_FAILURE = 3

uint8 ik_status
float32 ik_error

bool feedback_valid
bool command_initialized

uint8 STATE_UNINITIALIZED = 0
uint8 STATE_READY = 1
uint8 STATE_ACTIVE = 2
uint8 STATE_FAULT = 3

uint8 state