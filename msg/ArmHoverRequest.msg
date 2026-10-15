uint64 timestamp
uint64 request_id
uint64 task_id
# This stage only has PREPARE. It never moves the arm or closes the gripper.
uint8 PREPARE = 1
uint8 action
