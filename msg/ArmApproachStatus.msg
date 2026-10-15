uint64 timestamp
uint64 task_id
uint64 plan_id
uint64 target_id
uint64 scene_id
uint64 last_request_id
uint8 phase # 0 idle, 1 approaching, 2 holding, 3 paused, 4 fault, 5 canceled
uint8 fault
uint8 pause_reason # 0 none, 1 hover condition, 2 pending joint displacement / clearance
uint8 request_result # 0 none, 1 accepted, 3 rejected
bool ownership_locked
bool motion_authorized
bool close_authorized # always false; jaw target is unchanged
bool lease_valid
bool resume_ready # still requires an explicit same-task RESUME request
float32[3] goal_base
float32 goal_error
float32 clearance
float32 jaw_target
