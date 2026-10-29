uint64 timestamp
uint64 task_id
uint64 plan_id
uint64 target_id
uint64 scene_id
uint64 last_request_id
uint8 MODE_APPROACH = 0
uint8 MODE_FIXED_GRASP = 1
uint8 mode # Decode phase/fault using this mode; one owner publishes this topic.
uint8 phase # approach: 0 idle,1 approach,2 hold,3 paused,4 fault,5 canceled; fixed: 0 idle,1 approach,2 align,3 close,4 verify,5 lift,6 hold,7 done,8 fault,9 canceled
uint8 fault
uint8 pause_reason # 0 none, 1 hover condition, 2 pending joint displacement / clearance
uint8 request_result # 0 none, 1 accepted, 3 rejected
bool ownership_locked
bool motion_authorized
bool close_authorized # always false in MODE_APPROACH
bool lease_valid
bool resume_ready # still requires an explicit same-task RESUME request
float32[3] goal_base
float32 goal_error
float32 clearance
float32 jaw_target
# Shared unfold progress; remaining grasp diagnostics stay false/zero in MODE_APPROACH.
bool paused
bool unfold_complete
bool close_entry_ready
bool completed_success
float32 actual_lift
float32 feedback_age
float32 bilateral_fraction
float32 fixed_normal_n
float32 moving_normal_n
