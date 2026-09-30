uint64 timestamp
uint64 active_request_id
uint64 last_request_id
uint8 last_request_result # 0 none, 1 accepted, 2 busy, 3 invalid, 4 payload locked
uint8 phase # See grasp/GraspStateMachine.hpp; separate from arm_control_status.state
uint8 result # 0 none, 1 running, 2 succeeded, 3 canceled, 4 failed
uint8 fault_reason
uint8 failed_phase # Phase where a fault occurred, retained after exit
uint8 payload_state # 0 empty, 1 candidate, 2 clamped, 3 held, 4 lost, 5 unknown
bool completed_success # Historical hold completion; not proof of current retention
bool ownership_locked
bool strict_position_reached
bool bench_near
uint32 target_sequence
float32 phase_elapsed_s
float32 feedback_age
float32 alignment_error
float32 bilateral_fraction
float32 fixed_normal_n
float32 moving_normal_n
float32 vertical_force_n
float32 initial_motion_shift
float32 hold_drift
float32 actual_lift
