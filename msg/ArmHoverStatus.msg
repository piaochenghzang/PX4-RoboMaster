uint64 timestamp
uint64 task_id
uint64 source_sample_time_us
uint32 source_sequence
uint32 source_epoch
uint16 denial_mask
bool native_heading_ready
uint64 native_reset_key
bool start_ready
bool close_geometry_ready
bool reset_blocked
bool prepared
uint64 last_request_id
uint8 request_result # 0 none, 1 accepted prepare-only, 2 rejected, 3 revoked
bool motion_authorized # always false in this prepare-only stage
bool close_authorized # always false in this prepare-only stage
float32 body_speed
float32 body_angular_speed
float32 tilt
float32 hover_position_error
float32 motor_max
float32 alignment_error
float32 relative_speed
float32 orientation_error
