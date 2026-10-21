# Local SITL geometry input; not a permanent motion/closing permission.
uint64 timestamp
uint64 task_id
uint64 sample_time_us
uint32 source_sequence
uint32 source_epoch
bool valid
float32 body_speed
float32 body_angular_speed
float32 tilt
float32 hover_position_error
float32 alignment_error
float32 relative_speed
float32 orientation_error
# Optional no-close task contract. Zero fields never authorize an approach.
uint64 plan_id
uint64 target_id
uint64 scene_id
uint64 lease_sample_us
bool lease_valid
bool geometry_valid
float32[3] approach_base
float32 clearance
# Explicit opt-in fixed-point fine grasp; false keeps the old no-close contract.
bool fixed_grasp
float32[3] lift_base # same task's fixed world tool target + 30mm world up, in live arm frame
float32 jaw_clearance # independent full jaw-stroke pair-specific slack +16mm
