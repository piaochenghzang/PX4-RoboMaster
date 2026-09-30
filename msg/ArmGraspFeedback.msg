uint64 timestamp
uint64 timestamp_sample
uint64 sample_time_us
uint32 source_sequence
uint32 world_epoch
uint16 profile_id
uint32 target_id
uint8 source_system
uint8 source_component
uint8 SOURCE_SIM_GROUND_TRUTH = 1
uint8 source
uint16 CAP_CONTACT = 1
uint16 CAP_FORCE = 2
uint16 CAP_OBJECT = 4
uint16 CAP_CLEARANCE = 8
uint16 CAP_BENCH = 16
uint16 capabilities
uint32 contact_frames
float32 sample_window_s
float32 fixed_fraction
float32 moving_fraction
float32 bilateral_fraction
float32 support_fraction
float32 other_fraction
float32 support_arm_fraction
float32 fixed_normal_n
float32 moving_normal_n
float32 vertical_force_n
float32[3] object_center_gripper # m, actual gripper_link frame, not FK tool frame
float32 object_height_world # m, simulation ground truth only
float32 body_support_clearance # m, signed world +Y separation for this bench
