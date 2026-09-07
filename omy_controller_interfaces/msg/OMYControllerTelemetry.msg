builtin_interfaces/Time stamp
uint64 controller_seq
uint64 publish_missed_total
float64 period_s
float64 update_compute_s
bool collision_active

bool external_wrench_received
bool external_wrench_fresh
bool external_wrench_has_stamp
float64 external_wrench_received_ros_time_s
float64 external_wrench_msg_stamp_ros_time_s
float64 external_wrench_receive_age_s
float64 external_wrench_stamp_age_s

float64[] q
float64[] qdot_measured
float64[] qdot_kdl
float64[] qddot_kdl

float64[6] wrench_received
float64[6] wrench_used
float64[] tau_rne
float64[] tau_reflect
float64[] tau_spring
float64[] tau_sync
float64[] tau_friction
float64[] tau_pre_scale
float64[] tau_cmd
