# VLM-derived parameters for MPC control
builtin_interfaces/Time stamp

# Metadata
string source                    # "vlm" or "fallback_cached" or "fallback_rules"
float32 confidence              # 0.0-1.0
float32 age_sec                 # Age of this data

# Scene understanding
string scene_type               # corridor, lobby, open_space, doorway, queue, crossing, unknown
string crowd_density            # empty, sparse, medium, dense
string recommended_action       # go_ahead, slow_down_and_go, stop_and_wait, keep_left, keep_right, yield_to_pedestrian

# Control parameters
float32 speed_scale             # 0.0-1.0, multiplier for v_max
float32 min_personal_distance   # 0.5-2.0 meters
string side_preference          # left, right, neutral
bool need_to_wait              # Emergency stop flag
float32 adaptive_min_obstacle_distance  # 0.25-0.5 meters, VLM-adaptive soft obstacle threshold

# Explanation
string explanation
