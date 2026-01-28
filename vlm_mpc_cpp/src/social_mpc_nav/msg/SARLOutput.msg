# SARL attention weights and value function output for MPC integration
# Published at 10 Hz by sarl_bridge_node

builtin_interfaces/Time stamp

# Per-person attention weights (normalized, sum to 1.0)
# Same order as People2D.people array
float32[] attention_weights
string[] person_names

# State value from SARL value network V(s)
float32 state_value

# Robot state used for this computation (for staleness checking)
float32 robot_x
float32 robot_y
float32 robot_yaw

# Metadata
float32 inference_time_ms
bool is_valid
