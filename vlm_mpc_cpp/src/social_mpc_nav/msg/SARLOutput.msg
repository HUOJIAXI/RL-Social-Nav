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

# SARL recommended action (best action from value-based selection)
# World-frame holonomic velocity components
float32 recommended_vx       # World-frame holonomic velocity x (m/s)
float32 recommended_vy       # World-frame holonomic velocity y (m/s)
float32 recommended_speed    # Speed magnitude sqrt(vx^2 + vy^2) (m/s)
float32 recommended_heading  # World-frame heading atan2(vy, vx) (radians)
bool has_recommended_action  # True if action selection was performed

# Metadata
float32 inference_time_ms
bool is_valid
