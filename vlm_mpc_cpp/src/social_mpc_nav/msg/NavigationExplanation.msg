# Combined VLM + SARL interpretability output
# Merges scene-level reasoning (VLM) with person-level attention (SARL)

builtin_interfaces/Time stamp

# Scene context (from VLM)
string scene_type
string crowd_density
string recommended_action
float32 vlm_confidence

# Per-person analysis (from SARL)
string[] person_names
float32[] attention_weights
float32[] person_distances
string[] person_motions

# SARL state value
float32 state_value

# Applied control parameters (after VLM + SARL modulation)
float32 applied_v_max
float32 applied_w_social
float32 sarl_scene_multiplier

# Human-readable combined narrative
string explanation_text
