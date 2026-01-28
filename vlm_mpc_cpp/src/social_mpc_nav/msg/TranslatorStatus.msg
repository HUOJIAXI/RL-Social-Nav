# Status message for the VLM translator node
builtin_interfaces/Time stamp

# VLM health
bool vlm_active
float32 vlm_last_response_age
int32 vlm_responses_received
int32 vlm_parsing_errors

# Fallback state
bool using_fallback
string fallback_reason

# Performance
float32 translation_latency_ms
