# VLM Side Preference Bias - Analysis and Fixes

## Problem Summary

**CONFIRMED: Severe LEFT bias in VLM outputs**

- **0%** "right" side preferences (0 out of 62 decisions)
- **87.1%** "left" side preferences (54 out of 62)
- **12.9%** "neutral" preferences (8 out of 62)

The VLM **never** recommends passing on the right side, even when people are clearly on the left.

## Root Cause Analysis

### 1. Training Data Bias
VLMs are often trained on data with cultural biases:
- **Left-hand traffic** regions (US, Europe, etc.): People naturally walk on the right, pass on the left
- **Right-hand traffic** regions (UK, Japan, etc.): Opposite convention
- The training data likely has more examples of left-passing behavior

### 2. Prompt Ambiguity
Current prompt line (vlm_integration_node.cpp:1405-1406):
```
- Use 'left': when people are on the RIGHT side → pass on the left
- Use 'right': when people are on the LEFT side → pass on the right
```

This is correct but may not be emphatic enough to override the model's inherent bias.

### 3. Lack of Few-Shot Examples
The prompt doesn't include concrete examples showing successful "right" choices.

## Recommended Fixes

### Fix 1: Add Emphatic Instructions at Multiple Locations

**Location**: `vlm_integration_node.cpp` lines ~1403-1410

**Current Code**:
```cpp
prompt << "**side_preference** (left, right, neutral):\n";
prompt << "Which side of the path to goal the robot should bias toward (lateral offset from centerline).\n";
prompt << "- Use 'left': when people are on the RIGHT side → pass on the left\n";
prompt << "- Use 'right': when people are on the LEFT side → pass on the right\n";
prompt << "- Use 'neutral': symmetric scene, people on both sides equally, or open space with no people\n";
```

**Proposed Fix**:
```cpp
prompt << "**side_preference** (left, right, neutral) - CRITICAL PARAMETER:\n";
prompt << "Which side of the path to goal the robot should bias toward (lateral offset from centerline).\n";
prompt << "IMPORTANT: Both 'left' and 'right' are EQUALLY VALID choices. There is NO preferred direction!\n";
prompt << "\n";
prompt << "Decision rules (MUST FOLLOW EXACTLY):\n";
prompt << "  1. People on LEFT side  → side_preference = 'right' (pass on RIGHT to avoid them)\n";
prompt << "  2. People on RIGHT side → side_preference = 'left'  (pass on LEFT to avoid them)\n";
prompt << "  3. People on BOTH sides → side_preference = 'neutral' (symmetric scene)\n";
prompt << "  4. No people or open space → side_preference = 'neutral'\n";
prompt << "\n";
prompt << "SYMMETRY CHECK: Make sure you choose 'right' about 50% of the time when people are asymmetrically distributed!\n";
prompt << "If you find yourself never choosing 'right', you are making a mistake.\n";
```

### Fix 2: Add Explicit Few-Shot Examples

**Location**: After line ~1458 (before "Required JSON output format")

**Add This Section**:
```cpp
prompt << "### Few-Shot Examples (Learn from these)\n\n";

prompt << "Example 1: People on RIGHT side\n";
prompt << "Scene: Two people at 4m on front-right, one at 5m on right side\n";
prompt << "Correct answer:\n";
prompt << "{\n";
prompt << "  \"side_preference\": \"left\",\n";
prompt << "  \"explanation\": \"People are on the RIGHT side, so robot passes on the LEFT.\"\n";
prompt << "}\n\n";

prompt << "Example 2: People on LEFT side\n";
prompt << "Scene: Two people at 4m on front-left, one at 5m on left side\n";
prompt << "Correct answer:\n";
prompt << "{\n";
prompt << "  \"side_preference\": \"right\",\n";
prompt << "  \"explanation\": \"People are on the LEFT side, so robot passes on the RIGHT.\"\n";
prompt << "}\n\n";

prompt << "Example 3: Cluster on LEFT, single person on RIGHT\n";
prompt << "Scene: Three people clustered at 3-4m on left side, one person at 6m on right\n";
prompt << "Correct answer:\n";
prompt << "{\n";
prompt << "  \"side_preference\": \"right\",\n";
prompt << "  \"explanation\": \"Large cluster on LEFT (3 people) and only one person far on RIGHT, so robot passes on the RIGHT to avoid the cluster.\"\n";
prompt << "}\n\n";

prompt << "Example 4: People on both sides equally\n";
prompt << "Scene: One person at 4m front-left, one person at 4m front-right\n";
prompt << "Correct answer:\n";
prompt << "{\n";
prompt << "  \"side_preference\": \"neutral\",\n";
prompt << "  \"explanation\": \"People distributed symmetrically on both sides, so neutral preference.\"\n";
prompt << "}\n\n";

prompt << "Now analyze the current scene and make YOUR decision:\n\n";
```

### Fix 3: Add Post-Decision Validation

**Location**: `vlm_translator_node.cpp` in `parseAndValidateVLMResponse()` function

**Add a heuristic check**:
```cpp
// After parsing side_preference (around line 394)
params.side_preference = SimpleJSONParser::extractString(json_response, "side_preference");

// Add validation based on explanation
std::string explanation_lower = params.explanation;
std::transform(explanation_lower.begin(), explanation_lower.end(),
               explanation_lower.begin(), ::tolower);

// Check for contradiction
bool mentions_left_side =
    explanation_lower.find("left side") != std::string::npos ||
    explanation_lower.find("on the left") != std::string::npos;

bool mentions_right_side =
    explanation_lower.find("right side") != std::string::npos ||
    explanation_lower.find("on the right") != std::string::npos;

// If mentions "left side" but chose "left", that's likely wrong
if (mentions_left_side && !mentions_right_side && params.side_preference == "left") {
    RCLCPP_WARN(this->get_logger(),
        "Potential VLM bias: chose 'left' but mentions 'left side' in explanation. "
        "This might be incorrect. Consider overriding to 'right'.");
    // Optional: Force correction
    // params.side_preference = "right";
}

// Similarly for right side
if (mentions_right_side && !mentions_left_side && params.side_preference == "right") {
    RCLCPP_WARN(this->get_logger(),
        "Potential VLM bias: chose 'right' but mentions 'right side' in explanation.");
    // params.side_preference = "left";
}
```

### Fix 4: Use Chain-of-Thought Reasoning

**Location**: In the prompt, before JSON output format

**Add This**:
```cpp
prompt << "### Decision Process (think step-by-step):\n\n";
prompt << "Before giving your answer, think through:\n";
prompt << "Step 1: List which side each person/cluster is on (left, right, or center)\n";
prompt << "Step 2: Count people on left side vs right side\n";
prompt << "Step 3: Determine which side has MORE clearance\n";
prompt << "Step 4: Choose side_preference as the side WITH MORE CLEARANCE\n";
prompt << "Step 5: Double-check: Does your choice match the rule?\n";
prompt << "        - People on left → choose 'right'\n";
prompt << "        - People on right → choose 'left'\n\n";
```

Then modify the JSON format to include reasoning:
```cpp
prompt << "{\n";
prompt << "  \"reasoning\": {\n";
prompt << "    \"people_on_left\": <count>,\n";
prompt << "    \"people_on_right\": <count>,\n";
prompt << "    \"clearance_side\": \"<left or right>\"\n";
prompt << "  },\n";
prompt << "  \"scene_type\": \"...\",\n";
// ... rest of JSON
```

### Fix 5: Add a "Right Preference Encouragement" Section

**Location**: Right before the required output format

**Add**:
```cpp
prompt << "### CRITICAL REMINDER:\n";
prompt << "Statistical balance check: In a balanced dataset, you should choose:\n";
prompt << "  - 'left': ~40-50% of the time (when people are on RIGHT)\n";
prompt << "  - 'right': ~40-50% of the time (when people are on LEFT)\n";
prompt << "  - 'neutral': ~10-20% of the time (symmetric or empty)\n";
prompt << "\n";
prompt << "If you notice you are NEVER choosing 'right', you are making an error!\n";
prompt << "Both directions are equally valid and important for safe navigation.\n\n";
```

## Implementation Priority

1. **High Priority**: Fix 1 (Emphatic instructions) + Fix 2 (Few-shot examples)
   - These address the immediate bias issue
   - Can be implemented quickly

2. **Medium Priority**: Fix 5 (Balance encouragement)
   - Helps with statistical awareness
   - Easy to add

3. **Low Priority**: Fix 3 (Post-validation) + Fix 4 (Chain-of-thought)
   - More complex to implement
   - Consider if Fixes 1-2 don't work

## Testing Plan

After implementing fixes:

1. **Run test scenarios** (use `test_vlm_side_preference.py`):
   - Test "people on left" scenarios
   - Verify VLM now outputs "right"

2. **Monitor live runs**:
   ```bash
   # Watch for "right" preferences in real-time
   ros2 topic echo /vlm/mpc_parameters | grep -A5 "side_preference: right"
   ```

3. **Analyze new logs**:
   ```bash
   python3 scripts/analyze_vlm_side_bias.py
   ```
   - Target: At least 30-40% "right" preferences in balanced scenarios

4. **Check explanations**:
   - Verify VLM reasoning matches the choice
   - Look for "people on left → go right" pattern

## Expected Outcome

After fixes:
- **35-45%** "right" preferences (in scenarios with people on left)
- **35-45%** "left" preferences (in scenarios with people on right)
- **10-20%** "neutral" preferences (symmetric or empty)
- **0%** contradictions between explanation and choice

## Additional Notes

### Alternative: Model Temperature

Try increasing temperature parameter in VLM API call to get more diverse outputs:
```cpp
// In vlm_integration_node.cpp, API call section
"temperature": 0.7,  // Increase from default 0.0
```

### Alternative: Different Model

Some VLMs have less bias:
- Try GPT-4V (vision) with higher temperature
- Try Claude 3 Opus (less prone to directional bias in testing)
- Try Gemini Pro Vision

### Monitoring Script

Create a simple monitor to track preference distribution:
```bash
#!/bin/bash
# scripts/monitor_vlm_bias.sh
watch -n 5 "tail -20 ~/ros2_logs/social_mpc_nav/vlm_translation.csv | cut -d',' -f10 | sort | uniq -c"
```

This will show real-time counts of left/right/neutral choices.
