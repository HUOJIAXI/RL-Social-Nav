#!/usr/bin/env python3
"""
Test script to diagnose VLM side preference bias.
Creates test scenarios to check if VLM can correctly choose 'right' preference.
"""

import json

def create_test_scenarios():
    """
    Create test scenarios that should trigger different side preferences.
    """
    scenarios = []

    # Scenario 1: People clearly on the LEFT - should choose RIGHT
    scenarios.append({
        "name": "People on LEFT - should choose RIGHT",
        "description": "Two people standing on the left side of corridor, robot should pass right",
        "robot_state": {
            "linear_velocity": 0.3,
            "angular_velocity": 0.0,
            "goal_direction": "straight ahead",
            "goal_distance": 15.0
        },
        "humans": [
            {
                "id": 1,
                "distance": 4.5,
                "position": "front-left",
                "speed": 0.0,
                "motion": "standing",
                "direction": "stationary"
            },
            {
                "id": 2,
                "distance": 5.2,
                "position": "left side",
                "speed": 0.0,
                "motion": "standing",
                "direction": "stationary"
            }
        ],
        "expected_side_preference": "right",
        "rationale": "People are on the LEFT side, so robot should pass on the RIGHT"
    })

    # Scenario 2: People clearly on the RIGHT - should choose LEFT
    scenarios.append({
        "name": "People on RIGHT - should choose LEFT",
        "description": "Two people standing on the right side of corridor, robot should pass left",
        "robot_state": {
            "linear_velocity": 0.3,
            "angular_velocity": 0.0,
            "goal_direction": "straight ahead",
            "goal_distance": 15.0
        },
        "humans": [
            {
                "id": 1,
                "distance": 4.5,
                "position": "front-right",
                "speed": 0.0,
                "motion": "standing",
                "direction": "stationary"
            },
            {
                "id": 2,
                "distance": 5.2,
                "position": "right side",
                "speed": 0.0,
                "motion": "standing",
                "direction": "stationary"
            }
        ],
        "expected_side_preference": "left",
        "rationale": "People are on the RIGHT side, so robot should pass on the LEFT"
    })

    # Scenario 3: People on BOTH sides - should choose NEUTRAL or based on clearance
    scenarios.append({
        "name": "People on BOTH sides - should choose NEUTRAL",
        "description": "People distributed on both sides equally",
        "robot_state": {
            "linear_velocity": 0.3,
            "angular_velocity": 0.0,
            "goal_direction": "straight ahead",
            "goal_distance": 15.0
        },
        "humans": [
            {
                "id": 1,
                "distance": 4.5,
                "position": "front-left",
                "speed": 0.0,
                "motion": "standing",
                "direction": "stationary"
            },
            {
                "id": 2,
                "distance": 4.8,
                "position": "front-right",
                "speed": 0.0,
                "motion": "standing",
                "direction": "stationary"
            }
        ],
        "expected_side_preference": "neutral",
        "rationale": "People on both sides equally, symmetric scene"
    })

    # Scenario 4: Cluster on LEFT, single person on RIGHT - should strongly choose RIGHT
    scenarios.append({
        "name": "Cluster LEFT, single RIGHT - should choose RIGHT",
        "description": "Large group on left, one person on right, robot should pass right to avoid cluster",
        "robot_state": {
            "linear_velocity": 0.3,
            "angular_velocity": 0.0,
            "goal_direction": "straight ahead",
            "goal_distance": 12.0
        },
        "humans": [
            {
                "id": 1,
                "distance": 3.5,
                "position": "left side",
                "speed": 0.0,
                "motion": "standing",
                "direction": "stationary"
            },
            {
                "id": 2,
                "distance": 3.8,
                "position": "front-left",
                "speed": 0.0,
                "motion": "standing",
                "direction": "stationary"
            },
            {
                "id": 3,
                "distance": 4.1,
                "position": "front-left",
                "speed": 0.0,
                "motion": "standing",
                "direction": "stationary"
            },
            {
                "id": 4,
                "distance": 6.0,
                "position": "front-right",
                "speed": 0.0,
                "motion": "standing",
                "direction": "stationary"
            }
        ],
        "clusters": [
            {
                "id": 0,
                "size": 3,
                "centroid": {"distance": 3.8, "position": "left"},
                "members": [1, 2, 3]
            },
            {
                "id": 1,
                "size": 1,
                "centroid": {"distance": 6.0, "position": "right"},
                "members": [4]
            }
        ],
        "expected_side_preference": "right",
        "rationale": "Large cluster on LEFT (3 people), single person far on RIGHT - should strongly prefer RIGHT"
    })

    # Scenario 5: Person approaching from LEFT - should choose RIGHT
    scenarios.append({
        "name": "Person approaching from LEFT - should choose RIGHT",
        "description": "One person on left approaching robot, should pass right",
        "robot_state": {
            "linear_velocity": 0.4,
            "angular_velocity": 0.0,
            "goal_direction": "straight ahead",
            "goal_distance": 18.0
        },
        "humans": [
            {
                "id": 1,
                "distance": 3.0,
                "position": "front-left",
                "speed": 1.2,
                "motion": "walking",
                "direction": "approaching robot"
            }
        ],
        "expected_side_preference": "right",
        "rationale": "Person is on LEFT and APPROACHING, robot should pass on RIGHT to avoid collision"
    })

    return scenarios


def format_test_prompt(scenario):
    """
    Format a test scenario into a prompt similar to the VLM system.
    """
    prompt = f"""You are a social navigation advisor for a mobile robot.

### Test Scenario: {scenario['name']}
{scenario['description']}

### Robot state
- Current linear velocity: {scenario['robot_state']['linear_velocity']} m/s
- Current angular velocity: {scenario['robot_state']['angular_velocity']} rad/s
- Goal direction: {scenario['robot_state']['goal_direction']}
- Goal distance: {scenario['robot_state']['goal_distance']} m

### Detected humans
"""

    for i, human in enumerate(scenario['humans'], 1):
        prompt += f"""Human {i}:
  - Distance: {human['distance']} m ({human['position']})
  - Speed: {human['speed']} m/s ({human['motion']})
  - Direction: {human['direction']}
"""

    if 'clusters' in scenario:
        prompt += "\n### Human clusters\n"
        for cluster in scenario['clusters']:
            prompt += f"""Cluster {cluster['id']}:
  - Size: {cluster['size']} people
  - Centroid: {cluster['centroid']['distance']} m ({cluster['centroid']['position']})
  - Members: {cluster['members']}
"""

    prompt += """
### side_preference Guidelines (CRITICAL):
- Use 'left': when people are on the RIGHT side → pass on the left
- Use 'right': when people are on the LEFT side → pass on the right
- Use 'neutral': symmetric scene, people on both sides equally

### Required output format
Respond ONLY with a JSON object using this schema:

{
  "scene_type": "<corridor, lobby, open_space, etc>",
  "crowd_density": "<empty, sparse, medium, dense>",
  "recommended_action": "<go_ahead, slow_down_and_go, etc>",
  "speed_scale": <float 0.0-1.0>,
  "min_personal_distance": <float 0.5-2.0>,
  "side_preference": "<left, right, neutral>",
  "need_to_wait": <true or false>,
  "explanation": "<1-2 sentences explaining side_preference choice>"
}

IMPORTANT: Pay careful attention to which side the people are on!
- If people are on the LEFT → choose "right"
- If people are on the RIGHT → choose "left"
"""

    return prompt


def print_test_report():
    """
    Generate a comprehensive test report.
    """
    scenarios = create_test_scenarios()

    print("="*80)
    print("VLM SIDE PREFERENCE BIAS TEST SCENARIOS")
    print("="*80)
    print("\nThese scenarios should be tested with your VLM to diagnose the left bias.")
    print("The current issue: VLM NEVER chooses 'right', only 'left' or 'neutral'.\n")

    for i, scenario in enumerate(scenarios, 1):
        print(f"\n{'='*80}")
        print(f"SCENARIO {i}: {scenario['name']}")
        print(f"{'='*80}")
        print(f"\nDescription: {scenario['description']}")
        print(f"\nExpected side_preference: {scenario['expected_side_preference']}")
        print(f"Rationale: {scenario['rationale']}")

        print(f"\n--- Test Prompt ---")
        print(format_test_prompt(scenario))
        print("\n--- End Prompt ---\n")

    print("\n" + "="*80)
    print("HOW TO USE THESE SCENARIOS:")
    print("="*80)
    print("""
1. Send each scenario's prompt to your VLM
2. Check if the VLM returns the expected side_preference
3. Pay special attention to scenarios 1, 4, and 5 (should return 'right')

DIAGNOSTIC QUESTIONS:
- Does the VLM ever return side_preference: "right"?
- Does it correctly interpret "people on LEFT → go right"?
- Does it show reasoning in explanations matching the choice?

EXPECTED RESULTS IF NO BIAS:
- Scenario 1: side_preference = "right" (people on left)
- Scenario 2: side_preference = "left" (people on right)
- Scenario 3: side_preference = "neutral" (people on both sides)
- Scenario 4: side_preference = "right" (cluster on left)
- Scenario 5: side_preference = "right" (person approaching from left)

If VLM fails to return "right" for scenarios 1, 4, or 5, the LEFT BIAS is confirmed.
    """)


def save_scenarios_json():
    """
    Save test scenarios to JSON file for automated testing.
    """
    scenarios = create_test_scenarios()

    output_file = "vlm_side_bias_test_scenarios.json"
    with open(output_file, 'w') as f:
        json.dump(scenarios, f, indent=2)

    print(f"\n✓ Test scenarios saved to: {output_file}")
    print("  You can use this file for automated VLM testing.")


if __name__ == '__main__':
    print_test_report()
    save_scenarios_json()
