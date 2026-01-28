#!/usr/bin/env python3
"""
Analyze VLM side preference bias from vlm_translation.csv log file.
Checks if the VLM has a systematic preference for left/right recommendations.
"""

import csv
import re
from collections import Counter
from pathlib import Path

def analyze_side_bias(log_path):
    """
    Analyze the VLM's side preference decisions and check for bias.
    """
    # Read the log file
    data = []
    with open(log_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            data.append(row)

    print(f"\n{'='*80}")
    print(f"VLM Side Preference Bias Analysis")
    print(f"{'='*80}")
    print(f"Log file: {log_path}")
    print(f"Total entries: {len(data)}\n")

    # Count side preferences
    side_counts = Counter()
    side_with_people = {'left': [], 'right': [], 'neutral': []}

    # Patterns to detect pedestrian positions in explanations
    left_patterns = [
        r'\bleft\b.*\bpedestrian',
        r'\bleft\b.*\bpeople',
        r'\bleft\b.*\bperson',
        r'\bleft side\b',
        r'\bon the left\b',
        r'\bfront-left\b'
    ]

    right_patterns = [
        r'\bright\b.*\bpedestrian',
        r'\bright\b.*\bpeople',
        r'\bright\b.*\bperson',
        r'\bright side\b',
        r'\bon the right\b',
        r'\bfront-right\b'
    ]

    for row in data:
        side_pref = row['side_preference']
        explanation = row['explanation'].lower()
        crowd_density = row['crowd_density']

        side_counts[side_pref] += 1

        # Check where pedestrians are mentioned in explanation
        has_left = any(re.search(p, explanation) for p in left_patterns)
        has_right = any(re.search(p, explanation) for p in right_patterns)

        side_with_people[side_pref].append({
            'explanation': row['explanation'],
            'crowd_density': crowd_density,
            'has_left_mention': has_left,
            'has_right_mention': has_right,
            'timestamp': row['stamp_sec']
        })

    # Print overall statistics
    print(f"Side Preference Distribution:")
    print(f"  Left:    {side_counts['left']:3d} ({side_counts['left']/len(data)*100:.1f}%)")
    print(f"  Right:   {side_counts['right']:3d} ({side_counts['right']/len(data)*100:.1f}%)")
    print(f"  Neutral: {side_counts['neutral']:3d} ({side_counts['neutral']/len(data)*100:.1f}%)")

    # Analyze left preferences
    print(f"\n{'='*80}")
    print(f"Analysis of 'LEFT' preferences ({side_counts['left']} entries):")
    print(f"{'='*80}")

    left_with_right_mention = sum(1 for e in side_with_people['left'] if e['has_right_mention'])
    left_with_left_mention = sum(1 for e in side_with_people['left'] if e['has_left_mention'])
    left_with_neither = sum(1 for e in side_with_people['left']
                           if not e['has_right_mention'] and not e['has_left_mention'])

    print(f"  Mentions 'right' (people on right → go left = CORRECT):  {left_with_right_mention:3d} ({left_with_right_mention/max(side_counts['left'],1)*100:.1f}%)")
    print(f"  Mentions 'left'  (people on left → go left = INCORRECT): {left_with_left_mention:3d} ({left_with_left_mention/max(side_counts['left'],1)*100:.1f}%)")
    print(f"  Neither mentioned:                                       {left_with_neither:3d} ({left_with_neither/max(side_counts['left'],1)*100:.1f}%)")

    # Show some examples where LEFT preference might be wrong
    if left_with_left_mention > 0:
        print(f"\n⚠️  POTENTIAL ISSUES: 'LEFT' preference with 'left' mentioned in explanation:")
        for i, entry in enumerate([e for e in side_with_people['left'] if e['has_left_mention']][:5]):
            print(f"\n  Example {i+1}:")
            print(f"    Explanation: {entry['explanation']}")
            print(f"    Crowd: {entry['crowd_density']}")

    # Analyze right preferences
    print(f"\n{'='*80}")
    print(f"Analysis of 'RIGHT' preferences ({side_counts['right']} entries):")
    print(f"{'='*80}")

    if side_counts['right'] == 0:
        print("  ⚠️  WARNING: NO 'RIGHT' PREFERENCES FOUND!")
        print("  This suggests a strong LEFT BIAS in the VLM outputs.")
        print("  The VLM never recommends going right, even when people are on the left.")
    else:
        right_with_left_mention = sum(1 for e in side_with_people['right'] if e['has_left_mention'])
        right_with_right_mention = sum(1 for e in side_with_people['right'] if e['has_right_mention'])
        right_with_neither = sum(1 for e in side_with_people['right']
                                if not e['has_left_mention'] and not e['has_right_mention'])

        print(f"  Mentions 'left'  (people on left → go right = CORRECT):   {right_with_left_mention:3d} ({right_with_left_mention/max(side_counts['right'],1)*100:.1f}%)")
        print(f"  Mentions 'right' (people on right → go right = INCORRECT): {right_with_right_mention:3d} ({right_with_right_mention/max(side_counts['right'],1)*100:.1f}%)")
        print(f"  Neither mentioned:                                         {right_with_neither:3d} ({right_with_neither/max(side_counts['right'],1)*100:.1f}%)")

    # Analyze neutral preferences
    print(f"\n{'='*80}")
    print(f"Analysis of 'NEUTRAL' preferences ({side_counts['neutral']} entries):")
    print(f"{'='*80}")

    neutral_with_people = sum(1 for e in side_with_people['neutral']
                             if e['has_left_mention'] or e['has_right_mention'])
    neutral_empty = sum(1 for e in side_with_people['neutral'] if e['crowd_density'] == 'empty')

    print(f"  Empty crowd (neutral is correct):          {neutral_empty:3d} ({neutral_empty/max(side_counts['neutral'],1)*100:.1f}%)")
    print(f"  Mentions people but still neutral:         {neutral_with_people:3d} ({neutral_with_people/max(side_counts['neutral'],1)*100:.1f}%)")

    # Summary and recommendations
    print(f"\n{'='*80}")
    print(f"Summary and Recommendations:")
    print(f"{'='*80}")

    if side_counts['right'] == 0:
        print("❌ CONFIRMED: Strong LEFT BIAS detected!")
        print("\nThe VLM shows a systematic bias:")
        print("  • Never recommends 'right' side preference")
        print("  • Always chooses 'left' or 'neutral'")
        print("  • This occurs even when people should be on the left")
        print("\nPossible causes:")
        print("  1. Training data bias (more left-passing examples)")
        print("  2. Cultural/regional driving conventions in training data")
        print("  3. Prompt interpretation issues")
        print("  4. Model reasoning bias toward left side")
        print("\nRecommendations:")
        print("  1. Add explicit examples in prompt showing 'right' preference scenarios")
        print("  2. Emphasize symmetry: 'left' and 'right' are equally valid")
        print("  3. Test with scenarios explicitly stating 'people on LEFT side'")
        print("  4. Consider adding few-shot examples with balanced left/right decisions")
    elif side_counts['right'] < side_counts['left'] / 3:
        print("⚠️  MODERATE LEFT BIAS detected")
        print(f"  'left' recommendations are {side_counts['left']/max(side_counts['right'],1):.1f}x more common than 'right'")
    else:
        print("✓ No significant bias detected")

    # Show some example explanations
    print(f"\n{'='*80}")
    print(f"Sample Explanations with 'LEFT' preference:")
    print(f"{'='*80}")
    for i, entry in enumerate(side_with_people['left'][:5]):
        print(f"\n{i+1}. {entry['explanation']}")
        print(f"   Crowd: {entry['crowd_density']}, Has right mention: {entry['has_right_mention']}")


if __name__ == '__main__':
    # Default log file location
    log_file = Path.home() / 'ros2_logs' / 'social_mpc_nav' / 'vlm_translation.csv'

    if not log_file.exists():
        print(f"Error: Log file not found at {log_file}")
        print("Please provide the correct path to vlm_translation.csv")
        exit(1)

    analyze_side_bias(log_file)
