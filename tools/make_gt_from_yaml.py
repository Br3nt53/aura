#!/usr/bin/env python3
import argparse
import yaml
import json

def generate_gt(scenario_data):
    """Generates ground truth data from a single scenario dictionary."""
    gt_data = []
    sim_time_s = scenario_data.get('sim_time_s', 10)
    rate_hz = 10  # Standard rate for ground truth generation
    
    for t_step in range(int(sim_time_s * rate_hz)):
        time_s = t_step / rate_hz
        frame = t_step
        
        for target in scenario_data.get('targets', []):
            start_time = target.get('start_time_s', 0)
            end_time = target.get('end_time_s', sim_time_s)
            
            if start_time <= time_s <= end_time:
                path = target['path']
                
                # Linear interpolation between path points
                duration = end_time - start_time
                progress = (time_s - start_time) / duration if duration > 0 else 0
                
                # Find which two points to interpolate between
                segment_progress = progress * (len(path) - 1)
                p1_idx = int(segment_progress)
                p2_idx = min(p1_idx + 1, len(path) - 1)
                
                local_progress = segment_progress - p1_idx
                
                p1 = path[p1_idx]
                p2 = path[p2_idx]
                
                x = p1[0] + (p2[0] - p1[0]) * local_progress
                y = p1[1] + (p2[1] - p1[1]) * local_progress
                
                gt_data.append({
                    'frame': frame,
                    'id': target['id'],
                    'x': x,
                    'y': y
                })
                
    return gt_data

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate ground truth JSONL from a scenario YAML.")
    parser.add_argument('--scenario', required=True, help="Path to the scenario YAML file.")
    parser.add_argument('--out', required=True, help="Path to the output JSONL file.")
    # Add the new, optional argument
    parser.add_argument('--test-case', required=False, help="Specify a single test case to run from a scenario file.")
    args = parser.parse_args()

    with open(args.scenario, 'r') as f:
        data = yaml.safe_load(f)

    scenario_to_run = data
    if args.test_case:
        if 'test_cases' in data and args.test_case in data['test_cases']:
            scenario_to_run = data['test_cases'][args.test_case]
        else:
            raise ValueError(f"Test case '{args.test_case}' not found in {args.scenario}")

    gt_data = generate_gt(scenario_to_run)

    with open(args.out, 'w') as f:
        for entry in gt_data:
            f.write(json.dumps(entry) + '\n')

    print(f"Successfully generated ground truth for '{args.test_case or 'main scenario'}' to {args.out}")
