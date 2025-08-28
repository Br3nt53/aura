#!/usr/bin/env python3
import argparse, subprocess, time, json, os, signal, sys, yaml
from pathlib import Path

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", required=True)
    ap.add_argument("--workdir", required=True)
    ap.add_argument("--ros_pkg", default="aura_examples")
    ap.add_argument("--launch", default="bringup.launch.py")
    ap.add_argument("--buffer_sec", type=float, default=1.0, help="Extra time after scenario ends")
    args = ap.parse_args()

    # Read duration from scenario to know how long to run
    data = yaml.safe_load(open(args.scenario, "r"))
    duration = float(data.get("sim", {}).get("duration_sec", 10.0))

    cmd = [
        "ros2", "launch", args.ros_pkg, args.launch,
        f"scenario:={args.scenario}",
        f"workdir:={args.workdir}"
    ]
    print("[pipeline] starting:", " ".join(cmd))
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

    t_end = time.time() + duration + args.buffer_sec
    try:
        while time.time() < t_end:
            line = proc.stdout.readline()
            if not line:
                time.sleep(0.1)
                continue
            sys.stdout.write(line)
            sys.stdout.flush()
    finally:
        # Try to terminate cleanly
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()

    print("[pipeline] finished. Predictions should be in", os.path.join(args.workdir, "pred.jsonl"))

if __name__ == "__main__":
    main()
