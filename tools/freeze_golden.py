#!/usr/bin/env python3
"""
Freeze best experiment run metrics into golden.json.
"""

import csv
import json
from pathlib import Path


def main():
    best = None
    golden = None

    for p in Path("out/experiments").glob("*/summary.csv"):
        with open(p) as f:
            for row in csv.DictReader(f):
                try:
                    auc = float(row.get("auc", "nan"))
                except Exception:
                    continue
                if best is None or auc > best[0]:
                    best = (auc, row)
                    golden = row

    if golden is not None:
        with open("out/golden.json", "w") as f:
            json.dump(golden, f, indent=2)
        print("[freeze] wrote out/golden.json")


if __name__ == "__main__":
    main()
