#!/usr/bin/env python3
"""
Optimize experiment hyperparameters (sweep search).
"""

import argparse
from pathlib import Path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--outdir", required=True)
    args = ap.parse_args()

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    print(f"[optimize] results will be written under {outdir}")


if __name__ == "__main__":
    main()
