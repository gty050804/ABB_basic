#!/usr/bin/env python3
"""Run all IRB1300 kinematics/dynamics demos."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

SCRIPTS = [
    "01_forward_kinematics.py",
    "02_inverse_kinematics.py",
    "03_inverse_dynamics.py",
    "04_forward_dynamics.py",
    "05_verify_dynamics.py",
    "06_compare_pybullet.py",
]


def main() -> None:
    root = Path(__file__).resolve().parent
    for name in SCRIPTS:
        path = root / name
        print(f"\n{'=' * 60}\nRunning {name}\n{'=' * 60}")
        subprocess.run([sys.executable, str(path)], check=True)
    print("\nAll demos completed. Results in results/")


if __name__ == "__main__":
    main()
