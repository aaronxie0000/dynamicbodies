#!/usr/bin/env python
"""CLI entry point for actuated pendulum and humanoid robot simulations."""

import os
import sys
import subprocess
import argparse


def main():
    """Run the actuated pendulum simulation using mjpython."""
    # Construct the command to run mjpython with the main module
    parser = argparse.ArgumentParser(description='Run a simulation.')
    parser.add_argument('--pend', action='store_true', help='Run the actuated pendulum simulation.')
    parser.add_argument('--humanoid', action='store_true', help='Run the humanoid robot simulation.')
    args = parser.parse_args()
    
    if args.pend:
        cmd = ["mjpython", "-m", "dynamic_body_sim.actuated_pendulum.main"] + sys.argv[1:]
    elif args.humanoid:
        cmd = ["mjpython", "-m", "dynamic_body_sim.humanoid_robot.main"] + sys.argv[1:]
    else:
        print("No simulation selected. Please specify either --pend or --humanoid.")
        return 1
    
    # Execute the command
    return subprocess.call(cmd)


if __name__ == "__main__":
    sys.exit(main())