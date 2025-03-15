#!/usr/bin/env mjpython
"""CLI entry point for actuated pendulum and humanoid robot simulations."""

import sys
import subprocess
import argparse

def main():
    """Run the actuated pendulum or humanoid robot simulation using mjpython."""
    parser = argparse.ArgumentParser(description='Run a simulation.')
    parser.add_argument('--pend', action='store_true', help='Run the actuated pendulum simulation.')
    parser.add_argument('--humanoid', action='store_true', help='Run the humanoid robot simulation.')
    args = parser.parse_args()

    if args.pend:
        cmd = ["mjpython", "-m", "dynamic_body_sim.actuated_pendulum.main"]
    elif args.humanoid:
        cmd = ["mjpython", "-m", "dynamic_body_sim.humanoid_robot.main"]
    else:
        print("No simulation selected. Please specify either --pend or --humanoid.", file=sys.stderr)
        return 1

    # Execute the command and return the exit code
    result = subprocess.run(cmd, check=False)
    return result.returncode

if __name__ == "__main__":
    sys.exit(main())
