#!/usr/bin/env mjpython
"""CLI entry point for actuated pendulum and humanoid robot simulations."""

import sys
import subprocess
import argparse

def main():
    """Run the actuated pendulum or humanoid robot simulation using mjpython."""
    parser = argparse.ArgumentParser(description='Run a simulation.')
    parser.add_argument('--body', choices=['pend', 'humanoid'], default='pend', help='Choose the example to run.')
    args = parser.parse_args()

    if args.body == 'pend':
        cmd = ["mjpython", "-m", "dynamic_body_sim.actuated_pendulum.main"]
    elif args.body == 'humanoid':
        cmd = ["mjpython", "-m", "dynamic_body_sim.humanoid_robot.main"]
    else:
        return 1

    # Execute the command and return the exit code
    result = subprocess.run(cmd, check=False)
    return result.returncode

if __name__ == "__main__":
    sys.exit(main())
