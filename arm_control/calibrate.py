"""Dry-run entry point without ROS: python calibrate.py. See arm_control/calibrate.py for the commands."""
import sys
from pathlib import Path

here = Path(__file__).resolve().parent
sys.path.insert(0, str(here))
sys.path.insert(0, str(here.parent / "stm_bridge"))     # servo_frame, for the dry-run output
from arm_control.calibrate import main

if __name__ == "__main__":
    main()
