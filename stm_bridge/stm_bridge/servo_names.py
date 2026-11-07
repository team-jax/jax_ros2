"""Servo name -> CAN_STM motor_id (Stm32pinmap.md section 3, plan B, 2026-10-14).

/servo/command names these; arm_control's calibration.json carries the same names in `name`.
tests/check_protocol.py compares this table with the calibration.json of arm_control.
"""
SERVO_IDS = {
    "F1_1": 0, "F1_2": 1,
    "F2_1": 2, "F2_2": 3,
    "F3_1": 4, "F3_2": 5,
    "F_ROT": 6,
    "W_X": 7, "W_YAW": 8, "W_PITCH": 9,
}
