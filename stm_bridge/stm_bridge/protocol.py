"""CAN frame of the STM32 servo node (team-jax/CAN_STM).

Pure functions, standard library only, so the byte layout is checked without a bus
(tests/check_protocol.py).

team-jax/CAN_STM (Stm32servosepc.md section 3, src/main.c):
  standard 11-bit id 5, data [motor_id][angle_x10 H][angle_x10 L], motor_id 0..9,
  angle_x10 0..1800 (0.0..180.0 deg). The firmware clips out-of-range angles and drops
  motor_id > 9; we clip and reject before sending so the caller knows.
"""
from dataclasses import dataclass

SERVO_STDID = 5
SERVO_COUNT = 10
SERVO_X10_MAX = 1800


@dataclass(frozen=True)
class Frame:
    arb_id: int
    extended: bool
    data: bytes

    def __str__(self):
        # cansend/candump notation, so a printed frame can be compared with `candump can0`
        ident = f"{self.arb_id:08X}" if self.extended else f"{self.arb_id:03X}"
        return f"{ident}#{self.data.hex().upper()}"


def servo_frame(motor_id: int, angle_deg: float) -> Frame:
    """One servo command. The angle is clipped to 0..180 deg like the firmware does."""
    if not 0 <= motor_id < SERVO_COUNT:
        raise ValueError(f"servo motor_id {motor_id} is not 0..{SERVO_COUNT - 1}")
    x10 = min(max(round(angle_deg * 10), 0), SERVO_X10_MAX)
    return Frame(SERVO_STDID, False, bytes([motor_id, x10 >> 8, x10 & 0xFF]))
