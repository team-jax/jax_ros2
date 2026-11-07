"""CAN frames of the AK45-36 actuators (the servo frame is stm_bridge.protocol.servo_frame).

Pure functions, standard library only, so the byte layout is checked without a bus
(tests/check_protocol.py). arm_control publishes /ak45/command and never writes these frames itself;
they are used by calibration.Joint.frame, the dry-run of calibrate and the tests.

AK45-36, servo mode (CubeMars AK Series Driver Manual V1.0.18, section 5.1/5.2):
  extended 29-bit id = (control mode << 8) | controller id.
  Position mode 4: int32 big-endian, deg * 10000, range +-36000 deg (5.1.5).
  Feedback 0x29: int16 pos * 0.1 deg, int16 speed * 10 ERPM, int16 current * 0.01 A,
  int8 temperature (C), uint8 error code (5.2.1).
  The manual does not say whether the position is on the output shaft or on the rotor
  (36:1); ak45_can calls this V3 and it is still open. calibration.json's `scale` holds it.
"""
from dataclasses import dataclass
import struct

from stm_bridge.protocol import Frame

AK45_MODE_POS = 4
AK45_FEEDBACK = 0x29
AK45_POS_MAX_DEG = 36000.0

AK45_ERRORS = {0: "none", 1: "motor over-temperature", 2: "over-current", 3: "over-voltage",
               4: "under-voltage", 5: "encoder fault", 6: "MOSFET over-temperature", 7: "motor stall"}


def ak45_pos_frame(controller_id: int, deg: float) -> Frame:
    """Position mode (4). The manual's routine truncates pos * 10000 to int32; so do we."""
    if not 0 < controller_id <= 0xFF:
        raise ValueError(f"AK45 controller id {controller_id} is not 1..255")
    if abs(deg) > AK45_POS_MAX_DEG:
        raise ValueError(f"AK45 position {deg} deg is outside +-{AK45_POS_MAX_DEG:g}")
    return Frame((AK45_MODE_POS << 8) | controller_id, True, struct.pack(">i", int(deg * 10000.0)))


@dataclass(frozen=True)
class AK45State:
    controller_id: int
    position_deg: float
    speed_erpm: float
    current_a: float
    temperature_c: int
    error: int


def parse_ak45_feedback(arb_id: int, extended: bool, data: bytes):
    """AK45State for a 0x29 feedback frame, None for anything else."""
    if not extended or len(data) < 8 or (arb_id >> 8) != AK45_FEEDBACK:
        return None
    pos, spd, cur, temp, err = struct.unpack(">hhhbB", bytes(data[:8]))
    return AK45State(arb_id & 0xFF, pos * 0.1, spd * 10.0, cur * 0.01, temp, err)
