"""The learned grasp and lift after the approach (JaxIsaacLab deploy_2026-11-05 README 7.4-7.8, policy
grasp_lift_25550, section 8), NumPy only, no ROS, so it is tested on its own.

One step() per 1/60 s, in this order (README 7.4): policy -> finger closing (7.5) -> lift IK (7.6) or the
L return (7.7) -> joint limits -> 0.5 s blend for what goes to the motors -> done check (7.8).

Everything is in deploy joint angles (robot_v3 URDF, document sign convention, rad), DEPLOY_JOINTS order
(8 arm joints with RotationBody last, then the 6 finger joints). Positions are in the deploy world.

What the real robot gives instead of the simulator (README 7.6, 8.1 and what we have):
  - joint positions: the TARGETS sent (no encoders on the servos), as the policy was trained (25300+).
  - pad forces (N, 6, README 6.1 order) and ADC values (0..1): stm_bridge /pressure and /pressure_adc.
    The real sensors only see the normal force, so the thumb "magnitude" of 6.2 is the normal force here.
  - the cube: the camera position at the start. It is not seen while it is in the hand, so after the
    hand grips it (thumb and finger 1 or 2 >= 0.3 N) it is taken to move with the fingertip midpoint
    (cube = midpoint + the offset at the grip). Its velocity is the change of that estimate. The lift
    (7.7) and done (7.8) checks use this estimate.
  - the cube yaw (43-44): cube_yaw_deg (the camera yaw is not converted to the robot frame yet).
"""
from math import cos, radians, sin

import numpy as np

from .model_tables import ARM, ARM_LIMITS, DEPLOY_JOINTS, DT, smooth

ISAAC = ARM[:7] + ["hand_base_to_finger_1_1", "hand_base_to_finger_2_1", "hand_base_to_rotation_base",
                   "finger_1_1_to_finger_1_2", "finger_1_2_to_finger_2_2", "rotation_base_to_finger_3_1",
                   "finger_3_1_to_finger_3_2"]
_ISAAC_IDX = [DEPLOY_JOINTS.index(n) for n in ISAAC]
# README 8.1: observation normalization with the V2 limits (deg), Isaac order
OBS_LIMITS = np.deg2rad(np.array([[-45, 45], [0, 45], [-90, 240], [-105, 105], [-90, 90], [-90, 90], [-120, 120],
                                  [-90, 90], [-90, 90], [-180, 0], [0, 120], [0, 120], [-90, 90], [0, 120]], float))
# robot_v3 URDF limits of the 6 finger joints (FINGERS order), rad
FINGER_LIMITS = np.deg2rad(np.array([[-90, 90], [-90, 90], [-90, 90], [0, 120], [0, 120], [0, 120]], float))
LIMITS = np.vstack([ARM_LIMITS, FINGER_LIMITS])
OBS_REF = np.array([0.0, 0.0, 0.3575])            # README 4: observation reference point (world)
BASE_ROT = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])   # base -> world, Rz(-90)
IK_ARM = [0, 2, 3, 4, 5, 6]                       # README 7.6: no shoulder roll (Isaac 1)
Q_L = np.deg2rad([0, 0, 7.2, 90.0, 0, -2.6, 0])   # README 7.7 servo mode, 7 arm joints
ROTATION_BODY = np.deg2rad(-170.0)
OPEN_HAND = np.deg2rad([0, 0, -30, 0, 0, 0])
CONTACT_N = 0.3
CLOSE_GAIN, CLOSE_SPEED = 0.0012, 0.15            # rad/N, rad/s (README 7.5)
FIRST_LO, FIRST_HI = np.deg2rad([0, 0, -30]), np.deg2rad([30, 30, 0])
BLEND_TICKS, RISE_M, RISE_TICKS, RETURN_S, DONE_TICKS = 30, 0.04, 30, 2.5, 30


class MlpPolicy:
    """grasp_lift_25550: (obs - mean) / (std + 0.01) -> 256 ELU -> 256 ELU -> 11 (model/grasp_lift_25550.npz,
    exported from the .onnx), then +-1 clip and a = 0.25 new + 0.75 a (README 2)."""

    def __init__(self, npz):
        w = np.load(npz)
        self.w = {k: w[k].astype(np.float64) for k in w.files}
        self.filtered = np.zeros(11)

    def reset(self):
        self.filtered[:] = 0.0

    def raw(self, obs):
        obs = np.asarray(obs, dtype=np.float64)
        if obs.shape != (45,) or not np.isfinite(obs).all():
            raise ValueError("invalid policy observation")
        x = (obs - self.w["mean"]) / self.w["div"]
        for i in (0, 1):
            x = x @ self.w[f"w{i}"].T + self.w[f"b{i}"]
            x = np.where(x > 0, x, np.expm1(np.minimum(x, 0)))
        out = x @ self.w["w2"].T + self.w["b2"]
        if not np.isfinite(out).all():
            raise ValueError("invalid policy output")
        return out

    def __call__(self, obs, fixed_raw=None):
        raw = np.clip(self.raw(obs), -1, 1)
        if fixed_raw:
            for i, v in fixed_raw.items():
                raw[i] = v
        self.filtered = 0.25 * raw + 0.75 * self.filtered
        return self.filtered.copy()


def as_dict(q):
    return dict(zip(DEPLOY_JOINTS, q))


class GraspController:
    def __init__(self, robot, policy, start, cube_world, theta_deg, cube_yaw_deg=0.0, e=0.0, timeout_s=10.0):
        """start: the 14 deploy targets at the end of INSERT (rad). cube_world: the camera's cube centre."""
        self.robot, self.policy = robot, policy
        self.start = np.asarray(start, float).copy()
        self.start[8:] = OPEN_HAND                     # README 7.4: closing state reset to open
        self.target = self.start.copy()                # q_new, what the observation sees
        self.prev = self.start.copy()                  # README 8.1: the previous target, reset here
        self.hold = self.start[:8].copy()
        self.offset = np.zeros(8)
        self.first = OPEN_HAND[:3].copy()
        q = as_dict(self.target)
        self.p_des = robot.fingertip_mid(q)
        self.r0 = robot.hand(q)[0]
        self.cube0 = np.asarray(cube_world, float).copy()
        self.cube = self.cube0.copy()
        self.cube_vel = np.zeros(3)
        self.grip_offset = None
        self.lift_z = float(self.cube0[2])
        self.psi = radians(cube_yaw_deg - theta_deg)
        self.dz = -0.0575 + e                          # README 8.1 (dz = 0.415 - d, d = 0.4725 - e)
        self.timeout = int(timeout_s / DT)
        self.phase, self.k, self.phase_k = "GRASP", 0, 0
        self.rise_count = self.done_count = 0
        self.frozen = None
        self.return_start = None
        self.reason = ""
        policy.reset()

    # --- observation (README 8.1) ---------------------------------------------------------------
    def observation(self, pad_adc):
        q = self.target[_ISAAC_IDX]
        lo, hi = OBS_LIMITS[:, 0], OBS_LIMITS[:, 1]
        rate = (self.target - self.prev)[_ISAAC_IDX] / DT * 0.2
        mid = self.robot.fingertip_mid(as_dict(self.target))
        obs = np.r_[2 * (q - lo) / (hi - lo) - 1, rate,
                    BASE_ROT.T @ (self.cube - OBS_REF), BASE_ROT.T @ self.cube_vel, BASE_ROT.T @ (self.cube - mid),
                    np.asarray(pad_adc, float)]
        obs = np.clip(obs, -5, 5)
        obs[9], obs[23] = -0.7222222, 0.0
        obs[29] -= 0.115
        obs[30] -= self.dz
        return np.r_[obs, sin(4 * self.psi), cos(4 * self.psi)]

    # --- 7.5 closing ----------------------------------------------------------------------------
    def close(self, a, pad_n):
        f = np.asarray(pad_n[:3], float) + np.asarray(pad_n[3:], float)
        on = f >= CONTACT_N
        if on.all():
            s_f, s_t = max(1.2 + 0.8 * a[7], CONTACT_N), max(1.2 + 0.8 * a[9], CONTACT_N)
            want = np.array([s_f / 2, s_f / 2, s_t])
        else:
            want = np.full(3, CONTACT_N)
        step = np.where(on, np.clip(CLOSE_GAIN * (want - f), -CLOSE_SPEED * DT, CLOSE_SPEED * DT), CLOSE_SPEED * DT)
        self.first = np.clip(self.first + step, FIRST_LO, FIRST_HI)
        tips = np.deg2rad(60) * np.clip(a[[8, 8, 10]], 0, 1)
        return np.r_[self.first, tips], f

    # --- 7.6 lift IK ----------------------------------------------------------------------------
    def ik(self, a, q):
        mid = self.robot.fingertip_mid(q)
        v = np.array([0.005 * a[3], 0.005 * a[5], -0.035 * a[0]])
        self.p_des = self.p_des + v * DT
        v = v + 2 * np.clip(self.p_des - mid, -0.02, 0.02)
        w = rotation_error_world(self.r0, self.robot.hand(q)[0])
        jac = self.robot.point_jacobian(q, [ARM[i] for i in IK_ARM], mid)
        dq = jac.T @ np.linalg.solve(jac @ jac.T + 0.02 ** 2 * np.eye(6), np.r_[v, w])
        self.offset[IK_ARM] = np.clip(self.offset[IK_ARM] + np.clip(dq, -0.6, 0.6) * DT, -1.0, 1.0)
        return self.hold + self.offset

    # --- one 60 Hz step -------------------------------------------------------------------------
    def step(self, pad_n, pad_adc):
        """-> 14 deploy targets for the motors (rad). Phases GRASP, RETURN, DONE, TIMEOUT."""
        if self.phase in ("DONE", "TIMEOUT"):
            return self.target.copy()
        obs = self.observation(pad_adc)
        fixed = {7: 1.0} if self.phase == "RETURN" else None
        a = self.policy(obs, fixed)
        if self.phase == "RETURN":
            a[8:11] = self.frozen
            self.policy.filtered[8:11] = self.frozen
        fingers, f = self.close(a, pad_n)
        new = self.target.copy()
        new[8:] = fingers
        q = as_dict(new)
        if self.phase == "GRASP":
            arm = self.ik(a, q)
        else:
            arm = self.return_start.copy()
            arm[:7] = self.return_start[:7] + smooth(self.phase_k * DT / RETURN_S) * (Q_L - self.return_start[:7])
        arm[7] = ROTATION_BODY
        new[:8] = arm
        new = np.clip(new, LIMITS[:, 0], LIMITS[:, 1])
        self.prev, self.target = self.target, new
        self._track_cube(f)
        self._advance(a, f)
        if self.k < BLEND_TICKS:
            return self.start + smooth((self.k + 1) / BLEND_TICKS) * (new - self.start)
        return new.copy()

    def _track_cube(self, f):
        mid = self.robot.fingertip_mid(as_dict(self.target))
        if self.grip_offset is None and f[2] >= CONTACT_N and (f[0] >= CONTACT_N or f[1] >= CONTACT_N):
            self.grip_offset = self.cube0 - mid
        new = self.cube0 if self.grip_offset is None else mid + self.grip_offset
        self.cube_vel = (new - self.cube) / DT
        self.cube = new

    def _advance(self, a, f):
        rise = float(self.cube[2] - self.lift_z)
        self.k += 1
        self.phase_k += 1
        if self.phase == "GRASP":
            self.rise_count = self.rise_count + 1 if rise >= RISE_M else 0
            if self.rise_count >= RISE_TICKS:
                self.frozen = a[8:11].copy()
                self.return_start = self.target[:8].copy()
                self.phase, self.phase_k = "RETURN", 0
            elif self.k >= self.timeout:
                self.phase, self.reason = "TIMEOUT", f"the cube was not lifted {RISE_M * 100:g} cm within {self.timeout * DT:g} s"
        elif self.phase == "RETURN":
            still = np.max(np.abs(self.target[:7] - self.prev[:7])) < np.deg2rad(0.05)
            ok = (self.phase_k * DT >= RETURN_S and still and rise >= RISE_M and f[2] >= CONTACT_N
                  and (f[0] >= CONTACT_N or f[1] >= CONTACT_N))
            self.done_count = self.done_count + 1 if ok else 0
            if self.done_count >= DONE_TICKS:
                self.phase, self.reason = "DONE", "lifted and returned to the L pose, still gripping"
            elif self.phase_k * DT > RETURN_S + 5.0:
                self.phase, self.reason = "TIMEOUT", "the grip was lost during the L return"


def rotation_error_world(r_want, r_now):
    from .robot_v3 import rotation_error
    return rotation_error(r_want, r_now)


class FingerTest:
    """Fingers only, the arm holds still: a check of the finger servos at the pregrasp pose (no policy, no IK).

    FINGERS: the first joints close by README 7.5 (0.15 rad/s until a digit reads >= 0.3 N, then held at
    0.3 N by the 0.0012 rad/N rule); the tips bend to tip_deg. With no pressure sensors (pads 0) every
    finger closes to its limit (30 deg, thumb 0) in about 3.5 s. CLOSED: held hold_s. OPEN: back to the
    open hand (0, 0, -30, 0, 0, 0) over open_s, then DONE.
    """

    def __init__(self, start, tip_deg=60.0, hold_s=2.0, open_s=1.5, max_close_s=6.0):
        self.start = np.asarray(start, float).copy()
        self.target = self.start.copy()
        self.target[8:] = OPEN_HAND
        self.first = OPEN_HAND[:3].copy()
        self.tips = np.deg2rad(tip_deg)
        self.hold, self.open, self.max_close = int(hold_s / DT), int(open_s / DT), int(max_close_s / DT)
        self.phase, self.phase_k, self.still = "FINGERS", 0, 0
        self.closed = None
        self.reason = ""

    def step(self, pad_n):
        if self.phase == "DONE":
            return self.target.copy()
        self.phase_k += 1
        if self.phase == "FINGERS":
            f = np.asarray(pad_n[:3], float) + np.asarray(pad_n[3:], float)
            on = f >= CONTACT_N
            step = np.where(on, np.clip(CLOSE_GAIN * (CONTACT_N - f), -CLOSE_SPEED * DT, CLOSE_SPEED * DT),
                            CLOSE_SPEED * DT)
            new = np.clip(self.first + step, FIRST_LO, FIRST_HI)
            self.still = self.still + 1 if np.allclose(new, self.first, atol=1e-6) else 0
            self.first = new
            ramp = min(1.0, self.phase_k * DT / 1.0)
            self.target[8:] = np.r_[self.first, self.tips * ramp, self.tips * ramp, self.tips * ramp]
            if (self.still >= 30 and ramp >= 1.0) or self.phase_k >= self.max_close:
                self.closed = self.target[8:].copy()
                self.reason = ("fingers at " + ", ".join(f"{d:.1f}" for d in np.rad2deg(self.closed)) + " deg; pads "
                               + ", ".join(f"{v:.2f}" for v in f) + " N")
                self.phase, self.phase_k = "CLOSED", 0
        elif self.phase == "CLOSED":
            if self.phase_k >= self.hold:
                self.phase, self.phase_k = "OPEN", 0
        elif self.phase == "OPEN":
            self.target[8:] = self.closed + smooth(self.phase_k / self.open) * (OPEN_HAND - self.closed)
            if self.phase_k >= self.open:
                self.phase, self.reason = "DONE", "fingers opened again"
        return self.target.copy()
