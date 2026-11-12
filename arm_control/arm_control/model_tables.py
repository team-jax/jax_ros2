"""The model's approach (RAISE -> SETTLE -> INSERT) as joint targets, from the deploy lookup tables.

Implements IntergrateSim artifacts/deploy_2026-11-05/README.md 7.1-7.3 in servo mode (no gravity term),
the same rules IntergrateSim deploy/v3_controller.py runs with the 25550 assets:

  RAISE   0 .. 8.0 s   q_cmd(t) = P_geo(t) + S(t); s = 0.85 s + 0.15 q_cmd; 7 arm joints <= 0.5 rad/s;
                       clamp to joint limits. Thumb first joint -30 deg * smoothstep((t - 1) / 3).
  SETTLE  .. 8.65 s    the same filter; the real robot has no joint feedback, so the handover is the
                       fixed 8.65 s of README 7.2.
  INSERT  45 ticks     clamp(q_h + smoothstep(k / 30) * delta), k = 1..30, then 15 ticks at q_h + delta.
  HOLD                 the last frame (the learned grasp starts here once its sensors exist).

Everything is in deploy joint angles (robot_v3 URDF, rad) at 60 Hz. No torch, no MuJoCo: the tables are
torch zip files, read here with a pickle reader that only accepts tensors.
"""
import pickle
import zipfile
from collections import OrderedDict
from pathlib import Path

import numpy as np

DT = 1 / 60
TABLE_ORIGIN = np.array([-0.285, 0.26])   # deploy world xy of the reference cube (README 7.1)
HANDOVER_S = 8.65                          # README 7.2, real robot
INSERT_TICKS, INSERT_RAMP = 45, 30         # README 7.3

ARM = ["shoulder_pitch_to_shoulder_roll", "shoulder_roll_to_shoulder_yaw",
       "shoulder_yaw_to_upper_arm", "upper_arm_to_under_arm", "under_arm_to_wrist_pitch",
       "wrist_pitch_to_wrist_yaw", "wrist_yaw_to_hand_base", "hand_base_to_rotation_base"]
FINGERS = ["hand_base_to_finger_1_1", "hand_base_to_finger_2_1", "rotation_base_to_finger_3_1",
           "finger_1_1_to_finger_1_2", "finger_1_2_to_finger_2_2", "finger_3_1_to_finger_3_2"]
DEPLOY_JOINTS = ARM + FINGERS
# robot_v3/urdf/JAXV3_Assembly.urdf <limit lower upper> of the 8 arm joints (rad)
ARM_LIMITS = np.array([[-0.785398, 0.785398], [0.0, 0.872665], [-1.570796, 4.188790],
                       [-1.745329, 1.745329], [-1.570796, 1.570796], [-1.570796, 1.570796],
                       [-2.094395, 2.094395], [-3.141593, 0.0]])

# README 7.1 P_geo knots (deg), servo mode
TIMES = np.array([0, 5, 5.6, 6.2, 6.8, 7.4, 8.0])
GEOMETRIC = np.deg2rad(np.array([
    [0] * 8,
    [-28.8, 0, 2.5, 89.1, -5.6, 20.8, -6.6, -170.1],
    [-24.6, 0, 2.5, 87.0, -5.6, 18.7, -6.6, -170.1],
    [-20.4, 0, 2.5, 84.4, -5.6, 17.0, -6.6, -170.1],
    [-16.1, 0, 2.5, 81.4, -5.6, 15.8, -6.6, -170.1],
    [-11.7, 0, 2.5, 77.8, -5.6, 15.0, -6.6, -170.1],
    [-7.3, 0, 2.5, 73.7, -5.6, 14.7, -6.6, -170.1],
]))

_DTYPES = {"FloatStorage": "<f4", "DoubleStorage": "<f8", "HalfStorage": "<f2", "LongStorage": "<i8",
           "IntStorage": "<i4", "ShortStorage": "<i2", "CharStorage": "i1", "ByteStorage": "u1",
           "BoolStorage": "?"}


def read_table(path):
    """{name: ndarray} of the tensors in a torch.save zip file (torch is not needed)."""
    with zipfile.ZipFile(path) as z:
        prefix = z.namelist()[0].split("/")[0]
        if z.read(f"{prefix}/byteorder").strip() != b"little":
            raise ValueError(f"{path}: not little-endian")

        def rebuild(storage, offset, size, stride, *_):
            dtype, raw = storage
            base = np.frombuffer(raw, dtype=dtype)
            if not size:
                return base[offset:offset + 1].reshape(()).copy()
            item = base.itemsize
            return np.lib.stride_tricks.as_strided(
                base[offset:], shape=tuple(size), strides=tuple(s * item for s in stride)).copy()

        class Reader(pickle.Unpickler):
            def find_class(self, module, name):
                if (module, name) == ("torch._utils", "_rebuild_tensor_v2"):
                    return rebuild
                if module == "torch" and name in _DTYPES:
                    return _DTYPES[name]
                if (module, name) == ("collections", "OrderedDict"):
                    return OrderedDict
                raise pickle.UnpicklingError(f"{path}: {module}.{name} is not a plain tensor")

            def persistent_load(self, pid):
                kind, dtype, key, _location, _numel = pid
                if kind != "storage":
                    raise pickle.UnpicklingError(f"{path}: unknown persistent id {kind}")
                return dtype, z.read(f"{prefix}/data/{key}")

        data = Reader(z.open(f"{prefix}/data.pkl")).load()
    return {k: v for k, v in data.items() if isinstance(v, np.ndarray)}


def smooth(x):
    x = np.clip(x, 0, 1)
    return x * x * (3 - 2 * x)


def hermite(ts, values, t):
    """README 7.1 Catmull-Rom: inner slopes (Q[k+1] - Q[k-1]) / (t[k+1] - t[k-1]), end slopes 0."""
    ts, values = np.asarray(ts), np.asarray(values)
    slopes = np.zeros_like(values)
    slopes[1:-1] = (values[2:] - values[:-2]) / (ts[2:] - ts[:-2])[:, None]
    k = int(np.clip(np.searchsorted(ts, t, side="right") - 1, 0, len(ts) - 2))
    h = ts[k + 1] - ts[k]
    x = np.clip((t - ts[k]) / h, 0, 1)
    return ((2 * x**3 - 3 * x * x + 1) * values[k] + (x**3 - 2 * x * x + x) * h * slopes[k]
            + (-2 * x**3 + 3 * x * x) * values[k + 1] + (x**3 - x * x) * h * slopes[k + 1])


def _cell(g, value):
    value = np.clip(value, g[0], g[-1])
    f = float((value - g[0]) / (g[1] - g[0]))
    if abs(f - round(f)) < 2e-5:
        f = float(round(f))
    i = int(np.clip(np.floor(f), 0, len(g) - 2))
    return i, f - i


def _bilinear(table, values, xy):
    i, a = _cell(table["gx"], xy[0])
    j, b = _cell(table["gy"], xy[1])
    return ((1 - a) * (1 - b) * values[..., i, j, :] + a * (1 - b) * values[..., i + 1, j, :]
            + (1 - a) * b * values[..., i, j + 1, :] + a * b * values[..., i + 1, j + 1, :])


class ApproachTables:
    """README 7.1/7.3 lookups for one cube offset (dx, dy) = cube world xy - (-0.285, 0.26), m.

    e = 0.4725 - d (d: shoulder pitch axis to table top, m); the final setup is e = 0.
    lower_pregrasp: the 0.5 cm lower pregrasp of 25550 (e_F = max(e - 0.005, -0.01)).
    """

    def __init__(self, folder, offset_xy, e=0.0, lower_pregrasp=True):
        folder = Path(folder)
        self.xy = np.asarray(offset_xy, dtype=float)
        if self.xy.shape != (2,) or abs(e) > 0.03:
            raise ValueError("offset must be (dx, dy) and |e| <= 0.03 m")
        self.mask = read_table(folder / "reach_mask_v65.pt")
        if not self.valid(self.xy):
            raise ValueError(f"cube offset {np.round(self.xy, 4).tolist()} m is outside reach_mask_v65")
        self.raise_knots = self._knots(folder, "raise_grid_v65", e)
        self.final_knots = self._knots(folder, "pregrasp_grid_v65", max(e - 0.005, -0.01) if lower_pregrasp else e)
        theta = read_table(folder / "hand_theta_v65.pt")
        self.theta_deg = float(_bilinear(theta, theta["theta_deg"][..., None], self.xy)[0])

    def valid(self, xy):
        """Closed union of the valid 2x2 cells (V3Tables.valid, modern)."""
        if not np.isfinite(xy).all():
            return False
        gx, gy = self.mask["gx"], self.mask["gy"]
        if not (gx[0] - 1e-7 <= xy[0] <= gx[-1] + 1e-7 and gy[0] - 1e-7 <= xy[1] <= gy[-1] + 1e-7):
            return False
        fx = float((xy[0] - gx[0]) / (gx[1] - gx[0]))
        fy = float((xy[1] - gy[0]) / (gy[1] - gy[0]))
        for ix in {int(np.floor(fx - 1e-4)), int(np.floor(fx + 1e-4))}:
            for iy in {int(np.floor(fy - 1e-4)), int(np.floor(fy + 1e-4))}:
                if 0 <= ix < len(gx) - 1 and 0 <= iy < len(gy) - 1 and self.mask["valid"][ix:ix + 2, iy:iy + 2].all():
                    return True
        return False

    def _knots(self, folder, prefix, e):
        tables = [read_table(folder / f"{prefix}_{suffix}.pt") for suffix in ("basec", "m3c", "p3c")]
        q = [_bilinear(t, t["dq"], self.xy) for t in tables]
        return q[0] + (abs(e) / 0.03) * ((q[1] if e < 0 else q[2]) - q[0])

    def shift(self, t):
        return hermite([0, 2, 5, 5.6, 6.2, 6.8, 7.4, 8],
                       np.vstack([np.zeros(8), self.raise_knots[0], self.raise_knots]), t)

    def q_cmd(self, t):
        """README 7.1 arm command P_geo(t) + S(t), 8 arm joints, rad."""
        return hermite(TIMES, GEOMETRIC, t) + self.shift(t)

    def insertion(self):
        """README 7.3 delta = F - S_5, wrist yaw + 12 deg."""
        delta = self.final_knots[-1] - self.raise_knots[-1]
        delta[6] += np.deg2rad(12)
        return delta


class Approach:
    """One 60 Hz target per step(), in DEPLOY_JOINTS order (rad)."""

    def __init__(self, tables, handover_s=HANDOVER_S):
        self.tables = tables
        self.handover_s = handover_s
        self.phase, self.k, self.phase_k = "RAISE", 0, 0
        self.s = np.zeros(8)
        self.fingers = np.zeros(6)
        self.last = None

    @property
    def t(self):
        return self.k * DT

    def _target(self, arm):
        return np.r_[arm, self.fingers]

    def step(self):
        if self.phase == "HOLD":
            return self.last.copy()
        t, phase = self.t, self.phase
        if phase in ("RAISE", "SETTLE"):
            new = 0.85 * self.s + 0.15 * self.tables.q_cmd(t)
            new[:7] = self.s[:7] + np.clip(new[:7] - self.s[:7], -0.5 * DT, 0.5 * DT)
            self.s = np.clip(new, ARM_LIMITS[:, 0], ARM_LIMITS[:, 1])
            self.fingers = np.deg2rad([0, 0, -30 * smooth((t - 1) / 3), 0, 0, 0])
            out = self._target(self.s)
            if phase == "RAISE" and t >= 8 - 1e-9:
                self.phase = "SETTLE"
            elif phase == "SETTLE" and t >= self.handover_s - 1e-9:
                self.q_h, self.delta = self.s.copy(), self.tables.insertion()
                self.phase = "INSERT"
        else:   # INSERT
            arm = np.clip(self.q_h + smooth((self.phase_k + 1) / INSERT_RAMP) * self.delta,
                          ARM_LIMITS[:, 0], ARM_LIMITS[:, 1])
            out = self._target(arm)
            if self.phase_k >= INSERT_TICKS - 1:
                self.phase = "HOLD"
        self.phase_k = self.phase_k + 1 if self.phase == phase else 0
        self.k += 1
        self.last = out
        return out.copy()
