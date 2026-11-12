"""From a camera /object_pose to a checked approach plan (no ROS here, so it is tested on its own).

PoseGate decides which poses may start the arm: robot frame only (vision_node's camera_to_robot set),
`stable` poses in a row within tol_m, the newest no older than max_age_s.
plan_from_pose turns one pose into the 60 Hz /arm_control/joint_command frames of the model's
approach (model_tables.Approach). check_plan lists what calibration.json would do to them.
"""
from dataclasses import dataclass, field

import numpy as np

from . import model_frames as mf
from . import model_tables as mt
from .play import clipped_joints

ROBOT_FRAME = "shoulder_roll_axis"
TABLE_TOP_Z, CUBE_HALF = 0.735, 0.025      # deploy world (IntergrateSim README, ik_delivery.py)


class PoseGate:
    def __init__(self, stable=2, tol_m=0.01, max_age_s=3.0):
        self.stable, self.tol, self.max_age = stable, tol_m, max_age_s
        self.poses = []            # [(xyz ndarray, yaw deg, time)]
        self.reason = "no /object_pose yet"

    def add(self, frame_id, xyz, yaw_deg, now):
        if frame_id != ROBOT_FRAME:
            self.poses = []
            self.reason = (f"/object_pose is in frame '{frame_id}', not {ROBOT_FRAME}: "
                           "set vision_node camera_to_robot")
            return
        xyz = np.asarray(xyz, dtype=float)
        if not np.isfinite(xyz).all() or not np.isfinite(yaw_deg):
            self.reason = "non-finite /object_pose"
            return
        if self.poses and np.linalg.norm(xyz - self.poses[-1][0]) > self.tol:
            self.poses = []
        self.poses = (self.poses + [(xyz, float(yaw_deg), now)])[-self.stable:]
        self.reason = None

    def ready(self, now):
        """((xyz, yaw_deg), None) when a pose may be used, else (None, why)."""
        if self.reason:
            return None, self.reason
        if len(self.poses) < self.stable:
            return None, f"{len(self.poses)}/{self.stable} matching poses within {self.tol * 100:g} cm"
        age = now - self.poses[-1][2]
        if age > self.max_age:
            return None, f"newest /object_pose is {age:.1f} s old (max {self.max_age:g} s)"
        xyz = np.mean([p for p, _, _ in self.poses], axis=0)
        return (xyz, float(np.mean([y for _, y, _ in self.poses]))), None


@dataclass
class Plan:
    ros: np.ndarray            # cube in shoulder_roll_axis, m
    world: np.ndarray          # cube in the deploy world, m
    offset: np.ndarray         # (dx, dy) of the tables, m
    yaw_deg: float             # cube yaw wrapped into (-45, 45] (camera yaw, not converted yet)
    theta_deg: float           # hand turn of the tables at this offset
    frames: list = field(default_factory=list)   # [(phase, {ROS joint: rad})] at 60 Hz
    final_deploy: np.ndarray = None              # 8 arm joints at the end of INSERT, rad


def plan_from_pose(ros_xyz, yaw_deg, tables_dir, send_finger_rotation=False, e=0.0,
                   max_z_error_m=0.05, handover_s=mt.HANDOVER_S):
    """ValueError when the cube is off the reachable table or its height says the frame is wrong."""
    ros_xyz = np.asarray(ros_xyz, dtype=float)
    world = mf.ros_to_world(ros_xyz)
    want_z = TABLE_TOP_Z + CUBE_HALF
    if max_z_error_m >= 0 and abs(world[2] - want_z) > max_z_error_m:
        raise ValueError(f"cube centre z {world[2]:.3f} m in the deploy world, the table puts it at {want_z:.3f} "
                         f"(+-{max_z_error_m:g}): check camera_to_robot")
    offset = world[:2] - mt.TABLE_ORIGIN
    tables = mt.ApproachTables(tables_dir, offset, e=e)
    run = mt.Approach(tables, handover_s)
    frames = []
    while run.phase != "HOLD":
        phase = run.phase
        cmd = mf.to_ros_command(dict(zip(mt.DEPLOY_JOINTS, run.step())))
        if not send_finger_rotation:
            cmd.pop("finger_rotation")
        frames.append((phase, cmd))
    return Plan(ros_xyz, world, offset, mf.cube_yaw_deg(yaw_deg), tables.theta_deg, frames,
                run.last[:8].copy())


def check_plan(plan, cal):
    """Problems a run must not ignore without force: joints calibration.json would cut short, and
    joints whose calibration is not verified."""
    names = list(plan.frames[0][1])
    traj = {"joint_names": names, "positions": [[f[n] for n in names] for _, f in plan.frames]}
    problems = [f"{name} goes {lo:.1f}..{hi:.1f} deg but the calibration allows {ok[0]:.1f}..{ok[1]:.1f}: "
                "it would be cut short" for name, lo, hi, ok in clipped_joints(traj, cal)]
    unverified = [n for n in names if not cal[n].verified]
    if unverified:
        problems.append("calibration not verified (zero and direction are assumptions): " + ", ".join(unverified))
    return problems
