"""Sim-to-sim playback of the trained A1 reach policy in MuJoCo.

Mirrors unitree_rl_lab's a1leg/sim2sim.py, adapted to the reach task:
- fixed-base 7-DoF arm (no floating base)
- joint PD gains are specified HERE (cfg.robot) and written into the model at runtime,
  exactly like the reference. Two control modes (selected by cfg.sim.control_mode):
    * "position": MuJoCo position servo  (force = kp*(ctrl-q) - kv*qvel)
    * "motor"   : explicit torque PD computed in Python, ctrl = clip(kp*(tgt-q)-kd*qd, +-eff)
- observation fields match the Isaac Lab env (35-dim), with an explicit policy quaternion layout:
    joint_pos_rel(7), joint_vel_rel(7), pose_command(7), last_action(7), ee_pose_error(7)
- the target pose ("command") is a reachable Link7 pose obtained by FK of a random joint config.
- like Isaac `play`, both the end-effector frame and the goal frame are drawn as RGB triads
  (x=red, y=green, z=blue), since the task tracks orientation too.

Run:  python scripts/sim2sim.py --policy /path/to/exported/policy.pt
"""

import argparse
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
import torch

PROJECT_ROOT = Path(__file__).resolve().parents[1]
ASSET_ROOT = PROJECT_ROOT / "source" / "h1_reach" / "h1_reach" / "assets"


class SimToSimCfg:
    class path:
        # two MJCF versions: position-servo actuators vs direct-torque (motor) actuators
        pos_xml_path = ASSET_ROOT / "mjcf" / "A1" / "a1_right_position.xml"
        motor_xml_path = ASSET_ROOT / "mjcf" / "A1" / "a1_right_motor.xml"

    class sim:
        sim_duration = 10000.0
        dt = 1.0 / 60.0  # matches the env sim.dt
        decimation = 2  # matches the env decimation -> 30 Hz policy
        action_dim = 7
        control_mode = "motor"  # "position" or "motor" (loads the matching XML)

    class robot:
        # init/home pose, identical to A1_RIGHT_CFG init_state (joint1..joint7 order)
        default_dof_pos = np.array([0.0, -0.6, 0.0, 1.0, 0.0, 0.5, 0.0], dtype=np.float64)
        # joint PD / effort, same as the Isaac implicit actuators (j1-4 proximal, j5-7 distal)
        stiffness = np.array([60.0, 60.0, 60.0, 60.0, 30.0, 30.0, 30.0])
        damping = np.array([6.0, 6.0, 6.0, 6.0, 3.0, 3.0, 3.0])
        effort = np.array([30.0, 30.0, 30.0, 30.0, 12.0, 12.0, 12.0])
        # rotor armature (effective joint inertia), MUST match the training config (A1_RIGHT_CFG).
        # With it, both "position" and "motor" modes are stable at dt=1/60. (At armature=0 the
        # explicit "motor" mode blows up on the near-zero-inertia wrist joints.)
        armature = 0.05
        action_scale = 0.5  # JointPositionActionCfg(scale=0.5, use_default_offset=True)
        joint_range_scale = 0.8  # FkReachablePoseCommandCfg(joint_range_scale=0.8)
        ee_body = "Link7"
        joint_names = [
            "joint1-a1_r",
            "joint2-a1_r",
            "joint3-a1_r",
            "joint4-a1_r",
            "joint5-a1_r",
            "joint6-a1_r",
            "joint7-a1_r",
        ]

    class command:
        resample_time = 4.0  # matches resampling_time_range=(4.0, 4.0)

    class observation:
        # Preserve the legacy sim2sim default; beta2-trained policies require "xyzw".
        policy_quaternion_layout = "wxyz"


def quat_conjugate(q):  # wxyz
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_mul(a, b):  # Hamilton product, internal MuJoCo wxyz layout
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ]
    )


def format_policy_quat(q, layout):
    """Convert an internal MuJoCo wxyz quaternion to the policy's expected layout."""
    if layout == "wxyz":
        return q
    if layout == "xyzw":
        return np.roll(q, -1)
    raise ValueError(f"Unsupported policy quaternion layout: {layout}")


def draw_frame(scn, pos, mat9, length=0.18, width=0.0025):
    """Draw an RGB coordinate triad (x=red, y=green, z=blue) at pos with orientation mat9 (row-major)."""
    mat = np.asarray(mat9).reshape(3, 3)
    colors = [(1, 0, 0, 1), (0, 1, 0, 1), (0, 0, 1, 1)]
    for i in range(3):
        if scn.ngeom >= scn.maxgeom:
            return
        g = scn.geoms[scn.ngeom]
        axis = mat[:, i]
        to = np.asarray(pos) + length * axis
        mujoco.mjv_initGeom(
            g, mujoco.mjtGeom.mjGEOM_ARROW, np.zeros(3), np.zeros(3), np.zeros(9), np.array(colors[i], dtype=np.float32)
        )
        mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_ARROW, width, np.asarray(pos), to)
        scn.ngeom += 1


class MujocoRunner:
    def __init__(self, cfg: SimToSimCfg, policy_path: Path):
        self.cfg = cfg
        self.control_mode = cfg.sim.control_mode
        self.policy_quaternion_layout = cfg.observation.policy_quaternion_layout
        if self.control_mode not in {"position", "motor"}:
            raise ValueError(f"Unsupported control mode: {self.control_mode}")
        if self.policy_quaternion_layout not in {"wxyz", "xyzw"}:
            raise ValueError(f"Unsupported policy quaternion layout: {self.policy_quaternion_layout}")
        if not policy_path.is_file():
            raise FileNotFoundError(f"Policy checkpoint does not exist: {policy_path}")
        xml = cfg.path.pos_xml_path if self.control_mode == "position" else cfg.path.motor_xml_path
        self.model = mujoco.MjModel.from_xml_path(str(xml))
        self.model.opt.timestep = cfg.sim.dt
        self.data = mujoco.MjData(self.model)
        self.scratch = mujoco.MjData(self.model)  # for FK-based command sampling
        self.policy = torch.jit.load(str(policy_path), map_location="cpu")
        self.policy.eval()

        self.action_dim = cfg.sim.action_dim
        self.action_scale = cfg.robot.action_scale
        self.default_dof_pos = cfg.robot.default_dof_pos
        self.decimation = cfg.sim.decimation
        self.kp, self.kd, self.eff = cfg.robot.stiffness, cfg.robot.damping, cfg.robot.effort

        # joint addresses in Isaac order
        self.jids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in cfg.robot.joint_names]
        self.qadr = np.array([self.model.jnt_qposadr[j] for j in self.jids])
        self.vadr = np.array([self.model.jnt_dofadr[j] for j in self.jids])
        self.ee_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, cfg.robot.ee_body)
        self.jnt_range = self.model.jnt_range[self.jids]

        self._setup_actuators()

        self.action = np.zeros(self.action_dim, dtype=np.float32)
        self.des_pos = np.zeros(3)
        self.des_quat = np.array([1.0, 0.0, 0.0, 0.0])

        key_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, "home")
        mujoco.mj_resetDataKeyframe(self.model, self.data, key_id)
        mujoco.mj_forward(self.model, self.data)
        self.resample_command()
        self.last_resample = self.data.time

        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        self.viewer.cam.distance = 1.4
        self.viewer.cam.elevation = -20
        self.viewer.cam.azimuth = 135
        self.viewer.cam.lookat[:] = [0.0, 0.0, 0.3]
        print(f"control_mode={self.control_mode}  kp={self.kp[0]}/{self.kp[-1]}  kd={self.kd[0]}/{self.kd[-1]}")

    def _setup_actuators(self):
        """Write the joint PD into the model, mirroring the reference (position vs motor)."""
        ai = np.arange(self.action_dim)  # actuators are in joint order (joint1..joint7)
        if self.control_mode == "position":
            # MuJoCo position servo: force = kp*(ctrl - q) - kv*qvel
            self.model.actuator_gainprm[ai, 0] = self.kp
            self.model.actuator_biasprm[ai, 1] = -self.kp
            self.model.actuator_biasprm[ai, 2] = -self.kd
            self.model.dof_damping[self.vadr] = 0.0
        else:  # motor: actuator is a direct torque source, PD computed in Python
            self.model.actuator_gainprm[ai, 0] = 1.0
            self.model.actuator_biasprm[ai, 1] = 0.0
            self.model.actuator_biasprm[ai, 2] = 0.0
            self.model.dof_damping[self.vadr] = 0.0
        # armature for both modes (stabilizes the explicit motor PD on low-inertia joints)
        self.model.dof_armature[self.vadr] = self.cfg.robot.armature

    def resample_command(self):
        """Sample a reachable target Link7 pose via FK of a random joint config (matches training)."""
        lo, hi = self.jnt_range[:, 0], self.jnt_range[:, 1]
        center = 0.5 * (lo + hi)
        half = 0.5 * (hi - lo) * self.cfg.robot.joint_range_scale
        q = center - half + 2.0 * half * np.random.rand(self.action_dim)
        self.scratch.qpos[self.qadr] = q
        mujoco.mj_forward(self.model, self.scratch)
        self.des_pos = self.scratch.xpos[self.ee_id].copy()
        self.des_quat = self.scratch.xquat[self.ee_id].copy()  # wxyz
        self.data.mocap_pos[0] = self.des_pos
        self.data.mocap_quat[0] = self.des_quat

    def get_obs(self):
        q = self.data.qpos[self.qadr].astype(np.float32)
        qd = self.data.qvel[self.vadr].astype(np.float32)
        joint_pos_rel = q - self.default_dof_pos.astype(np.float32)
        joint_vel_rel = qd
        des_quat_obs = format_policy_quat(self.des_quat, self.policy_quaternion_layout)
        pose_command = np.concatenate([self.des_pos, des_quat_obs]).astype(np.float32)
        ee_pos = self.data.xpos[self.ee_id]
        ee_quat = self.data.xquat[self.ee_id]
        pos_err = (self.des_pos - ee_pos).astype(np.float32)
        quat_err = quat_mul(self.des_quat, quat_conjugate(ee_quat)).astype(np.float32)
        quat_err = format_policy_quat(quat_err, self.policy_quaternion_layout)
        return np.concatenate([joint_pos_rel, joint_vel_rel, pose_command, self.action, pos_err, quat_err]).astype(
            np.float32
        )

    def apply_control(self, target):
        if self.control_mode == "position":
            self.data.ctrl[:] = target
        else:
            q = self.data.qpos[self.qadr]
            qd = self.data.qvel[self.vadr]
            self.data.ctrl[:] = np.clip(self.kp * (target - q) - self.kd * qd, -self.eff, self.eff)

    def draw_markers(self):
        scn = self.viewer.user_scn
        scn.ngeom = 0
        # end-effector frame (Link7)
        draw_frame(scn, self.data.xpos[self.ee_id], self.data.xmat[self.ee_id])
        # goal frame
        goal_mat = np.zeros(9)
        mujoco.mju_quat2Mat(goal_mat, self.des_quat)
        draw_frame(scn, self.des_pos, goal_mat)

    def run(self):
        ctrl_dt = self.decimation * self.cfg.sim.dt
        while self.viewer.is_running() and self.data.time < self.cfg.sim.sim_duration:
            t0 = time.time()
            obs = self.get_obs()
            self.action = self.policy(torch.from_numpy(obs)).detach().numpy().astype(np.float32)
            target = self.default_dof_pos + self.action_scale * self.action
            for _ in range(self.decimation):
                self.apply_control(target)
                mujoco.mj_step(self.model, self.data)

            if self.data.time - self.last_resample > self.cfg.command.resample_time:
                self.resample_command()
                self.last_resample = self.data.time

            pos_err = np.linalg.norm(self.des_pos - self.data.xpos[self.ee_id])
            print(f"\rt={self.data.time:6.1f}s  pos_err={pos_err * 100:5.1f} cm", end="")

            self.draw_markers()
            self.viewer.sync()
            time.sleep(max(0.0, ctrl_dt - (time.time() - t0)))
        self.viewer.close()


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description="Run the OneRobotics A1 reach policy in MuJoCo.")
    parser.add_argument("--policy", type=Path, required=True, help="Path to an exported TorchScript policy.pt file.")
    parser.add_argument(
        "--control-mode",
        choices=("position", "motor"),
        default=SimToSimCfg.sim.control_mode,
        help="MuJoCo actuator mode (default: motor).",
    )
    parser.add_argument(
        "--policy-quaternion-layout",
        choices=("wxyz", "xyzw"),
        default=SimToSimCfg.observation.policy_quaternion_layout,
        help="Quaternion layout expected by the policy (beta2: xyzw; legacy default: wxyz).",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    cfg = SimToSimCfg()
    cfg.sim.control_mode = args.control_mode
    cfg.observation.policy_quaternion_layout = args.policy_quaternion_layout
    runner = MujocoRunner(cfg, args.policy.expanduser().resolve())
    runner.run()
