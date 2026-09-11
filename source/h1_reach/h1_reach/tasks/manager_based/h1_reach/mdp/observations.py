# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import quat_conjugate, quat_mul, subtract_frame_transforms

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def ee_pose_error_b(env: ManagerBasedRLEnv, command_name: str, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """End-effector pose error w.r.t. the commanded pose, in the robot base frame.

    Returns a (num_envs, 7) tensor: position error (3) followed by the orientation error
    quaternion in Isaac Lab's native layout (4; beta2: xyzw). Giving the policy its own tracking
    error closes the control loop and typically improves tracking precision a lot.
    """
    asset: RigidObject = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)  # target pose in base frame (N, 7)
    des_pos_b, des_quat_b = command[:, :3], command[:, 3:7]
    # current end-effector pose expressed in the base frame
    body_id = asset_cfg.body_ids[0]  # type: ignore
    curr_pos_b, curr_quat_b = subtract_frame_transforms(
        asset.data.root_pos_w.torch,
        asset.data.root_quat_w.torch,
        asset.data.body_pos_w.torch[:, body_id],
        asset.data.body_quat_w.torch[:, body_id],
    )
    pos_error = des_pos_b - curr_pos_b
    quat_error = quat_mul(des_quat_b, quat_conjugate(curr_quat_b))
    return torch.cat([pos_error, quat_error], dim=-1)
