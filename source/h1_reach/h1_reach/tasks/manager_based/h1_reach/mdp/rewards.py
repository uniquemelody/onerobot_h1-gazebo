# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import combine_frame_transforms, quat_error_magnitude, quat_mul

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def orientation_command_error_tanh(
    env: ManagerBasedRLEnv, std: float, command_name: str, asset_cfg: SceneEntityCfg
) -> torch.Tensor:
    """Reward orientation tracking with a tanh kernel (mirror of position_command_error_tanh)."""
    asset: RigidObject = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)
    des_quat_b = command[:, 3:7]
    des_quat_w = quat_mul(asset.data.root_quat_w.torch, des_quat_b)
    curr_quat_w = asset.data.body_quat_w.torch[:, asset_cfg.body_ids[0]]  # type: ignore
    error = quat_error_magnitude(curr_quat_w, des_quat_w)
    return 1 - torch.tanh(error / std)


def pose_command_error_tanh(
    env: ManagerBasedRLEnv, pos_std: float, ori_std: float, command_name: str, asset_cfg: SceneEntityCfg
) -> torch.Tensor:
    """Joint position+orientation tracking reward (multiplicative tanh kernels).

    The reward is only large when BOTH the position and the orientation errors are small, so the
    policy cannot maximize it by satisfying one objective while ignoring the other.
    """
    asset: RigidObject = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)
    # position error in world frame
    des_pos_w, des_quat_w = combine_frame_transforms(
        asset.data.root_pos_w.torch, asset.data.root_quat_w.torch, command[:, :3], command[:, 3:7]
    )
    body_id = asset_cfg.body_ids[0]  # type: ignore
    pos_error = torch.norm(asset.data.body_pos_w.torch[:, body_id] - des_pos_w, dim=1)
    ori_error = quat_error_magnitude(asset.data.body_quat_w.torch[:, body_id], des_quat_w)
    return (1 - torch.tanh(pos_error / pos_std)) * (1 - torch.tanh(ori_error / ori_std))
