# Beginner Gazebo Launcher and Documentation Design

Date: 2026-09-01

## Context

The OneRobotics A1 Gazebo integration is developed on the local
`feat/gazebo-harmonic-fuel-assets` branch of
`https://github.com/katazen/onerobot_h1.git`. This is intentionally different
from the `T1Amoo/IsaacLab` contribution branch: Isaac Lab robot and Reach-task
configuration belongs in the IsaacLab repository, while ordinary robot assets
for Gazebo do not belong in the Gazebo simulator-engine repository. They stay
with the vendor asset source and may later be published as separate Gazebo Fuel
resources.

The current guide exposes the safe low-level GUI setup, but a beginner should
not have to reproduce that shell state manually. The user selected an
interactive one-command launcher and then attempted the proposed command, which
confirms the interface choice.

## Goals

1. `bash /home/woan/桌面/onerobot_h1-gazebo/gazebo/scripts/open_demo.sh`
   works from any current directory on this configured machine.
2. The launcher presents a Chinese menu for right arm, left arm, bimanual
   stand, and exit; closing one Gazebo window returns to the menu.
3. Each launch uses a fresh deterministic model export, validates it before
   native execution, selects Gazebo Sim major version 8, resolves only the
   local export, preserves only the required display-session variables, and
   isolates HOME/XDG state.
4. The beginner README explains repository/branch relationships, one-time
   setup, the one launcher command, the visual checklist, automated validation,
   current limitations, and the fact that nothing has been uploaded.
5. Advanced Fuel approval/publication material remains available without being
   mixed into the beginner path.
6. The original acceptance requirement is restored by testing conservative
   commanded motion for both the right and left halves of the bimanual stand.

## Non-goals

- Do not modify or fork the Gazebo simulator engine.
- Do not reproduce the Isaac Lab Reach environment, rewards, PPO training, or
  policy playback in Gazebo.
- Do not install packages, use sudo, change shell/Conda configuration, contact
  Fuel, upload, push, merge, or request credentials.
- Do not hide a missing Python environment, Mamba environment, or graphical
  display by performing an implicit installation.
- The GUI is a visual load/render check, not proof of every physical property
  and not proof of online publication.

## Launcher interface

Create executable `gazebo/scripts/open_demo.sh`. It accepts no arguments and
prints this loop:

```text
请选择要查看的 Gazebo 模型：
1. 右臂
2. 左臂
3. 双臂站架
0. 退出
请输入 0、1、2 或 3：
```

`1`, `2`, and `3` map only to the three checked-in world filenames. Any other
input prints a short correction and displays the menu again. EOF and `0` exit
successfully. The script resolves its repository from `BASH_SOURCE[0]`, so the
caller's current directory is irrelevant.

Before showing the menu, the launcher:

1. requires `.venv/bin/python` (or the existing test-only `A1_PYTHON_BIN`
   convention) to be executable;
2. requires `DISPLAY` or `WAYLAND_DISPLAY`;
3. creates one private `mktemp` session root;
4. exports the three packages into that private root and runs the strict Python
   validator against the same export;
5. prepares the existing dedicated Harmonic Mamba environment and requires its
   prefix state to be `existing`.

For each selection it calls the existing `a1_harmonic_run` helper with fixed
inner environment values:

- `GZ_SIM_RESOURCE_PATH=<private validated export>`;
- private `HOME`, `XDG_CACHE_HOME`, `XDG_CONFIG_HOME`, and `XDG_DATA_HOME`;
- only present values among `DISPLAY`, `WAYLAND_DISPLAY`, `XAUTHORITY`,
  `XDG_RUNTIME_DIR`, and `DBUS_SESSION_BUS_ADDRESS`.

The fixed command is `gz sim --force-version 8 -v 4 <absolute world path>`.
Exit status 0 or 130 means the window closed normally and returns to the menu;
any other status exits nonzero with a concise Chinese error. A trap removes
only the exact private `mktemp` session root on exit or interruption. No user
Gazebo, Fuel, HOME, or XDG configuration is mutated.

## Documentation split

Rewrite `gazebo/README.md` as the beginner entry point, targeting roughly one
screen of concepts followed by four short sections: one-time setup, open a
model, automated verification, and scope/status. It must clearly say that a
fresh GitHub checkout currently lacks this unpushed local branch.

Move the existing thumbnail approval, authorization gate, manifest-bound
snapshot, secure-client handoff, and post-publication verification material to
`gazebo/PUBLISHING.md`. Preserve its fail-closed commands and warnings; do not
restore authenticated Fuel Tools 9 upload/download commands. The beginner
guide links to it only for an organization-authorized maintainer.

## Complete bimanual motion acceptance

Extend the live case table with
`("onerobotics_a1_bimanual_stand", "joint_l1", 0.05)`. Keep the existing
right-half case. Both targets are strictly inside their source limits and use
the same bounded process lifecycle, finite-state checks, minimum `0.02 rad`
motion delta, and cleanup requirements. User-facing success text must count
four motion cases across three models rather than calling four lines “models.”

The strict-cache aggregate uses the same live case table and therefore must
also run four cases within its existing total deadline; increase that deadline
only if a real quiet serial run proves the current bound insufficient.

## Testing and verification

Tests are written and observed failing before implementation. They cover:

- the launcher exists, is executable, strict Bash, and cwd-independent;
- menu choices map exactly to the three absolute checked-in worlds;
- invalid input retries and `0`/EOF perform no native launch;
- a missing display, Python tool environment, or Harmonic prefix fails before
  Gazebo execution with an actionable message;
- the native call uses exact forced major 8, the private validated resource
  root, isolated HOME/XDG state, and only the display-session allow-list;
- Ctrl+C status 130 returns to a live menu rather than killing the shell;
- no global mutation, network, credential, upload, or unpinned `gz sim` command
  is introduced;
- both bimanual halves appear in unit/runtime/cache contracts;
- the beginner guide contains only the short local path and the advanced guide
  retains every publication safety contract.

After focused tests pass, run full Ruff/format/Bash/documentation/non-runtime
gates, all four real commanded-motion cases, a real menu-driven GUI launch on
the available desktop, the strict-cache motion aggregate, source immutability,
`git diff --check`, and residual-process checks. No push or upload occurs.
