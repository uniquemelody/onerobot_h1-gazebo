#!/usr/bin/env bash
set -euo pipefail

script_dir=$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
gazebo_root=${script_dir%/scripts}
# shellcheck source=harmonic_env.sh
source "$script_dir/harmonic_env.sh"

if (( $# != 0 )); then
  printf '用法：bash %s（不需要参数）\n' "${0##*/}" >&2
  exit 2
fi
if [[ -z ${DISPLAY:-}${WAYLAND_DISPLAY:-} ]]; then
  printf '无法打开 Gazebo：当前终端没有图形桌面 DISPLAY 或 WAYLAND_DISPLAY。\n' >&2
  exit 1
fi

python_bin=${A1_PYTHON_BIN:-$gazebo_root/.venv/bin/python}
if [[ ! -x $python_bin ]]; then
  printf 'Gazebo Python 环境不存在，请先在仓库根目录执行：env -u PYTHONPATH uv sync --project gazebo --locked\n' >&2
  exit 1
fi

session_root=$(mktemp -d "${TMPDIR:-/tmp}/onerobotics-a1-gui.XXXXXX")
cleanup_session() {
  rm -rf -- "$session_root"
}
trap cleanup_session EXIT
models_root=$session_root/models

if ! build_output=$(env -u PYTHONPATH "$python_bin" -m onerobotics_a1_gazebo.package --output "$models_root" 2>&1); then
  printf '模型生成失败：%s\n' "$build_output" >&2
  exit 1
fi
if ! validation_output=$(env -u PYTHONPATH "$python_bin" -m onerobotics_a1_gazebo.validate "$models_root" 2>&1); then
  printf '模型验证失败：%s\n' "$validation_output" >&2
  exit 1
fi
printf '模型数据验证通过。\n'

if ! prefix_state=$(a1_harmonic_prefix_state); then
  exit 1
fi
if [[ $prefix_state != existing ]]; then
  printf 'Gazebo Harmonic 环境尚未安装，请先运行：bash %s/install_harmonic_conda.sh\n' "$script_dir" >&2
  exit 1
fi

mkdir -p "$session_root/home" "$session_root/cache" "$session_root/config" "$session_root/data"
gui_env=(
  "GZ_SIM_RESOURCE_PATH=$models_root"
  "HOME=$session_root/home"
  "XDG_CACHE_HOME=$session_root/cache"
  "XDG_CONFIG_HOME=$session_root/config"
  "XDG_DATA_HOME=$session_root/data"
)
for variable in DISPLAY WAYLAND_DISPLAY XAUTHORITY XDG_RUNTIME_DIR DBUS_SESSION_BUS_ADDRESS; do
  if [[ -v $variable ]]; then
    gui_env+=("$variable=${!variable}")
  fi
done

while true; do
  printf '\n请选择要查看的 Gazebo 模型：\n1. 右臂\n2. 左臂\n3. 双臂站架\n0. 退出\n'
  if ! IFS= read -r -p '请输入 0、1、2 或 3：' choice; then
    printf '\n'
    exit 0
  fi
  case $choice in
    0) exit 0 ;;
    1) world=onerobotics_a1_right_arm_demo.sdf ;;
    2) world=onerobotics_a1_left_arm_demo.sdf ;;
    3) world=onerobotics_a1_bimanual_stand_demo.sdf ;;
    *)
      printf '输入无效，请输入 0、1、2 或 3。\n'
      continue
      ;;
  esac

  set +e
  a1_harmonic_run env "${gui_env[@]}" gz sim --force-version 8 -v 4 "$gazebo_root/worlds/$world"
  gui_status=$?
  set -e
  if (( gui_status != 0 && gui_status != 130 )); then
    printf 'Gazebo 启动失败，退出状态：%s\n' "$gui_status" >&2
    exit "$gui_status"
  fi
done
