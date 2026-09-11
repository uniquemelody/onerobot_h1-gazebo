#!/usr/bin/env bash
set -euo pipefail

script_dir=$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
gazebo_root=${script_dir%/scripts}
project_python=$gazebo_root/.venv/bin/python

if (( $# != 2 )); then
  printf 'Usage: %s <export-root> <worlds-root>\n' "${0##*/}" >&2
  exit 2
fi
if [[ ! -x $project_python ]]; then
  printf 'Gazebo project Python is missing or not executable: %s\n' "$project_python" >&2
  exit 1
fi

env -u PYTHONPATH "$project_python" -m onerobotics_a1_gazebo.demo smoke-all \
  --models "$1" --worlds "$2"
