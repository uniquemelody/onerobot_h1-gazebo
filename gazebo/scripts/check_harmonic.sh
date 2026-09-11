#!/usr/bin/env bash
set -euo pipefail

_A1_CHECK_SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
# shellcheck source=harmonic_env.sh
source "$_A1_CHECK_SCRIPT_DIR/harmonic_env.sh"
gazebo_root=$(CDPATH= cd -- "$_A1_CHECK_SCRIPT_DIR/.." && pwd -P)

a1_check_error() {
  printf 'HARMONIC_FORMAT_ERROR: %s\n' "$*" >&2
  return 1
}

if (( $# != 1 )); then
  a1_check_error 'usage: check_harmonic.sh <export-root>'
  exit 2
fi
export_root=$1

if [[ -n ${A1_PYTHON_BIN:-} ]]; then
  python_bin=$A1_PYTHON_BIN
elif [[ -x $gazebo_root/.venv/bin/python ]]; then
  python_bin=$gazebo_root/.venv/bin/python
else
  python_bin=$(command -v python3 2>/dev/null || true)
fi
if [[ -z ${python_bin:-} || ! -x $python_bin ]]; then
  a1_check_error 'unable to find the Gazebo tooling Python; set A1_PYTHON_BIN'
  exit 1
fi

temporary_dir=$(mktemp -d "${TMPDIR:-/tmp}/onerobotics-a1-harmonic.XXXXXX")
a1_check_cleanup() {
  rm -rf -- "$temporary_dir"
}
trap a1_check_cleanup EXIT HUP INT TERM

snapshot_root=$temporary_dir/export
if ! snapshot_output=$(
  env -u PYTHONPATH "$python_bin" -m onerobotics_a1_gazebo.harmonic \
    --snapshot-export "$export_root" "$snapshot_root" 2>&1
); then
  a1_check_error "snapshot capture failed: $snapshot_output"
  exit 1
fi
export_root=$snapshot_root

if ! validation_output=$(env -u PYTHONPATH "$python_bin" -m onerobotics_a1_gazebo.validate "$export_root" 2>&1); then
  a1_check_error "pure-data validation failed: $validation_output"
  exit 1
fi

model_slugs=(
  onerobotics_a1_right_arm
  onerobotics_a1_left_arm
  onerobotics_a1_bimanual_stand
)
for slug in "${model_slugs[@]}"; do
  if [[ ! -d $export_root/$slug || -L $export_root/$slug ]]; then
    a1_check_error "validated model directory is unavailable: $slug"
    exit 1
  fi
done
a1_prepare_harmonic_env || exit 1
if [[ $(a1_harmonic_prefix_state) != existing ]]; then
  a1_check_error "Harmonic prefix does not exist: $_A1_HARMONIC_PREFIX"
  exit 1
fi

mkdir -p -- "$temporary_dir/home" "$temporary_dir/xdg/cache" \
  "$temporary_dir/xdg/config" "$temporary_dir/xdg/data"

validated_slugs=()
for slug in "${model_slugs[@]}"; do
  model_dir=$export_root/$slug
  parsed_sdf=$temporary_dir/$slug.parsed.sdf
  generated_config=$temporary_dir/$slug.model.config
  command_log=$temporary_dir/$slug.command.log

  if ! a1_harmonic_run gz sdf --force-version 14 -k "$model_dir/model.sdf" >"$command_log" 2>&1; then
    a1_check_error "native Harmonic command failed for $slug (source SDF check): $(<"$command_log")"
    exit 1
  fi
  if ! a1_harmonic_run gz sdf --force-version 14 -p "$model_dir/model.sdf" \
    >"$parsed_sdf" 2>"$command_log"; then
    a1_check_error "native Harmonic command failed for $slug (SDF parse): $(<"$command_log")"
    exit 1
  fi
  if [[ ! -s $parsed_sdf ]]; then
    a1_check_error "pretty SDF output is empty for $slug"
    exit 1
  fi
  if ! a1_harmonic_run gz sdf --force-version 14 -k "$parsed_sdf" >"$command_log" 2>&1; then
    a1_check_error "native Harmonic command failed for $slug (pretty SDF check): $(<"$command_log")"
    exit 1
  fi
  if ! a1_harmonic_run env \
    "HOME=$temporary_dir/home" \
    "XDG_CACHE_HOME=$temporary_dir/xdg/cache" \
    "XDG_CONFIG_HOME=$temporary_dir/xdg/config" \
    "XDG_DATA_HOME=$temporary_dir/xdg/data" \
    gz fuel --force-version 9 meta --pbtxt2config "$model_dir/metadata.pbtxt" \
    >"$generated_config" 2>"$command_log"; then
    a1_check_error "native Harmonic command failed for $slug (Fuel metadata): $(<"$command_log")"
    exit 1
  fi
  if [[ ! -s $generated_config ]]; then
    a1_check_error "generated model.config output is empty for $slug"
    exit 1
  fi
  if ! env -u PYTHONPATH "$python_bin" -m onerobotics_a1_gazebo.harmonic \
    "$model_dir/model.sdf" "$parsed_sdf" "$slug"; then
    exit 1
  fi
  validated_slugs+=("$slug")
done
for slug in "${validated_slugs[@]}"; do
  printf 'HARMONIC_FORMAT_VALID %s\n' "$slug"
done
printf 'HARMONIC_FORMAT_OK: 3 models\n'
