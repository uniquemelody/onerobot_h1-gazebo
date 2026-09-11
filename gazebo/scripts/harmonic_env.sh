#!/usr/bin/env bash
# Source-only helper for running the isolated Gazebo Harmonic environment.
set -euo pipefail

_A1_HARMONIC_HELPER_DIR=$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
A1_HARMONIC_ENV_NAME=${A1_HARMONIC_ENV_NAME:-onerobotics-a1-gz-harmonic}
_A1_HARMONIC_MAMBA_BIN=
_A1_HARMONIC_MAMBA_ROOT=
_A1_HARMONIC_PREFIX=

a1_validate_env_name() {
  local name=$A1_HARMONIC_ENV_NAME
  if (( ${#name} > 64 )) || [[ ! $name =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] \
    || [[ $name == base || $name == root ]]; then
    printf 'A1_HARMONIC_ENV_NAME must be a safe dedicated environment name, got: %s\n' "$name" >&2
    return 1
  fi
}

a1_find_mamba() {
  local candidate
  if [[ -n ${A1_MAMBA_BIN:-} ]]; then
    if [[ ! -x $A1_MAMBA_BIN ]]; then
      printf 'A1_MAMBA_BIN is not an executable mamba path: %s\n' "$A1_MAMBA_BIN" >&2
      return 1
    fi
    printf '%s\n' "$A1_MAMBA_BIN"
    return 0
  fi

  if candidate=$(command -v mamba 2>/dev/null) && [[ -x $candidate ]]; then
    printf '%s\n' "$candidate"
    return 0
  fi

  if [[ -n ${HOME:-} ]]; then
    for candidate in "$HOME/miniforge3/bin/mamba" "$HOME/mambaforge/bin/mamba"; do
      if [[ -x $candidate ]]; then
        printf '%s\n' "$candidate"
        return 0
      fi
    done
  fi

  printf 'Unable to find mamba; set A1_MAMBA_BIN to its executable path.\n' >&2
  return 1
}

a1_validate_prepared_harmonic_env() {
  local bin_directory envs_directory resolved root
  if [[ -z $_A1_HARMONIC_MAMBA_BIN || -z $_A1_HARMONIC_MAMBA_ROOT || -z $_A1_HARMONIC_PREFIX ]]; then
    printf 'The Harmonic environment paths have not been prepared.\n' >&2
    return 1
  fi
  if ! resolved=$(readlink -f -- "$_A1_HARMONIC_MAMBA_BIN" 2>/dev/null); then
    printf 'The prepared mamba executable is no longer resolvable: %s\n' "$_A1_HARMONIC_MAMBA_BIN" >&2
    return 1
  fi
  bin_directory=${resolved%/*}
  root=${bin_directory%/*}
  if [[ $resolved != "$_A1_HARMONIC_MAMBA_BIN" || -L $resolved || ! -f $resolved || ! -x $resolved \
    || ${resolved##*/} != mamba || -L $bin_directory || ! -d $bin_directory \
    || ${bin_directory##*/} != bin || $root != "$_A1_HARMONIC_MAMBA_ROOT" || $root == / \
    || -L $root || ! -d $root || ! -r $root || ! -x $root ]]; then
    printf 'The prepared mamba root is no longer safe: %s\n' "$_A1_HARMONIC_MAMBA_ROOT" >&2
    return 1
  fi

  envs_directory=$root/envs
  if [[ -L $envs_directory ]]; then
    printf 'Mamba envs directory must not be a symlink: %s\n' "$envs_directory" >&2
    return 1
  fi
  if [[ -e $envs_directory ]]; then
    if [[ ! -d $envs_directory || ! -r $envs_directory || ! -w $envs_directory || ! -x $envs_directory ]]; then
      printf 'Mamba envs directory is not a writable accessible directory: %s\n' "$envs_directory" >&2
      return 1
    fi
  elif [[ ! -w $root ]]; then
    printf 'Mamba root cannot create its envs directory: %s\n' "$root" >&2
    return 1
  fi
}

a1_prepare_harmonic_env() {
  local bin_directory candidate envs_directory resolved root
  if [[ -n $_A1_HARMONIC_MAMBA_BIN ]]; then
    a1_validate_prepared_harmonic_env
    return
  fi

  a1_validate_env_name || return 1
  candidate=$(a1_find_mamba) || return 1
  if ! resolved=$(readlink -f -- "$candidate" 2>/dev/null); then
    printf 'Unable to resolve the selected mamba executable: %s\n' "$candidate" >&2
    return 1
  fi
  bin_directory=${resolved%/*}
  root=${bin_directory%/*}
  if [[ ! -f $resolved || ! -x $resolved || ${resolved##*/} != mamba \
    || ${bin_directory##*/} != bin || -z $root || $root == / || ! -d $root \
    || ! -r $root || ! -x $root ]]; then
    printf 'Selected mamba must resolve to a safe <root>/bin/mamba executable: %s\n' "$candidate" >&2
    return 1
  fi

  envs_directory=$root/envs
  if [[ -L $envs_directory ]]; then
    printf 'Mamba envs directory must not be a symlink: %s\n' "$envs_directory" >&2
    return 1
  fi
  if [[ -e $envs_directory ]]; then
    if [[ ! -d $envs_directory || ! -r $envs_directory || ! -w $envs_directory || ! -x $envs_directory ]]; then
      printf 'Mamba envs directory is not a writable accessible directory: %s\n' "$envs_directory" >&2
      return 1
    fi
  elif [[ ! -w $root ]]; then
    printf 'Mamba root cannot create its envs directory: %s\n' "$root" >&2
    return 1
  fi

  _A1_HARMONIC_MAMBA_BIN=$resolved
  _A1_HARMONIC_MAMBA_ROOT=$root
  _A1_HARMONIC_PREFIX=$envs_directory/$A1_HARMONIC_ENV_NAME
  a1_validate_prepared_harmonic_env
}

a1_harmonic_prefix_state() {
  local envs_directory
  a1_prepare_harmonic_env || return 1
  envs_directory=${_A1_HARMONIC_PREFIX%/*}

  if [[ -L $envs_directory || ( -e $envs_directory && ! -d $envs_directory ) \
    || ( -e $envs_directory && ( ! -r $envs_directory || ! -w $envs_directory || ! -x $envs_directory ) ) ]]; then
    printf 'Cannot determine a safe Harmonic prefix below: %s\n' "$envs_directory" >&2
    return 1
  fi
  if [[ -L $_A1_HARMONIC_PREFIX ]]; then
    printf 'Harmonic prefix must not be a symlink: %s\n' "$_A1_HARMONIC_PREFIX" >&2
    return 1
  fi
  if [[ -e $_A1_HARMONIC_PREFIX ]]; then
    if [[ ! -d $_A1_HARMONIC_PREFIX || ! -r $_A1_HARMONIC_PREFIX \
      || ! -w $_A1_HARMONIC_PREFIX || ! -x $_A1_HARMONIC_PREFIX ]]; then
      printf 'Harmonic prefix is not a writable accessible directory: %s\n' "$_A1_HARMONIC_PREFIX" >&2
      return 1
    fi
    printf 'existing\n'
    return 0
  fi

  printf 'absent\n'
}

a1_invoke_mamba() {
  local expected_state invocation=${1:-} prefix_state
  if (( $# == 0 )); then
    printf 'A Mamba invocation kind is required.\n' >&2
    return 1
  fi
  shift
  case "$invocation" in
    run | update) expected_state=existing ;;
    create) expected_state=absent ;;
    *)
      printf 'Unsupported Mamba invocation kind: %s\n' "$invocation" >&2
      return 1
      ;;
  esac

  a1_prepare_harmonic_env || return 1
  prefix_state=$(a1_harmonic_prefix_state) || return 1
  if [[ $prefix_state != "$expected_state" ]]; then
    printf 'Harmonic prefix state changed before Mamba %s: expected %s, got %s\n' \
      "$invocation" "$expected_state" "$prefix_state" >&2
    return 1
  fi

  if [[ $invocation == run ]]; then
    env -i "$_A1_HARMONIC_MAMBA_BIN" --no-rc --no-env run \
      --root-prefix "$_A1_HARMONIC_MAMBA_ROOT" --clean-env \
      --prefix "$_A1_HARMONIC_PREFIX" "$@"
  else
    env -i "$_A1_HARMONIC_MAMBA_BIN" --no-rc --no-env env "$invocation" \
      --root-prefix "$_A1_HARMONIC_MAMBA_ROOT" --prefix "$_A1_HARMONIC_PREFIX" "$@"
  fi
}

a1_harmonic_run() {
  a1_invoke_mamba run "$@"
}
