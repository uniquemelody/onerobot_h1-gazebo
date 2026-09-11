#!/usr/bin/env bash
set -euo pipefail

_A1_INSTALL_SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
# shellcheck source=harmonic_env.sh
source "$_A1_INSTALL_SCRIPT_DIR/harmonic_env.sh"

environment_file=$(CDPATH= cd -- "$_A1_INSTALL_SCRIPT_DIR/.." && pwd -P)/environment-harmonic.yml

a1_install_error() {
  printf 'HARMONIC_INSTALL_ERROR: %s\n' "$*" >&2
  return 1
}

a1_require_prefix_state() {
  local current expected=$1
  if ! current=$(a1_harmonic_prefix_state); then
    a1_install_error 'the Harmonic prefix became unsafe before the environment transaction'
    return 1
  fi
  if [[ $current != "$expected" ]]; then
    a1_install_error "the Harmonic prefix changed before the environment transaction: expected $expected, got $current"
    return 1
  fi
}

a1_classic_snapshot() {
  local gazebo_output gazebo_sha gz_sha gz_target version_line
  if [[ -e /usr/bin/gz || -L /usr/bin/gz ]]; then
    gz_target=$(readlink -f -- /usr/bin/gz) || return 1
    gz_sha=$(sha256sum -- /usr/bin/gz) || return 1
    gz_sha=${gz_sha%% *}
  else
    gz_target='<missing>'
    gz_sha='<missing>'
  fi

  if [[ -e /usr/bin/gazebo || -L /usr/bin/gazebo ]]; then
    gazebo_sha=$(sha256sum -- /usr/bin/gazebo) || return 1
    gazebo_sha=${gazebo_sha%% *}
    if [[ -x /usr/bin/gazebo ]]; then
      gazebo_output=$(/usr/bin/gazebo --version 2>&1) || true
      if [[ -z $gazebo_output ]]; then
        return 1
      fi
      version_line=${gazebo_output%%$'\n'*}
    else
      version_line='<not-executable>'
    fi
  else
    gazebo_sha='<missing>'
    version_line='<missing>'
  fi

  printf 'gz_target=%s\ngz_sha256=%s\ngazebo_sha256=%s\nclassic_version=%s\n' \
    "$gz_target" "$gz_sha" "$gazebo_sha" "$version_line"
}

a1_print_classic_snapshot() {
  local snapshot=$1
  local line
  while IFS= read -r line; do
    case "$line" in
      gz_target=*) printf 'CLASSIC_GZ_TARGET %s\n' "${line#*=}" ;;
      gz_sha256=*) printf 'CLASSIC_GZ_SHA256 %s\n' "${line#*=}" ;;
      gazebo_sha256=*) printf 'CLASSIC_GAZEBO_SHA256 %s\n' "${line#*=}" ;;
      classic_version=*) printf 'CLASSIC_VERSION %s\n' "${line#*=}" ;;
    esac
  done <<< "$snapshot"
}

a1_probe_component() {
  local component=$1
  local major=$2
  local output pattern
  if ! output=$(a1_harmonic_run gz "$component" --force-version "$major" --versions 2>&1); then
    printf '%s expected major %s, but its version probe failed: %s\n' "$component" "$major" "$output" >&2
    return 1
  fi
  pattern="^[[:space:]]*(${major}([.][0-9]+)+)[[:space:]]*$"
  if [[ ! $output =~ $pattern ]]; then
    printf '%s expected major %s, but probe output was: %s\n' "$component" "$major" "$output" >&2
    return 1
  fi
  printf '%s\n' "${BASH_REMATCH[1]}"
}

a1_probe_all() {
  local fuel_version sdf_version sim_version
  sim_version=$(a1_probe_component sim 8) || return 1
  sdf_version=$(a1_probe_component sdf 14) || return 1
  fuel_version=$(a1_probe_component fuel 9) || return 1
  printf 'HARMONIC_COMPONENT sim %s\n' "$sim_version"
  printf 'HARMONIC_COMPONENT sdf %s\n' "$sdf_version"
  printf 'HARMONIC_COMPONENT fuel %s\n' "$fuel_version"
}

if [[ $(uname -s) != Linux || $(uname -m) != x86_64 ]]; then
  a1_install_error 'this installer supports only Linux x86-64'
  exit 1
fi
if [[ ! -r $environment_file ]]; then
  a1_install_error "missing environment specification: $environment_file"
  exit 1
fi
a1_validate_env_name || exit 1
a1_prepare_harmonic_env || exit 1
prefix_state=$(a1_harmonic_prefix_state) || exit 1
classic_before=$(a1_classic_snapshot) || {
  a1_install_error 'unable to record the Gazebo Classic state'
  exit 1
}

transaction_started=0
classic_verified=0
a1_install_exit_guard() {
  local after status=$?
  trap - EXIT
  if (( transaction_started && ! classic_verified )); then
    if ! after=$(a1_classic_snapshot) || [[ $after != "$classic_before" ]]; then
      printf 'HARMONIC_INSTALL_ERROR: Gazebo Classic changed during the environment transaction\n' >&2
      status=1
    else
      a1_print_classic_snapshot "$after"
      printf 'CLASSIC_UNCHANGED\n'
    fi
  fi
  exit "$status"
}
trap a1_install_exit_guard EXIT

if [[ $prefix_state == existing ]] && versions=$(a1_probe_all 2>/dev/null); then
  classic_after=$(a1_classic_snapshot) || {
    a1_install_error 'unable to verify the Gazebo Classic state'
    exit 1
  }
  if [[ $classic_after != "$classic_before" ]]; then
    a1_install_error 'Gazebo Classic changed while probing Harmonic'
    exit 1
  fi
  classic_verified=1
  printf '%s\n' "$versions"
  printf 'HARMONIC_ALREADY_READY\n'
  a1_print_classic_snapshot "$classic_after"
  printf 'CLASSIC_UNCHANGED\n'
  exit 0
elif [[ $prefix_state == existing ]]; then
  a1_require_prefix_state existing || exit 1
  transaction_started=1
  a1_invoke_mamba update --yes --file "$environment_file"
else
  a1_require_prefix_state absent || exit 1
  transaction_started=1
  a1_invoke_mamba create --yes \
    --file "$environment_file" --override-channels --strict-channel-priority --channel conda-forge
fi

if ! versions=$(a1_probe_all); then
  a1_install_error 'the Harmonic environment does not provide every required component major'
  exit 1
fi
classic_after=$(a1_classic_snapshot) || {
  a1_install_error 'unable to verify the Gazebo Classic state after the environment transaction'
  exit 1
}
if [[ $classic_after != "$classic_before" ]]; then
  a1_install_error 'Gazebo Classic changed during the environment transaction'
  exit 1
fi
classic_verified=1
printf '%s\n' "$versions"
a1_print_classic_snapshot "$classic_after"
printf 'CLASSIC_UNCHANGED\n'
