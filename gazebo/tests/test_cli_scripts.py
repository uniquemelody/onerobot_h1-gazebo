from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest
import yaml
from onerobotics_a1_gazebo.package import export_models

GAZEBO_ROOT = Path(__file__).resolve().parents[1]
ENVIRONMENT = GAZEBO_ROOT / "environment-harmonic.yml"
SCRIPTS = GAZEBO_ROOT / "scripts"
HELPER = SCRIPTS / "harmonic_env.sh"
INSTALLER = SCRIPTS / "install_harmonic_conda.sh"
CHECKER = SCRIPTS / "check_harmonic.sh"
OPEN_DEMO = SCRIPTS / "open_demo.sh"
SCRIPT_PATHS = (HELPER, INSTALLER, CHECKER, OPEN_DEMO)
ENV_NAME = "onerobotics-a1-gz-harmonic"


def _write_fake_mamba(root: Path) -> tuple[Path, Path, Path]:
    bin_directory = root / "bin"
    (root / "envs").mkdir(parents=True)
    bin_directory.mkdir()
    executable = bin_directory / "mamba"
    log = root / "mamba calls.tsv"
    state = root / "environment state"
    (root / "control").mkdir()
    executable.write_text(
        """#!/usr/bin/env bash
set -euo pipefail
self=$(readlink -f -- "$0")
bin_directory=${self%/*}
fake_root=${bin_directory%/*}
control_directory=$fake_root/control
fake_mamba_log=$fake_root/'mamba calls.tsv'
fake_mamba_state=$fake_root/'environment state'

read_control() {
  local path=$control_directory/$1 value=
  if [[ -f $path ]]; then
    IFS= read -r value < "$path" || [[ -n $value ]]
  fi
  printf '%s' "$value"
}

expected_root=$(read_control expected_root)
expected_prefix=$(read_control expected_prefix)
mode=$(read_control mode)
mode=${mode:-ready}
for ambient_name in \
  MAMBA_ROOT_PREFIX CONDA_ENVS_PATH CONDARC MAMBARC CONDA_PKGS_DIRS \
  ARBITRARY_HOSTILE_CONFIG HOSTILE_AMBIENT_MARKER; do
  [[ -z ${!ambient_name+x} ]] || exit 87
done
{
  printf 'CALL'
  for argument in "$@"; do
    printf '\\t%s' "$argument"
  done
  printf '\\n'
} >> "$fake_mamba_log"

joined=" $* "
fail_match=$(read_control fail_match)
if [[ -n $fail_match && $joined == *"$fail_match"* ]]; then
  exit 42
fi
mutate_export_root=$(read_control mutate_export_root)
mutation_marker=$(read_control mutation_marker)
if [[ -n $mutate_export_root && ! -e $mutation_marker ]]; then
  target=$mutate_export_root/onerobotics_a1_right_arm/model.sdf
  sed -i '0,/<mass>/{s/<mass>[^<]*<\\/mass>/<mass>999<\\/mass>/}' -- "$target"
  : > "$mutation_marker"
fi

[[ ${1:-} == --no-rc ]] || exit 89
shift
[[ ${1:-} == --no-env ]] || exit 90
shift

if [[ ${1:-} == run ]]; then
  shift
  [[ ${1:-} == --root-prefix ]] || exit 91
  shift
  [[ ${1:-} == "$expected_root" ]] || exit 92
  shift
  [[ ${1:-} == --clean-env ]] || exit 93
  shift
  [[ ${1:-} == --prefix ]] || exit 94
  shift
  [[ ${1:-} == "$expected_prefix" ]] || exit 95
  shift
  if [[ $mode == missing && ! -f $fake_mamba_state ]]; then
    printf 'environment does not exist\\n' >&2
    exit 1
  fi
  if [[ $mode == run_broken && ! -f $fake_mamba_state ]]; then
    printf 'existing environment is corrupt\\n' >&2
    exit 1
  fi
  if [[ $mode == swap_prefix && ! -f $fake_mamba_state ]]; then
    swap_destination=$(read_control swap_destination)
    [[ -n $swap_destination ]] || exit 110
    rmdir -- "$expected_prefix"
    ln -s -- "$swap_destination" "$expected_prefix"
    printf 'probe replaced the environment prefix\\n' >&2
    exit 1
  fi
  if [[ ${1:-} == env ]]; then
    shift
    while [[ ${1:-} == *=* ]]; do
      shift
    done
  fi
  [[ ${1:-} == gz ]] || exit 93
  component=${2:-}
  empty_kind=$(read_control empty_kind)
  mutate_pretty_slug=$(read_control mutate_pretty_slug)
  if [[ $component == sdf && " $* " == *" -p "* ]]; then
    if [[ $empty_kind != pretty ]]; then
      source_path=${!#}
      if [[ -n $mutate_pretty_slug && $source_path == *"$mutate_pretty_slug"* ]]; then
        sed 's/canonical_link="base_link"/canonical_link="mutated"/' -- "$source_path"
      else
        cat -- "$source_path"
      fi
    fi
    exit
  fi
  if [[ $component == fuel && " $* " == *" meta "* ]]; then
    if [[ $empty_kind != fuel ]]; then
      printf '<model><name>converted</name></model>\\n'
    fi
    exit
  fi
  if [[ $component == sdf && " $* " == *" -k "* ]]; then
    exit
  fi

  ready=1
  if [[ $mode == stale && ! -f $fake_mamba_state ]]; then
    ready=0
  elif [[ $mode == wrong ]]; then
    ready=0
  fi
  missing_component=$(read_control missing_component)
  missing_component=${missing_component:-sdf}
  if [[ $mode == missing_component && $component == $missing_component ]]; then
    printf 'component unavailable\\n'
    exit
  fi
  if [[ $mode == misleading ]]; then
    case "$component" in
      sim) printf '7.9.0 (dependency 8.0.0)\\n' ;;
      sdf) printf '13.8.0 (dependency 14.0.0)\\n' ;;
      fuel) printf '8.0.0 (dependency 9.0.0)\\n' ;;
      *) exit 97 ;;
    esac
    exit
  fi
  if (( ready )); then
    case "$component" in
      sim) printf '8.10.0\\n' ;;
      sdf) printf '14.7.0\\n' ;;
      fuel) printf '9.1.1\\n' ;;
      *) exit 94 ;;
    esac
  else
    case "$component" in
      sim) printf '7.9.0\\n' ;;
      sdf) printf '13.8.0\\n' ;;
      fuel) printf '8.0.0\\n' ;;
      *) exit 95 ;;
    esac
  fi
  exit
fi

if [[ ${1:-} == env && ${2:-} == create ]]; then
  shift 2
  [[ ${1:-} == --root-prefix ]] || exit 101
  [[ ${2:-} == "$expected_root" ]] || exit 102
  shift 2
  [[ ${1:-} == --prefix ]] || exit 103
  [[ ${2:-} == "$expected_prefix" ]] || exit 104
  mkdir -p -- "$expected_prefix"
  : > "$fake_mamba_state"
  exit
fi
if [[ ${1:-} == env && ${2:-} == update ]]; then
  shift 2
  [[ ${1:-} == --root-prefix ]] || exit 105
  [[ ${2:-} == "$expected_root" ]] || exit 106
  shift 2
  [[ ${1:-} == --prefix ]] || exit 107
  [[ ${2:-} == "$expected_prefix" ]] || exit 108
  : > "$fake_mamba_state"
  exit
fi
exit 109
""",
        encoding="utf-8",
    )
    executable.chmod(0o755)
    return executable, log, state


def _calls(log: Path) -> list[list[str]]:
    if not log.exists():
        return []
    calls: list[list[str]] = []
    for line in log.read_text(encoding="utf-8").splitlines():
        fields = line.split("\t")
        assert fields[0] == "CALL"
        calls.append(fields[1:])
    return calls


def _set_fake_control(environment: dict[str, str], name: str, value: str) -> None:
    root = Path(environment["FAKE_EXPECTED_ROOT"])
    (root / "control" / name).write_text(value, encoding="utf-8")


def _add_hostile_mamba_bootstrap_environment(environment: dict[str, str]) -> None:
    environment.update(
        {
            "MAMBA_ROOT_PREFIX": "/dev/null",
            "CONDA_ENVS_PATH": "/dev/null",
            "CONDARC": "/dev/null",
            "MAMBARC": "/dev/null",
            "CONDA_PKGS_DIRS": "/dev/null",
            "ARBITRARY_HOSTILE_CONFIG": "/dev/null",
            "HOSTILE_AMBIENT_MARKER": "must-not-reach-mamba",
        }
    )


def _fake_environment(tmp_path: Path, mode: str = "ready") -> tuple[dict[str, str], Path, Path]:
    root = tmp_path / "fake root with spaces"
    fake, log, state = _write_fake_mamba(root)
    prefix = root / "envs" / ENV_NAME
    if mode != "missing":
        prefix.mkdir()
    environment = os.environ.copy()
    environment.update(
        {
            "A1_MAMBA_BIN": str(fake),
            "FAKE_EXPECTED_ROOT": str(root),
            "FAKE_EXPECTED_PREFIX": str(prefix),
        }
    )
    _set_fake_control(environment, "expected_root", str(root))
    _set_fake_control(environment, "expected_prefix", str(prefix))
    _set_fake_control(environment, "mode", mode)
    return environment, log, state


def _run_installer(tmp_path: Path, mode: str = "ready") -> tuple[subprocess.CompletedProcess[str], list[list[str]]]:
    environment, log, _ = _fake_environment(tmp_path, mode)
    result = subprocess.run(
        ["bash", str(INSTALLER)],
        cwd=tmp_path,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    return result, _calls(log)


def _transaction(calls: list[list[str]]) -> list[list[str]]:
    return [
        call
        for call in calls
        if any(call[index : index + 2] in (["env", "create"], ["env", "update"]) for index in range(len(call)))
    ]


def _transaction_kind(call: list[str]) -> str:
    for index in range(len(call)):
        if call[index : index + 2] in (["env", "create"], ["env", "update"]):
            return call[index + 1]
    raise AssertionError(f"not a transaction: {call}")


def _run_calls(calls: list[list[str]]) -> list[list[str]]:
    return [call for call in calls if "run" in call]


def _run_payload(call: list[str]) -> list[str]:
    run_index = call.index("run")
    prefix_index = call.index("--prefix", run_index)
    return call[prefix_index + 2 :]


def _assert_pinned_call(call: list[str], environment: dict[str, str]) -> None:
    assert call[:2] == ["--no-rc", "--no-env"]
    assert call[call.index("--root-prefix") + 1] == environment["FAKE_EXPECTED_ROOT"]
    assert call[call.index("--prefix") + 1] == environment["FAKE_EXPECTED_PREFIX"]


def test_environment_file_is_the_exact_single_dependency_contract() -> None:
    document = yaml.safe_load(ENVIRONMENT.read_text(encoding="utf-8"))

    assert document == {
        "name": ENV_NAME,
        "channels": ["conda-forge", "nodefaults"],
        "dependencies": [
            "gz-tools2=2.*",
            "gz-sim8=8.*",
            "gz-fuel-tools9=9.*",
            "libsdformat14=14.*",
        ],
    }


@pytest.mark.parametrize("script", SCRIPT_PATHS)
def test_shell_scripts_are_strict_and_parse_as_bash(script: Path) -> None:
    text = script.read_text(encoding="utf-8")

    assert "set -euo pipefail" in text
    subprocess.run(["bash", "-n", str(script)], check=True)


def test_scripts_resolve_their_own_paths_and_helper_is_source_only() -> None:
    for script in SCRIPT_PATHS:
        text = script.read_text(encoding="utf-8")
        assert "BASH_SOURCE[0]" in text
    helper = HELPER.read_text(encoding="utf-8")
    assert "a1_find_mamba" in helper
    assert "a1_harmonic_run" in helper
    assert "source-only" in helper.lower()


@pytest.mark.parametrize("script", SCRIPT_PATHS)
def test_scripts_contain_no_forbidden_global_mutation_or_fuel_network_commands(script: Path) -> None:
    lowered = script.read_text(encoding="utf-8").lower()
    forbidden = (
        "sudo",
        "apt-get",
        "add-apt-repository",
        "conda init",
        "conda activate",
        "mamba activate",
        "conda config",
        "--prune",
        "gz fuel upload",
        "gz fuel download",
        "gz fuel list",
        "gz fuel config",
        "private-token",
        "fuel_token",
    )
    assert not [token for token in forbidden if token in lowered]


def test_installer_creates_missing_environment_from_checked_in_yaml(tmp_path: Path) -> None:
    result, calls = _run_installer(tmp_path, "missing")

    assert result.returncode == 0, result.stderr
    transactions = _transaction(calls)
    assert len(transactions) == 1
    assert _transaction_kind(transactions[0]) == "create"
    assert "--no-rc" in transactions[0]
    assert "--no-env" in transactions[0]
    assert "--override-channels" in transactions[0]
    assert "--strict-channel-priority" in transactions[0]
    assert transactions[0][transactions[0].index("--file") + 1] == str(ENVIRONMENT)
    assert "CLASSIC_UNCHANGED" in result.stdout


def test_installer_updates_existing_stale_environment_without_pruning(tmp_path: Path) -> None:
    result, calls = _run_installer(tmp_path, "stale")

    assert result.returncode == 0, result.stderr
    transactions = _transaction(calls)
    assert len(transactions) == 1
    assert _transaction_kind(transactions[0]) == "update"
    assert "--no-rc" in transactions[0]
    assert "--no-env" in transactions[0]
    assert "--prune" not in transactions[0]
    assert transactions[0][transactions[0].index("--file") + 1] == str(ENVIRONMENT)


def test_installer_ready_path_performs_no_transaction(tmp_path: Path) -> None:
    result, calls = _run_installer(tmp_path, "ready")

    assert result.returncode == 0, result.stderr
    assert _transaction(calls) == []
    assert "HARMONIC_ALREADY_READY" in result.stdout
    assert "CLASSIC_UNCHANGED" in result.stdout


def test_installer_pins_one_prefix_despite_hostile_mamba_environment_and_config(tmp_path: Path) -> None:
    environment, log, _ = _fake_environment(tmp_path, "ready")
    _add_hostile_mamba_bootstrap_environment(environment)

    result = subprocess.run(
        ["bash", str(INSTALLER)],
        cwd=tmp_path,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    calls = _calls(log)

    assert result.returncode == 0, result.stderr
    assert calls
    assert _transaction(calls) == []
    for call in calls:
        _assert_pinned_call(call, environment)


@pytest.mark.parametrize(
    ("mode", "expected_transaction", "expected_call_count"), [("missing", "create", 4), ("stale", "update", 5)]
)
def test_installer_transactions_start_mamba_with_an_empty_environment(
    tmp_path: Path, mode: str, expected_transaction: str, expected_call_count: int
) -> None:
    environment, log, _ = _fake_environment(tmp_path, mode)
    _add_hostile_mamba_bootstrap_environment(environment)

    result = subprocess.run(
        ["bash", str(INSTALLER)],
        cwd=tmp_path,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    calls = _calls(log)
    assert len(calls) == expected_call_count
    transactions = _transaction(calls)
    assert [_transaction_kind(call) for call in transactions] == [expected_transaction]


def test_installer_updates_existing_directory_even_when_every_run_fails(tmp_path: Path) -> None:
    result, calls = _run_installer(tmp_path, "run_broken")

    assert result.returncode == 0, result.stderr
    transactions = _transaction(calls)
    assert [_transaction_kind(call) for call in transactions] == ["update"]
    assert all(_run_payload(call) != ["true"] for call in _run_calls(calls))


def test_installer_revalidates_prefix_after_failed_probe_before_update(tmp_path: Path) -> None:
    environment, log, _ = _fake_environment(tmp_path, "swap_prefix")
    destination = tmp_path / "redirected after probe"
    destination.mkdir()
    _set_fake_control(environment, "swap_destination", str(destination))

    result = subprocess.run(
        ["bash", str(INSTALLER)],
        cwd=tmp_path,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode != 0
    assert "prefix" in result.stderr.lower()
    assert Path(environment["FAKE_EXPECTED_PREFIX"]).is_symlink()
    assert _transaction(_calls(log)) == []


@pytest.mark.parametrize("unsafe_kind", ["symlink", "file", "inaccessible"])
def test_installer_rejects_unsafe_exact_prefix_before_any_mamba_call(tmp_path: Path, unsafe_kind: str) -> None:
    environment, log, _ = _fake_environment(tmp_path, "ready")
    prefix = Path(environment["FAKE_EXPECTED_PREFIX"])
    prefix.rmdir()
    restore_permissions = False
    if unsafe_kind == "symlink":
        destination = tmp_path / "redirected environment"
        destination.mkdir()
        prefix.symlink_to(destination, target_is_directory=True)
    elif unsafe_kind == "file":
        prefix.write_text("not an environment directory", encoding="utf-8")
    else:
        prefix.mkdir()
        prefix.chmod(0)
        restore_permissions = True

    try:
        result = subprocess.run(
            ["bash", str(INSTALLER)],
            cwd=tmp_path,
            env=environment,
            text=True,
            capture_output=True,
            check=False,
        )
    finally:
        if restore_permissions:
            prefix.chmod(0o755)

    assert result.returncode != 0
    assert "prefix" in result.stderr.lower()
    assert _calls(log) == []


def test_installer_rejects_mamba_resolved_outside_bin_before_invocation(tmp_path: Path) -> None:
    marker = tmp_path / "was_invoked"
    executable = tmp_path / "unsafe tools" / "mamba"
    executable.parent.mkdir()
    executable.write_text(f"#!/usr/bin/env bash\n: > {marker!s}\n", encoding="utf-8")
    executable.chmod(0o755)
    environment = os.environ.copy()
    environment["A1_MAMBA_BIN"] = str(executable)

    result = subprocess.run(
        ["bash", str(INSTALLER)],
        cwd=tmp_path,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode != 0
    assert "bin/mamba" in result.stderr.lower()
    assert not marker.exists()


@pytest.mark.parametrize("component", ["sim", "sdf", "fuel"])
def test_installer_rejects_missing_component_version(tmp_path: Path, component: str) -> None:
    environment, log, _ = _fake_environment(tmp_path, "missing_component")
    _set_fake_control(environment, "missing_component", component)

    result = subprocess.run(
        ["bash", str(INSTALLER)],
        cwd=tmp_path,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode != 0
    assert f"{component} expected major" in result.stderr.lower()
    assert "HARMONIC_ALREADY_READY" not in result.stdout
    assert _transaction_kind(_transaction(_calls(log))[0]) == "update"


def test_installer_rejects_wrong_component_majors_after_update(tmp_path: Path) -> None:
    result, calls = _run_installer(tmp_path, "wrong")

    assert result.returncode != 0
    assert "expected major" in result.stderr.lower()
    assert [_transaction_kind(call) for call in _transaction(calls)] == ["update"]
    assert "CLASSIC_UNCHANGED" in result.stdout


def test_installer_does_not_accept_expected_major_hidden_in_unrelated_probe_text(tmp_path: Path) -> None:
    result, calls = _run_installer(tmp_path, "misleading")

    assert result.returncode != 0
    assert "expected major" in result.stderr.lower()
    assert [_transaction_kind(call) for call in _transaction(calls)] == ["update"]
    assert "HARMONIC_ALREADY_READY" not in result.stdout


def test_installer_rejects_missing_mamba_with_actionable_error(tmp_path: Path) -> None:
    environment = os.environ.copy()
    environment["A1_MAMBA_BIN"] = str(tmp_path / "does not exist")

    result = subprocess.run(
        ["bash", str(INSTALLER)],
        cwd=tmp_path,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode != 0
    assert "mamba" in result.stderr.lower()
    assert "A1_MAMBA_BIN" in result.stderr


def test_installer_keeps_space_containing_yaml_path_as_one_argument(tmp_path: Path) -> None:
    copied_root = tmp_path / "copied project with spaces" / "gazebo"
    copied_scripts = copied_root / "scripts"
    copied_scripts.mkdir(parents=True)
    shutil.copy2(ENVIRONMENT, copied_root / ENVIRONMENT.name)
    shutil.copy2(HELPER, copied_scripts / HELPER.name)
    shutil.copy2(INSTALLER, copied_scripts / INSTALLER.name)
    environment, log, _ = _fake_environment(tmp_path / "runner", "missing")

    result = subprocess.run(
        ["bash", str(copied_scripts / INSTALLER.name)],
        cwd=tmp_path,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    create = _transaction(_calls(log))[0]
    assert create[create.index("--file") + 1] == str(copied_root / ENVIRONMENT.name)


def test_helper_honors_environment_name_override_for_every_run(tmp_path: Path) -> None:
    environment, log, _ = _fake_environment(tmp_path, "ready")
    environment["A1_HARMONIC_ENV_NAME"] = "a1-harmonic-test"
    custom_prefix = Path(environment["FAKE_EXPECTED_ROOT"]) / "envs" / "a1-harmonic-test"
    custom_prefix.mkdir()
    environment["FAKE_EXPECTED_PREFIX"] = str(custom_prefix)
    _set_fake_control(environment, "expected_prefix", str(custom_prefix))

    result = subprocess.run(
        ["bash", str(INSTALLER)],
        cwd=tmp_path,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    run_calls = _run_calls(_calls(log))
    assert run_calls
    assert all(_run_payload(call)[0] == "gz" for call in run_calls)
    assert all(call[call.index("--prefix") + 1] == str(custom_prefix) for call in run_calls)


def test_fresh_shell_can_source_helper_and_run_without_prior_prepare(tmp_path: Path) -> None:
    environment, log, _ = _fake_environment(tmp_path, "ready")
    _add_hostile_mamba_bootstrap_environment(environment)

    result = subprocess.run(
        [
            "bash",
            "-c",
            'source "$1"; a1_harmonic_run gz sim --force-version 8 --versions',
            "a1-helper-test",
            str(HELPER),
        ],
        cwd=tmp_path,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert result.stdout == "8.10.0\n"
    calls = _calls(log)
    assert len(calls) == 1
    _assert_pinned_call(calls[0], environment)
    assert _run_payload(calls[0]) == ["gz", "sim", "--force-version", "8", "--versions"]


@pytest.mark.parametrize("unsafe_name", ["base", "root", "../shared", "/tmp/shared", "name/child"])
def test_installer_rejects_unsafe_environment_name_before_any_mamba_call(tmp_path: Path, unsafe_name: str) -> None:
    environment, log, _ = _fake_environment(tmp_path, "missing")
    environment["A1_HARMONIC_ENV_NAME"] = unsafe_name

    result = subprocess.run(
        ["bash", str(INSTALLER)],
        cwd=tmp_path,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode != 0
    assert "safe dedicated environment name" in result.stderr.lower()
    assert _calls(log) == []


def test_every_installer_probe_uses_clean_env_and_exact_forced_major(tmp_path: Path) -> None:
    result, calls = _run_installer(tmp_path, "ready")

    assert result.returncode == 0, result.stderr
    run_calls = _run_calls(calls)
    assert run_calls
    assert all(call[:2] == ["--no-rc", "--no-env"] for call in run_calls)
    assert all("--root-prefix" in call and "--prefix" in call for call in run_calls)
    for component, major in (("sim", "8"), ("sdf", "14"), ("fuel", "9")):
        probes = [call for call in run_calls if _run_payload(call)[:2] == ["gz", component]]
        assert probes
        assert all(_run_payload(call)[2:] == ["--force-version", major, "--versions"] for call in probes)


@pytest.fixture(scope="module")
def exported_models(tmp_path_factory: pytest.TempPathFactory) -> Path:
    root = tmp_path_factory.mktemp("harmonic export with spaces") / "fuel output"
    export_models(root)
    return root


def _run_checker(
    tmp_path: Path,
    export_root: Path,
    *,
    fail_match: str | None = None,
    empty_kind: str | None = None,
) -> tuple[subprocess.CompletedProcess[str], list[list[str]]]:
    environment, log, _ = _fake_environment(tmp_path, "ready")
    environment["A1_PYTHON_BIN"] = sys.executable
    if fail_match is not None:
        _set_fake_control(environment, "fail_match", fail_match)
    if empty_kind is not None:
        _set_fake_control(environment, "empty_kind", empty_kind)
    result = subprocess.run(
        ["bash", str(CHECKER), str(export_root)],
        cwd=tmp_path,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    return result, _calls(log)


def test_checker_pins_same_prefix_despite_hostile_mamba_environment_and_config(
    tmp_path: Path, exported_models: Path
) -> None:
    environment, log, _ = _fake_environment(tmp_path, "ready")
    environment["A1_PYTHON_BIN"] = sys.executable
    _add_hostile_mamba_bootstrap_environment(environment)

    result = subprocess.run(
        ["bash", str(CHECKER), str(exported_models)],
        cwd=tmp_path,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    calls = _calls(log)

    assert result.returncode == 0, result.stderr
    assert calls
    for call in calls:
        _assert_pinned_call(call, environment)
    assert "HARMONIC_FORMAT_OK: 3 models" in result.stdout


def test_format_checker_runs_all_native_operations_and_ignores_archives(tmp_path: Path, exported_models: Path) -> None:
    result, calls = _run_checker(tmp_path, exported_models)

    assert result.returncode == 0, result.stderr
    assert result.stdout == (
        "HARMONIC_FORMAT_VALID onerobotics_a1_right_arm\n"
        "HARMONIC_FORMAT_VALID onerobotics_a1_left_arm\n"
        "HARMONIC_FORMAT_VALID onerobotics_a1_bimanual_stand\n"
        "HARMONIC_FORMAT_OK: 3 models\n"
    )
    run_calls = _run_calls(calls)
    assert run_calls
    assert all(call[:2] == ["--no-rc", "--no-env"] for call in run_calls)
    payloads = [_run_payload(call) for call in run_calls]
    sdf_calls = [payload for payload in payloads if payload[:4] == ["gz", "sdf", "--force-version", "14"]]
    fuel_calls = [
        payload
        for payload in payloads
        if any(payload[index : index + 4] == ["gz", "fuel", "--force-version", "9"] for index in range(len(payload)))
    ]
    assert sum("-p" in call for call in sdf_calls) == 3
    assert sum("-k" in call and "/model.sdf" in " ".join(call) for call in sdf_calls) == 3
    assert sum("-k" in call and ".parsed.sdf" in " ".join(call) for call in sdf_calls) == 3
    assert len(fuel_calls) == 3
    assert all("meta" in call and "--pbtxt2config" in call for call in fuel_calls)
    for call in fuel_calls:
        assert "env" in call
        assert any(argument.startswith("HOME=") for argument in call)
        assert any(argument.startswith("XDG_CACHE_HOME=") for argument in call)
        assert any(argument.startswith("XDG_CONFIG_HOME=") for argument in call)
        assert any(argument.startswith("XDG_DATA_HOME=") for argument in call)
    assert not any("archives" in argument for payload in payloads for argument in payload)


def test_format_checker_uses_one_private_snapshot_after_pure_validation(tmp_path: Path, exported_models: Path) -> None:
    environment, log, _ = _fake_environment(tmp_path, "ready")
    environment["A1_PYTHON_BIN"] = sys.executable
    _set_fake_control(environment, "mutate_export_root", str(exported_models))
    _set_fake_control(environment, "mutation_marker", str(tmp_path / "mutation happened"))
    model_sdf = exported_models / "onerobotics_a1_right_arm" / "model.sdf"
    original = model_sdf.read_bytes()
    try:
        result = subprocess.run(
            ["bash", str(CHECKER), str(exported_models)],
            cwd=tmp_path,
            env=environment,
            text=True,
            capture_output=True,
            check=False,
        )
        calls = _calls(log)
        assert model_sdf.read_bytes() != original
    finally:
        model_sdf.write_bytes(original)

    assert result.returncode == 0, result.stderr
    assert "HARMONIC_FORMAT_OK: 3 models" in result.stdout
    assert calls
    assert not any(str(exported_models) in argument for call in calls for argument in call)


@pytest.mark.parametrize(
    "fail_match",
    [
        " gz sdf --force-version 14 -k ",
        " gz sdf --force-version 14 -p ",
        " gz fuel --force-version 9 meta ",
    ],
)
def test_format_checker_emits_no_success_footer_after_native_command_failure(
    tmp_path: Path, exported_models: Path, fail_match: str
) -> None:
    result, _ = _run_checker(tmp_path, exported_models, fail_match=fail_match)

    assert result.returncode != 0
    assert "native harmonic command failed" in result.stderr.lower()
    assert "HARMONIC_FORMAT_VALID" not in result.stdout
    assert "HARMONIC_FORMAT_OK" not in result.stdout


@pytest.mark.parametrize("kind", ["pretty", "fuel"])
def test_format_checker_rejects_empty_generated_output(tmp_path: Path, exported_models: Path, kind: str) -> None:
    result, _ = _run_checker(tmp_path, exported_models, empty_kind=kind)

    assert result.returncode != 0
    assert "empty" in result.stderr.lower()
    assert "HARMONIC_FORMAT_VALID" not in result.stdout
    assert "HARMONIC_FORMAT_OK" not in result.stdout


def test_format_checker_defers_all_success_lines_until_every_semantic_gate_passes(
    tmp_path: Path, exported_models: Path
) -> None:
    environment, log, _ = _fake_environment(tmp_path, "ready")
    environment["A1_PYTHON_BIN"] = sys.executable
    _set_fake_control(environment, "mutate_pretty_slug", "onerobotics_a1_bimanual_stand")

    result = subprocess.run(
        ["bash", str(CHECKER), str(exported_models)],
        cwd=tmp_path,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode != 0
    assert "canonical link" in result.stderr.lower()
    assert "HARMONIC_FORMAT_VALID" not in result.stdout
    assert "HARMONIC_FORMAT_OK" not in result.stdout
    assert _calls(log)


def test_format_checker_runs_pure_validator_before_any_gazebo_command(tmp_path: Path) -> None:
    invalid_export = tmp_path / "invalid export"
    invalid_export.mkdir()

    result, calls = _run_checker(tmp_path / "runner", invalid_export)

    assert result.returncode != 0
    assert calls == []
    assert "pure-data validation failed" in result.stderr.lower()
    assert "HARMONIC_FORMAT_OK" not in result.stdout
