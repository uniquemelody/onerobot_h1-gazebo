from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

GAZEBO_ROOT = Path(__file__).resolve().parents[1]
OPEN_DEMO = GAZEBO_ROOT / "scripts" / "open_demo.sh"
WORLDS = GAZEBO_ROOT / "worlds"
ENV_NAME = "onerobotics-a1-gz-harmonic"


def _write_fake_mamba(root: Path, *, prefix_exists: bool = True) -> Path:
    bin_directory = root / "bin"
    bin_directory.mkdir(parents=True)
    (root / "envs").mkdir()
    if prefix_exists:
        (root / "envs" / ENV_NAME).mkdir()
    (root / "control").mkdir()
    executable = bin_directory / "mamba"
    executable.write_text(
        """#!/usr/bin/env bash
set -euo pipefail
self=$(readlink -f -- "$0")
bin_directory=${self%/*}
fake_root=${bin_directory%/*}
log=$fake_root/calls.tsv
{
  printf 'CALL'
  for argument in "$@"; do
    printf '\\t%s' "$argument"
  done
  printf '\\n'
} >> "$log"
[[ ${1:-} == --no-rc ]] || exit 89
[[ ${2:-} == --no-env ]] || exit 90
[[ ${3:-} == run ]] || exit 91
[[ ${4:-} == --root-prefix ]] || exit 92
[[ ${5:-} == "$fake_root" ]] || exit 93
[[ ${6:-} == --clean-env ]] || exit 94
[[ ${7:-} == --prefix ]] || exit 95
[[ ${8:-} == "$fake_root/envs/onerobotics-a1-gz-harmonic" ]] || exit 96
shift 8
[[ ${1:-} == env ]] || exit 97
shift
while [[ ${1:-} == *=* ]]; do
  shift
done
[[ ${1:-} == gz && ${2:-} == sim ]] || exit 98
status=0
if [[ -f $fake_root/control/gui_status ]]; then
  IFS= read -r status < "$fake_root/control/gui_status"
fi
exit "$status"
""",
        encoding="utf-8",
    )
    executable.chmod(0o755)
    return executable


def _calls(root: Path) -> list[list[str]]:
    log = root / "calls.tsv"
    if not log.exists():
        return []
    return [line.split("\t")[1:] for line in log.read_text(encoding="utf-8").splitlines()]


def _write_recording_python(root: Path) -> tuple[Path, Path]:
    log = root / "python-calls.jsonl"
    executable = root / "recording-python"
    executable.write_text(
        f"""#!{sys.executable}
import json
import os
import sys

with open(os.environ["A1_PYTHON_CALLS_LOG"], "a", encoding="utf-8") as handle:
    json.dump(sys.argv[1:], handle)
    handle.write("\\n")
os.execv(sys.executable, [sys.executable, *sys.argv[1:]])
""",
        encoding="utf-8",
    )
    executable.chmod(0o755)
    return executable, log


def _python_calls(log: Path) -> list[list[str]]:
    if not log.exists():
        return []
    return [json.loads(line) for line in log.read_text(encoding="utf-8").splitlines()]


def _run_launcher(
    tmp_path: Path,
    stdin: str,
    *,
    display: bool = True,
    gui_status: int | None = None,
    extra_environment: dict[str, str] | None = None,
    arguments: tuple[str, ...] = (),
    python_bin: str | None = None,
    prefix_exists: bool = True,
) -> tuple[subprocess.CompletedProcess[str], list[list[str]]]:
    root = tmp_path / "fake mamba root"
    mamba = _write_fake_mamba(root, prefix_exists=prefix_exists)
    if gui_status is not None:
        (root / "control" / "gui_status").write_text(f"{gui_status}\n", encoding="utf-8")
    environment = os.environ.copy()
    environment.update(
        {
            "A1_MAMBA_BIN": str(mamba),
            "A1_PYTHON_BIN": python_bin or sys.executable,
            "HOME": "/hostile/home",
            "XDG_CACHE_HOME": "/hostile/cache",
            "XDG_CONFIG_HOME": "/hostile/config",
            "XDG_DATA_HOME": "/hostile/data",
            "GZ_SIM_RESOURCE_PATH": "/hostile/gz",
            "OGRE_RESOURCE_PATH": "/hostile/ogre",
            "IGN_GAZEBO_RESOURCE_PATH": "/hostile/ign",
            "HOSTILE_NATIVE_PAYLOAD": "must-not-pass",
        }
    )
    for variable in (
        "DISPLAY",
        "WAYLAND_DISPLAY",
        "XAUTHORITY",
        "XDG_RUNTIME_DIR",
        "DBUS_SESSION_BUS_ADDRESS",
    ):
        environment.pop(variable, None)
    if display:
        environment["DISPLAY"] = ":99"
    else:
        environment.pop("DISPLAY", None)
        environment.pop("WAYLAND_DISPLAY", None)
    if extra_environment:
        environment.update(extra_environment)
    result = subprocess.run(
        ["bash", str(OPEN_DEMO), *arguments],
        cwd=tmp_path,
        env=environment,
        input=stdin,
        text=True,
        capture_output=True,
        check=False,
    )
    return result, _calls(root)


def _run_launcher_with_recording_python(
    tmp_path: Path,
    stdin: str,
    *,
    gui_status: int | None = None,
) -> tuple[subprocess.CompletedProcess[str], list[list[str]], list[list[str]], Path]:
    runtime_tmpdir = tmp_path / "dedicated runtime tmpdir"
    runtime_tmpdir.mkdir()
    python_bin, log = _write_recording_python(tmp_path)
    result, calls = _run_launcher(
        tmp_path,
        stdin,
        gui_status=gui_status,
        python_bin=str(python_bin),
        extra_environment={
            "A1_PYTHON_CALLS_LOG": str(log),
            "TMPDIR": str(runtime_tmpdir),
        },
    )
    return result, calls, _python_calls(log), runtime_tmpdir


def _payload(call: list[str]) -> list[str]:
    prefix_index = call.index("--prefix")
    return call[prefix_index + 2 :]


def _inner_environment(call: list[str]) -> dict[str, str]:
    payload = _payload(call)
    assert payload[0] == "env"
    gz_index = payload.index("gz")
    assignments = payload[1:gz_index]
    assert all("=" in assignment for assignment in assignments)
    return dict(assignment.split("=", 1) for assignment in assignments)


def _assert_build_lifecycle(calls: list[list[str]], python_calls: list[list[str]], runtime_tmpdir: Path) -> Path:
    assert len(calls) == 1
    assert len(python_calls) == 2
    package_call, validate_call = python_calls
    assert package_call[:3] == ["-m", "onerobotics_a1_gazebo.package", "--output"]
    assert validate_call[:2] == ["-m", "onerobotics_a1_gazebo.validate"]
    models_root = Path(package_call[3])
    assert package_call == [
        "-m",
        "onerobotics_a1_gazebo.package",
        "--output",
        str(models_root),
    ]
    assert validate_call == ["-m", "onerobotics_a1_gazebo.validate", str(models_root)]

    session_root = models_root.parent
    assert session_root.parent == runtime_tmpdir
    inner_environment = _inner_environment(calls[0])
    assert inner_environment["GZ_SIM_RESOURCE_PATH"] == str(models_root)
    assert inner_environment["HOME"] == str(session_root / "home")
    assert inner_environment["XDG_CACHE_HOME"] == str(session_root / "cache")
    assert inner_environment["XDG_CONFIG_HOME"] == str(session_root / "config")
    assert inner_environment["XDG_DATA_HOME"] == str(session_root / "data")
    return session_root


def test_launcher_exists_is_executable_and_parses_as_bash() -> None:
    assert OPEN_DEMO.is_file()
    assert os.access(OPEN_DEMO, os.X_OK)
    subprocess.run(["bash", "-n", str(OPEN_DEMO)], check=True)


@pytest.mark.parametrize(
    ("choice", "world"),
    [
        ("1", "onerobotics_a1_right_arm_demo.sdf"),
        ("2", "onerobotics_a1_left_arm_demo.sdf"),
        ("3", "onerobotics_a1_bimanual_stand_demo.sdf"),
    ],
)
def test_menu_choice_launches_only_the_fixed_world_from_any_cwd(
    tmp_path: Path,
    choice: str,
    world: str,
) -> None:
    result, calls = _run_launcher(tmp_path, f"{choice}\n0\n")

    assert result.returncode == 0, result.stderr
    assert len(calls) == 1
    assert calls[0][-7:] == [
        "gz",
        "sim",
        "--force-version",
        "8",
        "-v",
        "4",
        str(WORLDS / world),
    ]


def test_native_payload_is_private_and_only_forwards_display_session_variables(tmp_path: Path) -> None:
    result, calls = _run_launcher(
        tmp_path,
        "1\n0\n",
        extra_environment={
            "WAYLAND_DISPLAY": "wayland-1",
            "XAUTHORITY": "/tmp/xauth",
            "XDG_RUNTIME_DIR": "/run/user/1000",
            "DBUS_SESSION_BUS_ADDRESS": "unix:path=/run/user/1000/bus",
        },
    )

    assert result.returncode == 0, result.stderr
    assert len(calls) == 1
    inner_environment = _inner_environment(calls[0])
    assert set(inner_environment) == {
        "GZ_SIM_RESOURCE_PATH",
        "HOME",
        "XDG_CACHE_HOME",
        "XDG_CONFIG_HOME",
        "XDG_DATA_HOME",
        "DISPLAY",
        "WAYLAND_DISPLAY",
        "XAUTHORITY",
        "XDG_RUNTIME_DIR",
        "DBUS_SESSION_BUS_ADDRESS",
    }
    assert not inner_environment["GZ_SIM_RESOURCE_PATH"].startswith("/hostile")
    assert all(
        not inner_environment[name].startswith("/hostile")
        for name in ("HOME", "XDG_CACHE_HOME", "XDG_CONFIG_HOME", "XDG_DATA_HOME")
    )
    assert inner_environment["DISPLAY"] == ":99"
    assert inner_environment["WAYLAND_DISPLAY"] == "wayland-1"
    assert inner_environment["XAUTHORITY"] == "/tmp/xauth"
    assert inner_environment["XDG_RUNTIME_DIR"] == "/run/user/1000"
    assert inner_environment["DBUS_SESSION_BUS_ADDRESS"] == "unix:path=/run/user/1000/bus"
    assert "HOSTILE_NATIVE_PAYLOAD" not in inner_environment
    assert "gz sim --force-version 8" in OPEN_DEMO.read_text(encoding="utf-8")


def test_native_payload_omits_absent_display_session_variables(tmp_path: Path) -> None:
    result, calls = _run_launcher(tmp_path, "1\n0\n")

    assert result.returncode == 0, result.stderr
    assert len(calls) == 1
    inner_environment = _inner_environment(calls[0])
    assert set(inner_environment) == {
        "GZ_SIM_RESOURCE_PATH",
        "HOME",
        "XDG_CACHE_HOME",
        "XDG_CONFIG_HOME",
        "XDG_DATA_HOME",
        "DISPLAY",
    }


def test_extra_arguments_exit_with_usage_error_before_mamba(tmp_path: Path) -> None:
    result, calls = _run_launcher(tmp_path, "", arguments=("unexpected",))

    assert result.returncode == 2
    assert "用法" in result.stderr
    assert calls == []


def test_missing_display_stops_before_mamba(tmp_path: Path) -> None:
    result, calls = _run_launcher(tmp_path, "", display=False)

    assert result.returncode != 0
    assert "DISPLAY" in result.stderr
    assert calls == []


@pytest.mark.parametrize("exists", [False, True])
def test_unusable_python_tooling_stops_before_mamba(tmp_path: Path, exists: bool) -> None:
    python_bin = tmp_path / "python-tooling"
    if exists:
        python_bin.write_text("not executable", encoding="utf-8")
    result, calls = _run_launcher(tmp_path, "", python_bin=str(python_bin))

    assert result.returncode != 0
    assert "Python 环境不存在" in result.stderr
    assert calls == []


def test_absent_harmonic_prefix_does_not_install_or_run_native_commands(tmp_path: Path) -> None:
    result, calls = _run_launcher(tmp_path, "", prefix_exists=False)

    assert result.returncode != 0
    assert "Harmonic 环境尚未安装" in result.stderr
    assert calls == []


def test_invalid_input_retries_without_native_launch(tmp_path: Path) -> None:
    result, calls = _run_launcher(tmp_path, "bogus\n0\n")

    assert result.returncode == 0, result.stderr
    assert "输入无效" in result.stdout
    assert calls == []


def test_eof_exits_successfully_without_native_launch(tmp_path: Path) -> None:
    result, calls = _run_launcher(tmp_path, "")

    assert result.returncode == 0, result.stderr
    assert calls == []


def test_launcher_uses_one_ephemeral_models_root_for_build_validation_and_gui(tmp_path: Path) -> None:
    result, calls, python_calls, runtime_tmpdir = _run_launcher_with_recording_python(tmp_path, "1\n0\n")

    assert result.returncode == 0, result.stderr
    session_root = _assert_build_lifecycle(calls, python_calls, runtime_tmpdir)
    assert not session_root.exists()


def test_gui_interrupt_status_returns_to_the_menu(tmp_path: Path) -> None:
    result, calls = _run_launcher(tmp_path, "1\n0\n", gui_status=130)

    assert result.returncode == 0, result.stderr
    assert len(calls) == 1
    assert result.stdout.count("请选择要查看的 Gazebo 模型：") == 2


def test_unexpected_gui_failure_exits_nonzero(tmp_path: Path) -> None:
    result, calls, python_calls, runtime_tmpdir = _run_launcher_with_recording_python(tmp_path, "1\n", gui_status=42)

    assert result.returncode == 42
    session_root = _assert_build_lifecycle(calls, python_calls, runtime_tmpdir)
    assert not session_root.exists()
    assert "Gazebo 启动失败" in result.stderr
