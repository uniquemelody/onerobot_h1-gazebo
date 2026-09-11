from __future__ import annotations

import errno
import io
import json
import os
import re
import signal
import subprocess
import time
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest
from onerobotics_a1_gazebo import cache as cache_module
from onerobotics_a1_gazebo import demo
from onerobotics_a1_gazebo import render as render_module
from onerobotics_a1_gazebo.package import export_models
from onerobotics_a1_gazebo.spec import load_model_specs

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
EXPORT_ROOT = REPOSITORY_ROOT / "dist/gazebo-fuel"
WORLDS_ROOT = REPOSITORY_ROOT / "gazebo/worlds"
RIGHT_MODEL = "onerobotics_a1_right_arm"
RIGHT_JOINT = "joint1-a1_r"
RIGHT_COMMAND_TOPIC = f"/a1_demo/{RIGHT_MODEL}/{RIGHT_JOINT}/cmd_pos"
RIGHT_STATE_TOPIC = f"/world/a1_demo/model/{RIGHT_MODEL}/joint_state"


def _state(position: float | None, *, joint: str = RIGHT_JOINT) -> str:
    axis: dict[str, float] = {} if position is None else {"position": position}
    return json.dumps(
        {
            "name": RIGHT_MODEL,
            "joint": [
                {"name": "unrelated", "axis1": {"position": -1.25}},
                {"name": joint, "axis1": axis},
            ],
        }
    )


class _FakeProcess:
    def __init__(self, harness: _Harness, stdout: Any, stderr: Any) -> None:
        self._harness = harness
        self.pid = 48129
        self.returncode = harness.early_returncode
        self.stdout = (
            harness.server_stdout_stream
            if stdout == subprocess.PIPE and harness.server_stdout_stream is not None
            else io.StringIO(harness.server_stdout)
            if stdout == subprocess.PIPE
            else None
        )
        self.stderr = io.StringIO(harness.server_stderr) if stderr == subprocess.PIPE else None

    def poll(self) -> int | None:
        return self.returncode

    def wait(self, timeout: float | None = None) -> int:
        self._harness.wait_timeouts.append(timeout)
        behavior = self._harness.wait_behaviors.pop(0) if self._harness.wait_behaviors else "exit"
        if behavior == "timeout":
            raise subprocess.TimeoutExpired("gz sim", timeout)
        self.returncode = 0
        self._harness.alive_groups.discard(self.pid)
        return 0


class _FakeCommandProcess:
    def __init__(self, harness: _Harness, args: list[str], options: dict[str, Any], pid: int) -> None:
        self._harness = harness
        self._args = args
        self._options = options
        self.pid = pid
        self.returncode: int | None = None

    def communicate(self, timeout: float | None = None) -> tuple[str, str]:
        completed = self._harness.dispatch(self._args, timeout)
        self.returncode = completed.returncode
        if not self._harness.command_group_survives_return:
            self._harness.alive_groups.discard(self.pid)
        return completed.stdout, completed.stderr

    def poll(self) -> int | None:
        return self.returncode

    def wait(self, timeout: float | None = None) -> int:
        self.returncode = -signal.SIGKILL
        return self.returncode


class _Harness:
    def __init__(
        self,
        *,
        states: list[str] | None = None,
        topic_lists: list[str] | None = None,
        topic_info: str = "Publishers [Address, Message Type]:\nnode, gz.msgs.Model\n",
        service_stdout: str = "data: true\n",
        service_returncode: int = 0,
        state_timeout_at: int | None = None,
        early_returncode: int | None = None,
        wait_behaviors: list[str] | None = None,
        server_stdout: str = "Gazebo server ready\n",
        server_stdout_stream: Any | None = None,
        server_stderr: str = "",
        crash_after_state_at: int | None = None,
        interrupt_state_at: int | None = None,
        os_error_state_at: int | None = None,
        leader_exited_group_alive: bool = False,
        sigterm_leaves_group_alive: bool | None = None,
        sigkill_leaves_group_alive: bool = False,
        child_sigkill_leaves_group_alive: bool = False,
        service_timeout: bool = False,
        command_group_survives_return: bool = False,
        exit_during_service_returncode: int | None = None,
        disappear_on_server_probe_after_service: bool = False,
    ) -> None:
        self.states = list(states or [_state(None), _state(0.03)])
        self.topic_lists = list(topic_lists or [f"{RIGHT_COMMAND_TOPIC}\n{RIGHT_STATE_TOPIC}\n"])
        self.topic_info = topic_info
        self.service_stdout = service_stdout
        self.service_returncode = service_returncode
        self.state_timeout_at = state_timeout_at
        self.early_returncode = early_returncode
        self.wait_behaviors = list(wait_behaviors or ["exit"])
        self.server_stdout = server_stdout
        self.server_stdout_stream = server_stdout_stream
        self.server_stderr = server_stderr
        self.crash_after_state_at = crash_after_state_at
        self.interrupt_state_at = interrupt_state_at
        self.os_error_state_at = os_error_state_at
        self.sigterm_leaves_group_alive = (
            bool(wait_behaviors and wait_behaviors[0] == "timeout")
            if sigterm_leaves_group_alive is None
            else sigterm_leaves_group_alive
        )
        self.sigkill_leaves_group_alive = sigkill_leaves_group_alive
        self.child_sigkill_leaves_group_alive = child_sigkill_leaves_group_alive
        self.service_timeout = service_timeout
        self.command_group_survives_return = command_group_survives_return
        self.exit_during_service_returncode = exit_during_service_returncode
        self.disappear_on_server_probe_after_service = disappear_on_server_probe_after_service
        self.service_dispatched = False
        self.server_probe_disappearance_triggered = False
        self.command_calls: list[tuple[list[str], dict[str, Any]]] = []
        self.server_calls: list[tuple[list[str], dict[str, Any]]] = []
        self.kill_calls: list[tuple[int, signal.Signals]] = []
        self.wait_timeouts: list[float | None] = []
        self.state_calls = 0
        self.process: _FakeProcess | None = None
        self.alive_groups: set[int] = {48129} if early_returncode is None or leader_exited_group_alive else set()

    @staticmethod
    def inner(args: list[str]) -> list[str]:
        gz_index = args.index("gz")
        return args[gz_index:]

    def popen(self, args: list[str], **kwargs: Any) -> _FakeProcess | _FakeCommandProcess:
        args = list(args)
        inner = self.inner(args)
        if inner[:2] == ["gz", "sim"]:
            self.server_calls.append((args, dict(kwargs)))
            self.process = _FakeProcess(self, kwargs.get("stdout"), kwargs.get("stderr"))
            return self.process
        self.command_calls.append((args, dict(kwargs)))
        pid = 49000 + len(self.command_calls)
        self.alive_groups.add(pid)
        return _FakeCommandProcess(self, args, dict(kwargs), pid)

    def dispatch(self, args: list[str], timeout: float | None) -> subprocess.CompletedProcess[str]:
        inner = self.inner(args)
        if inner[:5] == ["gz", "topic", "--force-version", "13", "-l"]:
            output = self.topic_lists.pop(0) if len(self.topic_lists) > 1 else self.topic_lists[0]
            return subprocess.CompletedProcess(args, 0, output, "")
        if "-i" in inner:
            return subprocess.CompletedProcess(args, 0, self.topic_info, "")
        if "-e" in inner:
            self.state_calls += 1
            if self.state_timeout_at == self.state_calls:
                raise subprocess.TimeoutExpired(args, timeout, output="partial", stderr="")
            if self.interrupt_state_at == self.state_calls:
                raise KeyboardInterrupt
            if self.os_error_state_at == self.state_calls:
                raise OSError("state transport broke")
            output = self.states.pop(0) if len(self.states) > 1 else self.states[0]
            if self.crash_after_state_at == self.state_calls:
                assert self.process is not None
                self.process.returncode = 31
            return subprocess.CompletedProcess(args, 0, output, "")
        if inner[:2] == ["gz", "service"]:
            self.service_dispatched = True
            if self.service_timeout:
                raise subprocess.TimeoutExpired(args, timeout, output="partial", stderr="")
            if self.exit_during_service_returncode is not None:
                assert self.process is not None
                self.process.returncode = self.exit_during_service_returncode
            return subprocess.CompletedProcess(args, self.service_returncode, self.service_stdout, "")
        if "-p" in inner and "gz.msgs.Double" in inner:
            return subprocess.CompletedProcess(args, 0, "", "")
        raise AssertionError(f"unexpected command: {inner}")

    def killpg(self, pid: int, sent_signal: signal.Signals | int) -> None:
        if sent_signal == 0:
            if (
                pid == 48129
                and self.disappear_on_server_probe_after_service
                and self.service_dispatched
                and not self.server_probe_disappearance_triggered
            ):
                self.server_probe_disappearance_triggered = True
                assert self.process is not None
                self.process.returncode = 37
                self.alive_groups.discard(pid)
                raise ProcessLookupError(pid)
            if pid not in self.alive_groups:
                raise ProcessLookupError(pid)
            return
        self.kill_calls.append((pid, sent_signal))
        child_survives_sigkill = pid >= 49000 and self.child_sigkill_leaves_group_alive
        if (sent_signal == signal.SIGKILL and not self.sigkill_leaves_group_alive and not child_survives_sigkill) or (
            sent_signal == signal.SIGTERM and not self.sigterm_leaves_group_alive
        ):
            self.alive_groups.discard(pid)


@pytest.fixture(scope="module")
def unit_assets(tmp_path_factory: pytest.TempPathFactory) -> tuple[Path, Path]:
    root = tmp_path_factory.mktemp("runtime-unit")
    export_root = root / "models"
    worlds_root = root / "worlds"
    export_models(export_root)
    demo.generate_worlds(models_root=export_root, output_root=worlds_root)
    return export_root, worlds_root


def _install_harness(monkeypatch: pytest.MonkeyPatch, harness: _Harness) -> None:
    monkeypatch.setattr(demo.subprocess, "Popen", harness.popen)
    monkeypatch.setattr(demo.os, "killpg", harness.killpg)
    monkeypatch.setattr(demo.secrets, "token_hex", lambda count: "9f4a2c7e1b6d8035")


def _smoke(unit_assets: tuple[Path, Path], **overrides: Any):
    export_root, worlds_root = unit_assets
    arguments = {
        "model": RIGHT_MODEL,
        "joint": RIGHT_JOINT,
        "target": 0.05,
        "export_root": export_root,
        "worlds_root": worlds_root,
    }
    arguments.update(overrides)
    return demo.smoke_test(**arguments)


def test_parse_joint_position_uses_name_not_array_order() -> None:
    assert demo.parse_joint_position(_state(0.03125), RIGHT_JOINT) == 0.03125


def test_parse_joint_position_treats_omitted_proto3_scalar_as_zero() -> None:
    assert demo.parse_joint_position(_state(None), RIGHT_JOINT) == 0.0


def test_parse_joint_position_rejects_concatenated_documents() -> None:
    with pytest.raises(ValueError, match="malformed"):
        demo.parse_joint_position(_state(0.01) + "\n" + _state(0.02), RIGHT_JOINT)


@pytest.mark.parametrize(
    ("payload", "message"),
    [
        ("not-json", "malformed"),
        ("[]", "object"),
        ('{"joint": {}}', "array"),
        ('{"joint": []}', "missing"),
        ('{"joint": [{"name": "other", "axis1": {}}]}', "missing"),
        (
            '{"joint": [{"name": "joint1-a1_r", "axis1": {}}, {"name": "joint1-a1_r", "axis1": {}}]}',
            "duplicate",
        ),
        ('{"joint": [{"name": "joint1-a1_r"}]}', "axis1"),
        ('{"joint": [{"name": "joint1-a1_r", "axis1": []}]}', "axis1"),
    ],
)
def test_parse_joint_position_rejects_malformed_or_missing_shapes(payload: str, message: str) -> None:
    with pytest.raises(ValueError, match=message):
        demo.parse_joint_position(payload, RIGHT_JOINT)


@pytest.mark.parametrize("value", ["true", '"0.1"', "null", "[]", "{}"])
def test_parse_joint_position_rejects_wrong_position_types(value: str) -> None:
    payload = f'{{"joint": [{{"name": "{RIGHT_JOINT}", "axis1": {{"position": {value}}}}}]}}'

    with pytest.raises(ValueError, match="position"):
        demo.parse_joint_position(payload, RIGHT_JOINT)


@pytest.mark.parametrize("value", ["NaN", "Infinity", "-Infinity"])
def test_parse_joint_position_rejects_nonfinite_positions(value: str) -> None:
    payload = f'{{"joint": [{{"name": "{RIGHT_JOINT}", "axis1": {{"position": {value}}}}}]}}'

    with pytest.raises(ValueError, match="finite"):
        demo.parse_joint_position(payload, RIGHT_JOINT)


def test_parse_joint_position_rejects_deep_and_oversized_json() -> None:
    deep = (
        '{"joint": [{"name": "joint1-a1_r", "axis1": {"position": 0}, "deep": ' + "[" * 100 + ("0" + "]" * 100 + "}]}")
    )
    oversized = '{"padding":"' + "x" * (1024 * 1024 + 1) + '"}'

    with pytest.raises(ValueError, match="nesting"):
        demo.parse_joint_position(deep, RIGHT_JOINT)
    with pytest.raises(ValueError, match="size"):
        demo.parse_joint_position(oversized, RIGHT_JOINT)


def test_oversized_character_payload_is_rejected_before_utf8_copy() -> None:
    class OversizedPayload(str):
        def encode(self, *args: Any, **kwargs: Any) -> bytes:
            raise AssertionError("oversized payload must not be encoded")

    payload = OversizedPayload("x" * (demo._MAX_JSON_BYTES + 1))

    with pytest.raises(ValueError, match="size"):
        demo.parse_joint_position(payload, RIGHT_JOINT)
    with pytest.raises(ValueError, match="size"):
        demo._parse_transport_frames(payload, RIGHT_JOINT)


def test_parse_joint_position_normalizes_invalid_unicode_and_huge_integer() -> None:
    surrogate = '{"joint": [], "padding": "\ud800"}'
    huge_integer = f'{{"joint": [{{"name": "{RIGHT_JOINT}", "axis1": {{"position": {"9" * 400}}}}}]}}'

    with pytest.raises(ValueError, match="joint-state"):
        demo.parse_joint_position(surrogate, RIGHT_JOINT)
    with pytest.raises(ValueError, match="position"):
        demo.parse_joint_position(huge_integer, RIGHT_JOINT)


@pytest.mark.parametrize(
    "payload",
    [
        r'{"ignored":"\ud800","joint":[{"name":"joint1-a1_r","axis1":{"position":0}}]}',
        r'{"\ud800":"ignored","joint":[{"name":"joint1-a1_r","axis1":{"position":0}}]}',
    ],
)
def test_parsers_reject_escaped_lone_surrogate_in_any_json_text(payload: str) -> None:
    with pytest.raises(ValueError, match="malformed"):
        demo.parse_joint_position(payload, RIGHT_JOINT)
    with pytest.raises(ValueError, match="malformed"):
        demo._parse_transport_frames(payload, RIGHT_JOINT)


def test_parser_accepts_valid_escaped_surrogate_pair() -> None:
    payload = r'{"ignored":"\ud83d\ude00","joint":[{"name":"joint1-a1_r","axis1":{"position":0}}]}'

    assert demo.parse_joint_position(payload, RIGHT_JOINT) == 0.0


def test_all_json_numeric_tokens_are_bounded_before_integer_conversion() -> None:
    payload = f'{{"padding": {"9" * 2048}, "joint": [{{"name": "{RIGHT_JOINT}", "axis1": {{"position": 0}}}}]}}'

    with pytest.raises(ValueError, match="numeric token exceeds"):
        demo.parse_joint_position(payload, RIGHT_JOINT)
    with pytest.raises(ValueError, match="numeric token exceeds"):
        demo._parse_transport_frames(payload, RIGHT_JOINT)


def test_exception_notes_have_python_310_compatible_storage() -> None:
    error = SimpleNamespace()

    demo._add_exception_note(error, "diagnostic marker")

    assert demo._exception_notes(error) == ("diagnostic marker",)


def test_harmonic_argv_accepts_valid_inner_environment_without_changing_default(
    unit_assets: tuple[Path, Path],
) -> None:
    export_root, _ = unit_assets
    default = demo._harmonic_argv("partition", export_root, ["topic", "-l"])
    configured = demo._harmonic_argv(
        "partition",
        export_root,
        ["topic", "-l"],
        extra_environment=(
            "QT_QPA_PLATFORM=offscreen",
            "LIBGL_ALWAYS_SOFTWARE=1",
            "HOME=/tmp/private-home",
            "XDG_CACHE_HOME=/tmp/private-cache",
        ),
    )

    default_gz = default.index("gz")
    configured_gz = configured.index("gz")
    assert default[default_gz - 3 : default_gz] == [
        "env",
        "GZ_PARTITION=partition",
        f"GZ_SIM_RESOURCE_PATH={export_root}",
    ]
    assert configured[configured_gz - 7 : configured_gz] == [
        "env",
        "GZ_PARTITION=partition",
        f"GZ_SIM_RESOURCE_PATH={export_root}",
        "QT_QPA_PLATFORM=offscreen",
        "LIBGL_ALWAYS_SOFTWARE=1",
        "HOME=/tmp/private-home",
        "XDG_CACHE_HOME=/tmp/private-cache",
    ]
    assert "MESA_LOADER_DRIVER_OVERRIDE=llvmpipe" not in configured


@pytest.mark.parametrize(
    "extra_environment",
    [
        ("GZ_PARTITION=attacker",),
        ("GZ_SIM_RESOURCE_PATH=/tmp/attacker",),
        ("MESA_LOADER_DRIVER_OVERRIDE=llvmpipe",),
        ("NOT AN ENV=value",),
        ("EMPTY",),
        ("HOME=/tmp/one", "HOME=/tmp/two"),
        ("HOME=/tmp/line\nbreak",),
    ],
)
def test_harmonic_argv_rejects_unsafe_or_duplicate_inner_environment(
    unit_assets: tuple[Path, Path],
    extra_environment: tuple[str, ...],
) -> None:
    export_root, _ = unit_assets

    with pytest.raises(ValueError, match="environment|reserved|duplicate|MESA"):
        demo._harmonic_argv(
            "partition",
            export_root,
            ["topic", "-l"],
            extra_environment=extra_environment,
        )


def test_run_harmonic_forwards_inner_environment_to_the_child_argv(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    harness = _Harness()
    _install_harness(monkeypatch, harness)

    demo._run_harmonic(
        partition="partition",
        private_export=Path("/private"),
        command=["topic", "--force-version", "13", "-l"],
        timeout=3.0,
        timeout_message="topic timeout",
        extra_environment=("HOME=/private/home", "XDG_CACHE_HOME=/private/cache"),
    )

    argv = harness.command_calls[0][0]
    gz_index = argv.index("gz")
    assert argv[gz_index - 2 : gz_index] == ["HOME=/private/home", "XDG_CACHE_HOME=/private/cache"]


def test_smoke_success_uses_exact_versions_variables_and_sequence(
    unit_assets: tuple[Path, Path],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    harness = _Harness()
    _install_harness(monkeypatch, harness)

    result = _smoke(unit_assets)

    assert result.initial_position == 0.0
    assert result.final_position == 0.03
    assert result.delta == pytest.approx(0.03)
    assert result.finite and result.moved_toward_target
    assert len(harness.server_calls) == 1
    server_call, server_options = harness.server_calls[0]
    assert _Harness.inner(server_call)[:-1] == ["gz", "sim", "--force-version", "8", "-s", "-r", "-v", "4"]
    assert Path(_Harness.inner(server_call)[-1]).name == "onerobotics_a1_right_arm_demo.sdf"
    private_world = Path(_Harness.inner(server_call)[-1])
    assert server_options["start_new_session"] is True
    assert server_options["stdout"] == subprocess.PIPE
    assert server_options["stderr"] == subprocess.PIPE

    all_calls = [server_call, *(call for call, _ in harness.command_calls)]
    environments = []
    for call in all_calls:
        gz_index = call.index("gz")
        assert call[gz_index - 3] == "env"
        environments.append(tuple(call[gz_index - 2 : gz_index]))
    assert len(set(environments)) == 1
    partition, resource = environments[0]
    assert re.fullmatch(r"GZ_PARTITION=onerobotics-a1-smoke-\d+-[0-9a-f]{16}", partition)
    assert resource.startswith("GZ_SIM_RESOURCE_PATH=/")
    private_export = Path(resource.split("=", 1)[1])
    assert private_world.parents[1] == private_export.parent
    assert not private_export.exists()

    commands = [_Harness.inner(call) for call, _ in harness.command_calls]
    assert commands == [
        ["gz", "topic", "--force-version", "13", "-l"],
        ["gz", "topic", "--force-version", "13", "-i", "-t", RIGHT_STATE_TOPIC],
        ["gz", "topic", "--force-version", "13", "-e", "-n", "1", "--json-output", "-t", RIGHT_STATE_TOPIC],
        [
            "gz",
            "topic",
            "--force-version",
            "13",
            "-t",
            RIGHT_COMMAND_TOPIC,
            "-m",
            "gz.msgs.Double",
            "-p",
            "data: 0.05",
        ],
        ["gz", "topic", "--force-version", "13", "-e", "-n", "1", "--json-output", "-t", RIGHT_STATE_TOPIC],
        [
            "gz",
            "service",
            "--force-version",
            "13",
            "-s",
            "/server_control",
            "--reqtype",
            "gz.msgs.ServerControl",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "2000",
            "--req",
            "stop: true",
        ],
    ]
    assert harness.kill_calls == []


def test_original_smoke_path_never_uses_the_cache_validator(
    unit_assets: tuple[Path, Path],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    harness = _Harness()
    _install_harness(monkeypatch, harness)
    monkeypatch.setattr(
        cache_module,
        "validate_cache_models",
        lambda **kwargs: (_ for _ in ()).throw(AssertionError("cache validator used by exact smoke")),
    )

    result = _smoke(unit_assets)

    assert result.moved_toward_target
    assert len(harness.server_calls) == 1


def test_validated_snapshot_core_reuses_the_existing_bounded_motion_lifecycle(
    unit_assets: tuple[Path, Path],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    export_root, worlds_root = unit_assets
    specs = demo.load_model_specs()
    spec = specs[0]
    models_snapshot = demo._validated_export_snapshot(export_root)
    worlds_snapshot = demo._world_snapshot(worlds_root, specs)
    harness = _Harness()
    _install_harness(monkeypatch, harness)

    result = demo._smoke_validated_snapshot(
        spec=spec,
        joint=RIGHT_JOINT,
        target=0.05,
        models_snapshot=models_snapshot,
        limit_snapshot=models_snapshot,
        worlds_snapshot=worlds_snapshot,
    )

    assert result.moved_toward_target
    assert len(harness.server_calls) == 1
    assert harness.kill_calls == []
    private_resource = next(
        value.split("=", 1)[1] for value in harness.server_calls[0][0] if value.startswith("GZ_SIM_RESOURCE_PATH=")
    )
    assert not Path(private_resource).exists()


def test_cache_smoke_validates_once_then_uses_normalized_models_and_trusted_limits(
    unit_assets: tuple[Path, Path],
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    trusted_models, worlds_root = unit_assets
    cache_models = tmp_path / "untrusted cache path"
    trusted_snapshot = object()
    normalized_snapshot = object()
    world_snapshot = object()
    validation_calls: list[dict[str, Path]] = []
    motion_calls: list[dict[str, object]] = []

    def validate_cache_models(**kwargs: Path) -> SimpleNamespace:
        validation_calls.append(kwargs)
        return SimpleNamespace(
            trusted_export=trusted_snapshot,
            normalized_models=normalized_snapshot,
        )

    def smoke_validated_snapshot(**kwargs: object) -> SimpleNamespace:
        motion_calls.append(kwargs)
        spec = kwargs["spec"]
        assert hasattr(spec, "slug")
        return SimpleNamespace(
            model=spec.slug,
            joint=kwargs["joint"],
            target=kwargs["target"],
            initial_position=0.0,
            final_position=0.03,
            delta=0.03,
            lower=-1.0,
            upper=1.0,
            finite=True,
            moved_toward_target=True,
        )

    monkeypatch.setattr(cache_module, "validate_cache_models", validate_cache_models)
    monkeypatch.setattr(demo, "_world_snapshot", lambda path, specs: world_snapshot)
    monkeypatch.setattr(demo, "_smoke_validated_snapshot", smoke_validated_snapshot, raising=False)

    results = demo.smoke_cache_all(
        trusted_models=trusted_models,
        cache_models=cache_models,
        worlds_root=worlds_root,
    )

    assert validation_calls == [{"trusted_models": trusted_models, "cache_models": cache_models}]
    assert len(results) == 4
    assert len(motion_calls) == 4
    assert all(call["models_snapshot"] is normalized_snapshot for call in motion_calls)
    assert all(call["limit_snapshot"] is trusted_snapshot for call in motion_calls)
    assert all(call["worlds_snapshot"] is world_snapshot for call in motion_calls)
    assert all(str(cache_models) not in repr(call) for call in motion_calls)
    assert len({call["deadline"] for call in motion_calls}) == 1
    assert all(isinstance(call["deadline"], float) for call in motion_calls)


def test_cache_validation_failure_prevents_every_motion_launch(
    unit_assets: tuple[Path, Path],
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    trusted_models, worlds_root = unit_assets
    motion_calls = 0

    def rejected(**kwargs: Path) -> None:
        raise ValueError("dedicated cache validation rejected")

    def motion(**kwargs: object) -> None:
        nonlocal motion_calls
        motion_calls += 1

    monkeypatch.setattr(cache_module, "validate_cache_models", rejected)
    monkeypatch.setattr(demo, "_smoke_validated_snapshot", motion, raising=False)

    with pytest.raises(ValueError, match="dedicated cache validation rejected"):
        demo.smoke_cache_all(
            trusted_models=trusted_models,
            cache_models=tmp_path / "cache",
            worlds_root=worlds_root,
        )

    assert motion_calls == 0


@pytest.mark.parametrize("target", [-1000.0, 1000.0, float("nan"), True, 10**400])
def test_smoke_rejects_invalid_target_before_launch(
    unit_assets: tuple[Path, Path],
    monkeypatch: pytest.MonkeyPatch,
    target: object,
) -> None:
    harness = _Harness()
    _install_harness(monkeypatch, harness)

    with pytest.raises(ValueError, match="target"):
        _smoke(unit_assets, target=target)

    assert harness.server_calls == []


def test_smoke_rejects_unknown_model_and_joint_before_launch(
    unit_assets: tuple[Path, Path],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    harness = _Harness()
    _install_harness(monkeypatch, harness)

    with pytest.raises(ValueError, match="unknown model"):
        _smoke(unit_assets, model="unknown")
    with pytest.raises(ValueError, match="unknown actuated joint"):
        _smoke(unit_assets, joint="unknown")
    assert harness.server_calls == []


def test_smoke_rejects_checked_in_world_byte_drift_before_launch(
    unit_assets: tuple[Path, Path],
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    export_root, worlds_root = unit_assets
    changed = tmp_path / "worlds"
    changed.mkdir()
    for world in worlds_root.iterdir():
        (changed / world.name).write_bytes(world.read_bytes())
    (changed / "onerobotics_a1_right_arm_demo.sdf").write_bytes(b"<sdf/>")
    harness = _Harness()
    _install_harness(monkeypatch, harness)

    with pytest.raises(ValueError, match="world bytes"):
        demo.smoke_test(
            model=RIGHT_MODEL,
            joint=RIGHT_JOINT,
            target=0.05,
            export_root=export_root,
            worlds_root=changed,
        )

    assert harness.server_calls == []


def test_smoke_rejects_path_separator_in_private_resource_root_before_launch(
    unit_assets: tuple[Path, Path],
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    hostile_root = tmp_path / "attacker:extra-resource-root"
    hostile_root.mkdir()
    real_temporary_directory = demo.tempfile.TemporaryDirectory

    class FixedTemporaryDirectory:
        def __enter__(self) -> str:
            return str(hostile_root)

        def __exit__(self, *args: object) -> None:
            return None

    def controlled_temporary_directory(*args: Any, **kwargs: Any) -> Any:
        if kwargs.get("prefix") == "onerobotics-a1-smoke-":
            return FixedTemporaryDirectory()
        return real_temporary_directory(*args, **kwargs)

    harness = _Harness()
    _install_harness(monkeypatch, harness)
    monkeypatch.setattr(demo.tempfile, "TemporaryDirectory", controlled_temporary_directory)

    with pytest.raises(ValueError, match="resource root.*path separator"):
        _smoke(unit_assets)

    assert harness.server_calls == []


def test_server_popen_failure_has_empty_log_and_topic_diagnostics(
    unit_assets: tuple[Path, Path],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    harness = _Harness()
    _install_harness(monkeypatch, harness)

    def failed_popen(*args: Any, **kwargs: Any) -> Any:
        raise OSError("server spawn failed")

    monkeypatch.setattr(demo.subprocess, "Popen", failed_popen)

    with pytest.raises(RuntimeError) as caught:
        _smoke(unit_assets)

    message = str(caught.value)
    assert "unable to start Gazebo server: server spawn failed" in message
    assert "server log tail:\n<empty>" in message
    assert "latest topic list:\n<none>" in message


def test_partial_log_start_failure_has_diagnostics_and_cleanup(
    unit_assets: tuple[Path, Path],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    harness = _Harness()
    _install_harness(monkeypatch, harness)

    def failed_start(tail: demo._BoundedLogTail, *args: Any, **kwargs: Any) -> None:
        tail._append("stderr", "PARTIAL_START_LOG\n")
        raise RuntimeError("log drain startup failed")

    monkeypatch.setattr(demo._BoundedLogTail, "start", failed_start)

    with pytest.raises(RuntimeError) as caught:
        _smoke(unit_assets)

    message = str(caught.value)
    assert "log drain startup failed" in message
    assert "server log tail" in message
    assert "PARTIAL_START_LOG" in message
    assert "latest topic list:\n<none>" in message
    assert 48129 not in harness.alive_groups


def test_joint_state_timeout_maps_exactly_and_still_cleans_up(
    unit_assets: tuple[Path, Path],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    harness = _Harness(state_timeout_at=1, service_stdout="Request timed out", wait_behaviors=["exit"])
    _install_harness(monkeypatch, harness)

    with pytest.raises(RuntimeError) as caught:
        _smoke(unit_assets)

    assert str(caught.value) == "joint-state timeout"
    assert any(
        "server log tail" in note and "latest topic list" in note for note in demo._exception_notes(caught.value)
    )
    assert harness.kill_calls == [(49003, signal.SIGKILL), (48129, signal.SIGTERM)]


def test_server_early_exit_is_diagnostic_and_cleaned(
    unit_assets: tuple[Path, Path],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    harness = _Harness(early_returncode=23, server_stderr="physics engine crashed\n")
    _install_harness(monkeypatch, harness)

    with pytest.raises(RuntimeError, match="server exited.*23") as caught:
        _smoke(unit_assets)

    assert "physics engine crashed" in str(caught.value)
    assert "latest topic list" in str(caught.value)


def test_server_early_exit_defers_pipe_finish_until_after_group_cleanup() -> None:
    class TrackingTail:
        finish_called = False

        def finish(self) -> None:
            self.finish_called = True

        def text(self) -> str:
            return ""

        def critical_error(self) -> None:
            return None

    tail = TrackingTail()
    server = SimpleNamespace(poll=lambda: 23)

    with pytest.raises(RuntimeError, match="server exited.*23"):
        demo._check_server(server, tail)

    assert not tail.finish_called


def test_missing_state_type_fails_with_diagnostics_and_cleanup(
    unit_assets: tuple[Path, Path],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    harness = _Harness(topic_info="Publishers:\nnode, gz.msgs.String\n")
    _install_harness(monkeypatch, harness)

    with pytest.raises(RuntimeError, match="gz.msgs.Model") as caught:
        _smoke(unit_assets)

    assert "latest topic list" in str(caught.value)


def test_state_type_requires_model_on_publisher_not_only_subscriber(
    unit_assets: tuple[Path, Path],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    harness = _Harness(
        topic_info=(
            "Publishers [Address, Message Type]:\n"
            "tcp://publisher, gz.msgs.String\n"
            "Subscribers [Address, Message Type]:\n"
            "tcp://subscriber, gz.msgs.Model\n"
        )
    )
    _install_harness(monkeypatch, harness)

    with pytest.raises(RuntimeError, match="publish.*gz.msgs.Model"):
        _smoke(unit_assets)


@pytest.mark.parametrize(
    "publisher_row",
    [
        "tcp://publisher",
        "tcp://publisher, gz.msgs.String, gz.msgs.Model",
    ],
)
def test_publisher_type_parser_rejects_missing_or_extra_columns(publisher_row: str) -> None:
    information = f"Publishers [Address, Message Type]:\n{publisher_row}\n"

    assert demo._publisher_message_types(information) == ()


def test_topic_discovery_polls_until_both_exact_topics_exist(
    unit_assets: tuple[Path, Path],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    harness = _Harness(topic_lists=[f"{RIGHT_COMMAND_TOPIC}-wrong\n", f"{RIGHT_COMMAND_TOPIC}\n{RIGHT_STATE_TOPIC}\n"])
    _install_harness(monkeypatch, harness)
    monkeypatch.setattr(demo, "_POLL_INTERVAL", 0.0)

    _smoke(unit_assets)

    list_calls = [call for call, _ in harness.command_calls if _Harness.inner(call)[-1] == "-l"]
    assert len(list_calls) == 2


def test_missing_topics_timeout_with_latest_list_and_cleanup(
    unit_assets: tuple[Path, Path],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    harness = _Harness(topic_lists=["/wrong/topic\n"])
    _install_harness(monkeypatch, harness)
    monkeypatch.setattr(demo, "_DISCOVERY_MOTION_TIMEOUT", 0.005)
    monkeypatch.setattr(demo, "_POLL_INTERVAL", 0.0)

    with pytest.raises(RuntimeError, match="topic discovery timeout") as caught:
        _smoke(unit_assets)

    assert "/wrong/topic" in str(caught.value)
    assert harness.process is not None and harness.process.returncode == 0


@pytest.mark.parametrize("later", [0.0, -0.03])
def test_no_progress_toward_target_times_out(
    unit_assets: tuple[Path, Path],
    monkeypatch: pytest.MonkeyPatch,
    later: float,
) -> None:
    harness = _Harness(states=[_state(None), _state(later)])
    _install_harness(monkeypatch, harness)
    monkeypatch.setattr(demo, "_DISCOVERY_MOTION_TIMEOUT", 0.01)
    monkeypatch.setattr(demo, "_POLL_INTERVAL", 0.0)

    with pytest.raises(RuntimeError, match="motion timeout"):
        _smoke(unit_assets)

    assert harness.state_calls >= 2
    assert harness.process is not None and harness.process.returncode == 0


def test_bounds_violation_in_later_state_fails_and_cleans_up(
    unit_assets: tuple[Path, Path],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    harness = _Harness(states=[_state(None), _state(1000.0)])
    _install_harness(monkeypatch, harness)

    with pytest.raises(RuntimeError, match="outside source limits"):
        _smoke(unit_assets)

    assert harness.process is not None and harness.process.returncode == 0


def test_runtime_capture_safely_accepts_complete_concatenated_transport_frames(
    unit_assets: tuple[Path, Path],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    harness = _Harness(states=[_state(None), _state(0.02) + "\n" + _state(0.03)])
    _install_harness(monkeypatch, harness)

    result = _smoke(unit_assets)

    assert result.final_position == 0.03


def test_runtime_capture_rejects_any_out_of_bounds_concatenated_frame(
    unit_assets: tuple[Path, Path],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    harness = _Harness(states=[_state(None), _state(0.03) + "\n" + _state(1000.0)])
    _install_harness(monkeypatch, harness)

    with pytest.raises(RuntimeError, match="outside source limits"):
        _smoke(unit_assets)


def test_success_is_rejected_if_server_crashes_during_final_capture(
    unit_assets: tuple[Path, Path],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    harness = _Harness(crash_after_state_at=2, service_stdout="data: false\n")
    _install_harness(monkeypatch, harness)

    with pytest.raises(RuntimeError, match="server exited.*31"):
        _smoke(unit_assets)


def test_final_sample_arriving_after_deadline_is_not_accepted(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    spec = demo.load_model_specs()[0]
    command_topic = f"/a1_demo/{spec.slug}/{RIGHT_JOINT}/cmd_pos"
    state_topic = f"/world/a1_demo/model/{spec.slug}/joint_state"
    clock = SimpleNamespace(value=0.0)
    captures = 0

    def fake_run_harmonic(**kwargs: Any) -> demo._CommandResult:
        command = kwargs["command"]
        if command[-1] == "-l":
            return demo._CommandResult(0, f"{command_topic}\n{state_topic}\n", "")
        if "-i" in command:
            return demo._CommandResult(0, "Publishers:\nnode, gz.msgs.Model\n", "")
        return demo._CommandResult(0, "", "")

    def fake_capture_position(**kwargs: Any) -> float:
        nonlocal captures
        captures += 1
        if captures == 1:
            return 0.0
        clock.value = 11.0
        return 0.03

    monkeypatch.setattr(demo, "_run_harmonic", fake_run_harmonic)
    monkeypatch.setattr(demo, "_capture_position", fake_capture_position)
    monkeypatch.setattr(demo.time, "monotonic", lambda: clock.value)
    server = SimpleNamespace(poll=lambda: None)
    tail = demo._BoundedLogTail()

    with pytest.raises(RuntimeError, match="motion timeout"):
        demo._run_motion(
            server=server,
            log_tail=tail,
            partition="partition",
            private_export=Path("/private"),
            spec=spec,
            joint=RIGHT_JOINT,
            target=0.05,
            lower=-1.0,
            upper=1.0,
            deadline=10.0,
            latest_topics=[""],
        )


def test_final_server_check_crossing_deadline_is_not_accepted(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    spec = demo.load_model_specs()[0]
    command_topic = f"/a1_demo/{spec.slug}/{RIGHT_JOINT}/cmd_pos"
    state_topic = f"/world/a1_demo/model/{spec.slug}/joint_state"
    clock = SimpleNamespace(value=0.0)
    captures = iter((0.0, 0.03))
    server_checks = 0

    def fake_run_harmonic(**kwargs: Any) -> demo._CommandResult:
        command = kwargs["command"]
        if command[-1] == "-l":
            return demo._CommandResult(0, f"{command_topic}\n{state_topic}\n", "")
        if "-i" in command:
            return demo._CommandResult(0, "Publishers:\nnode, gz.msgs.Model\n", "")
        return demo._CommandResult(0, "", "")

    def final_check_crosses_deadline(*args: Any, **kwargs: Any) -> None:
        nonlocal server_checks
        server_checks += 1
        if server_checks == 4:
            clock.value = 11.0

    monkeypatch.setattr(demo, "_run_harmonic", fake_run_harmonic)
    monkeypatch.setattr(demo, "_capture_position", lambda **kwargs: next(captures))
    monkeypatch.setattr(demo, "_check_server", final_check_crosses_deadline)
    monkeypatch.setattr(demo.time, "monotonic", lambda: clock.value)

    with pytest.raises(RuntimeError, match="motion timeout"):
        demo._run_motion(
            server=SimpleNamespace(poll=lambda: None),
            log_tail=demo._BoundedLogTail(),
            partition="partition",
            private_export=Path("/private"),
            spec=spec,
            joint=RIGHT_JOINT,
            target=0.05,
            lower=-1.0,
            upper=1.0,
            deadline=10.0,
            latest_topics=[""],
        )


def test_command_deadline_is_recomputed_after_slow_popen(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    harness = _Harness()
    _install_harness(monkeypatch, harness)
    clock = SimpleNamespace(value=5.0)

    def delayed_popen(args: list[str], **kwargs: Any) -> _FakeProcess | _FakeCommandProcess:
        process = harness.popen(args, **kwargs)
        clock.value = 11.0
        return process

    monkeypatch.setattr(demo.subprocess, "Popen", delayed_popen)
    monkeypatch.setattr(demo.time, "monotonic", lambda: clock.value)

    with pytest.raises(RuntimeError, match="command deadline"):
        demo._run_harmonic(
            partition="partition",
            private_export=Path("/private"),
            command=["topic", "--force-version", "13", "-l"],
            timeout=3.0,
            timeout_message="command deadline",
            deadline=10.0,
        )

    assert harness.kill_calls == [(49001, signal.SIGKILL)]


def test_bounded_communicate_rejects_combined_stdout_and_stderr() -> None:
    stdout_read, stdout_write = os.pipe()
    stderr_read, stderr_write = os.pipe()
    os.write(stdout_write, b"a" * 40)
    os.write(stderr_write, b"b" * 40)
    os.close(stdout_write)
    os.close(stderr_write)
    stdout = os.fdopen(stdout_read, "r", encoding="utf-8")
    stderr = os.fdopen(stderr_read, "r", encoding="utf-8")
    process = SimpleNamespace(stdout=stdout, stderr=stderr, poll=lambda: 0, wait=lambda timeout=None: 0)
    try:
        with pytest.raises(RuntimeError, match="output exceeds"):
            demo._communicate_bounded(process, timeout=1.0, output_limit_bytes=64)
    finally:
        stdout.close()
        stderr.close()


def test_run_harmonic_routes_limited_commands_through_bounded_reader(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    harness = _Harness()
    _install_harness(monkeypatch, harness)
    calls: list[tuple[float, int]] = []

    def bounded(process: _FakeCommandProcess, *, timeout: float, output_limit_bytes: int) -> tuple[str, str]:
        calls.append((timeout, output_limit_bytes))
        process.returncode = 0
        harness.alive_groups.discard(process.pid)
        return "bounded stdout", ""

    monkeypatch.setattr(demo, "_communicate_bounded", bounded, raising=False)

    result = demo._run_harmonic(
        partition="partition",
        private_export=Path("/private"),
        command=["topic", "--force-version", "13", "-l"],
        timeout=3.0,
        timeout_message="topic timeout",
        output_limit_bytes=64,
    )

    assert result.stdout == "bounded stdout"
    assert calls == [(3.0, 64)]
    assert harness.kill_calls == []


def test_success_is_rejected_if_server_exits_during_unverified_stop_request(
    unit_assets: tuple[Path, Path],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    harness = _Harness(
        service_stdout="data: false\n",
        exit_during_service_returncode=37,
    )
    _install_harness(monkeypatch, harness)

    with pytest.raises(RuntimeError, match="exited before verified stop.*37"):
        _smoke(unit_assets)


def test_unverified_server_disappearance_before_fallback_signal_is_rejected(
    unit_assets: tuple[Path, Path],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    harness = _Harness(
        service_stdout="data: false\n",
        disappear_on_server_probe_after_service=True,
    )
    _install_harness(monkeypatch, harness)

    with pytest.raises(RuntimeError, match="exited before verified stop.*37"):
        _smoke(unit_assets)

    assert harness.server_probe_disappearance_triggered
    assert not any(pid == 48129 and sent_signal != 0 for pid, sent_signal in harness.kill_calls)


def test_success_checks_fatal_log_drained_only_during_finish(
    unit_assets: tuple[Path, Path],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class LateCriticalTail:
        finished = False

        def start(self, stdout: Any, stderr: Any) -> None:
            return None

        def finish(self) -> None:
            self.finished = True

        def text(self) -> str:
            return "[Err] late resource load failure" if self.finished else ""

        def critical_error(self) -> str | None:
            return "[Err] late resource load failure" if self.finished else None

        def problems(self) -> tuple[str, ...]:
            return ()

    harness = _Harness()
    _install_harness(monkeypatch, harness)
    monkeypatch.setattr(demo, "_BoundedLogTail", LateCriticalTail)

    with pytest.raises(RuntimeError, match="late resource load failure") as caught:
        _smoke(unit_assets)

    assert "server log tail" in str(caught.value)
    assert "latest topic list" in str(caught.value)


def test_plugin_or_physics_load_error_is_rejected(
    unit_assets: tuple[Path, Path],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    harness = _Harness(server_stderr="[Err] Failed to load physics plugin gz-physics-dartsim-plugin\n")
    _install_harness(monkeypatch, harness)

    with pytest.raises(RuntimeError, match="plugin|physics"):
        _smoke(unit_assets)


@pytest.mark.parametrize(
    "message",
    [
        "[Err] [MeshManager.cc:1] Unable to find mesh missing.stl\n",
        "[Err] [Server.cc:1] Failed to load resource model.sdf\n",
        "[Fatal] [Server.cc:1] transport invariant broke\n",
    ],
)
def test_every_error_or_fatal_level_log_is_sticky(message: str) -> None:
    tail = demo._BoundedLogTail(maximum=128)
    tail._append("stderr", message)
    tail._append("stderr", "x" * 4096)
    server = SimpleNamespace(poll=lambda: None)

    with pytest.raises(RuntimeError, match="error|fatal|load"):
        demo._check_server(server, tail)


def test_physics_warning_is_sticky_after_bounded_tail_rolls_over() -> None:
    tail = demo._BoundedLogTail(maximum=128)
    tail._append("stderr", "[Wrn] [Physics.cc:1] physics engine fallback\n")
    tail._append("stderr", "x" * 4096)
    server = SimpleNamespace(poll=lambda: None)

    with pytest.raises(RuntimeError, match="physics"):
        demo._check_server(server, tail)


def test_sticky_physics_warning_survives_stream_chunk_boundary() -> None:
    tail = demo._BoundedLogTail(maximum=128)
    tail._append("stderr", "prefix [Wr")
    tail._append("stderr", "n] [Physics.cc:1] fallback\n")
    tail._append("stderr", "x" * 4096)
    server = SimpleNamespace(poll=lambda: None)

    with pytest.raises(RuntimeError, match="physics"):
        demo._check_server(server, tail)


def test_short_live_log_line_is_drained_before_server_pipe_closes() -> None:
    read_descriptor, write_descriptor = os.pipe()
    reader = os.fdopen(read_descriptor, "r", encoding="utf-8")
    tail = demo._BoundedLogTail()
    tail.start(None, reader)
    try:
        os.write(write_descriptor, b"[Wrn] physics live marker\n")
        deadline = time.monotonic() + 0.5
        while tail.critical_error() is None and time.monotonic() < deadline:
            time.sleep(0.01)
        assert tail.critical_error() is not None
    finally:
        os.close(write_descriptor)
        tail.finish()


def test_log_drain_read_failure_prevents_smoke_success_with_diagnostics(
    unit_assets: tuple[Path, Path],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class BrokenStream(io.StringIO):
        def readline(self, *args: Any, **kwargs: Any) -> str:
            raise OSError("pipe read failed")

    harness = _Harness(server_stdout_stream=BrokenStream())
    _install_harness(monkeypatch, harness)

    with pytest.raises(RuntimeError) as caught:
        _smoke(unit_assets)

    message = str(caught.value)
    assert "server cleanup failed" in message
    assert "stdout log drain failed: pipe read failed" in message
    assert "server log tail" in message
    assert "latest topic list" in message


def test_nonterminating_log_drain_thread_prevents_smoke_success(
    unit_assets: tuple[Path, Path],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class NeverStoppingThread:
        def __init__(self, *args: Any, **kwargs: Any) -> None:
            pass

        def start(self) -> None:
            pass

        def join(self, timeout: float | None = None) -> None:
            pass

        def is_alive(self) -> bool:
            return True

    harness = _Harness()
    _install_harness(monkeypatch, harness)
    monkeypatch.setattr(demo.threading, "Thread", NeverStoppingThread)

    with pytest.raises(RuntimeError) as caught:
        _smoke(unit_assets)

    message = str(caught.value)
    assert "server cleanup failed" in message
    assert "log drain thread did not stop" in message
    assert "server log tail" in message
    assert "latest topic list" in message


def test_log_drain_first_join_timeout_prevents_smoke_success(
    unit_assets: tuple[Path, Path],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class EventuallyStoppingThread:
        def __init__(self, *args: Any, **kwargs: Any) -> None:
            self.alive_checks = 0

        def start(self) -> None:
            pass

        def join(self, timeout: float | None = None) -> None:
            pass

        def is_alive(self) -> bool:
            self.alive_checks += 1
            return self.alive_checks == 1

    harness = _Harness()
    _install_harness(monkeypatch, harness)
    monkeypatch.setattr(demo.threading, "Thread", EventuallyStoppingThread)

    with pytest.raises(RuntimeError) as caught:
        _smoke(unit_assets)

    assert "log drain thread did not stop" in str(caught.value)


def test_final_log_read_error_prevents_smoke_success(
    unit_assets: tuple[Path, Path],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class BrokenStream(io.StringIO):
        def readline(self, *args: Any, **kwargs: Any) -> str:
            raise OSError("final drain failed")

    class DrainDuringJoinThread:
        def __init__(self, *, target: Any, args: tuple[Any, ...], **kwargs: Any) -> None:
            self.target = target
            self.args = args
            self.drained = False

        def start(self) -> None:
            pass

        def join(self, timeout: float | None = None) -> None:
            if not self.drained:
                self.drained = True
                self.target(*self.args)

        def is_alive(self) -> bool:
            return False

    harness = _Harness(server_stdout_stream=BrokenStream())
    _install_harness(monkeypatch, harness)
    monkeypatch.setattr(demo.threading, "Thread", DrainDuringJoinThread)

    with pytest.raises(RuntimeError) as caught:
        _smoke(unit_assets)

    assert "stdout log drain failed: final drain failed" in str(caught.value)


def test_exit_zero_service_timeout_text_is_not_graceful_success(
    unit_assets: tuple[Path, Path],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    harness = _Harness(service_stdout="Service call timed out\n", service_returncode=0, wait_behaviors=["exit"])
    _install_harness(monkeypatch, harness)

    _smoke(unit_assets)

    assert harness.kill_calls == [(48129, signal.SIGTERM)]


def test_surviving_server_control_helper_group_is_a_cleanup_failure(
    unit_assets: tuple[Path, Path],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    harness = _Harness(
        service_timeout=True,
        child_sigkill_leaves_group_alive=True,
    )
    _install_harness(monkeypatch, harness)

    with pytest.raises(RuntimeError) as caught:
        _smoke(unit_assets)

    message = str(caught.value)
    assert "server cleanup failed" in message
    assert "child process group 49006 survived SIGKILL" in message
    assert "server log tail" in message
    assert "latest topic list" in message


def test_server_control_cleanup_interrupt_preserves_notes_and_diagnostics(
    unit_assets: tuple[Path, Path],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    harness = _Harness()
    _install_harness(monkeypatch, harness)
    real_run_harmonic = demo._run_harmonic

    def interrupted_service(**kwargs: Any) -> demo._CommandResult:
        if kwargs["command"][0] == "service":
            error = KeyboardInterrupt("service cleanup interrupted")
            demo._add_exception_note(error, "child process group 49006 survived SIGKILL")
            raise error
        return real_run_harmonic(**kwargs)

    monkeypatch.setattr(demo, "_run_harmonic", interrupted_service)

    with pytest.raises(KeyboardInterrupt, match="service cleanup interrupted") as caught:
        _smoke(unit_assets)

    notes = "\n".join(demo._exception_notes(caught.value))
    assert "child process group 49006 survived SIGKILL" in notes
    assert "server log tail" in notes
    assert "latest topic list" in notes
    assert 48129 not in harness.alive_groups


def test_real_server_control_timeout_with_cleanup_interrupt_cannot_be_swallowed(
    unit_assets: tuple[Path, Path],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    harness = _Harness(service_timeout=True)
    _install_harness(monkeypatch, harness)
    real_kill_child_group = demo._kill_child_group

    def interrupted_helper_cleanup(process: Any) -> demo._GroupCleanupResult:
        if process.pid >= 49000 and harness.service_dispatched:
            harness.alive_groups.discard(process.pid)
            return demo._GroupCleanupResult(True, KeyboardInterrupt("helper cleanup interrupted"))
        return real_kill_child_group(process)

    monkeypatch.setattr(demo, "_kill_child_group", interrupted_helper_cleanup)

    with pytest.raises(KeyboardInterrupt, match="helper cleanup interrupted") as caught:
        _smoke(unit_assets)

    notes = "\n".join(demo._exception_notes(caught.value))
    assert "server-control timeout" in notes
    assert "server log tail" in notes
    assert "latest topic list" in notes
    assert 48129 not in harness.alive_groups


def test_graceful_stop_requires_semantic_true_response(
    unit_assets: tuple[Path, Path],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    harness = _Harness(service_stdout="data: true\n", wait_behaviors=["exit"])
    _install_harness(monkeypatch, harness)

    _smoke(unit_assets)

    assert harness.kill_calls == []
    assert harness.wait_timeouts == [5.0]


def test_stop_falls_back_from_sigterm_to_sigkill(
    unit_assets: tuple[Path, Path],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    harness = _Harness(
        service_stdout="data: false\n",
        wait_behaviors=["timeout", "exit"],
    )
    _install_harness(monkeypatch, harness)

    _smoke(unit_assets)

    assert harness.kill_calls == [(48129, signal.SIGTERM), (48129, signal.SIGKILL)]
    assert harness.wait_timeouts == [5.0, 5.0]


def test_stop_kills_surviving_group_after_leader_has_exited(
    unit_assets: tuple[Path, Path],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    export_root, _ = unit_assets
    harness = _Harness(
        early_returncode=0,
        leader_exited_group_alive=True,
        sigterm_leaves_group_alive=True,
        service_stdout="data: false\n",
    )
    _install_harness(monkeypatch, harness)
    monkeypatch.setattr(demo, "_STOP_TIMEOUT", 0.0)
    server = harness.popen(
        demo._harmonic_argv("partition", export_root, ["sim", "world.sdf"]),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    with pytest.raises(RuntimeError, match="exited before verified stop.*0"):
        demo._stop_server(server, partition="partition", private_export=export_root)

    assert harness.kill_calls == [(48129, signal.SIGTERM), (48129, signal.SIGKILL)]
    assert 48129 not in harness.alive_groups


def test_cleanup_failure_on_success_has_server_and_topic_diagnostics(
    unit_assets: tuple[Path, Path],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    harness = _Harness(
        server_stderr="CLEANUP_LOG_MARKER\n",
        service_stdout="data: false\n",
        sigterm_leaves_group_alive=True,
        sigkill_leaves_group_alive=True,
        wait_behaviors=["timeout", "timeout"],
    )
    _install_harness(monkeypatch, harness)
    monkeypatch.setattr(demo, "_STOP_TIMEOUT", 0.0)

    with pytest.raises(RuntimeError) as caught:
        _smoke(unit_assets)

    message = str(caught.value)
    assert "server cleanup failed" in message
    assert "server log tail" in message
    assert "CLEANUP_LOG_MARKER" in message
    assert "latest topic list" in message


@pytest.mark.parametrize("motion_failure", [False, True])
def test_cleanup_error_notes_survive_diagnostic_arbitration(
    unit_assets: tuple[Path, Path],
    monkeypatch: pytest.MonkeyPatch,
    motion_failure: bool,
) -> None:
    topic_info = "Publishers:\nnode, gz.msgs.String\n" if motion_failure else None
    harness = _Harness(**({"topic_info": topic_info} if topic_info is not None else {}))
    _install_harness(monkeypatch, harness)

    def noted_stop_failure(*args: Any, **kwargs: Any) -> None:
        error = RuntimeError("stop failed")
        demo._add_exception_note(error, "STOP_FAILURE_NOTE")
        raise error

    monkeypatch.setattr(demo, "_stop_server", noted_stop_failure)
    monkeypatch.setattr(
        demo,
        "_kill_child_group",
        lambda process: demo._GroupCleanupResult(False, None),
    )

    with pytest.raises(RuntimeError) as caught:
        _smoke(unit_assets)

    rendered = str(caught.value) + "\n" + "\n".join(demo._exception_notes(caught.value))
    assert "STOP_FAILURE_NOTE" in rendered
    assert "survived emergency SIGKILL" in rendered


@pytest.mark.parametrize("preexisting_error", ["motion", "cleanup"])
def test_nested_emergency_cleanup_notes_survive_arbitration(
    unit_assets: tuple[Path, Path],
    monkeypatch: pytest.MonkeyPatch,
    preexisting_error: str,
) -> None:
    topic_info = (
        "Publishers:\nnode, gz.msgs.String\n" if preexisting_error == "motion" else "Publishers:\nnode, gz.msgs.Model\n"
    )
    harness = _Harness(topic_info=topic_info)
    _install_harness(monkeypatch, harness)
    real_kill_child_group = demo._kill_child_group

    if preexisting_error == "cleanup":
        monkeypatch.setattr(
            demo,
            "_stop_server",
            lambda *args, **kwargs: (_ for _ in ()).throw(RuntimeError("stop failed")),
        )

    def emergency_cleanup_with_nested_note(process: Any) -> demo._GroupCleanupResult:
        if process.pid == 48129:
            interruption = KeyboardInterrupt("first cleanup interruption")
            demo._add_exception_note(interruption, "SECOND_CLEANUP_NOTE")
            harness.alive_groups.discard(process.pid)
            return demo._GroupCleanupResult(True, interruption)
        return real_kill_child_group(process)

    monkeypatch.setattr(demo, "_kill_child_group", emergency_cleanup_with_nested_note)

    with pytest.raises(RuntimeError) as caught:
        _smoke(unit_assets)

    rendered = str(caught.value) + "\n" + "\n".join(demo._exception_notes(caught.value))
    assert "first cleanup interruption" in rendered
    assert "SECOND_CLEANUP_NOTE" in rendered


def test_cleanup_keyboard_interrupt_still_emergency_kills_server_group(
    unit_assets: tuple[Path, Path],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    harness = _Harness()
    _install_harness(monkeypatch, harness)

    def interrupted_stop(*args: Any, **kwargs: Any) -> None:
        raise KeyboardInterrupt

    monkeypatch.setattr(demo, "_stop_server", interrupted_stop)

    with pytest.raises(KeyboardInterrupt):
        _smoke(unit_assets)

    assert (48129, signal.SIGKILL) in harness.kill_calls
    assert 48129 not in harness.alive_groups


@pytest.mark.parametrize("error_type", [RuntimeError, KeyboardInterrupt])
def test_log_tail_start_failure_cleans_server_group(
    unit_assets: tuple[Path, Path],
    monkeypatch: pytest.MonkeyPatch,
    error_type: type[BaseException],
) -> None:
    harness = _Harness()
    _install_harness(monkeypatch, harness)

    def failed_start(*args: Any, **kwargs: Any) -> None:
        raise error_type("log drain startup failed")

    monkeypatch.setattr(demo._BoundedLogTail, "start", failed_start)

    with pytest.raises(error_type, match="log drain startup failed"):
        _smoke(unit_assets)

    assert 48129 not in harness.alive_groups


@pytest.mark.parametrize("interrupted_phase", ["after_setup", "after_motion"])
def test_async_interrupt_between_server_phases_cannot_escape_cleanup(
    unit_assets: tuple[Path, Path],
    monkeypatch: pytest.MonkeyPatch,
    interrupted_phase: str,
) -> None:
    harness = _Harness(service_stdout="data: false\n")
    _install_harness(monkeypatch, harness)

    def checkpoint(phase: str) -> None:
        if phase == interrupted_phase:
            raise KeyboardInterrupt(f"interrupted {phase}")

    monkeypatch.setattr(demo, "_lifecycle_checkpoint", checkpoint, raising=False)

    with pytest.raises(KeyboardInterrupt, match=interrupted_phase):
        _smoke(unit_assets)

    assert 48129 not in harness.alive_groups


def test_emergency_cleanup_defers_keyboard_interrupt_until_group_is_dead(
    unit_assets: tuple[Path, Path],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    harness = _Harness()
    _install_harness(monkeypatch, harness)
    interrupted = False

    def apparently_successful_stop(*args: Any, **kwargs: Any) -> None:
        return None

    def interrupt_once(pid: int, sent_signal: signal.Signals | int) -> None:
        nonlocal interrupted
        if sent_signal == signal.SIGKILL and not interrupted:
            interrupted = True
            raise KeyboardInterrupt
        harness.killpg(pid, sent_signal)

    monkeypatch.setattr(demo, "_stop_server", apparently_successful_stop)
    monkeypatch.setattr(demo.os, "killpg", interrupt_once)

    with pytest.raises(KeyboardInterrupt):
        _smoke(unit_assets)

    assert interrupted
    assert (48129, signal.SIGKILL) in harness.kill_calls
    assert 48129 not in harness.alive_groups


def test_timed_out_child_kills_surviving_group_after_leader_has_exited(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    harness = _Harness(sigterm_leaves_group_alive=True)
    _install_harness(monkeypatch, harness)
    process = _FakeCommandProcess(harness, [], {}, 49999)
    process.returncode = 0
    harness.alive_groups.add(process.pid)

    demo._kill_child_group(process)

    assert harness.kill_calls == [(49999, signal.SIGKILL)]
    assert process.pid not in harness.alive_groups


def test_successful_command_rejects_and_kills_surviving_process_group(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    harness = _Harness(command_group_survives_return=True)
    _install_harness(monkeypatch, harness)

    with pytest.raises(RuntimeError, match="process group remained alive"):
        demo._run_harmonic(
            partition="partition",
            private_export=Path("/private"),
            command=["topic", "--force-version", "13", "-l"],
            timeout=3.0,
            timeout_message="topic timeout",
        )

    assert harness.kill_calls == [(49001, signal.SIGKILL)]
    assert 49001 not in harness.alive_groups


def test_interrupt_after_helper_popen_cannot_escape_group_cleanup(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    harness = _Harness()
    _install_harness(monkeypatch, harness)

    def interrupted_clock() -> float:
        raise KeyboardInterrupt("clock interrupted after Popen")

    monkeypatch.setattr(demo.time, "monotonic", interrupted_clock)

    with pytest.raises(KeyboardInterrupt, match="clock interrupted"):
        demo._run_harmonic(
            partition="partition",
            private_export=Path("/private"),
            command=["topic", "--force-version", "13", "-l"],
            timeout=3.0,
            timeout_message="topic timeout",
            deadline=10.0,
        )

    assert harness.kill_calls == [(49001, signal.SIGKILL)]
    assert 49001 not in harness.alive_groups


@pytest.mark.parametrize("cleaned", [True, False])
def test_child_cleanup_interruption_keeps_operation_failure_primary(
    monkeypatch: pytest.MonkeyPatch,
    cleaned: bool,
) -> None:
    harness = _Harness(state_timeout_at=1)
    _install_harness(monkeypatch, harness)

    def interrupted_incomplete_cleanup(process: Any) -> demo._GroupCleanupResult:
        return demo._GroupCleanupResult(cleaned, KeyboardInterrupt("cleanup interrupted"))

    monkeypatch.setattr(demo, "_kill_child_group", interrupted_incomplete_cleanup)

    with pytest.raises(RuntimeError) as caught:
        demo._run_harmonic(
            partition="partition",
            private_export=Path("/private"),
            command=[
                "topic",
                "--force-version",
                "13",
                "-e",
                "-n",
                "1",
                "--json-output",
                "-t",
                RIGHT_STATE_TOPIC,
            ],
            timeout=3.0,
            timeout_message="joint-state timeout",
        )

    assert str(caught.value) == "joint-state timeout"
    notes = "\n".join(demo._exception_notes(caught.value))
    assert "cleanup interruption: KeyboardInterrupt: cleanup interrupted" in notes
    if cleaned:
        assert "survived SIGKILL" not in notes
    else:
        assert "child process group 49001 survived SIGKILL" in notes


def test_eintr_group_probe_does_not_false_report_child_cleanup(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    harness = _Harness()
    _install_harness(monkeypatch, harness)
    process = _FakeCommandProcess(harness, [], {}, 49999)
    harness.alive_groups.add(process.pid)
    first_probe = True

    def interrupted_once(pid: int, sent_signal: signal.Signals | int) -> None:
        nonlocal first_probe
        if sent_signal == 0 and first_probe:
            first_probe = False
            raise OSError(errno.EINTR, "group probe interrupted")
        harness.killpg(pid, sent_signal)

    monkeypatch.setattr(demo.os, "killpg", interrupted_once)

    cleanup = demo._kill_child_group(process)

    assert cleanup.cleaned
    assert harness.kill_calls == [(49999, signal.SIGKILL)]
    assert process.pid not in harness.alive_groups


@pytest.mark.parametrize("failure", ["interrupt", "oserror"])
def test_child_communication_failure_cleans_transient_and_server_groups(
    unit_assets: tuple[Path, Path],
    monkeypatch: pytest.MonkeyPatch,
    failure: str,
) -> None:
    options = {"interrupt_state_at": 1} if failure == "interrupt" else {"os_error_state_at": 1}
    harness = _Harness(service_stdout="data: false\n", **options)
    _install_harness(monkeypatch, harness)

    expected = KeyboardInterrupt if failure == "interrupt" else RuntimeError
    with pytest.raises(expected):
        _smoke(unit_assets)

    assert not harness.alive_groups
    assert any(pid >= 49000 and sent_signal == signal.SIGKILL for pid, sent_signal in harness.kill_calls)


def test_failure_log_tail_is_bounded_and_keeps_latest_bytes(
    unit_assets: tuple[Path, Path],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    ending = "LATEST_SERVER_LOG_MARKER"
    harness = _Harness(
        early_returncode=17,
        server_stderr="x" * (256 * 1024) + ending,
    )
    _install_harness(monkeypatch, harness)

    with pytest.raises(RuntimeError) as caught:
        _smoke(unit_assets)

    message = str(caught.value)
    assert ending in message
    assert len(message) < 70 * 1024


def test_cli_never_prints_final_success_when_third_model_fails(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
    tmp_path: Path,
) -> None:
    calls = 0

    def fake_smoke_test(**kwargs: Any) -> SimpleNamespace:
        nonlocal calls
        calls += 1
        if calls == 3:
            raise RuntimeError("third model failed")
        return SimpleNamespace(
            model=kwargs["model"],
            joint=kwargs["joint"],
            initial_position=0.0,
            final_position=0.03,
            delta=0.03,
        )

    monkeypatch.setattr(demo, "smoke_test", fake_smoke_test)

    status = demo.main(
        [
            "smoke-all",
            "--models",
            str(tmp_path / "models"),
            "--worlds",
            str(tmp_path / "worlds"),
        ]
    )

    captured = capsys.readouterr()
    assert status == 1
    assert calls == 3
    assert "SMOKE_TEST_VALID" not in captured.out
    assert "SMOKE_TEST_OK" not in captured.out
    assert "third model failed" in captured.err


def test_cli_prints_truthful_four_case_success_footer_after_all_motions_pass(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
    tmp_path: Path,
) -> None:
    calls: list[tuple[str, str, float]] = []

    def fake_smoke_test(**kwargs: Any) -> SimpleNamespace:
        calls.append((kwargs["model"], kwargs["joint"], kwargs["target"]))
        return SimpleNamespace(
            model=kwargs["model"],
            joint=kwargs["joint"],
            initial_position=0.0,
            final_position=0.03,
            delta=0.03,
        )

    monkeypatch.setattr(demo, "smoke_test", fake_smoke_test)

    status = demo.main(
        [
            "smoke-all",
            "--models",
            str(tmp_path / "models"),
            "--worlds",
            str(tmp_path / "worlds"),
        ]
    )

    assert status == 0
    assert calls == list(demo._LIVE_CASES)
    assert capsys.readouterr().out.splitlines() == [
        "SMOKE_TEST_VALID onerobotics_a1_right_arm: joint=joint1-a1_r initial=0.000000 final=0.030000 delta=0.030000",
        "SMOKE_TEST_VALID onerobotics_a1_left_arm: joint=joint1-a1_l initial=0.000000 final=0.030000 delta=0.030000",
        "SMOKE_TEST_VALID onerobotics_a1_bimanual_stand: joint=joint_r1 initial=0.000000 final=0.030000 delta=0.030000",
        "SMOKE_TEST_VALID onerobotics_a1_bimanual_stand: joint=joint_l1 initial=0.000000 final=0.030000 delta=0.030000",
        "SMOKE_TEST_OK: 4 cases across 3 models",
    ]


def test_cli_prints_exception_notes_for_joint_state_timeout(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
    tmp_path: Path,
) -> None:
    def timed_out(**kwargs: Any) -> SimpleNamespace:
        error = RuntimeError("joint-state timeout")
        demo._add_exception_note(error, "server log tail:\nFATAL_MARKER\nlatest topic list:\n/state")
        raise error

    monkeypatch.setattr(demo, "smoke_test", timed_out)

    status = demo.main(
        [
            "smoke-all",
            "--models",
            str(tmp_path / "models"),
            "--worlds",
            str(tmp_path / "worlds"),
        ]
    )

    captured = capsys.readouterr()
    assert status == 1
    assert "joint-state timeout" in captured.err
    assert "server log tail" in captured.err
    assert "FATAL_MARKER" in captured.err
    assert "latest topic list" in captured.err


def test_cli_prints_compatibility_notes_before_reraising_keyboard_interrupt(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
    tmp_path: Path,
) -> None:
    def interrupted(**kwargs: Any) -> SimpleNamespace:
        error = KeyboardInterrupt("cleanup interrupted")
        setattr(
            error,
            demo._COMPAT_NOTES_ATTRIBUTE,
            ("COMPAT_SERVER_LOG_AND_TOPICS",),
        )
        raise error

    monkeypatch.setattr(demo, "smoke_test", interrupted)

    with pytest.raises(KeyboardInterrupt, match="cleanup interrupted"):
        demo.main(
            [
                "smoke-all",
                "--models",
                str(tmp_path / "models"),
                "--worlds",
                str(tmp_path / "worlds"),
            ]
        )

    captured = capsys.readouterr()
    assert "COMPAT_SERVER_LOG_AND_TOPICS" in captured.err
    assert "SMOKE_TEST_OK" not in captured.out


def test_shell_wrapper_is_cwd_independent_and_has_no_false_footer() -> None:
    script = REPOSITORY_ROOT / "gazebo/scripts/run_smoke_tests.sh"
    text = script.read_text(encoding="utf-8")

    assert text.startswith("#!/usr/bin/env bash\n")
    assert "set -euo pipefail" in text
    assert "BASH_SOURCE[0]" in text
    assert os.access(script, os.X_OK)
    assert ".venv/bin/python" in text
    assert "env -u PYTHONPATH" in text
    assert "SMOKE_TEST_OK: 3 cases across 3 models" not in text


@pytest.mark.runtime
@pytest.mark.parametrize(
    ("model", "joint", "target"),
    [
        ("onerobotics_a1_right_arm", "joint1-a1_r", 0.05),
        ("onerobotics_a1_left_arm", "joint1-a1_l", 0.05),
        ("onerobotics_a1_bimanual_stand", "joint_r1", 0.05),
        ("onerobotics_a1_bimanual_stand", "joint_l1", 0.05),
    ],
)
def test_position_command_moves_joint_toward_target(model: str, joint: str, target: float) -> None:
    result = demo.smoke_test(
        model=model,
        joint=joint,
        target=target,
        export_root=EXPORT_ROOT,
        worlds_root=WORLDS_ROOT,
    )

    assert result.model == model
    assert result.joint == joint
    assert result.target == target
    assert result.finite
    assert result.moved_toward_target
    assert result.delta >= 0.02
    assert result.lower < target < result.upper
    assert result.lower <= result.initial_position <= result.upper
    assert result.lower <= result.final_position <= result.upper


@pytest.mark.runtime
@pytest.mark.render_runtime
def test_real_ogre2_render_captures_all_three_models(tmp_path: Path) -> None:
    candidates = tmp_path / "candidates"

    result = render_module.render_candidates(EXPORT_ROOT, candidates)
    captured = render_module.capture_thumbnail_set(candidates)

    assert result == candidates
    assert tuple(captured.png_by_slug) == tuple(spec.slug for spec in load_model_specs())
