"""Generate deterministic Gazebo Harmonic demo worlds for the public A1 models."""

from __future__ import annotations

import argparse
import contextlib
import errno
import io
import json
import math
import os
import re
import secrets
import selectors
import signal
import stat
import subprocess
import sys
import tempfile
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from xml.etree import ElementTree
from xml.etree.ElementTree import Element, SubElement

from onerobotics_a1_gazebo.sdf import add_text, serialize_sdf
from onerobotics_a1_gazebo.spec import ModelSpec, load_model_specs
from onerobotics_a1_gazebo.validate import ValidationFailure, validate_all
from onerobotics_a1_gazebo.validation_snapshot import DirectorySnapshot, SnapshotError, capture_directory

_WORLD_NAME = "a1_demo"
_CONTROLLER_FILENAME = "gz-sim-joint-position-controller-system"
_CONTROLLER_NAME = "gz::sim::systems::JointPositionController"
_STATE_FILENAME = "gz-sim-joint-state-publisher-system"
_STATE_NAME = "gz::sim::systems::JointStatePublisher"
_DISCOVERY_MOTION_TIMEOUT = 30.0
_CACHE_SMOKE_TOTAL_TIMEOUT = 150.0
_CHILD_TIMEOUT = 3.0
_POLL_INTERVAL = 0.1
_MAX_JSON_BYTES = 1024 * 1024
_MAX_JSON_DEPTH = 64
_MAX_JSON_NUMBER_CHARS = 1024
_MAX_LOG_TAIL = 64 * 1024
_STOP_TIMEOUT = 5.0
_HARMONIC_HELPER = Path(__file__).resolve().parents[1] / "scripts/harmonic_env.sh"
_FRESH_SHELL = 'set -euo pipefail\nsource "$1"\nshift\na1_harmonic_run "$@"'
_LIVE_CASES = (
    ("onerobotics_a1_right_arm", "joint1-a1_r", 0.05),
    ("onerobotics_a1_left_arm", "joint1-a1_l", 0.05),
    ("onerobotics_a1_bimanual_stand", "joint_r1", 0.05),
    ("onerobotics_a1_bimanual_stand", "joint_l1", 0.05),
)
# Roughly 1 cm below each public model's zero-pose collision envelope.
_GROUND_POSE_BY_SLUG = {
    "onerobotics_a1_right_arm": "0 0 -0.025 0 0 0",
    "onerobotics_a1_left_arm": "0 0 -0.025 0 0 0",
    "onerobotics_a1_bimanual_stand": "0 0 -0.25 0 0 0",
}
_COMPAT_NOTES_ATTRIBUTE = "_onerobotics_a1_diagnostic_notes"


@dataclass(frozen=True)
class SmokeResult:
    """Finite source-bounded motion observed for one commanded Gazebo joint."""

    model: str
    joint: str
    target: float
    initial_position: float
    final_position: float
    delta: float
    lower: float
    upper: float
    finite: bool
    moved_toward_target: bool


@dataclass(frozen=True)
class _CommandResult:
    returncode: int
    stdout: str
    stderr: str


@dataclass(frozen=True)
class _GroupCleanupResult:
    cleaned: bool
    deferred_error: BaseException | None


class _JsonNumberError(ValueError):
    pass


class _HarmonicCommandError(RuntimeError):
    def __init__(
        self,
        message: str,
        *,
        cleanup_interruption: BaseException | None,
    ) -> None:
        super().__init__(message)
        self.cleanup_interruption = cleanup_interruption


class _ChildGroupCleanupError(_HarmonicCommandError):
    pass


def _add_exception_note(error: object, note: str) -> None:
    native_add_note = getattr(error, "add_note", None)
    if callable(native_add_note):
        native_add_note(note)
        return
    notes = list(getattr(error, _COMPAT_NOTES_ATTRIBUTE, ()))
    notes.append(note)
    setattr(error, _COMPAT_NOTES_ATTRIBUTE, tuple(notes))


def _exception_notes(error: object) -> tuple[str, ...]:
    native_notes = tuple(getattr(error, "__notes__", ()))
    compatibility_notes = tuple(getattr(error, _COMPAT_NOTES_ATTRIBUTE, ()))
    return native_notes + compatibility_notes


def _copy_exception_notes(source: object, destination: object) -> None:
    for note in _exception_notes(source):
        _add_exception_note(destination, note)


def _merge_cleanup_error(
    existing: BaseException | None,
    added: BaseException,
    *,
    note: str | None = None,
) -> BaseException:
    if existing is None:
        return added
    _add_exception_note(existing, note or str(added))
    return existing


class _BoundedLogTail:
    def __init__(
        self,
        maximum: int = _MAX_LOG_TAIL,
        *,
        classifier: Callable[[str], str | None] | None = None,
    ) -> None:
        self._maximum = maximum
        self._classifier = classifier
        self._text = ""
        self._lock = threading.Lock()
        self._threads: list[threading.Thread] = []
        self._streams: list[io.TextIOBase] = []
        self._labels: list[str] = []
        self._sticky_critical: str | None = None
        self._critical_scan_tail = ""
        self._drain_problems: list[str] = []

    def _append(self, label: str, chunk: str) -> None:
        tagged = f"[{label}] {chunk}"
        with self._lock:
            scan_text = self._critical_scan_tail + chunk
            if self._sticky_critical is None:
                self._sticky_critical = _critical_server_error(scan_text)
                if self._sticky_critical is None and self._classifier is not None:
                    self._sticky_critical = self._classifier(scan_text)
            self._critical_scan_tail = scan_text[-self._maximum :]
            self._text = (self._text + tagged)[-self._maximum :]

    def _drain(self, stream: io.TextIOBase, label: str) -> None:
        try:
            while chunk := stream.readline(4096):
                self._append(label, chunk)
        except (OSError, ValueError) as error:
            with self._lock:
                self._drain_problems.append(f"{label} log drain failed: {error}")

    def start(self, stdout: io.TextIOBase | None, stderr: io.TextIOBase | None) -> None:
        for stream, label in ((stdout, "stdout"), (stderr, "stderr")):
            if stream is None:
                continue
            thread = threading.Thread(target=self._drain, args=(stream, label), daemon=True)
            try:
                thread.start()
            except BaseException:
                with contextlib.suppress(OSError):
                    stream.close()
                raise
            self._streams.append(stream)
            self._threads.append(thread)
            self._labels.append(label)

    def finish(self) -> None:
        for thread, label in zip(self._threads, self._labels, strict=True):
            thread.join(timeout=0.5)
            if thread.is_alive():
                with self._lock:
                    self._drain_problems.append(f"{label} log drain thread did not stop")
                thread.join(timeout=0.2)
        for stream, thread in zip(self._streams, self._threads, strict=True):
            if not thread.is_alive():
                with contextlib.suppress(OSError, ValueError):
                    stream.close()

    def text(self) -> str:
        with self._lock:
            return self._text

    def critical_error(self) -> str | None:
        with self._lock:
            return self._sticky_critical

    def problems(self) -> tuple[str, ...]:
        with self._lock:
            return tuple(self._drain_problems)


def world_filename(spec: ModelSpec) -> str:
    """Return the checked-in world filename for one locked model specification."""
    return f"{spec.slug}_demo.sdf"


def _add_world_systems(world: Element) -> None:
    physics = SubElement(
        world,
        "plugin",
        {
            "filename": "gz-sim-physics-system",
            "name": "gz::sim::systems::Physics",
        },
    )
    engine = SubElement(physics, "engine")
    add_text(engine, "filename", "gz-physics-dartsim-plugin")
    SubElement(
        world,
        "plugin",
        {
            "filename": "gz-sim-user-commands-system",
            "name": "gz::sim::systems::UserCommands",
        },
    )
    SubElement(
        world,
        "plugin",
        {
            "filename": "gz-sim-scene-broadcaster-system",
            "name": "gz::sim::systems::SceneBroadcaster",
        },
    )


def _add_ground(world: Element, spec: ModelSpec) -> None:
    ground = SubElement(world, "model", {"name": "ground_plane"})
    add_text(ground, "static", "true")
    add_text(ground, "pose", _GROUND_POSE_BY_SLUG[spec.slug])
    link = SubElement(ground, "link", {"name": "ground_link"})
    collision = SubElement(link, "collision", {"name": "ground_collision"})
    collision_plane = SubElement(SubElement(collision, "geometry"), "plane")
    add_text(collision_plane, "normal", "0 0 1")
    add_text(collision_plane, "size", "100 100")
    visual = SubElement(link, "visual", {"name": "ground_visual"})
    visual_plane = SubElement(SubElement(visual, "geometry"), "plane")
    add_text(visual_plane, "normal", "0 0 1")
    add_text(visual_plane, "size", "100 100")
    material = SubElement(visual, "material")
    add_text(material, "ambient", "0.8 0.8 0.8 1")
    add_text(material, "diffuse", "0.8 0.8 0.8 1")


def _add_light(world: Element) -> None:
    light = SubElement(world, "light", {"name": "sun", "type": "directional"})
    add_text(light, "cast_shadows", "true")
    add_text(light, "pose", "0 0 10 0 0 0")
    add_text(light, "diffuse", "0.8 0.8 0.8 1")
    add_text(light, "specular", "0.2 0.2 0.2 1")
    attenuation = SubElement(light, "attenuation")
    add_text(attenuation, "range", "1000")
    add_text(light, "direction", "-0.5 0.1 -0.9")


def _add_controlled_include(world: Element, spec: ModelSpec) -> None:
    include = SubElement(world, "include")
    add_text(include, "uri", f"model://{spec.slug}")
    add_text(include, "name", spec.slug)
    for joint_name in spec.actuated_joints:
        controller = SubElement(
            include,
            "plugin",
            {"filename": _CONTROLLER_FILENAME, "name": _CONTROLLER_NAME},
        )
        add_text(controller, "joint_name", joint_name)
        add_text(controller, "joint_index", "0")
        add_text(controller, "topic", f"/{_WORLD_NAME}/{spec.slug}/{joint_name}/cmd_pos")
        add_text(controller, "use_velocity_commands", "true")
        add_text(controller, "cmd_max", "0.1")
        add_text(controller, "cmd_min", "-0.1")
        add_text(controller, "initial_position", "0")
    publisher = SubElement(
        include,
        "plugin",
        {"filename": _STATE_FILENAME, "name": _STATE_NAME},
    )
    for joint_name in spec.actuated_joints:
        add_text(publisher, "joint_name", joint_name)


def build_demo_world(spec: ModelSpec) -> ElementTree.ElementTree:
    """Build one controller-injected world without modifying the Fuel model."""
    sdf = Element("sdf", {"version": "1.11"})
    world = SubElement(sdf, "world", {"name": _WORLD_NAME})
    add_text(world, "gravity", "0 0 -9.8")
    physics = SubElement(world, "physics", {"name": "one_millisecond", "type": "ignored"})
    add_text(physics, "max_step_size", "0.001")
    add_text(physics, "real_time_update_rate", "1000")
    _add_world_systems(world)
    _add_ground(world, spec)
    _add_light(world)
    _add_controlled_include(world, spec)
    return ElementTree.ElementTree(sdf)


def _materialize_snapshot(snapshot: DirectorySnapshot, destination: Path) -> None:
    destination.mkdir(mode=0o700)
    for relative in sorted(snapshot.directories, key=lambda path: (len(path.parts), path.as_posix())):
        (destination / Path(*relative.parts)).mkdir(mode=0o700)
    for relative, data in sorted(snapshot.files.items(), key=lambda item: item[0].as_posix()):
        path = destination / Path(*relative.parts)
        with path.open("xb") as stream:
            stream.write(data)


def _validated_export_snapshot(
    models_root: Path,
    *,
    validator: Callable[[Path], None] | None = None,
) -> DirectorySnapshot:
    try:
        snapshot = capture_directory(models_root)
    except SnapshotError as error:
        raise ValueError(f"model export snapshot failed: {error}") from None
    if snapshot.problems:
        details = "; ".join(f"{problem.message}: {problem.path}" for problem in snapshot.problems)
        raise ValueError(f"model export snapshot failed: {details}")
    try:
        with tempfile.TemporaryDirectory(prefix="onerobotics-a1-world-validation-") as temporary:
            private_export = Path(temporary) / "models"
            _materialize_snapshot(snapshot, private_export)
            with contextlib.redirect_stdout(io.StringIO()):
                (validator or validate_all)(private_export)
    except (OSError, SnapshotError, ValidationFailure, ValueError) as error:
        raise ValueError(f"model export validation failed: {error}") from None
    return snapshot


def _verify_output_root_identity(output_root: Path, descriptor: int) -> None:
    try:
        path_details = output_root.lstat()
        descriptor_details = os.fstat(descriptor)
    except OSError as error:
        raise ValueError(f"output directory was changed or replaced: {error}") from None
    if (
        stat.S_ISLNK(path_details.st_mode)
        or not stat.S_ISDIR(path_details.st_mode)
        or (path_details.st_dev, path_details.st_ino) != (descriptor_details.st_dev, descriptor_details.st_ino)
    ):
        raise ValueError(f"output directory was changed or replaced: {output_root}")


def _prepare_output_root(output_root: Path, expected_names: set[str]) -> int:
    try:
        inspected = output_root.lstat()
    except FileNotFoundError:
        output_root.mkdir(parents=True, mode=0o755)
        inspected = output_root.lstat()
    except OSError as error:
        raise ValueError(f"unable to inspect output directory: {error}") from None
    if stat.S_ISLNK(inspected.st_mode):
        raise ValueError(f"output directory must not be a symlink: {output_root}")
    if not stat.S_ISDIR(inspected.st_mode):
        raise ValueError(f"output path must be a regular directory: {output_root}")

    try:
        descriptor = os.open(output_root, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
    except OSError as error:
        raise ValueError(f"unable to open output directory safely: {error}") from None
    try:
        try:
            _verify_output_root_identity(output_root, descriptor)
            entries = list(os.scandir(descriptor))
        except OSError as error:
            raise ValueError(f"unable to inspect output directory: {error}") from None
        for entry in entries:
            if entry.name not in expected_names:
                raise ValueError(f"unexpected output entry: {entry.name}")
            try:
                details = entry.stat(follow_symlinks=False)
            except OSError as error:
                raise ValueError(f"unable to inspect output entry: {entry.name}: {error}") from None
            if not stat.S_ISREG(details.st_mode):
                raise ValueError(f"output entry must be a regular file: {entry.name}")
        return descriptor
    except BaseException:
        os.close(descriptor)
        raise


def _atomic_write(output_root: Path, directory_descriptor: int, name: str, data: bytes) -> None:
    temporary_name = f".{name}.{secrets.token_hex(16)}"
    descriptor = -1
    try:
        descriptor = os.open(
            temporary_name,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW,
            0o600,
            dir_fd=directory_descriptor,
        )
        os.fchmod(descriptor, 0o644)
        with os.fdopen(descriptor, "wb", closefd=True) as stream:
            descriptor = -1
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        _verify_output_root_identity(output_root, directory_descriptor)
        os.replace(
            temporary_name,
            name,
            src_dir_fd=directory_descriptor,
            dst_dir_fd=directory_descriptor,
        )
        _verify_output_root_identity(output_root, directory_descriptor)
    except OSError as error:
        raise ValueError(f"unable to write output atomically: {error}") from None
    finally:
        if descriptor >= 0:
            os.close(descriptor)
        with contextlib.suppress(FileNotFoundError):
            os.unlink(temporary_name, dir_fd=directory_descriptor)


def generate_worlds(*, models_root: Path, output_root: Path) -> tuple[Path, ...]:
    """Validate one export snapshot and atomically write the three demo worlds."""
    _validated_export_snapshot(Path(models_root))
    specs = load_model_specs()
    documents = tuple((world_filename(spec), serialize_sdf(build_demo_world(spec))) for spec in specs)
    output_root = Path(output_root)
    directory_descriptor = _prepare_output_root(output_root, {name for name, _ in documents})
    try:
        for name, data in documents:
            _atomic_write(output_root, directory_descriptor, name, data)
        os.fsync(directory_descriptor)
        _verify_output_root_identity(output_root, directory_descriptor)
    finally:
        os.close(directory_descriptor)
    return tuple(output_root / name for name, _ in documents)


def _json_depth_exceeds(document: object) -> bool:
    pending = [(document, 1)]
    while pending:
        value, depth = pending.pop()
        if depth > _MAX_JSON_DEPTH:
            return True
        if isinstance(value, dict):
            pending.extend((child, depth + 1) for child in value.values())
        elif isinstance(value, list):
            pending.extend((child, depth + 1) for child in value)
    return False


def _json_text_is_utf8(document: object) -> bool:
    pending = [document]
    while pending:
        value = pending.pop()
        if isinstance(value, str):
            try:
                value.encode("utf-8")
            except UnicodeEncodeError:
                return False
        elif isinstance(value, dict):
            pending.extend(value.keys())
            pending.extend(value.values())
        elif isinstance(value, list):
            pending.extend(value)
    return True


def _bounded_json_int(token: str) -> int:
    if len(token) > _MAX_JSON_NUMBER_CHARS:
        raise _JsonNumberError("joint-state JSON numeric token exceeds limit")
    try:
        return int(token)
    except ValueError:
        raise _JsonNumberError("malformed joint-state JSON numeric token") from None


def _bounded_json_float(token: str) -> float:
    if len(token) > _MAX_JSON_NUMBER_CHARS:
        raise _JsonNumberError("joint-state JSON numeric token exceeds limit")
    try:
        value = float(token)
    except ValueError:
        raise _JsonNumberError("malformed joint-state JSON numeric token") from None
    if not math.isfinite(value):
        raise _JsonNumberError("joint-state JSON numeric value must be finite")
    return value


def _reject_json_constant(token: str) -> float:
    raise _JsonNumberError(f"joint-state JSON numeric value must be finite: {token}")


def parse_joint_position(payload: str, joint_name: str) -> float:
    """Extract one exact joint's finite scalar position from gz.msgs.Model JSON."""
    if not isinstance(payload, str):
        raise ValueError("joint-state payload must be text")
    if len(payload) > _MAX_JSON_BYTES:
        raise ValueError("joint-state JSON exceeds size limit")
    try:
        encoded_size = len(payload.encode("utf-8"))
    except UnicodeEncodeError:
        raise ValueError("malformed joint-state JSON text") from None
    if encoded_size > _MAX_JSON_BYTES:
        raise ValueError("joint-state JSON exceeds size limit")
    try:
        document = json.loads(
            payload,
            parse_int=_bounded_json_int,
            parse_float=_bounded_json_float,
            parse_constant=_reject_json_constant,
        )
    except _JsonNumberError as error:
        raise ValueError(str(error)) from None
    except (json.JSONDecodeError, RecursionError, UnicodeError) as error:
        message = (
            "joint-state JSON exceeds nesting limit"
            if isinstance(error, RecursionError)
            else "malformed joint-state JSON"
        )
        raise ValueError(message) from None
    if not _json_text_is_utf8(document):
        raise ValueError("malformed joint-state JSON text")
    if _json_depth_exceeds(document):
        raise ValueError("joint-state JSON exceeds nesting limit")
    if not isinstance(document, dict):
        raise ValueError("joint-state JSON root must be an object")
    joints = document.get("joint")
    if not isinstance(joints, list):
        raise ValueError("joint-state joint field must be an array")
    matches: list[dict[str, object]] = []
    for entry in joints:
        if not isinstance(entry, dict):
            raise ValueError("every joint-state joint must be an object")
        name = entry.get("name")
        if not isinstance(name, str):
            raise ValueError("every joint-state joint name must be text")
        if name == joint_name:
            matches.append(entry)
    if not matches:
        raise ValueError(f"missing joint state: {joint_name}")
    if len(matches) != 1:
        raise ValueError(f"duplicate joint state: {joint_name}")
    axis = matches[0].get("axis1")
    if not isinstance(axis, dict):
        raise ValueError(f"joint state {joint_name} axis1 must be an object")
    position = axis.get("position", 0.0)
    if isinstance(position, bool) or not isinstance(position, (int, float)):
        raise ValueError(f"joint state {joint_name} position must be numeric")
    try:
        result = float(position)
    except (OverflowError, ValueError):
        raise ValueError(f"joint state {joint_name} position must be finite") from None
    if not math.isfinite(result):
        raise ValueError(f"joint state {joint_name} position must be finite")
    return result


def _capture_clean_directory(path: Path, label: str) -> DirectorySnapshot:
    try:
        snapshot = capture_directory(path)
    except SnapshotError as error:
        raise ValueError(f"{label} snapshot failed: {error}") from None
    if snapshot.problems:
        details = "; ".join(f"{problem.message}: {problem.path}" for problem in snapshot.problems)
        raise ValueError(f"{label} snapshot failed: {details}")
    return snapshot


def _world_snapshot(worlds_root: Path, specs: tuple[ModelSpec, ...]) -> DirectorySnapshot:
    snapshot = _capture_clean_directory(worlds_root, "world")
    expected = {PurePosixPath(world_filename(spec)) for spec in specs}
    actual = set(snapshot.files)
    if snapshot.directories or actual != expected:
        raise ValueError(
            "world snapshot must contain exactly the three generated files; "
            f"missing={sorted(str(path) for path in expected - actual)}, "
            f"extra={sorted(str(path) for path in actual - expected)}"
        )
    return snapshot


def _joint_interval(snapshot: DirectorySnapshot, spec: ModelSpec, joint_name: str) -> tuple[float, float]:
    relative = PurePosixPath(spec.slug) / "model.sdf"
    data = snapshot.files.get(relative)
    if data is None:
        raise ValueError(f"validated model snapshot is missing {relative}")
    try:
        root = ElementTree.fromstring(data)
    except (ElementTree.ParseError, RecursionError) as error:
        raise ValueError(f"unable to parse validated model limits for {joint_name}") from error
    joints = root.findall(f"model/joint[@name='{joint_name}']")
    if len(joints) != 1:
        raise ValueError(f"validated model must contain exactly one joint {joint_name}")
    limits = joints[0].findall("axis/limit")
    if len(limits) != 1:
        raise ValueError(f"validated joint {joint_name} must contain exactly one limit")
    try:
        lower = float(limits[0].findtext("lower", ""))
        upper = float(limits[0].findtext("upper", ""))
    except ValueError as error:
        raise ValueError(f"validated joint {joint_name} limits must be numeric") from error
    if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
        raise ValueError(f"validated joint {joint_name} limits must be one finite interval")
    return lower, upper


def _harmonic_argv(
    partition: str,
    private_export: Path,
    command: list[str],
    *,
    extra_environment: tuple[str, ...] = (),
) -> list[str]:
    resource_root = os.fspath(private_export)
    if not private_export.is_absolute() or not resource_root or os.pathsep in resource_root:
        raise ValueError("private resource root must be absolute and contain no path separator")
    environment_names = {"GZ_PARTITION", "GZ_SIM_RESOURCE_PATH"}
    validated_environment: list[str] = []
    for assignment in extra_environment:
        if not isinstance(assignment, str) or "=" not in assignment or "\x00" in assignment or "\n" in assignment:
            raise ValueError("extra environment entries must be safe NAME=value strings")
        name, _ = assignment.split("=", 1)
        if re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", name) is None:
            raise ValueError(f"invalid extra environment name: {name}")
        if name == "MESA_LOADER_DRIVER_OVERRIDE":
            raise ValueError("MESA_LOADER_DRIVER_OVERRIDE is forbidden for this Harmonic runtime")
        if name in {"GZ_PARTITION", "GZ_SIM_RESOURCE_PATH"}:
            raise ValueError(f"extra environment name is reserved: {name}")
        if name in environment_names:
            raise ValueError(f"duplicate extra environment name: {name}")
        environment_names.add(name)
        validated_environment.append(assignment)
    return [
        "bash",
        "-c",
        _FRESH_SHELL,
        "a1-harmonic-runtime",
        str(_HARMONIC_HELPER),
        "env",
        f"GZ_PARTITION={partition}",
        f"GZ_SIM_RESOURCE_PATH={resource_root}",
        *validated_environment,
        "gz",
        *command,
    ]


def _kill_child_group(process: subprocess.Popen[str]) -> _GroupCleanupResult:
    deferred_error: BaseException | None = None

    def defer(error: BaseException) -> None:
        nonlocal deferred_error
        if deferred_error is None:
            deferred_error = error
        else:
            _add_exception_note(
                deferred_error,
                f"additional cleanup interruption: {type(error).__name__}: {error}",
            )

    for _ in range(3):
        try:
            if not _process_group_exists(process.pid):
                try:
                    process.poll()
                except BaseException as error:
                    defer(error)
                return _GroupCleanupResult(True, deferred_error)
        except BaseException as error:
            defer(error)
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            return _GroupCleanupResult(True, deferred_error)
        except OSError:
            continue
        except BaseException as error:
            defer(error)
            continue
        try:
            process.wait(timeout=2.0)
        except (OSError, subprocess.TimeoutExpired):
            pass
        except BaseException as error:
            defer(error)
    try:
        cleaned = not _process_group_exists(process.pid)
    except BaseException as error:
        defer(error)
        cleaned = False
    return _GroupCleanupResult(cleaned, deferred_error)


def _process_group_exists(process_group: int) -> bool:
    try:
        os.killpg(process_group, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    except OSError as error:
        return error.errno != errno.ESRCH
    return True


def _wait_for_process_group_exit(process_group: int, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    while _process_group_exists(process_group):
        if time.monotonic() >= deadline:
            return False
        time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
    return True


def _wait_for_server_group_exit(server: subprocess.Popen[str], timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    if server.poll() is None:
        try:
            server.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            return not _process_group_exists(server.pid)
        except OSError:
            pass
    return _wait_for_process_group_exit(server.pid, max(0.0, deadline - time.monotonic()))


def _communicate_bounded(
    process: subprocess.Popen[str],
    *,
    timeout: float,
    output_limit_bytes: int,
) -> tuple[str, str]:
    """Drain both command pipes while enforcing one combined byte limit."""
    if isinstance(output_limit_bytes, bool) or not isinstance(output_limit_bytes, int) or output_limit_bytes <= 0:
        raise ValueError("command output limit must be a positive integer")
    if process.stdout is None or process.stderr is None:
        raise RuntimeError("bounded command capture requires stdout and stderr pipes")
    selector = selectors.DefaultSelector()
    chunks: dict[str, list[bytes]] = {"stdout": [], "stderr": []}
    total = 0
    deadline = time.monotonic() + timeout
    try:
        selector.register(process.stdout, selectors.EVENT_READ, "stdout")
        selector.register(process.stderr, selectors.EVENT_READ, "stderr")
        while selector.get_map():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise subprocess.TimeoutExpired("Harmonic command", timeout)
            events = selector.select(remaining)
            if not events:
                raise subprocess.TimeoutExpired("Harmonic command", timeout)
            for key, _ in events:
                data = os.read(key.fd, min(64 * 1024, output_limit_bytes - total + 1))
                if not data:
                    selector.unregister(key.fileobj)
                    continue
                chunks[key.data].append(data)
                total += len(data)
                if total > output_limit_bytes:
                    raise RuntimeError("Harmonic command output exceeds the byte limit")
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise subprocess.TimeoutExpired("Harmonic command", timeout)
        if process.poll() is None:
            process.wait(timeout=remaining)
    finally:
        selector.close()
    try:
        stdout = b"".join(chunks["stdout"]).decode("utf-8")
        stderr = b"".join(chunks["stderr"]).decode("utf-8")
    except UnicodeDecodeError:
        raise RuntimeError("Harmonic command output must be UTF-8") from None
    return stdout, stderr


def _run_harmonic(
    *,
    partition: str,
    private_export: Path,
    command: list[str],
    timeout: float,
    timeout_message: str,
    deadline: float | None = None,
    extra_environment: tuple[str, ...] = (),
    output_limit_bytes: int | None = None,
) -> _CommandResult:
    argv = _harmonic_argv(
        partition,
        private_export,
        command,
        extra_environment=extra_environment,
    )
    process: subprocess.Popen[str] | None = None
    try:
        try:
            process = subprocess.Popen(
                argv,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                start_new_session=True,
            )
        except OSError as error:
            raise RuntimeError(f"unable to start Harmonic command: {error}") from error

        communication_timeout = timeout
        if deadline is not None:
            communication_timeout = min(timeout, deadline - time.monotonic())
            if communication_timeout <= 0:
                raise RuntimeError(timeout_message)
        try:
            if output_limit_bytes is None:
                stdout, stderr = process.communicate(timeout=communication_timeout)
            else:
                stdout, stderr = _communicate_bounded(
                    process,
                    timeout=communication_timeout,
                    output_limit_bytes=output_limit_bytes,
                )
        except subprocess.TimeoutExpired:
            raise RuntimeError(timeout_message) from None
        except OSError as error:
            raise RuntimeError(f"Harmonic command communication failed: {error}") from error
        if deadline is not None and time.monotonic() >= deadline:
            raise RuntimeError(timeout_message)
        if _process_group_exists(process.pid):
            raise RuntimeError("Harmonic command process group remained alive after command completion")
        return _CommandResult(process.returncode or 0, stdout, stderr)
    except BaseException as operation_error:
        if process is None:
            raise
        cleanup = _kill_child_group(process)
        terminal_error = operation_error
        if cleanup.deferred_error is not None:
            _add_exception_note(
                terminal_error,
                f"cleanup interruption: {type(cleanup.deferred_error).__name__}: {cleanup.deferred_error}",
            )
            _copy_exception_notes(cleanup.deferred_error, terminal_error)
        if isinstance(operation_error, RuntimeError) and (cleanup.deferred_error is not None or not cleanup.cleaned):
            error_type = _ChildGroupCleanupError if not cleanup.cleaned else _HarmonicCommandError
            terminal_error = error_type(
                str(operation_error),
                cleanup_interruption=cleanup.deferred_error,
            )
            _copy_exception_notes(operation_error, terminal_error)
        if not cleanup.cleaned:
            _add_exception_note(
                terminal_error,
                f"child process group {process.pid} survived SIGKILL",
            )
        if terminal_error is operation_error:
            raise
        raise terminal_error from operation_error


def _require_command(result: _CommandResult, label: str) -> str:
    if result.returncode != 0:
        output = (result.stdout + result.stderr)[-8192:]
        raise RuntimeError(f"{label} failed with exit {result.returncode}: {output}")
    return result.stdout


def _remaining(deadline: float, timeout_message: str) -> float:
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        raise RuntimeError(timeout_message)
    return min(_CHILD_TIMEOUT, remaining)


def _critical_server_error(log: str) -> str | None:
    patterns = (
        r"(?im)^.*\[(?:err|error|fatal|ftl)\].*$",
        r"(?im)^.*(?:\[wrn\]|\bwarning\b).*\bphysics\b.*$",
        r"(?im)^.*(?:\[err\]|\berror\b).*(?:plugin|physics).*$",
        r"(?im)^.*(?:failed|unable|could not) to (?:load|initialize).*(?:plugin|physics).*$",
        r"(?im)^.*(?:plugin|physics).*(?:failed|unable|could not).*$",
    )
    for pattern in patterns:
        match = re.search(pattern, log)
        if match is not None:
            return match.group(0)[-2048:]
    return None


def _check_server(server: subprocess.Popen[str], log_tail: _BoundedLogTail) -> None:
    returncode = server.poll()
    if returncode is not None:
        raise RuntimeError(f"Gazebo server exited early with code {returncode}")
    critical = log_tail.critical_error() or _critical_server_error(log_tail.text())
    if critical is not None:
        raise RuntimeError(f"Gazebo plugin/load/physics error: {critical}")


def _diagnostic(message: str, log_tail: _BoundedLogTail, latest_topics: str) -> str:
    return (
        f"{message}\nserver log tail:\n{log_tail.text() or '<empty>'}\nlatest topic list:\n{latest_topics or '<none>'}"
    )


def _capture_position(
    *,
    partition: str,
    private_export: Path,
    state_topic: str,
    joint: str,
    lower: float,
    upper: float,
    deadline: float,
) -> float:
    result = _run_harmonic(
        partition=partition,
        private_export=private_export,
        command=[
            "topic",
            "--force-version",
            "13",
            "-e",
            "-n",
            "1",
            "--json-output",
            "-t",
            state_topic,
        ],
        timeout=_remaining(deadline, "joint-state timeout"),
        timeout_message="joint-state timeout",
        deadline=deadline,
    )
    payload = _require_command(result, "joint-state capture")
    try:
        positions = _parse_transport_frames(payload, joint)
    except ValueError as error:
        raise RuntimeError(f"invalid joint state: {error}") from error
    for position in positions:
        if not lower <= position <= upper:
            raise RuntimeError(f"joint position {position} is outside source limits [{lower}, {upper}]")
    return positions[-1]


def _parse_transport_frames(payload: str, joint: str) -> tuple[float, ...]:
    """Split Transport's occasional whitespace-delimited JSON burst safely."""
    if len(payload) > _MAX_JSON_BYTES:
        raise ValueError("joint-state JSON exceeds size limit")
    try:
        encoded_size = len(payload.encode("utf-8"))
    except UnicodeEncodeError:
        raise ValueError("malformed joint-state JSON stream") from None
    if encoded_size > _MAX_JSON_BYTES:
        raise ValueError("joint-state JSON exceeds size limit")
    decoder = json.JSONDecoder(
        parse_int=_bounded_json_int,
        parse_float=_bounded_json_float,
        parse_constant=_reject_json_constant,
    )
    documents: list[str] = []
    index = 0
    try:
        while index < len(payload):
            whitespace_start = index
            while index < len(payload) and payload[index].isspace():
                index += 1
            if index == len(payload):
                break
            if documents and index == whitespace_start:
                raise ValueError("malformed joint-state JSON stream")
            _, end = decoder.raw_decode(payload, index)
            documents.append(payload[index:end])
            if len(documents) > 32:
                raise ValueError("joint-state JSON stream contains too many documents")
            index = end
    except _JsonNumberError as error:
        raise ValueError(str(error)) from None
    except (json.JSONDecodeError, RecursionError) as error:
        message = (
            "joint-state JSON exceeds nesting limit"
            if isinstance(error, RecursionError)
            else "malformed joint-state JSON stream"
        )
        raise ValueError(message) from None
    if not documents:
        raise ValueError("malformed joint-state JSON stream")
    return tuple(parse_joint_position(document, joint) for document in documents)


def _publisher_message_types(information: str) -> tuple[str, ...]:
    section = re.search(
        r"(?ims)^\s*Publishers(?:\s*\[[^\]\r\n]*\])?\s*:\s*\n"
        r"(?P<body>.*?)(?=^\s*Subscribers(?:\s*\[[^\]\r\n]*\])?\s*:|\Z)",
        information,
    )
    if section is None:
        return ()
    message_types: list[str] = []
    for line in section.group("body").splitlines():
        columns = [column.strip() for column in line.split(",")]
        if len(columns) == 2 and re.fullmatch(r"[A-Za-z_]\w*(?:\.[A-Za-z_]\w*)+", columns[1]):
            message_types.append(columns[1])
    return tuple(message_types)


def _stop_server(
    server: subprocess.Popen[str],
    *,
    partition: str,
    private_export: Path,
    extra_environment: tuple[str, ...] = (),
) -> None:
    graceful = False
    cleanup_problems: list[str] = []
    request_interruption: BaseException | None = None

    def raise_cleanup_outcome() -> None:
        if request_interruption is not None:
            for problem in cleanup_problems:
                _add_exception_note(request_interruption, problem)
            raise request_interruption
        if cleanup_problems:
            raise RuntimeError("; ".join(cleanup_problems))

    def record_unverified_exit() -> None:
        if any("before verified stop" in problem for problem in cleanup_problems):
            return
        returncode = server.poll()
        if returncode is None:
            cleanup_problems.append("Gazebo server process group disappeared before verified stop")
        else:
            cleanup_problems.append(f"Gazebo server exited before verified stop with code {returncode}")

    returncode_before_request = server.poll()
    if returncode_before_request is not None:
        record_unverified_exit()
    else:
        try:
            response = _run_harmonic(
                partition=partition,
                private_export=private_export,
                command=[
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
                timeout=3.0,
                timeout_message="server-control timeout",
                extra_environment=extra_environment,
            )
            graceful = response.returncode == 0 and (
                re.search(r"(?m)^\s*data:\s*true\s*$", response.stdout) is not None
            )
        except _HarmonicCommandError as error:
            if error.cleanup_interruption is not None:
                request_interruption = error.cleanup_interruption
                _add_exception_note(
                    request_interruption,
                    f"server-control command failure: {type(error).__name__}: {error}",
                )
                _copy_exception_notes(error, request_interruption)
            if isinstance(error, _ChildGroupCleanupError):
                cleanup_problems.append(
                    "; ".join(
                        (
                            f"server-control helper cleanup failed: {error}",
                            *_exception_notes(error),
                        )
                    )
                )
            graceful = False
        except RuntimeError:
            graceful = False
        except BaseException as error:
            request_interruption = error
            graceful = False

    if not graceful and returncode_before_request is None:
        returncode_after_request = server.poll()
        if returncode_after_request is not None:
            record_unverified_exit()

    if graceful:
        if _wait_for_server_group_exit(server, _STOP_TIMEOUT):
            graceful_returncode = server.poll()
            if graceful_returncode not in (None, 0):
                raise RuntimeError(f"Gazebo server exited after stop request with code {graceful_returncode}")
            return

    if _process_group_exists(server.pid):
        try:
            os.killpg(server.pid, signal.SIGTERM)
        except ProcessLookupError:
            record_unverified_exit()
        except OSError as error:
            cleanup_problems.append(f"SIGTERM failed: {error}")
    else:
        record_unverified_exit()
    if _wait_for_server_group_exit(server, _STOP_TIMEOUT):
        raise_cleanup_outcome()
        return

    try:
        os.killpg(server.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    except OSError as error:
        cleanup_problems.append(f"SIGKILL failed: {error}")
    if not _wait_for_server_group_exit(server, _STOP_TIMEOUT):
        cleanup_problems.append(f"process group {server.pid} survived SIGKILL")
    raise_cleanup_outcome()


def _run_motion(
    *,
    server: subprocess.Popen[str],
    log_tail: _BoundedLogTail,
    partition: str,
    private_export: Path,
    spec: ModelSpec,
    joint: str,
    target: float,
    lower: float,
    upper: float,
    deadline: float,
    latest_topics: list[str],
) -> SmokeResult:
    command_topic = f"/{_WORLD_NAME}/{spec.slug}/{joint}/cmd_pos"
    state_topic = f"/world/{_WORLD_NAME}/model/{spec.slug}/joint_state"
    while True:
        _check_server(server, log_tail)
        listing = _run_harmonic(
            partition=partition,
            private_export=private_export,
            command=["topic", "--force-version", "13", "-l"],
            timeout=_remaining(deadline, "topic discovery timeout"),
            timeout_message="topic discovery timeout",
            deadline=deadline,
        )
        latest_topics[0] = _require_command(listing, "topic discovery")
        topics = {line.strip() for line in latest_topics[0].splitlines() if line.strip()}
        if {command_topic, state_topic} <= topics:
            break
        if time.monotonic() >= deadline:
            raise RuntimeError("topic discovery timeout")
        time.sleep(min(_POLL_INTERVAL, max(0.0, deadline - time.monotonic())))

    _check_server(server, log_tail)
    information = _run_harmonic(
        partition=partition,
        private_export=private_export,
        command=["topic", "--force-version", "13", "-i", "-t", state_topic],
        timeout=_remaining(deadline, "topic type timeout"),
        timeout_message="topic type timeout",
        deadline=deadline,
    )
    type_text = _require_command(information, "joint-state topic inspection")
    publisher_types = _publisher_message_types(type_text)
    if "gz.msgs.Model" not in publisher_types:
        raise RuntimeError(f"joint-state topic publisher is not gz.msgs.Model: {type_text[-2048:]}")

    initial = _capture_position(
        partition=partition,
        private_export=private_export,
        state_topic=state_topic,
        joint=joint,
        lower=lower,
        upper=upper,
        deadline=deadline,
    )
    if not lower <= initial <= upper:
        raise RuntimeError(f"initial joint position {initial} is outside source limits [{lower}, {upper}]")
    publication = _run_harmonic(
        partition=partition,
        private_export=private_export,
        command=[
            "topic",
            "--force-version",
            "13",
            "-t",
            command_topic,
            "-m",
            "gz.msgs.Double",
            "-p",
            f"data: {target}",
        ],
        timeout=_remaining(deadline, "joint command timeout"),
        timeout_message="joint command timeout",
        deadline=deadline,
    )
    _require_command(publication, "joint command publication")

    initial_distance = abs(target - initial)
    direction = 1.0 if target > initial else -1.0
    while True:
        _check_server(server, log_tail)
        if time.monotonic() >= deadline:
            raise RuntimeError("motion timeout")
        position = _capture_position(
            partition=partition,
            private_export=private_export,
            state_topic=state_topic,
            joint=joint,
            lower=lower,
            upper=upper,
            deadline=deadline,
        )
        if time.monotonic() >= deadline:
            raise RuntimeError("motion timeout")
        if not lower <= position <= upper:
            raise RuntimeError(f"joint position {position} is outside source limits [{lower}, {upper}]")
        progress = (position - initial) * direction
        moved_toward = progress >= 0.02 and abs(target - position) < initial_distance
        if moved_toward:
            _check_server(server, log_tail)
            if time.monotonic() >= deadline:
                raise RuntimeError("motion timeout")
            return SmokeResult(
                model=spec.slug,
                joint=joint,
                target=target,
                initial_position=initial,
                final_position=position,
                delta=abs(position - initial),
                lower=lower,
                upper=upper,
                finite=True,
                moved_toward_target=True,
            )
        time.sleep(min(_POLL_INTERVAL, max(0.0, deadline - time.monotonic())))


def _lifecycle_checkpoint(phase: str) -> None:
    """Expose phase boundaries so cleanup ownership can be regression-tested."""
    del phase


def _validated_target(spec: ModelSpec, joint: str, target: object) -> float:
    if joint not in spec.actuated_joints:
        raise ValueError(f"unknown actuated joint for {spec.slug}: {joint}")
    if isinstance(target, bool) or not isinstance(target, (int, float)):
        raise ValueError("target must be one finite numeric value")
    try:
        result = float(target)
    except (OverflowError, ValueError):
        raise ValueError("target must be one finite numeric value") from None
    if not math.isfinite(result):
        raise ValueError("target must be one finite numeric value")
    return result


def _smoke_validated_snapshot(
    *,
    spec: ModelSpec,
    joint: str,
    target: object,
    models_snapshot: DirectorySnapshot,
    limit_snapshot: DirectorySnapshot,
    worlds_snapshot: DirectorySnapshot,
    deadline: float | None = None,
) -> SmokeResult:
    """Run the existing bounded lifecycle using only already-captured snapshots."""
    target = _validated_target(spec, joint, target)
    expected_world = serialize_sdf(build_demo_world(spec))
    world_relative = PurePosixPath(world_filename(spec))
    checked_in_world = worlds_snapshot.files.get(world_relative)
    if checked_in_world is None:
        raise ValueError(f"checked-in world snapshot is missing: {world_relative}")
    if checked_in_world != expected_world:
        raise ValueError(f"checked-in world bytes do not match generated expectation: {world_relative}")
    lower, upper = _joint_interval(limit_snapshot, spec, joint)
    if not lower < target < upper:
        raise ValueError(f"target must be strictly inside source limits ({lower}, {upper})")

    partition = f"onerobotics-a1-smoke-{os.getpid()}-{secrets.token_hex(8)}"
    with tempfile.TemporaryDirectory(prefix="onerobotics-a1-smoke-") as temporary:
        run_root = Path(temporary).resolve()
        private_export = run_root / "models"
        private_worlds = run_root / "worlds"
        _materialize_snapshot(models_snapshot, private_export)
        private_worlds.mkdir(mode=0o700)
        private_world = private_worlds / world_relative.name
        with private_world.open("xb") as stream:
            stream.write(checked_in_world)
        if deadline is not None and time.monotonic() >= deadline:
            raise RuntimeError("cache smoke total timeout")

        argv = _harmonic_argv(
            partition,
            private_export,
            ["sim", "--force-version", "8", "-s", "-r", "-v", "4", str(private_world)],
        )
        log_tail = _BoundedLogTail()
        latest_topics = [""]
        server: subprocess.Popen[str] | None = None
        motion_result: SmokeResult | None = None
        motion_error: BaseException | None = None
        cleanup_error: BaseException | None = None
        emergency_cleanup = _GroupCleanupResult(True, None)
        try:
            try:
                server = subprocess.Popen(
                    argv,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    start_new_session=True,
                )
            except OSError as error:
                raise RuntimeError(f"unable to start Gazebo server: {error}") from error
            log_tail.start(server.stdout, server.stderr)
            motion_deadline = time.monotonic() + _DISCOVERY_MOTION_TIMEOUT
            if deadline is not None:
                motion_deadline = min(motion_deadline, deadline)
                if motion_deadline <= time.monotonic():
                    raise RuntimeError("cache smoke total timeout")
            _lifecycle_checkpoint("after_setup")
            motion_result = _run_motion(
                server=server,
                log_tail=log_tail,
                partition=partition,
                private_export=private_export,
                spec=spec,
                joint=joint,
                target=target,
                lower=lower,
                upper=upper,
                deadline=motion_deadline,
                latest_topics=latest_topics,
            )
            _lifecycle_checkpoint("after_motion")
        except BaseException as error:
            motion_error = error
        finally:
            if server is not None:
                try:
                    _stop_server(server, partition=partition, private_export=private_export)
                except BaseException as error:
                    cleanup_error = error
                try:
                    emergency_cleanup = _kill_child_group(server)
                except BaseException as error:
                    emergency_cleanup = _GroupCleanupResult(False, error)
                try:
                    log_tail.finish()
                except BaseException as error:
                    cleanup_error = _merge_cleanup_error(
                        cleanup_error,
                        error,
                        note=f"log cleanup interruption: {type(error).__name__}: {error}",
                    )

        log_problems = log_tail.problems()
        if log_problems:
            log_error = RuntimeError("; ".join(log_problems))
            cleanup_error = _merge_cleanup_error(cleanup_error, log_error)

        if emergency_cleanup.deferred_error is not None:
            interruption_detail = (
                "emergency cleanup interruption: "
                f"{type(emergency_cleanup.deferred_error).__name__}: {emergency_cleanup.deferred_error}"
            )
            if motion_error is not None:
                _add_exception_note(motion_error, interruption_detail)
                _copy_exception_notes(emergency_cleanup.deferred_error, motion_error)
            elif cleanup_error is not None:
                _add_exception_note(cleanup_error, interruption_detail)
                _copy_exception_notes(emergency_cleanup.deferred_error, cleanup_error)
            else:
                cleanup_error = emergency_cleanup.deferred_error

        if not emergency_cleanup.cleaned:
            emergency_error = RuntimeError(f"process group {server.pid} survived emergency SIGKILL")
            cleanup_error = _merge_cleanup_error(cleanup_error, emergency_error)

        late_critical = log_tail.critical_error() or _critical_server_error(log_tail.text())
        if motion_error is None and motion_result is not None and late_critical is not None:
            motion_error = RuntimeError(f"Gazebo plugin/load/physics error: {late_critical}")

        if cleanup_error is not None:
            cleanup_detail = _diagnostic(
                f"server cleanup failed: {cleanup_error}",
                log_tail,
                latest_topics[0],
            )
            cleanup_notes = _exception_notes(cleanup_error)
            if motion_error is not None:
                _add_exception_note(motion_error, cleanup_detail)
                for note in cleanup_notes:
                    _add_exception_note(motion_error, note)
            elif isinstance(cleanup_error, RuntimeError):
                diagnostic_error = RuntimeError(cleanup_detail)
                for note in cleanup_notes:
                    _add_exception_note(diagnostic_error, note)
                raise diagnostic_error from cleanup_error
            else:
                _add_exception_note(cleanup_error, cleanup_detail)
                raise cleanup_error

        if motion_error is not None:
            if not isinstance(motion_error, RuntimeError):
                _add_exception_note(
                    motion_error,
                    _diagnostic(str(motion_error), log_tail, latest_topics[0]),
                )
                raise motion_error
            detail = _diagnostic(str(motion_error), log_tail, latest_topics[0])
            if str(motion_error) == "joint-state timeout":
                _add_exception_note(motion_error, detail)
                raise motion_error
            diagnostic_error = RuntimeError(detail)
            _copy_exception_notes(motion_error, diagnostic_error)
            raise diagnostic_error from motion_error

        if motion_result is None:
            raise RuntimeError(_diagnostic("motion check returned no result", log_tail, latest_topics[0]))
        return motion_result


def smoke_test(
    *,
    model: str,
    joint: str,
    target: float,
    export_root: Path,
    worlds_root: Path,
) -> SmokeResult:
    """Run one exact-export, bounded Gazebo Harmonic commanded-motion check."""
    specs = load_model_specs()
    by_slug = {spec.slug: spec for spec in specs}
    spec = by_slug.get(model)
    if spec is None:
        raise ValueError(f"unknown model: {model}")
    validated_target = _validated_target(spec, joint, target)
    export_snapshot = _validated_export_snapshot(Path(export_root))
    worlds_snapshot = _world_snapshot(Path(worlds_root), specs)
    return _smoke_validated_snapshot(
        spec=spec,
        joint=joint,
        target=validated_target,
        models_snapshot=export_snapshot,
        limit_snapshot=export_snapshot,
        worlds_snapshot=worlds_snapshot,
    )


def smoke_cache_all(
    *,
    trusted_models: Path,
    cache_models: Path,
    worlds_root: Path,
) -> tuple[SmokeResult, ...]:
    """Validate one immutable Fuel cache snapshot and run four cases across three models."""
    from onerobotics_a1_gazebo.cache import validate_cache_models

    specs = load_model_specs()
    by_slug = {spec.slug: spec for spec in specs}
    deadline = time.monotonic() + _CACHE_SMOKE_TOTAL_TIMEOUT
    validated = validate_cache_models(
        trusted_models=Path(trusted_models),
        cache_models=Path(cache_models),
    )
    worlds_snapshot = _world_snapshot(Path(worlds_root), specs)
    results: list[SmokeResult] = []
    for model, joint, target in _LIVE_CASES:
        spec = by_slug[model]
        results.append(
            _smoke_validated_snapshot(
                spec=spec,
                joint=joint,
                target=target,
                models_snapshot=validated.normalized_models,
                limit_snapshot=validated.trusted_export,
                worlds_snapshot=worlds_snapshot,
                deadline=deadline,
            )
        )
    return tuple(results)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    generate = commands.add_parser("generate", help="generate the three deterministic demo worlds")
    generate.add_argument("--models", type=Path, required=True)
    generate.add_argument("--output", type=Path, required=True)
    smoke_all = commands.add_parser("smoke-all", help="run four real commanded-motion cases across three models")
    smoke_all.add_argument("--models", type=Path, required=True)
    smoke_all.add_argument("--worlds", type=Path, required=True)
    smoke_cache_all_parser = commands.add_parser(
        "smoke-cache-all",
        help="validate three Fuel cache models and run four real commanded-motion cases across three models",
    )
    smoke_cache_all_parser.add_argument("--trusted-models", type=Path, required=True)
    smoke_cache_all_parser.add_argument("--cache-models", type=Path, required=True)
    smoke_cache_all_parser.add_argument("--worlds", type=Path, required=True)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.command == "generate":
            paths = generate_worlds(models_root=args.models, output_root=args.output)
            for path in paths:
                print(f"DEMO_WORLD_OK {path.name}")
            print(f"DEMO_WORLD_OK: {len(paths)} worlds")
            return 0
        if args.command == "smoke-all":
            lines: list[str] = []
            for model, joint, target in _LIVE_CASES:
                result = smoke_test(
                    model=model,
                    joint=joint,
                    target=target,
                    export_root=args.models,
                    worlds_root=args.worlds,
                )
                lines.append(
                    f"SMOKE_TEST_VALID {result.model}: joint={result.joint} "
                    f"initial={result.initial_position:.6f} final={result.final_position:.6f} "
                    f"delta={result.delta:.6f}"
                )
            for line in lines:
                print(line)
            print(f"SMOKE_TEST_OK: {len(lines)} cases across 3 models")
            return 0
        if args.command == "smoke-cache-all":
            cache_results = smoke_cache_all(
                trusted_models=args.trusted_models,
                cache_models=args.cache_models,
                worlds_root=args.worlds,
            )
            lines = [
                f"CACHE_SMOKE_TEST_VALID {result.model}: joint={result.joint} "
                f"initial={result.initial_position:.6f} final={result.final_position:.6f} "
                f"delta={result.delta:.6f}"
                for result in cache_results
            ]
            for line in lines:
                print(line)
            print(f"CACHE_SMOKE_TEST_OK: {len(lines)} cases across 3 models")
            return 0
    except (OSError, RuntimeError, ValueError) as error:
        if args.command == "smoke-all":
            prefix = "SMOKE_TEST_ERROR"
        elif args.command == "smoke-cache-all":
            prefix = "CACHE_SMOKE_TEST_ERROR"
        else:
            prefix = "DEMO_ERROR"
        print(f"{prefix}: {error}", file=sys.stderr)
        for note in _exception_notes(error):
            print(note, file=sys.stderr)
        return 1
    except BaseException as error:
        for note in tuple(getattr(error, _COMPAT_NOTES_ATTRIBUTE, ())):
            print(note, file=sys.stderr)
        raise
    raise AssertionError("unreachable command")


if __name__ == "__main__":
    raise SystemExit(main())
