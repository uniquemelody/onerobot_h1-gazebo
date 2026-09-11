"""Build private render worlds and capture reproducible Gazebo thumbnails."""

from __future__ import annotations

import argparse
import hashlib
import math
import os
import re
import secrets
import signal
import stat
import subprocess
import sys
import tempfile
import time
import uuid
from collections.abc import Callable, Mapping
from contextlib import suppress
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from types import MappingProxyType
from xml.etree import ElementTree
from xml.etree.ElementTree import Element, SubElement

import yaml

from onerobotics_a1_gazebo import demo as _demo
from onerobotics_a1_gazebo.sdf import add_text, serialize_sdf
from onerobotics_a1_gazebo.spec import load_model_specs
from onerobotics_a1_gazebo.thumbnail import (
    MANIFEST_NAME,
    ThumbnailSet,
    approved_assets_root,
    build_render_manifest,
    capture_render_config_bytes,
    capture_thumbnail_set,
    current_model_sdf_sha256,
    current_render_config_sha256,
    validate_thumbnail_bytes,
    validate_thumbnail_snapshot,
)
from onerobotics_a1_gazebo.validate import validate_render_input
from onerobotics_a1_gazebo.validation_snapshot import DirectorySnapshot, SnapshotError, capture_directory

_WORLD_NAME = "a1_render"
_FRAME_PATTERN_PREFIX = "camera_rig::link::render_camera_"
_FRAME_PATTERN_SUFFIX = ".png"
_CAMERA_TOPIC = "/world/a1_render/model/camera_rig/link/link/sensor/render_camera/image"
_RENDER_TIMEOUT = 45.0
_POLL_INTERVAL = 0.1
_run_harmonic = _demo._run_harmonic


@dataclass(frozen=True)
class LightSettings:
    name: str
    pose: tuple[float, ...]
    diffuse: tuple[float, ...]
    specular: tuple[float, ...]
    direction: tuple[float, ...]


@dataclass(frozen=True)
class RenderSettings:
    slug: str
    camera_pose: tuple[float, ...]
    ground_pose: tuple[float, ...]
    lights: tuple[LightSettings, ...]


@dataclass(frozen=True)
class RenderConfigSnapshot:
    settings: Mapping[str, RenderSettings]
    sha256: str


@dataclass(frozen=True)
class SelectedFrame:
    path: Path
    data: bytes
    suffix: int


@dataclass(frozen=True)
class RenderObservation:
    slug: str
    elapsed_seconds: float
    selected_frame_suffix: int
    png_sha256: str


@dataclass
class _ProcessOwner:
    process: subprocess.Popen[str] | None = None


class _UniqueKeyLoader(yaml.SafeLoader):
    pass


def _construct_unique_mapping(loader: _UniqueKeyLoader, node: yaml.nodes.MappingNode, deep: bool = False):
    mapping: dict[object, object] = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise ValueError(f"render config has duplicate key: {key}")
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    _construct_unique_mapping,
)


def render_config_path() -> Path:
    return Path(__file__).resolve().parents[1] / "config/render.yaml"


def _finite_tuple(value: object, length: int, field: str) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f"{field} must be a length-{length} numeric list")
    result: list[float] = []
    for item in value:
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            raise ValueError(f"{field} values must be finite numbers")
        number = float(item)
        if not math.isfinite(number):
            raise ValueError(f"{field} values must be finite numbers")
        result.append(number)
    return tuple(result)


def _parse_render_config(data: bytes) -> Mapping[str, RenderSettings]:
    try:
        document = yaml.load(data.decode("utf-8"), Loader=_UniqueKeyLoader)
    except ValueError:
        raise
    except (UnicodeError, yaml.YAMLError) as error:
        raise ValueError(f"unable to read render config: {error}") from None
    if not isinstance(document, dict) or set(document) != {"schema_version", "models"}:
        raise ValueError("render config root fields must be exact")
    if document["schema_version"] != 1 or isinstance(document["schema_version"], bool):
        raise ValueError("render config schema_version must be 1")
    models = document["models"]
    slugs = tuple(spec.slug for spec in load_model_specs())
    if not isinstance(models, dict) or set(models) != set(slugs):
        raise ValueError("render config models must contain exactly the three locked slugs")
    parsed: dict[str, RenderSettings] = {}
    for slug in slugs:
        value = models[slug]
        if not isinstance(value, dict) or set(value) != {"camera_pose", "ground_pose", "lights"}:
            raise ValueError(f"{slug} render fields must be exact")
        lights_value = value["lights"]
        if not isinstance(lights_value, list) or len(lights_value) != 2:
            raise ValueError(f"{slug}.lights must contain exactly two entries")
        lights: list[LightSettings] = []
        for index, light in enumerate(lights_value):
            if not isinstance(light, dict) or set(light) != {
                "name",
                "pose",
                "diffuse",
                "specular",
                "direction",
            }:
                raise ValueError(f"{slug}.lights[{index}] fields must be exact")
            name = light["name"]
            if not isinstance(name, str) or name not in {"key", "fill"}:
                raise ValueError(f"{slug}.lights[{index}].name must be key or fill")
            lights.append(
                LightSettings(
                    name=name,
                    pose=_finite_tuple(light["pose"], 6, f"{slug}.lights[{index}].pose"),
                    diffuse=_finite_tuple(light["diffuse"], 4, f"{slug}.lights[{index}].diffuse"),
                    specular=_finite_tuple(light["specular"], 4, f"{slug}.lights[{index}].specular"),
                    direction=_finite_tuple(light["direction"], 3, f"{slug}.lights[{index}].direction"),
                )
            )
        if {light.name for light in lights} != {"key", "fill"}:
            raise ValueError(f"{slug}.lights must contain key and fill exactly once")
        parsed[slug] = RenderSettings(
            slug=slug,
            camera_pose=_finite_tuple(value["camera_pose"], 6, f"{slug}.camera_pose"),
            ground_pose=_finite_tuple(value["ground_pose"], 6, f"{slug}.ground_pose"),
            lights=tuple(lights),
        )
    return MappingProxyType(parsed)


def capture_render_config(path: Path | None = None) -> RenderConfigSnapshot:
    """Capture one immutable config byte set, then parse and hash only those bytes."""
    config_path = Path(path) if path is not None else render_config_path()
    data = capture_render_config_bytes(config_path)
    return RenderConfigSnapshot(_parse_render_config(data), hashlib.sha256(data).hexdigest())


def load_render_config(path: Path | None = None) -> Mapping[str, RenderSettings]:
    """Strictly load the exact three-model public render configuration."""
    return capture_render_config(path).settings


def _numbers(values: tuple[float, ...]) -> str:
    return " ".join(format(value, ".12g") for value in values)


def _add_world_systems(world: Element) -> None:
    physics = SubElement(
        world,
        "plugin",
        {"filename": "gz-sim-physics-system", "name": "gz::sim::systems::Physics"},
    )
    engine = SubElement(physics, "engine")
    add_text(engine, "filename", "gz-physics-dartsim-plugin")
    SubElement(
        world,
        "plugin",
        {"filename": "gz-sim-user-commands-system", "name": "gz::sim::systems::UserCommands"},
    )
    SubElement(
        world,
        "plugin",
        {"filename": "gz-sim-scene-broadcaster-system", "name": "gz::sim::systems::SceneBroadcaster"},
    )
    sensors = SubElement(
        world,
        "plugin",
        {"filename": "gz-sim-sensors-system", "name": "gz::sim::systems::Sensors"},
    )
    add_text(sensors, "render_engine", "ogre2")


def _add_ground(world: Element, settings: RenderSettings) -> None:
    ground = SubElement(world, "model", {"name": "ground_plane"})
    add_text(ground, "static", "true")
    add_text(ground, "pose", _numbers(settings.ground_pose))
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
    add_text(material, "ambient", "0.55 0.55 0.55 1")
    add_text(material, "diffuse", "0.65 0.65 0.65 1")


def _add_lights(world: Element, settings: RenderSettings) -> None:
    for light_settings in settings.lights:
        light = SubElement(world, "light", {"name": light_settings.name, "type": "directional"})
        add_text(light, "cast_shadows", "true" if light_settings.name == "key" else "false")
        add_text(light, "pose", _numbers(light_settings.pose))
        add_text(light, "diffuse", _numbers(light_settings.diffuse))
        add_text(light, "specular", _numbers(light_settings.specular))
        attenuation = SubElement(light, "attenuation")
        add_text(attenuation, "range", "1000")
        add_text(light, "direction", _numbers(light_settings.direction))


def _add_camera(world: Element, settings: RenderSettings, frame_dir: Path) -> None:
    camera_model = SubElement(world, "model", {"name": "camera_rig"})
    add_text(camera_model, "static", "true")
    add_text(camera_model, "pose", _numbers(settings.camera_pose))
    link = SubElement(camera_model, "link", {"name": "link"})
    sensor = SubElement(link, "sensor", {"name": "render_camera", "type": "camera"})
    add_text(sensor, "always_on", "true")
    add_text(sensor, "update_rate", "20")
    add_text(sensor, "visualize", "false")
    camera = SubElement(sensor, "camera")
    add_text(camera, "horizontal_fov", "0.9")
    image = SubElement(camera, "image")
    add_text(image, "width", "512")
    add_text(image, "height", "512")
    add_text(image, "format", "R8G8B8")
    clip = SubElement(camera, "clip")
    add_text(clip, "near", "0.05")
    add_text(clip, "far", "20")
    save = SubElement(camera, "save", {"enabled": "true"})
    add_text(save, "path", os.fspath(frame_dir))


def _safe_model_slug(model_dir: Path) -> str:
    try:
        snapshot = capture_directory(model_dir)
    except SnapshotError as error:
        raise ValueError(f"render model snapshot failed: {error}") from None
    if snapshot.problems:
        raise ValueError("render model snapshot contains unsafe entries")
    if PurePosixPath("model.sdf") not in snapshot.files:
        raise ValueError("render model directory is missing regular model.sdf")
    slugs = {spec.slug for spec in load_model_specs()}
    if model_dir.name not in slugs:
        raise ValueError(f"unknown render model slug: {model_dir.name}")
    return model_dir.name


def _require_private_directory(path: Path, label: str) -> Path:
    if not path.is_absolute():
        raise ValueError(f"{label} must be absolute")
    try:
        details = path.lstat()
    except OSError as error:
        raise ValueError(f"{label} must be an existing directory: {error}") from None
    if stat.S_ISLNK(details.st_mode):
        raise ValueError(f"{label} must not be a symlink")
    if not stat.S_ISDIR(details.st_mode):
        raise ValueError(f"{label} must be a regular directory")
    return path


def _write_world(output_path: Path, data: bytes) -> None:
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    try:
        if output_path.exists() or output_path.is_symlink():
            details = output_path.lstat()
            if stat.S_ISLNK(details.st_mode) or not stat.S_ISREG(details.st_mode):
                raise ValueError("render world output must be a regular non-symlink file")
        temporary = output_path.parent / f".{output_path.name}.{secrets.token_hex(16)}"
        descriptor = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
        try:
            with os.fdopen(descriptor, "wb", closefd=True) as stream:
                descriptor = -1
                stream.write(data)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, output_path)
        finally:
            if descriptor >= 0:
                os.close(descriptor)
            if temporary.exists():
                temporary.unlink()
    except ValueError:
        raise
    except OSError as error:
        raise ValueError(f"unable to write render world safely: {error}") from None


def build_render_world(
    model_dir: Path,
    output_path: Path,
    frame_dir: Path,
    *,
    render_settings: RenderSettings | None = None,
) -> Path:
    """Write one deterministic private Ogre2 render world."""
    model_dir = Path(model_dir)
    slug = _safe_model_slug(model_dir)
    frame_dir = _require_private_directory(Path(frame_dir), "frame directory")
    settings = render_settings if render_settings is not None else load_render_config()[slug]
    if settings.slug != slug:
        raise ValueError("render settings slug does not match the model directory")
    sdf = Element("sdf", {"version": "1.11"})
    world = SubElement(sdf, "world", {"name": _WORLD_NAME})
    add_text(world, "gravity", "0 0 0")
    physics = SubElement(world, "physics", {"name": "render_step", "type": "ignored"})
    add_text(physics, "max_step_size", "0.001")
    add_text(physics, "real_time_update_rate", "1000")
    _add_world_systems(world)
    include = SubElement(world, "include")
    add_text(include, "uri", f"model://{slug}")
    add_text(include, "name", slug)
    _add_ground(world, settings)
    _add_lights(world, settings)
    _add_camera(world, settings, frame_dir)
    _write_world(Path(output_path), serialize_sdf(ElementTree.ElementTree(sdf)))
    return Path(output_path)


def _frame_suffix(path: PurePosixPath) -> int | None:
    name = path.name
    if not name.startswith(_FRAME_PATTERN_PREFIX) or not name.endswith(_FRAME_PATTERN_SUFFIX):
        return None
    token = name[len(_FRAME_PATTERN_PREFIX) : -len(_FRAME_PATTERN_SUFFIX)]
    if not token or (len(token) > 1 and token.startswith("0")) or not token.isascii() or not token.isdigit():
        return None
    return int(token)


def _capture_frames(frame_dir: Path) -> tuple[DirectorySnapshot, tuple[tuple[int, PurePosixPath, bytes], ...]]:
    try:
        snapshot = capture_directory(Path(frame_dir))
    except SnapshotError as error:
        raise ValueError(f"frame snapshot failed: {error}") from None
    if snapshot.problems:
        details = "; ".join(f"{problem.message}: {problem.path}" for problem in snapshot.problems)
        raise ValueError(f"frame snapshot failed: {details}")
    if snapshot.directories:
        raise ValueError("frame directory contains unexpected directories")
    frames: list[tuple[int, PurePosixPath, bytes]] = []
    for path, data in snapshot.files.items():
        suffix = _frame_suffix(path)
        if suffix is None:
            raise ValueError(f"unexpected frame entry: {path}")
        try:
            validate_thumbnail_bytes(data)
        except ValueError as error:
            raise ValueError(f"frame is not a complete PNG: {path}: {error}") from None
        frames.append((suffix, path, data))
    frames.sort(key=lambda item: item[0])
    if len(frames) < 3:
        raise ValueError("frame directory must contain at least three complete PNG frames")
    return snapshot, tuple(frames)


def capture_selected_frame(frame_dir: Path) -> SelectedFrame:
    """Capture and return the numerically greatest frame bytes from one snapshot."""
    _, frames = _capture_frames(Path(frame_dir))
    suffix, relative, data = frames[-1]
    return SelectedFrame(Path(frame_dir) / Path(*relative.parts), data, suffix)


def select_frame(frame_dir: Path) -> Path:
    """Select the numerically greatest complete frame, never by mtime or text order."""
    return capture_selected_frame(frame_dir).path


def _path_identity(path: Path) -> tuple[int, int, int]:
    try:
        details = path.lstat()
    except OSError as error:
        raise ValueError(f"path changed or was replaced: {path}: {error}") from None
    return details.st_dev, details.st_ino, stat.S_IFMT(details.st_mode)


def _verify_directory_identity(path: Path, expected: tuple[int, int, int], label: str) -> None:
    try:
        actual = _path_identity(path)
    except ValueError as error:
        raise ValueError(f"{label} changed or was replaced: {error}") from None
    if actual != expected or actual[2] != stat.S_IFDIR:
        raise ValueError(f"{label} changed or was replaced: {path}")


@dataclass
class _HeldDirectory:
    path: Path
    descriptor: int
    identity: tuple[int, int, int]


@dataclass
class _HeldChildDirectory:
    name: str
    descriptor: int
    identity: tuple[int, int, int]


def _details_identity(details: os.stat_result) -> tuple[int, int, int]:
    return details.st_dev, details.st_ino, stat.S_IFMT(details.st_mode)


def _open_held_directory(path: Path, label: str) -> _HeldDirectory:
    path = Path(path)
    if not path.is_absolute() or not path.name or path.name in {".", ".."}:
        raise ValueError(f"{label} must be one absolute directory path")
    try:
        inspected = path.lstat()
    except OSError as error:
        raise ValueError(f"{label} is inaccessible: {error}") from None
    if stat.S_ISLNK(inspected.st_mode):
        raise ValueError(f"{label} must not be a symlink")
    if not stat.S_ISDIR(inspected.st_mode):
        raise ValueError(f"{label} must be a regular directory")
    try:
        descriptor = os.open(path, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC)
    except OSError as error:
        raise ValueError(f"{label} changed while opening: {error}") from None
    identity = _details_identity(inspected)
    if _details_identity(os.fstat(descriptor)) != identity:
        os.close(descriptor)
        raise ValueError(f"{label} changed while opening")
    return _HeldDirectory(path, descriptor, identity)


def _open_or_create_direct_parent(path: Path, label: str) -> _HeldDirectory:
    path = Path(path)
    try:
        path.lstat()
    except FileNotFoundError:
        if not path.is_absolute() or not path.name or path.name in {".", ".."}:
            raise ValueError(f"{label} must be one absolute direct parent") from None
        grandparent = _open_held_directory(path.parent, f"{label} grandparent")
        try:
            _verify_held_directory(grandparent, f"{label} grandparent")
            with suppress(FileExistsError):
                os.mkdir(path.name, mode=0o755, dir_fd=grandparent.descriptor)
            try:
                inspected = os.stat(path.name, dir_fd=grandparent.descriptor, follow_symlinks=False)
            except OSError as error:
                raise ValueError(f"{label} changed during creation: {error}") from None
            if stat.S_ISLNK(inspected.st_mode) or not stat.S_ISDIR(inspected.st_mode):
                raise ValueError(f"{label} must be a regular non-symlink directory")
            descriptor = os.open(
                path.name,
                os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC,
                dir_fd=grandparent.descriptor,
            )
            identity = _details_identity(inspected)
            if _details_identity(os.fstat(descriptor)) != identity:
                os.close(descriptor)
                raise ValueError(f"{label} changed while opening")
            return _HeldDirectory(path, descriptor, identity)
        finally:
            os.close(grandparent.descriptor)
    return _open_held_directory(path, label)


def _verify_held_directory(directory: _HeldDirectory, label: str) -> None:
    try:
        opened = _details_identity(os.fstat(directory.descriptor))
        named = _details_identity(directory.path.lstat())
    except OSError as error:
        raise ValueError(f"{label} changed or was replaced: {error}") from None
    if opened != directory.identity or named != directory.identity or opened[2] != stat.S_IFDIR:
        raise ValueError(f"{label} changed or was replaced")


def _entry_details(directory_fd: int, name: str) -> os.stat_result | None:
    try:
        return os.stat(name, dir_fd=directory_fd, follow_symlinks=False)
    except FileNotFoundError:
        return None


def _open_child_directory(parent: _HeldDirectory, name: str, label: str) -> _HeldChildDirectory:
    try:
        inspected = os.stat(name, dir_fd=parent.descriptor, follow_symlinks=False)
    except OSError as error:
        raise ValueError(f"{label} is inaccessible: {error}") from None
    if stat.S_ISLNK(inspected.st_mode):
        raise ValueError(f"{label} must not be a symlink")
    if not stat.S_ISDIR(inspected.st_mode):
        raise ValueError(f"{label} must be a regular directory")
    try:
        descriptor = os.open(
            name,
            os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC,
            dir_fd=parent.descriptor,
        )
    except OSError as error:
        raise ValueError(f"{label} changed while opening: {error}") from None
    identity = _details_identity(inspected)
    if _details_identity(os.fstat(descriptor)) != identity:
        os.close(descriptor)
        raise ValueError(f"{label} changed while opening")
    return _HeldChildDirectory(name, descriptor, identity)


def _verify_child_directory(
    parent: _HeldDirectory,
    child: _HeldChildDirectory,
    name: str,
    label: str,
) -> None:
    try:
        opened = _details_identity(os.fstat(child.descriptor))
        inspected = os.stat(name, dir_fd=parent.descriptor, follow_symlinks=False)
        named = _details_identity(inspected)
    except OSError as error:
        raise ValueError(f"{label} changed or was replaced: {error}") from None
    if opened != child.identity or named != child.identity or opened[2] != stat.S_IFDIR:
        raise ValueError(f"{label} changed or was replaced")


def _held_child_location(
    parent: _HeldDirectory,
    child: _HeldChildDirectory,
    names: tuple[str, str],
    label: str,
) -> str:
    try:
        if _details_identity(os.fstat(parent.descriptor)) != parent.identity:
            raise ValueError(f"{label} parent descriptor changed")
        if _details_identity(os.fstat(child.descriptor)) != child.identity:
            raise ValueError(f"{label} child descriptor changed")
        matches = []
        for name in names:
            details = _entry_details(parent.descriptor, name)
            if details is not None and _details_identity(details) == child.identity:
                matches.append(name)
    except OSError as error:
        raise ValueError(f"unable to reconcile {label}: {error}") from None
    if len(matches) != 1:
        raise ValueError(f"unable to reconcile {label}: expected one retained child name, found {matches}")
    return matches[0]


def _rename_reached_destination_after_error(
    operation_error: BaseException,
    parent: _HeldDirectory,
    child: _HeldChildDirectory,
    source_name: str,
    destination_name: str,
    label: str,
) -> bool | None:
    try:
        location = _held_child_location(parent, child, (source_name, destination_name), label)
    except BaseException as error:
        _demo._add_exception_note(
            operation_error,
            f"{label} outcome reconciliation failed: {type(error).__name__}: {error}",
        )
        _demo._copy_exception_notes(error, operation_error)
        return None
    return location == destination_name


def _replace_held_child_reconciled(
    parent: _HeldDirectory,
    child: _HeldChildDirectory,
    source_name: str,
    destination_name: str,
    label: str,
    record_outcome: Callable[[bool], None],
) -> None:
    try:
        os.replace(
            source_name,
            destination_name,
            src_dir_fd=parent.descriptor,
            dst_dir_fd=parent.descriptor,
        )
    except BaseException as operation_error:
        outcome = _rename_reached_destination_after_error(
            operation_error,
            parent,
            child,
            source_name,
            destination_name,
            label,
        )
        if outcome is not None:
            record_outcome(outcome)
        raise
    else:
        record_outcome(True)


def _create_child_directory(parent: _HeldDirectory, prefix: str) -> _HeldChildDirectory:
    for _ in range(128):
        name = f"{prefix}{secrets.token_hex(16)}"
        try:
            os.mkdir(name, mode=0o700, dir_fd=parent.descriptor)
        except FileExistsError:
            continue
        return _open_child_directory(parent, name, "private staging directory")
    raise ValueError("unable to allocate a unique private staging directory")


def _write_regular_file_at(directory_fd: int, name: str, data: bytes) -> None:
    descriptor = os.open(
        name,
        os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
        0o600,
        dir_fd=directory_fd,
    )
    try:
        with os.fdopen(descriptor, "wb", closefd=True) as stream:
            descriptor = -1
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
    finally:
        if descriptor >= 0:
            os.close(descriptor)


def _write_thumbnail_snapshot(directory_fd: int, snapshot: ThumbnailSet) -> None:
    for slug, data in snapshot.png_by_slug.items():
        _write_regular_file_at(directory_fd, f"{slug}.png", data)
    _write_regular_file_at(directory_fd, MANIFEST_NAME, snapshot.manifest_bytes)
    os.fsync(directory_fd)


def _read_regular_file_at(directory_fd: int, name: str) -> bytes:
    inspected = os.stat(name, dir_fd=directory_fd, follow_symlinks=False)
    if stat.S_ISLNK(inspected.st_mode) or not stat.S_ISREG(inspected.st_mode):
        raise ValueError(f"thumbnail entry must be a regular non-symlink file: {name}")
    if inspected.st_size > 4 * 1024 * 1024:
        raise ValueError(f"thumbnail entry exceeds size limit: {name}")
    descriptor = os.open(
        name,
        os.O_RDONLY | os.O_NONBLOCK | os.O_NOFOLLOW | os.O_CLOEXEC,
        dir_fd=directory_fd,
    )
    try:
        opened = os.fstat(descriptor)
        identity = (inspected.st_dev, inspected.st_ino, inspected.st_size, inspected.st_mtime_ns, inspected.st_ctime_ns)
        if identity != (opened.st_dev, opened.st_ino, opened.st_size, opened.st_mtime_ns, opened.st_ctime_ns):
            raise ValueError(f"thumbnail entry changed while opening: {name}")
        remaining = opened.st_size
        chunks: list[bytes] = []
        while remaining:
            chunk = os.read(descriptor, min(remaining, 1024 * 1024))
            if not chunk:
                break
            chunks.append(chunk)
            remaining -= len(chunk)
        if remaining or os.read(descriptor, 1):
            raise ValueError(f"thumbnail entry changed while reading: {name}")
        after = os.fstat(descriptor)
        named = os.stat(name, dir_fd=directory_fd, follow_symlinks=False)
        after_identity = (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns, after.st_ctime_ns)
        if identity != after_identity or identity != (
            named.st_dev,
            named.st_ino,
            named.st_size,
            named.st_mtime_ns,
            named.st_ctime_ns,
        ):
            raise ValueError(f"thumbnail entry changed while reading: {name}")
        return b"".join(chunks)
    finally:
        os.close(descriptor)


def _post_write_thumbnail_check(
    directory_fd: int,
    *,
    expected_render_config_sha256: str,
    expected_model_sdf_sha256: Mapping[str, str],
) -> ThumbnailSet:
    files: dict[PurePosixPath, bytes] = {}
    before = os.fstat(directory_fd)
    with os.scandir(directory_fd) as entries:
        names = sorted(entry.name for entry in entries)
    for name in names:
        files[PurePosixPath(name)] = _read_regular_file_at(directory_fd, name)
    with os.scandir(directory_fd) as entries:
        after_names = sorted(entry.name for entry in entries)
    after = os.fstat(directory_fd)
    before_identity = (
        before.st_dev,
        before.st_ino,
        before.st_size,
        before.st_mtime_ns,
        before.st_ctime_ns,
    )
    after_identity = (
        after.st_dev,
        after.st_ino,
        after.st_size,
        after.st_mtime_ns,
        after.st_ctime_ns,
    )
    if names != after_names or before_identity != after_identity:
        raise ValueError("thumbnail directory changed while validating installed bytes")
    snapshot = DirectorySnapshot(MappingProxyType(files), frozenset())
    return validate_thumbnail_snapshot(
        snapshot,
        expected_render_config_sha256=expected_render_config_sha256,
        expected_model_sdf_sha256=expected_model_sdf_sha256,
    )


def _remove_entry_at(parent_fd: int, name: str) -> None:
    details = _entry_details(parent_fd, name)
    if details is None:
        return
    if stat.S_ISDIR(details.st_mode) and not stat.S_ISLNK(details.st_mode):
        child_fd = os.open(
            name,
            os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC,
            dir_fd=parent_fd,
        )
        try:
            identity = _details_identity(details)
            if _details_identity(os.fstat(child_fd)) != identity:
                raise ValueError(f"cleanup directory changed while opening: {name}")
            with os.scandir(child_fd) as entries:
                children = [entry.name for entry in entries]
            for child in children:
                _remove_entry_at(child_fd, child)
            named = os.stat(name, dir_fd=parent_fd, follow_symlinks=False)
            if _details_identity(named) != identity:
                raise ValueError(f"cleanup directory changed while removing: {name}")
        finally:
            os.close(child_fd)
        os.rmdir(name, dir_fd=parent_fd)
    else:
        os.unlink(name, dir_fd=parent_fd)


def _remove_held_child_at(
    parent: _HeldDirectory,
    child: _HeldChildDirectory,
    name: str,
    label: str,
) -> None:
    _verify_child_directory(parent, child, name, label)
    with os.scandir(child.descriptor) as entries:
        children = [entry.name for entry in entries]
    for entry in children:
        _remove_entry_at(child.descriptor, entry)
    _verify_child_directory(parent, child, name, label)
    os.rmdir(name, dir_fd=parent.descriptor)


def _same_thumbnail_set(left: ThumbnailSet, right: ThumbnailSet) -> bool:
    return left.manifest_bytes == right.manifest_bytes and dict(left.png_by_slug) == dict(right.png_by_slug)


def _finish_resource_cleanup(
    operation_error: BaseException | None,
    actions: tuple[tuple[str, Callable[[], None]], ...],
) -> None:
    cleanup_error: BaseException | None = None
    for label, action in actions:
        try:
            action()
        except BaseException as error:
            if operation_error is not None:
                _demo._add_exception_note(
                    operation_error,
                    f"{label}: {type(error).__name__}: {error}",
                )
                _demo._copy_exception_notes(error, operation_error)
            elif cleanup_error is None:
                cleanup_error = error
                _demo._add_exception_note(error, label)
            else:
                _demo._add_exception_note(
                    cleanup_error,
                    f"{label}: {type(error).__name__}: {error}",
                )
                _demo._copy_exception_notes(error, cleanup_error)
    if operation_error is None and cleanup_error is not None:
        raise cleanup_error


def approve_candidates(
    candidate_root: Path,
    *,
    approved_root: Path | None = None,
    expected_render_config_sha256: str | None = None,
    expected_model_sdf_sha256: Mapping[str, str] | None = None,
) -> Path:
    """Atomically install the exact bytes of one already-reviewed candidate set."""
    fixed_config_sha256 = (
        expected_render_config_sha256 if expected_render_config_sha256 is not None else current_render_config_sha256()
    )
    fixed_model_sha256 = (
        expected_model_sdf_sha256 if expected_model_sdf_sha256 is not None else current_model_sdf_sha256()
    )
    candidate_root = Path(candidate_root)
    candidate_identity = _path_identity(candidate_root)
    if candidate_identity[2] == stat.S_IFLNK:
        raise ValueError("candidate root must not be a symlink")
    if candidate_identity[2] != stat.S_IFDIR:
        raise ValueError("candidate root must be a regular directory")
    candidate = capture_thumbnail_set(
        candidate_root,
        expected_render_config_sha256=fixed_config_sha256,
        expected_model_sdf_sha256=fixed_model_sha256,
    )
    _verify_directory_identity(candidate_root, candidate_identity, "candidate root")

    requested = Path(approved_root) if approved_root is not None else approved_assets_root()
    if not requested.is_absolute() or not requested.name or requested.name in {".", ".."}:
        raise ValueError("approved root must name a directory")
    parent = _open_or_create_direct_parent(requested.parent, "approved root parent")
    destination_name = requested.name
    previous: _HeldChildDirectory | None = None
    try:
        details = _entry_details(parent.descriptor, destination_name)
        if details is not None:
            if stat.S_ISLNK(details.st_mode):
                raise ValueError("approved root must not be a symlink")
            if not stat.S_ISDIR(details.st_mode):
                raise ValueError("approved root must be a regular directory")
            previous = _open_child_directory(parent, destination_name, "approved root")
        staging = _create_child_directory(parent, f".{destination_name}.stage-")
    except BaseException as operation_error:
        actions: list[tuple[str, Callable[[], None]]] = []
        if previous is not None:
            actions.append(("approved previous descriptor cleanup failed", lambda: os.close(previous.descriptor)))
        actions.append(("approved parent descriptor cleanup failed", lambda: os.close(parent.descriptor)))
        _finish_resource_cleanup(operation_error, tuple(actions))
        raise
    backup_name = f".{destination_name}.backup-{uuid.uuid4().hex}"
    installed = False
    moved_previous = False
    committed = False
    active_error: BaseException | None = None

    def record_previous_move(outcome: bool) -> None:
        nonlocal moved_previous
        moved_previous = outcome

    def record_install(outcome: bool) -> None:
        nonlocal installed
        installed = outcome

    try:
        _write_thumbnail_snapshot(staging.descriptor, candidate)
        staged_snapshot = _post_write_thumbnail_check(
            staging.descriptor,
            expected_render_config_sha256=fixed_config_sha256,
            expected_model_sdf_sha256=fixed_model_sha256,
        )
        if staged_snapshot is not None and not _same_thumbnail_set(candidate, staged_snapshot):
            raise ValueError("staged approved thumbnail bytes differ from the candidate snapshot")
        _verify_directory_identity(candidate_root, candidate_identity, "candidate root")
        _verify_held_directory(parent, "approved root parent")
        if previous is not None:
            _verify_child_directory(parent, previous, destination_name, "approved root")
            _replace_held_child_reconciled(
                parent,
                previous,
                destination_name,
                backup_name,
                "approved backup rename",
                record_previous_move,
            )
            _verify_child_directory(parent, previous, backup_name, "approved root backup")
        else:
            if _entry_details(parent.descriptor, destination_name) is not None:
                raise ValueError("approved root appeared during installation")
        _verify_child_directory(parent, staging, staging.name, "approved staging directory")
        _replace_held_child_reconciled(
            parent,
            staging,
            staging.name,
            destination_name,
            "approved install rename",
            record_install,
        )
        _verify_held_directory(parent, "approved root parent")
        _verify_child_directory(parent, staging, destination_name, "installed approved root")
        installed_snapshot = _post_write_thumbnail_check(
            staging.descriptor,
            expected_render_config_sha256=fixed_config_sha256,
            expected_model_sdf_sha256=fixed_model_sha256,
        )
        if installed_snapshot is not None and not _same_thumbnail_set(candidate, installed_snapshot):
            raise ValueError("approved thumbnail bytes differ after atomic installation")
        _verify_held_directory(parent, "approved root parent")
        committed = True
        if moved_previous:
            if previous is None:
                raise ValueError("approved root backup lost its retained directory")
            _remove_held_child_at(parent, previous, backup_name, "approved root backup")
            moved_previous = False
        _verify_held_directory(parent, "approved root parent")
        return requested
    except BaseException as operation_error:
        active_error = operation_error
        rollback_errors: list[str] = []
        if installed and not committed:
            try:
                _verify_child_directory(parent, staging, destination_name, "failed approved root")
                _remove_held_child_at(parent, staging, destination_name, "failed approved root")
                installed = False
            except BaseException as error:
                rollback_errors.append(f"unable to remove failed approved install: {error}")
        if moved_previous and not committed:
            try:
                os.replace(
                    backup_name,
                    destination_name,
                    src_dir_fd=parent.descriptor,
                    dst_dir_fd=parent.descriptor,
                )
                moved_previous = False
                if previous is not None:
                    _verify_child_directory(parent, previous, destination_name, "restored approved root")
            except BaseException as error:
                rollback_errors.append(f"unable to restore previous approved root: {error}")
        if rollback_errors:
            _demo._add_exception_note(operation_error, "; ".join(rollback_errors))
        raise
    finally:

        def remove_uninstalled_staging() -> None:
            if not installed:
                if _entry_details(parent.descriptor, staging.name) is not None:
                    _remove_held_child_at(parent, staging, staging.name, "approved staging directory")

        cleanup_actions: list[tuple[str, Callable[[], None]]] = [
            ("approved staging cleanup failed", remove_uninstalled_staging),
            ("approved staging descriptor close failed", lambda: os.close(staging.descriptor)),
        ]
        if previous is not None:
            cleanup_actions.append(("approved previous descriptor close failed", lambda: os.close(previous.descriptor)))
        cleanup_actions.append(("approved parent descriptor close failed", lambda: os.close(parent.descriptor)))
        _finish_resource_cleanup(active_error, tuple(cleanup_actions))


def _render_critical_error(log: str) -> str | None:
    inherited = _demo._critical_server_error(log)
    if inherited is not None:
        return inherited
    patterns = (
        r"(?im)^.*(?:warning|\[wrn\]).*(?:sensor|ogre|render|mesh).*(?:fail|unable|error|missing).*$",
        r"(?im)^(?=.*\b(?:sensors?|ogre2?|render(?:ing)?|mesh)\b)"
        r"(?=.*\b(?:fail(?:ed|s|ure)?|unable|error|missing|could\s+not|cannot)\b).*$",
    )
    for pattern in patterns:
        match = re.search(pattern, log)
        if match is not None:
            return match.group(0)[-2048:]
    return None


def _check_render_server(server: subprocess.Popen[str], log_tail: _demo._BoundedLogTail) -> None:
    _demo._check_server(server, log_tail)
    critical = _render_critical_error(log_tail.text())
    if critical is not None:
        raise RuntimeError(f"Gazebo Sensors/Ogre2/mesh load error: {critical}")


def _require_harmonic(result: _demo._CommandResult, label: str) -> str:
    if result.returncode != 0:
        output = (result.stdout + result.stderr)[-8192:]
        raise RuntimeError(f"{label} failed with exit {result.returncode}: {output}")
    return result.stdout


def _remaining(deadline: float, message: str) -> float:
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        raise RuntimeError(message)
    return min(3.0, remaining)


def _topic_information(
    *,
    partition: str,
    private_export: Path,
    extra_environment: tuple[str, ...],
    deadline: float,
) -> str:
    result = _run_harmonic(
        partition=partition,
        private_export=private_export,
        command=["topic", "--force-version", "13", "-i", "-t", _CAMERA_TOPIC],
        timeout=_remaining(deadline, "camera topic inspection timeout"),
        timeout_message="camera topic inspection timeout",
        deadline=deadline,
        extra_environment=extra_environment,
    )
    return _require_harmonic(result, "camera topic inspection")


def _subscriber_message_types(information: str) -> tuple[str, ...]:
    lines = information.splitlines()
    mentions = [index for index, line in enumerate(lines) if line.strip().startswith("Subscribers")]
    exact = [index for index in mentions if lines[index].strip() == "Subscribers [Address, Message Type]:"]
    if not exact:
        if mentions:
            raise ValueError("camera subscriber section header is malformed")
        return ()
    if len(mentions) != 1 or len(exact) != 1:
        raise ValueError("camera subscriber section must appear exactly once")
    message_types: list[str] = []
    for line in lines[exact[0] + 1 :]:
        if not line.strip():
            continue
        columns = [column.strip() for column in line.split(",")]
        if len(columns) != 2 or not all(columns):
            raise ValueError("camera subscriber row is malformed")
        message_type = columns[1]
        if message_type not in {"gz.msgs.Image", "google.protobuf.Message"}:
            raise ValueError(f"camera subscriber type is not an Image-compatible type: {message_type}")
        message_types.append(message_type)
    return tuple(message_types)


def _publisher_message_types(information: str) -> tuple[str, ...]:
    lines = information.splitlines()
    mentions = [index for index, line in enumerate(lines) if line.strip().startswith("Publishers")]
    exact = [index for index in mentions if lines[index].strip() == "Publishers [Address, Message Type]:"]
    if not exact:
        if mentions:
            raise ValueError("camera publisher section header is malformed")
        raise ValueError("camera publisher section is missing")
    if len(mentions) != 1 or len(exact) != 1:
        raise ValueError("camera publisher section must appear exactly once")
    message_types: list[str] = []
    publisher_rows = lines[exact[0] + 1 :]
    no_subscriber_terminator = f"No subscribers on topic [{_CAMERA_TOPIC}]"
    for index, line in enumerate(publisher_rows):
        stripped = line.strip()
        if stripped.startswith("Subscribers"):
            break
        if stripped == no_subscriber_terminator:
            if any(trailing.strip() for trailing in publisher_rows[index + 1 :]):
                raise ValueError("camera publisher no-subscriber terminator must be final")
            break
        if not stripped:
            continue
        columns = [column.strip() for column in line.split(",")]
        if len(columns) != 2 or not all(columns):
            raise ValueError("camera publisher row is malformed")
        message_types.append(columns[1])
    return tuple(message_types)


def _check_subscriber(subscriber: subprocess.Popen[str]) -> None:
    returncode = subscriber.poll()
    if returncode not in (None, 0):
        raise RuntimeError(f"camera subscriber exited after connection with code {returncode}")


def _cleanup_subscriber_group(
    subscriber: subprocess.Popen[str],
    record_interruption: Callable[[BaseException, str], None],
    cleanup_errors: list[str],
) -> None:
    for _attempt in range(2):
        try:
            cleanup = _demo._kill_child_group(subscriber)
        except BaseException as error:
            record_interruption(error, "camera subscriber cleanup interrupted")
            continue
        if cleanup.deferred_error is not None:
            record_interruption(cleanup.deferred_error, "camera subscriber cleanup interrupted")
        if cleanup.cleaned:
            return

    try:
        os.killpg(subscriber.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    except BaseException as error:
        record_interruption(error, "camera subscriber direct SIGKILL interrupted")
    try:
        subscriber.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        cleanup_errors.append("camera subscriber wait timed out after direct SIGKILL")
    except BaseException as error:
        record_interruption(error, "camera subscriber post-SIGKILL wait interrupted")
    try:
        cleaned = _demo._wait_for_process_group_exit(subscriber.pid, 3.0)
    except BaseException as error:
        record_interruption(error, "camera subscriber process-group verification interrupted")
        cleaned = False
    if not cleaned:
        cleanup_errors.append(f"camera subscriber process group {subscriber.pid} survived direct SIGKILL")


def _world_pause(
    *,
    paused: bool,
    partition: str,
    private_export: Path,
    extra_environment: tuple[str, ...],
    deadline: float,
) -> None:
    action = "pause" if paused else "unpause"
    result = _run_harmonic(
        partition=partition,
        private_export=private_export,
        command=[
            "service",
            "--force-version",
            "13",
            "-s",
            f"/world/{_WORLD_NAME}/control",
            "--reqtype",
            "gz.msgs.WorldControl",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "2000",
            "--req",
            f"pause: {'true' if paused else 'false'}",
        ],
        timeout=_remaining(deadline, f"world {action} timeout"),
        timeout_message=f"world {action} timeout",
        deadline=deadline,
        extra_environment=extra_environment,
    )
    if result.returncode != 0 or re.search(r"(?m)^\s*data:\s*true\s*$", result.stdout) is None:
        output = (result.stdout + result.stderr)[-8192:]
        raise RuntimeError(f"world {action} did not return semantic data: true: {output}")


def _discover_camera_and_connect(
    *,
    server: subprocess.Popen[str],
    log_tail: _demo._BoundedLogTail,
    partition: str,
    private_export: Path,
    extra_environment: tuple[str, ...],
    deadline: float,
    latest_transport: list[str],
    subscriber_owner: _ProcessOwner,
) -> subprocess.Popen[str]:
    while True:
        _check_render_server(server, log_tail)
        listing = _run_harmonic(
            partition=partition,
            private_export=private_export,
            command=["topic", "--force-version", "13", "-l"],
            timeout=_remaining(deadline, "camera topic discovery timeout"),
            timeout_message="camera topic discovery timeout",
            deadline=deadline,
            extra_environment=extra_environment,
        )
        latest_transport[0] = _require_harmonic(listing, "camera topic discovery")
        topics = {line.strip() for line in latest_transport[0].splitlines() if line.strip()}
        if _CAMERA_TOPIC in topics:
            break
        if time.monotonic() >= deadline:
            raise RuntimeError("camera topic discovery timeout")
        time.sleep(min(_POLL_INTERVAL, max(0.0, deadline - time.monotonic())))

    information = _topic_information(
        partition=partition,
        private_export=private_export,
        extra_environment=extra_environment,
        deadline=deadline,
    )
    latest_transport[0] = information
    try:
        publisher_types = _publisher_message_types(information)
    except ValueError as error:
        raise RuntimeError(str(error)) from error
    if publisher_types != ("gz.msgs.Image",):
        raise RuntimeError(f"camera topic must have exactly one gz.msgs.Image publisher: {information[-2048:]}")
    try:
        baseline_subscribers = len(_subscriber_message_types(information))
    except ValueError as error:
        raise RuntimeError(str(error)) from error

    argv = _demo._harmonic_argv(
        partition,
        private_export,
        [
            "topic",
            "--force-version",
            "13",
            "-e",
            "-n",
            "3",
            "--json-output",
            "-t",
            _CAMERA_TOPIC,
        ],
        extra_environment=extra_environment,
    )
    try:
        subscriber = subprocess.Popen(
            argv,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            text=True,
            start_new_session=True,
        )
    except OSError as error:
        raise RuntimeError(f"unable to start camera subscriber: {error}") from error
    subscriber_owner.process = subscriber

    while True:
        _check_render_server(server, log_tail)
        if subscriber.poll() is not None:
            raise RuntimeError(f"camera subscriber exited before connection with code {subscriber.returncode}")
        information = _topic_information(
            partition=partition,
            private_export=private_export,
            extra_environment=extra_environment,
            deadline=deadline,
        )
        latest_transport[0] = information
        try:
            subscribers = _subscriber_message_types(information)
        except ValueError as error:
            raise RuntimeError(str(error)) from error
        if len(subscribers) > baseline_subscribers:
            return subscriber
        if time.monotonic() >= deadline:
            raise RuntimeError("camera subscriber connection timeout")
        time.sleep(min(_POLL_INTERVAL, max(0.0, deadline - time.monotonic())))


def _wait_for_complete_frame(
    *,
    frame_dir: Path,
    server: subprocess.Popen[str],
    subscriber: subprocess.Popen[str],
    log_tail: _demo._BoundedLogTail,
    deadline: float,
    latest_frame_error: list[str],
) -> SelectedFrame:
    while True:
        _check_render_server(server, log_tail)
        _check_subscriber(subscriber)
        try:
            return capture_selected_frame(frame_dir)
        except ValueError as error:
            latest_frame_error[0] = str(error)
        if time.monotonic() >= deadline:
            raise RuntimeError("frame capture timeout")
        time.sleep(min(_POLL_INTERVAL, max(0.0, deadline - time.monotonic())))


def _stable_paused_frame(
    frame_dir: Path,
    server: subprocess.Popen[str],
    subscriber: subprocess.Popen[str],
    log_tail: _demo._BoundedLogTail,
    deadline: float,
) -> SelectedFrame:
    previous: SelectedFrame | None = None
    while True:
        _check_render_server(server, log_tail)
        _check_subscriber(subscriber)
        if time.monotonic() >= deadline:
            raise RuntimeError("paused frame stability timeout")
        try:
            current = capture_selected_frame(frame_dir)
        except ValueError:
            previous = None
        else:
            if previous is not None and current.suffix == previous.suffix and current.data == previous.data:
                return current
            previous = current
        time.sleep(min(_POLL_INTERVAL, max(0.0, deadline - time.monotonic())))


def _render_diagnostic(
    message: str,
    log_tail: _demo._BoundedLogTail,
    latest_transport: str,
    latest_frame_error: str,
) -> str:
    return (
        f"{message}\nserver log tail:\n{log_tail.text() or '<empty>'}"
        f"\nlatest camera transport:\n{latest_transport or '<none>'}"
        f"\nlatest frame status:\n{latest_frame_error or '<none>'}"
    )


def _record_render_observation(
    observations: list[RenderObservation] | None,
    *,
    model_dir: Path,
    started: float,
    selected_frame: SelectedFrame,
) -> None:
    if observations is None:
        return
    observations.append(
        RenderObservation(
            slug=Path(model_dir).name,
            elapsed_seconds=time.monotonic() - started,
            selected_frame_suffix=selected_frame.suffix,
            png_sha256=hashlib.sha256(selected_frame.data).hexdigest(),
        )
    )


def _require_render_result(
    result: bytes | None,
    selected_frame: SelectedFrame | None,
    log_tail: _demo._BoundedLogTail,
    latest_transport: str,
    latest_frame_error: str,
) -> SelectedFrame:
    if result is None:
        raise RuntimeError(
            _render_diagnostic("render returned no frame", log_tail, latest_transport, latest_frame_error)
        )
    if selected_frame is None or selected_frame.data != result:
        raise RuntimeError("render selected no stable frame")
    validate_thumbnail_bytes(result)
    return selected_frame


def capture_model_thumbnail(
    *,
    model_dir: Path,
    private_export: Path,
    run_root: Path,
    timeout: float = _RENDER_TIMEOUT,
    render_settings: RenderSettings | None = None,
    observations: list[RenderObservation] | None = None,
) -> bytes:
    """Capture one real server-side Ogre2 frame with bounded process cleanup."""
    started = time.monotonic()
    if isinstance(timeout, bool) or not isinstance(timeout, (int, float)) or not math.isfinite(float(timeout)):
        raise ValueError("render timeout must be one finite positive number")
    timeout = float(timeout)
    if timeout <= 0:
        raise ValueError("render timeout must be one finite positive number")
    private_export = _require_private_directory(Path(private_export), "private export")
    run_root = _require_private_directory(Path(run_root), "private run root")
    frames = run_root / "frames"
    worlds = run_root / "worlds"
    home = run_root / "home"
    cache = run_root / "cache"
    for directory in (frames, worlds, home, cache):
        directory.mkdir(mode=0o700)
    world = build_render_world(
        Path(model_dir),
        worlds / f"{Path(model_dir).name}.sdf",
        frames,
        render_settings=render_settings,
    )
    partition = f"onerobotics-a1-render-{os.getpid()}-{secrets.token_hex(8)}"
    inner_environment = (
        "QT_QPA_PLATFORM=offscreen",
        "LIBGL_ALWAYS_SOFTWARE=1",
        f"HOME={home}",
        f"XDG_CACHE_HOME={cache}",
    )
    argv = _demo._harmonic_argv(
        partition,
        private_export,
        ["sim", "--force-version", "8", "-s", "--headless-rendering", "-v", "4", str(world)],
        extra_environment=inner_environment,
    )
    server: subprocess.Popen[str] | None = None
    subscriber: subprocess.Popen[str] | None = None
    subscriber_owner = _ProcessOwner()
    log_tail = _demo._BoundedLogTail(classifier=_render_critical_error)
    latest_transport = [""]
    latest_frame_error = [""]
    result: bytes | None = None
    selected_frame: SelectedFrame | None = None
    operation_error: BaseException | None = None
    cleanup_errors: list[str] = []
    cleanup_interruption: BaseException | None = None

    def record_cleanup_interruption(error: BaseException, label: str) -> None:
        nonlocal cleanup_interruption
        if cleanup_interruption is None:
            cleanup_interruption = error
            _demo._add_exception_note(error, label)
        else:
            _demo._add_exception_note(
                cleanup_interruption,
                f"{label}: {type(error).__name__}: {error}",
            )
            _demo._copy_exception_notes(error, cleanup_interruption)

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
            raise RuntimeError(f"unable to start Gazebo render server: {error}") from error
        log_tail.start(server.stdout, server.stderr)
        deadline = time.monotonic() + timeout
        subscriber = _discover_camera_and_connect(
            server=server,
            log_tail=log_tail,
            partition=partition,
            private_export=private_export,
            extra_environment=inner_environment,
            deadline=deadline,
            latest_transport=latest_transport,
            subscriber_owner=subscriber_owner,
        )
        _check_subscriber(subscriber)
        _world_pause(
            paused=False,
            partition=partition,
            private_export=private_export,
            extra_environment=inner_environment,
            deadline=deadline,
        )
        _check_subscriber(subscriber)
        _wait_for_complete_frame(
            frame_dir=frames,
            server=server,
            subscriber=subscriber,
            log_tail=log_tail,
            deadline=deadline,
            latest_frame_error=latest_frame_error,
        )
        _check_subscriber(subscriber)
        _world_pause(
            paused=True,
            partition=partition,
            private_export=private_export,
            extra_environment=inner_environment,
            deadline=deadline,
        )
        _check_subscriber(subscriber)
        selected_frame = _stable_paused_frame(frames, server, subscriber, log_tail, deadline)
        result = selected_frame.data
        _check_render_server(server, log_tail)
        _check_subscriber(subscriber)
    except BaseException as error:
        operation_error = error
    finally:
        if subscriber is None:
            subscriber = subscriber_owner.process
        if subscriber is not None:
            _cleanup_subscriber_group(subscriber, record_cleanup_interruption, cleanup_errors)
        if server is not None:
            try:
                _demo._stop_server(
                    server,
                    partition=partition,
                    private_export=private_export,
                    extra_environment=inner_environment,
                )
            except BaseException as error:
                record_cleanup_interruption(error, "render server graceful cleanup interrupted")
            try:
                emergency = _demo._kill_child_group(server)
            except BaseException as error:
                record_cleanup_interruption(error, "render server emergency cleanup interrupted")
            else:
                if emergency.deferred_error is not None:
                    record_cleanup_interruption(
                        emergency.deferred_error,
                        "render server emergency cleanup interrupted",
                    )
                if not emergency.cleaned:
                    cleanup_errors.append(f"render server process group {server.pid} survived emergency cleanup")
        try:
            log_tail.finish()
        except BaseException as error:
            record_cleanup_interruption(error, "render log cleanup interrupted")
        try:
            cleanup_errors.extend(log_tail.problems())
        except BaseException as error:
            record_cleanup_interruption(error, "render log problem collection interrupted")

    late_critical = log_tail.critical_error() or _render_critical_error(log_tail.text())
    if operation_error is None and late_critical is not None:
        operation_error = RuntimeError(f"Gazebo Sensors/Ogre2/mesh load error: {late_critical}")
    if cleanup_interruption is not None:
        if operation_error is None:
            operation_error = cleanup_interruption
        elif operation_error is not cleanup_interruption:
            _demo._add_exception_note(
                operation_error,
                f"cleanup interruption: {type(cleanup_interruption).__name__}: {cleanup_interruption}",
            )
            _demo._copy_exception_notes(cleanup_interruption, operation_error)
    if cleanup_errors:
        cleanup_message = "; ".join(cleanup_errors)
        if operation_error is None:
            operation_error = RuntimeError(cleanup_message)
        else:
            _demo._add_exception_note(operation_error, cleanup_message)
    if operation_error is not None:
        diagnostic = _render_diagnostic(
            str(operation_error),
            log_tail,
            latest_transport[0],
            latest_frame_error[0],
        )
        if not isinstance(operation_error, RuntimeError):
            _demo._add_exception_note(operation_error, diagnostic)
            raise operation_error
        wrapped = RuntimeError(diagnostic)
        _demo._copy_exception_notes(operation_error, wrapped)
        raise wrapped from operation_error
    selected_frame = _require_render_result(
        result,
        selected_frame,
        log_tail,
        latest_transport[0],
        latest_frame_error[0],
    )
    _record_render_observation(
        observations,
        model_dir=Path(model_dir),
        started=started,
        selected_frame=selected_frame,
    )
    return selected_frame.data


def _write_candidate_files(
    directory_fd: int,
    png_by_slug: Mapping[str, bytes],
    manifest_bytes: bytes,
) -> None:
    for slug, data in png_by_slug.items():
        _write_regular_file_at(directory_fd, f"{slug}.png", data)
    _write_regular_file_at(directory_fd, MANIFEST_NAME, manifest_bytes)
    os.fsync(directory_fd)


def render_candidates(
    models_root: Path,
    candidate_root: Path | None = None,
    *,
    observations: list[RenderObservation] | None = None,
) -> Path:
    """Render all three models and atomically install an exact candidate set."""
    models_root = Path(models_root)
    snapshot = _demo._validated_export_snapshot(models_root, validator=validate_render_input)
    config = capture_render_config()
    if candidate_root is None:
        parent_path = Path(tempfile.mkdtemp(prefix="onerobotics-a1-render-candidates-")).resolve()
        requested = parent_path / "candidates"
    else:
        requested = Path(candidate_root)
    if not requested.is_absolute() or not requested.name or requested.name in {".", ".."}:
        raise ValueError("candidate root must name one absolute directory")
    parent = _open_held_directory(requested.parent, "candidate root parent")
    destination_name = requested.name
    if _entry_details(parent.descriptor, destination_name) is not None:
        operation_error = ValueError(f"candidate root must not already exist: {requested}")
        _finish_resource_cleanup(
            operation_error,
            (("candidate parent descriptor close failed", lambda: os.close(parent.descriptor)),),
        )
        raise operation_error

    outer_error: BaseException | None = None
    try:
        specs = load_model_specs()
        model_hashes: dict[str, str] = {}
        for spec in specs:
            relative = PurePosixPath(spec.slug) / "model.sdf"
            data = snapshot.files.get(relative)
            if data is None:
                raise ValueError(f"validated export snapshot is missing {relative}")
            model_hashes[spec.slug] = hashlib.sha256(data).hexdigest()

        pngs: dict[str, bytes] = {}
        with tempfile.TemporaryDirectory(prefix="onerobotics-a1-render-run-") as temporary:
            private_root = Path(temporary).resolve()
            private_export = private_root / "models"
            _demo._materialize_snapshot(snapshot, private_export)
            runs = private_root / "runs"
            runs.mkdir(mode=0o700)
            for spec in specs:
                run_root = runs / spec.slug
                run_root.mkdir(mode=0o700)
                pngs[spec.slug] = capture_model_thumbnail(
                    model_dir=private_export / spec.slug,
                    private_export=private_export,
                    run_root=run_root,
                    render_settings=config.settings[spec.slug],
                    observations=observations,
                )

        config_digest = config.sha256
        manifest = build_render_manifest(
            pngs,
            render_config_sha256=config_digest,
            model_sdf_sha256=model_hashes,
        )
        _verify_held_directory(parent, "candidate root parent")
        staging = _create_child_directory(parent, f".{destination_name}.stage-")
        installed = False
        active_error: BaseException | None = None

        def record_install(outcome: bool) -> None:
            nonlocal installed
            installed = outcome

        try:
            _write_candidate_files(staging.descriptor, pngs, manifest)
            staged_snapshot = _post_write_thumbnail_check(
                staging.descriptor,
                expected_render_config_sha256=config_digest,
                expected_model_sdf_sha256=model_hashes,
            )
            if staged_snapshot is not None and (
                dict(staged_snapshot.png_by_slug) != pngs or staged_snapshot.manifest_bytes != manifest
            ):
                raise ValueError("staged candidate bytes differ from rendered bytes")
            _verify_held_directory(parent, "candidate root parent")
            if _entry_details(parent.descriptor, destination_name) is not None:
                raise ValueError("candidate root appeared during installation")
            _verify_child_directory(parent, staging, staging.name, "candidate staging directory")
            _replace_held_child_reconciled(
                parent,
                staging,
                staging.name,
                destination_name,
                "candidate install rename",
                record_install,
            )
            _verify_held_directory(parent, "candidate root parent")
            _verify_child_directory(parent, staging, destination_name, "installed candidate root")
            installed_snapshot = _post_write_thumbnail_check(
                staging.descriptor,
                expected_render_config_sha256=config_digest,
                expected_model_sdf_sha256=model_hashes,
            )
            if installed_snapshot is not None and (
                dict(installed_snapshot.png_by_slug) != pngs or installed_snapshot.manifest_bytes != manifest
            ):
                raise ValueError("candidate bytes differ after atomic installation")
            _verify_held_directory(parent, "candidate root parent")
            return requested
        except BaseException as operation_error:
            active_error = operation_error
            rollback_errors: list[str] = []
            if installed:
                try:
                    _verify_child_directory(parent, staging, destination_name, "failed candidate root")
                    _remove_held_child_at(parent, staging, destination_name, "failed candidate root")
                    installed = False
                except BaseException as error:
                    rollback_errors.append(f"unable to remove failed candidate install: {error}")
            if rollback_errors:
                _demo._add_exception_note(operation_error, "; ".join(rollback_errors))
            raise
        finally:

            def remove_uninstalled_staging() -> None:
                if not installed:
                    if _entry_details(parent.descriptor, staging.name) is not None:
                        _remove_held_child_at(parent, staging, staging.name, "candidate staging directory")

            _finish_resource_cleanup(
                active_error,
                (
                    ("candidate staging cleanup failed", remove_uninstalled_staging),
                    ("candidate staging descriptor close failed", lambda: os.close(staging.descriptor)),
                ),
            )
    except BaseException as operation_error:
        outer_error = operation_error
        raise
    finally:
        _finish_resource_cleanup(
            outer_error,
            (("candidate parent descriptor close failed", lambda: os.close(parent.descriptor)),),
        )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    render = commands.add_parser("render", help="render a candidate set without approving it")
    render.add_argument("--models", type=Path, required=True)
    render.add_argument("--candidates", type=Path)
    approve = commands.add_parser("approve", help="approve exact already-reviewed candidate bytes")
    approve.add_argument("--candidates", type=Path, required=True)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.command == "render":
            observations: list[RenderObservation] = []
            destination = render_candidates(args.models, args.candidates, observations=observations)
            for observation in observations:
                print(
                    f"RENDER_MODEL_OK: {observation.slug} "
                    f"elapsed={observation.elapsed_seconds:.3f}s "
                    f"selected_frame={observation.selected_frame_suffix} "
                    f"sha256={observation.png_sha256}"
                )
            print(f"RENDER_OK: {destination}")
            return 0
        destination = approve_candidates(args.candidates)
        print(f"APPROVAL_OK: {destination}")
        return 0
    except (OSError, RuntimeError, ValueError) as error:
        prefix = "RENDER_ERROR" if args.command == "render" else "APPROVAL_ERROR"
        print(f"{prefix}: {error}", file=sys.stderr)
        for note in _demo._exception_notes(error):
            print(note, file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
