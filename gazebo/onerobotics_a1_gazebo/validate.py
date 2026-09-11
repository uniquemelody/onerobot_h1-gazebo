"""Fail-closed, pure-data validation for exported OneRobotics A1 packages."""

from __future__ import annotations

import argparse
import gzip
import hashlib
import io
import json
import math
import re
import sys
import tarfile
import tempfile
import zlib
from dataclasses import dataclass, replace
from pathlib import Path, PurePosixPath
from xml.etree import ElementTree
from xml.etree.ElementTree import Element

from onerobotics_a1_gazebo.package import (
    CHANGES_STATEMENT,
    EXPORT_MARKER,
    PACKAGE_VERSION,
    _render_template,
)
from onerobotics_a1_gazebo.sdf import convert_urdf, serialize_sdf
from onerobotics_a1_gazebo.spec import (
    SOURCE_COMMIT,
    SOURCE_REPOSITORY,
    ModelSpec,
    load_hardware_overlay,
    load_model_specs,
    source_root,
)
from onerobotics_a1_gazebo.thumbnail import (
    ThumbnailSet,
    approved_assets_root,
    capture_thumbnail_set,
    validate_thumbnail_bytes,
)
from onerobotics_a1_gazebo.validation_snapshot import (
    DirectorySnapshot,
    SnapshotError,
    capture_directory,
)

_SDF_VERSION = "1.11"
_REQUIRED_FILES = (
    "model.sdf",
    "model.config",
    "metadata.pbtxt",
    "README.md",
    "NOTICE",
    "LICENSE",
    "SOURCE_MANIFEST.json",
)
_INERTIA_COMPONENTS = ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
_NUMERIC_TEXT_COUNTS = {
    "pose": 6,
    "mass": 1,
    "ixx": 1,
    "ixy": 1,
    "ixz": 1,
    "iyy": 1,
    "iyz": 1,
    "izz": 1,
    "xyz": 3,
    "lower": 1,
    "upper": 1,
    "effort": 1,
    "velocity": 1,
    "scale": 3,
    "size": 3,
    "radius": 1,
    "length": 1,
    "ambient": 4,
    "diffuse": 4,
}
_MAX_DOCUMENT_DEPTH = 256
_MAX_ORPHAN_ARCHIVE_SIZE = 64 * 1024 * 1024
_MAX_ORPHAN_ARCHIVE_MEMBERS = 512
_MAX_EXPECTED_ARCHIVE_SIZE = 160 * 1024 * 1024


@dataclass(frozen=True)
class ValidationIssue:
    """One independently actionable package validation failure."""

    model: str
    message: str


class ValidationFailure(ValueError):
    """Raised by :func:`validate_all` when any model package is invalid."""

    def __init__(self, issues: tuple[ValidationIssue, ...]) -> None:
        self.issues = issues
        super().__init__(f"package validation failed with {len(issues)} issue(s)")


@dataclass(frozen=True)
class _ExpectedData:
    model: Element
    sources: dict[str, str]
    mesh_digests: dict[str, str]
    model_sdf_sha256: str


class _PackageReader:
    """Issue-reporting reads over one already captured immutable snapshot."""

    def __init__(self, snapshot: DirectorySnapshot, model: str, issues: list[ValidationIssue]) -> None:
        self.snapshot = snapshot
        self.model = model
        self.issues = issues
        self.known_missing: set[str] = set()
        self.problem_paths = {problem.path for problem in snapshot.problems}

    def issue(self, message: str) -> None:
        self.issues.append(ValidationIssue(self.model, message))

    @staticmethod
    def _path(relative: str) -> PurePosixPath | None:
        portable = PurePosixPath(relative)
        if (
            not relative
            or "\\" in relative
            or portable.is_absolute()
            or any(part in {"", ".", ".."} for part in portable.parts)
        ):
            return None
        return portable

    def read(self, relative: str) -> bytes | None:
        path = self._path(relative)
        if path is None:
            self.issue(f"non-portable package path: {relative}")
            return None
        data = self.snapshot.files.get(path)
        if data is not None:
            return data
        if path in self.problem_paths:
            return None
        if path in self.snapshot.directories:
            self.issue(f"required file is not regular: {relative}")
        elif relative not in self.known_missing and path.parts[0] not in self.known_missing:
            self.issue(f"missing required file: {relative}")
        return None

    def inventory(self, relative: str) -> tuple[str, ...] | None:
        path = self._path(relative)
        if path is None:
            self.issue(f"non-portable package path: {relative}")
            return None
        if path not in self.snapshot.directories:
            if relative not in self.known_missing:
                self.issue(f"missing directory: {relative}")
            return None
        depth = len(path.parts) + 1
        file_names = {item.name for item in self.snapshot.files if len(item.parts) == depth and item.parent == path}
        directory_names = {
            item.name for item in self.snapshot.directories if len(item.parts) == depth and item.parent == path
        }
        problem_names = {
            item.path.name
            for item in self.snapshot.problems
            if len(item.path.parts) == depth and item.path.parent == path
        }
        for name in sorted(directory_names):
            self.issue(f"package entry is not a regular file: {relative}/{name}")
        return tuple(sorted(file_names | directory_names | problem_names))

    def report_snapshot_problems(self) -> None:
        for problem in self.snapshot.problems:
            if len(problem.path.parts) == 1 and problem.message.startswith(
                "snapshot entry must be a regular file or directory"
            ):
                self.issue(f"required file is not regular: {problem.path}")
            else:
                self.issue(f"{problem.message}: {problem.path}")

    def validate_root_inventory(
        self,
        *,
        expected_files: set[str],
        expected_directories: set[str],
        label: str,
    ) -> None:
        actual_files = {path.name for path in self.snapshot.files if len(path.parts) == 1}
        actual_directories = {path.name for path in self.snapshot.directories if len(path.parts) == 1}
        problem_names = {problem.path.name for problem in self.snapshot.problems if len(problem.path.parts) == 1}
        names = actual_files | actual_directories | problem_names
        expected = expected_files | expected_directories
        for name in sorted(names - expected):
            self.issue(f"unexpected {label} root entry: {name}")
        for name in sorted(actual_files & expected_directories):
            self.issue(f"{label} root entry must be a regular directory: {name}")
        for name in sorted(actual_directories & expected_files):
            self.issue(f"{label} root entry must be a regular file: {name}")
        for name in sorted(expected - names):
            self.known_missing.add(name)
            if label == "package" and name in expected_files:
                self.issue(f"missing required file: {name}")
            elif label == "package":
                self.issue(f"missing directory: {name}")
            else:
                self.issue(f"missing {label} root entry: {name}")


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _decode(reader: _PackageReader, relative: str) -> str | None:
    data = reader.read(relative)
    if data is None:
        return None
    try:
        return data.decode("utf-8")
    except UnicodeDecodeError:
        reader.issue(f"required text file is not UTF-8: {relative}")
        return None


def _parse_xml(reader: _PackageReader, relative: str) -> Element | None:
    data = reader.read(relative)
    if data is None:
        return None
    try:
        return ElementTree.fromstring(data)
    except RecursionError:
        reader.issue(f"{relative} exceeds nesting limit")
        return None
    except (ElementTree.ParseError, LookupError, ValueError):
        reader.issue(f"malformed {relative}")
        return None


def _element_depth_exceeds(root: Element, maximum: int = _MAX_DOCUMENT_DEPTH) -> bool:
    pending = [(root, 1)]
    while pending:
        element, depth = pending.pop()
        if depth > maximum:
            return True
        pending.extend((child, depth + 1) for child in element)
    return False


def _json_depth_exceeds(text: str, maximum: int = _MAX_DOCUMENT_DEPTH) -> bool:
    depth = 0
    quoted = False
    escaped = False
    for character in text:
        if quoted:
            if escaped:
                escaped = False
            elif character == "\\":
                escaped = True
            elif character == '"':
                quoted = False
            continue
        if character == '"':
            quoted = True
        elif character in "[{":
            depth += 1
            if depth > maximum:
                return True
        elif character in "]}":
            depth -= 1
    return False


def _finite_values(
    element: Element | None,
    count: int,
    field: str,
    reader: _PackageReader,
) -> tuple[float, ...] | None:
    text = element.text if element is not None else None
    if text is None:
        reader.issue(f"{field} must be a finite number")
        return None
    parts = text.split()
    if len(parts) != count:
        reader.issue(f"{field} must be a finite number")
        return None
    try:
        values = tuple(float(part) for part in parts)
    except ValueError:
        reader.issue(f"{field} must be a finite number")
        return None
    if not all(math.isfinite(value) for value in values):
        reader.issue(f"{field} must be a finite number")
        return None
    return values


def _signature(element: Element) -> tuple[object, ...]:
    return (
        element.tag,
        tuple(sorted(element.attrib.items())),
        " ".join((element.text or "").split()),
        tuple(_signature(child) for child in element),
    )


def _joint_topology(joints: list[Element]) -> dict[str, tuple[str | None, str | None, str | None]]:
    topology: dict[str, tuple[str | None, str | None, str | None]] = {}
    for joint in joints:
        name = joint.attrib.get("name")
        if name and name not in topology:
            topology[name] = (
                joint.attrib.get("type"),
                joint.findtext("parent"),
                joint.findtext("child"),
            )
    return topology


def _validate_inertials(model: Element, reader: _PackageReader) -> None:
    for link in model.findall("link"):
        name = link.attrib.get("name", "<unnamed>")
        inertial = link.find("inertial")
        mass_values = _finite_values(
            inertial.find("mass") if inertial is not None else None,
            1,
            "mass",
            reader,
        )
        if mass_values is not None and mass_values[0] <= 0:
            reader.issue(f"link {name} mass must be positive")
        inertia = inertial.find("inertia") if inertial is not None else None
        values: dict[str, float] = {}
        for component in _INERTIA_COMPONENTS:
            parsed = _finite_values(
                inertia.find(component) if inertia is not None else None,
                1,
                f"inertia {component}",
                reader,
            )
            if parsed is not None:
                values[component] = parsed[0]
        if len(values) != len(_INERTIA_COMPONENTS):
            continue
        scale = max(abs(value) for value in values.values())
        if scale == 0:
            reader.issue(f"link {name} inertia is not positive definite")
            continue
        ixx, ixy, ixz, iyy, iyz, izz = (values[component] / scale for component in _INERTIA_COMPONENTS)
        minor_1 = ixx
        minor_2 = ixx * iyy - ixy * ixy
        determinant = ixx * (iyy * izz - iyz * iyz) - ixy * (ixy * izz - iyz * ixz) + ixz * (ixy * iyz - iyy * ixz)
        minors = (minor_1, minor_2, determinant)
        if not all(math.isfinite(value) and value > 0 for value in minors):
            reader.issue(f"link {name} inertia is not positive definite")


def _validate_numbers(model: Element, reader: _PackageReader) -> None:
    for element in model.iter():
        count = _NUMERIC_TEXT_COUNTS.get(element.tag)
        if count is not None:
            _finite_values(element, count, element.tag, reader)


def _validate_position_intervals(model: Element, reader: _PackageReader) -> None:
    for joint in model.findall("joint"):
        limit = joint.find("axis/limit")
        if limit is None:
            continue
        name = joint.attrib.get("name", "<unnamed>")
        lower = _finite_values(limit.find("lower"), 1, f"joint {name} lower", reader)
        upper = _finite_values(limit.find("upper"), 1, f"joint {name} upper", reader)
        if lower is None or upper is None:
            continue
        if lower[0] >= upper[0]:
            reader.issue(f"joint {name} position lower limit must be less than upper limit")
        if not lower[0] <= 0 <= upper[0]:
            reader.issue(f"joint {name} position limit must contain zero")


def _validate_limits(model: Element, expected: Element, spec: ModelSpec, reader: _PackageReader) -> None:
    for name in spec.actuated_joints:
        actual_joint = model.find(f"joint[@name='{name}']")
        expected_joint = expected.find(f"joint[@name='{name}']")
        if actual_joint is None or expected_joint is None:
            continue
        actual_limit = actual_joint.find("axis/limit")
        expected_limit = expected_joint.find("axis/limit")
        if actual_limit is None or expected_limit is None:
            reader.issue(f"joint {name} position limit mismatch")
            reader.issue(f"joint {name} effort limit mismatch")
            reader.issue(f"joint {name} velocity limit mismatch")
            continue
        parsed: dict[str, float] = {}
        for field in ("lower", "upper", "effort", "velocity"):
            values = _finite_values(actual_limit.find(field), 1, f"joint {name} {field}", reader)
            if values is not None:
                parsed[field] = values[0]
        if "lower" in parsed and "upper" in parsed:
            if parsed["lower"] >= parsed["upper"]:
                reader.issue(f"joint {name} position lower limit must be less than upper limit")
            if not parsed["lower"] <= 0 <= parsed["upper"]:
                reader.issue(f"joint {name} position limit must contain zero")
        for field, label in (
            ("lower", "position limit mismatch"),
            ("upper", "position limit mismatch"),
            ("effort", "effort limit mismatch"),
            ("velocity", "velocity limit mismatch"),
        ):
            actual_text = actual_limit.findtext(field)
            expected_text = expected_limit.findtext(field)
            if actual_text != expected_text:
                reader.issue(f"joint {name} {label}")


def _validate_physical_fields(model: Element, expected: Element, reader: _PackageReader) -> None:
    for expected_link in expected.findall("link"):
        name = expected_link.attrib["name"]
        actual_link = model.find(f"link[@name='{name}']")
        if actual_link is None:
            continue
        actual_mass = actual_link.findtext("inertial/mass")
        expected_mass = expected_link.findtext("inertial/mass")
        if actual_mass != expected_mass:
            reader.issue(f"link {name} mass mismatch")
        for component in _INERTIA_COMPONENTS:
            if actual_link.findtext(f"inertial/inertia/{component}") != expected_link.findtext(
                f"inertial/inertia/{component}"
            ):
                reader.issue(f"link {name} inertia {component} mismatch")


def _validate_sdf(
    root: Element,
    spec: ModelSpec,
    expected: Element | None,
    reader: _PackageReader,
) -> set[str] | None:
    _validate_numbers(root, reader)
    if root.tag != "sdf" or root.attrib != {"version": _SDF_VERSION}:
        reader.issue('SDF root mismatch: expected exactly <sdf version="1.11">')
    if root.findall(".//static"):
        reader.issue("published model must not contain <static>")
    if root.findall(".//plugin"):
        reader.issue("published model must not contain plugins")
    if root.findall(".//sensor"):
        reader.issue("published model must not contain sensors")
    serialized = ElementTree.tostring(root, encoding="unicode").lower()
    if re.search(r"gazebo[_-]ros|lib[^<\"']*ros|\bros2?\b|/ros(?:/|\b)", serialized):
        reader.issue("published model must not contain ROS dependencies")

    mesh_uri_elements = set(root.findall(".//geometry/mesh/uri"))
    for uri_element in root.findall(".//uri"):
        uri = (uri_element.text or "").strip()
        if uri_element not in mesh_uri_elements and not _portable_uri(uri):
            reader.issue(f"non-portable URI: {uri or '<empty>'}")

    models = root.findall("model")
    if len(models) != 1:
        reader.issue("SDF must contain exactly one model")
    for local_model in root.findall(".//model"):
        _validate_inertials(local_model, reader)
        _validate_position_intervals(local_model, reader)
    expected_models = [model for model in models if model.attrib.get("name") == spec.slug]
    if len(expected_models) == 1:
        model = expected_models[0]
    elif len(models) == 1:
        model = models[0]
    else:
        return {(uri.text or "").strip() for uri in mesh_uri_elements}
    if len(root) != 1 or root[0] is not model:
        reader.issue("SDF root must contain only the published model")
    if model.attrib.get("name") != spec.slug:
        reader.issue("model name mismatch")
    if model.attrib.get("canonical_link") != "base_link":
        reader.issue("canonical link mismatch")
    links = model.findall("link")
    actual_link_names = [link.attrib.get("name") for link in links]
    if actual_link_names != list(spec.physical_links):
        reader.issue("link names mismatch")
    joints = model.findall("joint")
    actual_joint_names = [joint.attrib.get("name") for joint in joints]
    expected_joint_names = ["world_to_base", *spec.fixed_joints, *spec.actuated_joints]
    if set(actual_joint_names) != set(expected_joint_names) or len(actual_joint_names) != len(expected_joint_names):
        reader.issue("joint names mismatch")

    if expected is not None:
        expected_topology = _joint_topology(expected.findall("joint"))
        actual_topology = _joint_topology(joints)
        expected_fixed = {name: topology for name, topology in expected_topology.items() if topology[0] == "fixed"}
        actual_fixed = {name: topology for name, topology in actual_topology.items() if topology[0] == "fixed"}
        if actual_fixed != expected_fixed:
            reader.issue("fixed-joint topology mismatch")
        expected_actuated = {name: expected_topology.get(name) for name in spec.actuated_joints}
        actual_actuated = {name: actual_topology.get(name) for name in spec.actuated_joints}
        if actual_actuated != expected_actuated:
            reader.issue("actuated-joint topology mismatch")
        _validate_limits(model, expected, spec, reader)
        _validate_physical_fields(model, expected, reader)
        if _signature(model) != _signature(expected):
            reader.issue("published model does not exactly preserve locked source data")

    return {(uri.text or "").strip() for uri in mesh_uri_elements}


def _source_lock(reader: _PackageReader) -> dict[str, str] | None:
    lock_path = Path(__file__).resolve().parents[1] / "generated-manifest.json"
    try:
        document = json.loads(lock_path.read_text(encoding="utf-8"))
    except (OSError, RecursionError, ValueError):
        reader.issue("unable to read immutable source lock")
        return None
    if not isinstance(document, dict):
        reader.issue("immutable source lock is malformed")
        return None
    expected_metadata = {
        "schema_version": 1,
        "source_repository": SOURCE_REPOSITORY,
        "source_commit": SOURCE_COMMIT,
    }
    metadata_valid = True
    for field, expected in expected_metadata.items():
        if document.get(field) != expected:
            reader.issue(f"immutable source lock {field} mismatch")
            metadata_valid = False
    files = document.get("files")
    if not isinstance(files, dict) or not all(
        isinstance(path, str) and isinstance(digest, str) for path, digest in files.items()
    ):
        reader.issue("immutable source lock is malformed")
        return None
    return files if metadata_valid else None


def _validate_source_snapshot(
    snapshot: DirectorySnapshot,
    lock: dict[str, str],
    reader: _PackageReader,
) -> bool:
    actual_paths = {path.as_posix() for path in snapshot.files}
    expected_paths = set(lock)
    valid = True
    for problem in snapshot.problems:
        reader.issue(f"immutable source snapshot problem: {problem.message}: {problem.path}")
        valid = False
    if actual_paths != expected_paths:
        reader.issue("immutable source lock mismatch: file inventory")
        valid = False
    expected_directories = {
        PurePosixPath(*path.parts[:depth])
        for relative in expected_paths
        for path in (PurePosixPath(relative),)
        for depth in range(1, len(path.parts))
    }
    if snapshot.directories != expected_directories:
        reader.issue("immutable source lock mismatch: directory inventory")
        valid = False
    for path in sorted(actual_paths & expected_paths):
        if _sha256(snapshot.files[PurePosixPath(path)]) != lock[path]:
            reader.issue(f"immutable source lock mismatch: digest for {path}")
            valid = False
    return valid


def _materialize_snapshot(snapshot: DirectorySnapshot, destination: Path) -> None:
    for directory in sorted(snapshot.directories, key=lambda path: (len(path.parts), path.as_posix())):
        (destination / Path(*directory.parts)).mkdir()
    for relative, data in snapshot.files.items():
        (destination / Path(*relative.parts)).write_bytes(data)


def _expected_sources(
    spec: ModelSpec,
    lock: dict[str, str],
    public_root: Path,
    reader: _PackageReader,
) -> tuple[dict[str, str], dict[str, str]] | None:
    try:
        urdf_relative = spec.urdf.relative_to(public_root).as_posix()
        source_urdf = ElementTree.parse(spec.urdf).getroot()
    except (OSError, ValueError, ElementTree.ParseError):
        reader.issue("unable to read locked source URDF")
        return None
    source_paths = {urdf_relative, "model_parameters.yaml"}
    mesh_digests: dict[str, str] = {}
    for mesh in source_urdf.findall(".//mesh"):
        filename = mesh.attrib.get("filename", "")
        portable = PurePosixPath(filename.replace("\\", "/"))
        if portable.is_absolute() or ".." in portable.parts or not portable.name:
            reader.issue("locked source URDF contains a non-portable mesh path")
            return None
        try:
            relative = (spec.urdf.parent / Path(*portable.parts)).relative_to(public_root)
        except (OSError, ValueError):
            reader.issue("locked source mesh escapes the source root")
            return None
        source_path = relative.as_posix()
        source_paths.add(source_path)
        digest = lock.get(source_path)
        if digest is None:
            reader.issue(f"locked source mesh is absent from source lock: {source_path}")
            return None
        previous = mesh_digests.setdefault(portable.name, digest)
        if previous != digest:
            reader.issue(f"locked source mesh basename collision: {portable.name}")
            return None
    try:
        sources = {path: lock[path] for path in source_paths}
    except KeyError as error:
        reader.issue(f"locked package source is absent from source lock: {error.args[0]}")
        return None
    return sources, mesh_digests


def _build_expected_data(
    specs: tuple[ModelSpec, ...],
    reader: _PackageReader,
) -> dict[str, _ExpectedData]:
    try:
        source_base = source_root().resolve(strict=True)
        source_snapshot = capture_directory(source_base)
    except (OSError, SnapshotError) as error:
        reader.issue(f"immutable source snapshot failed: {error}")
        return {}
    lock = _source_lock(reader)
    if lock is None or not _validate_source_snapshot(source_snapshot, lock, reader):
        return {}

    expected: dict[str, _ExpectedData] = {}
    try:
        with tempfile.TemporaryDirectory(prefix="onerobotics-a1-source-snapshot-") as temporary:
            snapshot_root = Path(temporary)
            _materialize_snapshot(source_snapshot, snapshot_root)
            overlay = load_hardware_overlay(snapshot_root / "model_parameters.yaml")
            for spec in specs:
                relative_urdf = spec.urdf.relative_to(source_base)
                snapshot_spec = replace(spec, urdf=snapshot_root / relative_urdf)
                converted_tree = convert_urdf(snapshot_spec, overlay)
                converted = converted_tree.getroot().find("model")
                if converted is None:
                    reader.issue(f"unable to derive locked source model: {spec.slug}")
                    continue
                provenance = _expected_sources(snapshot_spec, lock, snapshot_root, reader)
                if provenance is None:
                    continue
                sources, mesh_digests = provenance
                expected[spec.key] = _ExpectedData(
                    converted,
                    sources,
                    mesh_digests,
                    _sha256(serialize_sdf(converted_tree)),
                )
    except (OSError, ValueError, ElementTree.ParseError) as error:
        reader.issue(f"unable to derive locked source model: {error}")
    return expected


def _portable_uri(uri: str) -> bool:
    path = PurePosixPath(uri)
    return bool(
        uri
        and "\\" not in uri
        and ":" not in uri.split("/", 1)[0]
        and not path.is_absolute()
        and all(part not in {"", ".", ".."} for part in path.parts)
        and len(path.parts) == 2
        and path.parts[0] == "meshes"
    )


def _validate_meshes(
    uris: set[str] | None,
    expected_meshes: dict[str, str] | None,
    reader: _PackageReader,
) -> None:
    valid_uris: set[str] = set()
    if uris is not None:
        for uri in sorted(uris):
            if not _portable_uri(uri):
                reader.issue(f"non-portable URI: {uri or '<empty>'}")
            else:
                valid_uris.add(uri)
    inventory = reader.inventory("meshes")
    if expected_meshes is None:
        return
    expected_names = set(expected_meshes)
    if uris is not None:
        referenced_names = {PurePosixPath(uri).name for uri in valid_uris}
        if referenced_names != expected_names:
            for missing in sorted(expected_names - referenced_names):
                reader.issue(f"missing mesh reference: meshes/{missing}")
            for unexpected in sorted(referenced_names - expected_names):
                reader.issue(f"unexpected mesh reference: meshes/{unexpected}")
    actual_names = set(inventory) if inventory is not None else None
    if actual_names is not None:
        for missing in sorted(expected_names - actual_names):
            reader.issue(f"missing mesh: meshes/{missing}")
        for unexpected in sorted(actual_names - expected_names):
            reader.issue(f"unexpected mesh: meshes/{unexpected}")
    for name, expected_digest in sorted(expected_meshes.items()):
        if actual_names is not None and name not in actual_names:
            continue
        data = reader.read(f"meshes/{name}")
        if data is not None and _sha256(data) != expected_digest:
            reader.issue(f"package mesh digest mismatch: meshes/{name}")


def _expected_model_hashes(
    specs: tuple[ModelSpec, ...],
    expected: dict[str, _ExpectedData],
) -> dict[str, str] | None:
    if any(spec.key not in expected for spec in specs):
        return None
    return {spec.slug: expected[spec.key].model_sdf_sha256 for spec in specs}


def _capture_approved_thumbnails(
    reader: _PackageReader,
    expected_model_sdf_sha256: dict[str, str] | None,
) -> ThumbnailSet | None:
    if expected_model_sdf_sha256 is None:
        return None
    try:
        return capture_thumbnail_set(
            approved_assets_root(),
            expected_model_sdf_sha256=expected_model_sdf_sha256,
        )
    except (OSError, ValueError) as error:
        reader.issue(f"approved thumbnail snapshot invalid: {error}")
        return None


def _validate_thumbnail(
    spec: ModelSpec,
    expected: _ExpectedData | None,
    approved: ThumbnailSet | None,
    reader: _PackageReader,
) -> None:
    inventory = reader.inventory("thumbnails")
    if inventory is None:
        return
    actual = set(inventory)
    expected_inventory = {"0.png"}
    for missing in sorted(expected_inventory - actual):
        reader.issue(f"missing thumbnail: thumbnails/{missing}")
    for unexpected in sorted(actual - expected_inventory):
        reader.issue(f"unexpected thumbnail: thumbnails/{unexpected}")
    if "0.png" not in actual:
        return

    data = reader.read("thumbnails/0.png")
    if data is None:
        return
    try:
        validate_thumbnail_bytes(data)
    except ValueError as error:
        reader.issue(f"thumbnail PNG invalid: {error}")

    if approved is None:
        return
    approved_data = approved.png_by_slug[spec.slug]
    if data != approved_data:
        reader.issue("approved thumbnail mismatch: thumbnails/0.png")

    record = approved.records_by_slug[spec.slug]
    model_sdf = reader.read("model.sdf")
    if model_sdf is not None and _sha256(model_sdf) != record.model_sdf_sha256:
        reader.issue("thumbnail manifest model.sdf hash mismatch")
    if expected is not None and record.model_sdf_sha256 != expected.model_sdf_sha256:
        reader.issue("approved thumbnail manifest locked model hash mismatch")


def _validate_manifest(
    spec: ModelSpec,
    expected_sources: dict[str, str] | None,
    reader: _PackageReader,
) -> None:
    text = _decode(reader, "SOURCE_MANIFEST.json")
    if text is None:
        return
    if _json_depth_exceeds(text):
        reader.issue("SOURCE_MANIFEST.json exceeds nesting limit")
        return
    try:
        manifest = json.loads(text)
    except RecursionError:
        reader.issue("SOURCE_MANIFEST.json exceeds nesting limit")
        return
    except ValueError:
        reader.issue("malformed SOURCE_MANIFEST.json")
        return
    if not isinstance(manifest, dict):
        reader.issue("source manifest must be a JSON object")
        return
    expected_scalars: dict[str, object] = {
        "schema_version": 1,
        "source_url": SOURCE_REPOSITORY,
        "source_commit": SOURCE_COMMIT,
        "model_key": spec.key,
        "changes": CHANGES_STATEMENT,
    }
    for field, expected in expected_scalars.items():
        if manifest.get(field) != expected:
            reader.issue(f"source manifest {field} mismatch")
    sources = manifest.get("sources")
    if not isinstance(sources, list):
        reader.issue("source manifest sources must be a list")
        return
    actual: dict[str, str] = {}
    for index, entry in enumerate(sources):
        if not isinstance(entry, dict) or set(entry) != {"path", "sha256"}:
            reader.issue(f"source manifest entry {index} is malformed")
            continue
        path = entry.get("path")
        digest = entry.get("sha256")
        if not isinstance(path, str) or not isinstance(digest, str):
            reader.issue(f"source manifest entry {index} is malformed")
            continue
        portable = PurePosixPath(path)
        if "\\" in path or portable.is_absolute() or any(part in {"", ".", ".."} for part in portable.parts):
            reader.issue(f"non-portable source manifest path: {path}")
            continue
        if path in actual:
            reader.issue(f"duplicate source manifest path: {path}")
            continue
        actual[path] = digest
    if expected_sources is None:
        return
    if set(actual) != set(expected_sources):
        reader.issue("package source inventory mismatch")
    for path in sorted(set(actual) & set(expected_sources)):
        if actual[path] != expected_sources[path]:
            reader.issue(f"package source digest mismatch: {path}")


def _validate_legal(spec: ModelSpec, reader: _PackageReader) -> None:
    repository_root = Path(__file__).resolve().parents[2]
    try:
        expected_license = (repository_root / "LICENSES/CC-BY-4.0.txt").read_bytes()
    except OSError:
        reader.issue("unable to read trusted CC BY 4.0 license text")
    else:
        license_data = reader.read("LICENSE")
        if license_data is not None and license_data != expected_license:
            reader.issue("exact CC BY 4.0 license text missing")

    metadata = _decode(reader, "metadata.pbtxt")
    expected_metadata = _render_template("metadata.pbtxt", spec)
    if metadata is not None and metadata != expected_metadata:
        reader.issue("CC BY 4.0 metadata missing or not exact")

    notice = _decode(reader, "NOTICE")
    expected_notice = _render_template("NOTICE", spec)
    if notice is not None and notice != expected_notice:
        reader.issue("NOTICE source URL, commit, changes, or attribution mismatch")

    model_config = _decode(reader, "model.config")
    if model_config is not None and model_config != _render_template("model.config.xml", spec):
        reader.issue("model.config metadata mismatch")

    readme = _decode(reader, "README.md")
    if readme is not None and readme != _render_template("README.md", spec):
        reader.issue("README provenance metadata mismatch")


def _expected_archive_bytes(package: DirectorySnapshot, spec: ModelSpec) -> bytes:
    buffer = io.BytesIO()
    archive_root = PurePosixPath(f"{spec.slug}-{PACKAGE_VERSION}")
    with gzip.GzipFile(filename="", mode="wb", fileobj=buffer, mtime=0) as zipped:
        with tarfile.open(fileobj=zipped, mode="w", format=tarfile.PAX_FORMAT) as archive:
            entries = [(path, None) for path in package.directories]
            entries.extend(package.files.items())
            for relative, data in sorted(entries, key=lambda entry: entry[0].as_posix()):
                info = tarfile.TarInfo((archive_root / relative).as_posix())
                info.uid = 0
                info.gid = 0
                info.uname = ""
                info.gname = ""
                info.mtime = 0
                if data is None:
                    info.type = tarfile.DIRTYPE
                    info.mode = 0o755
                    archive.addfile(info)
                else:
                    info.type = tarfile.REGTYPE
                    info.mode = 0o644
                    info.size = len(data)
                    archive.addfile(info, io.BytesIO(data))
    return buffer.getvalue()


class _ArchiveExpansionError(ValueError):
    pass


def _decompress_gzip_bounded(data: bytes, maximum: int) -> bytes:
    decompressor = zlib.decompressobj(16 + zlib.MAX_WBITS)
    output = decompressor.decompress(data, maximum + 1)
    if len(output) > maximum or decompressor.unconsumed_tail:
        raise _ArchiveExpansionError
    output += decompressor.flush(maximum + 1 - len(output))
    if len(output) > maximum:
        raise _ArchiveExpansionError
    if not decompressor.eof or decompressor.unused_data:
        raise zlib.error("incomplete or concatenated gzip stream")
    return output


def _validate_archive(
    data: bytes,
    spec: ModelSpec,
    package: DirectorySnapshot | None,
    reader: _PackageReader,
) -> None:
    relative = f"archives/{spec.slug}-{PACKAGE_VERSION}.tar.gz"
    if len(data) < 10 or data[:4] != b"\x1f\x8b\x08\x00" or data[4:8] != b"\x00\x00\x00\x00" or data[8:] == b"":
        reader.issue(f"malformed archive: {relative}")
        return
    if data[8] != 2 or data[9] != 255:
        reader.issue(f"archive gzip metadata mismatch: {relative}")
    expected_root = f"{spec.slug}-{PACKAGE_VERSION}"
    try:
        expected_archive = _expected_archive_bytes(package, spec) if package is not None else None
        maximum = (
            len(_decompress_gzip_bounded(expected_archive, _MAX_EXPECTED_ARCHIVE_SIZE))
            if expected_archive is not None
            else _MAX_ORPHAN_ARCHIVE_SIZE
        )
    except (_ArchiveExpansionError, MemoryError, zlib.error):
        reader.issue(f"archive validation exceeded resource limit: {relative}")
        return
    expected_directories = (
        {(PurePosixPath(expected_root) / path).as_posix() for path in package.directories}
        if package is not None
        else set()
    )
    expected_files = (
        {(PurePosixPath(expected_root) / path).as_posix(): contents for path, contents in package.files.items()}
        if package is not None
        else {}
    )
    try:
        raw_tar = _decompress_gzip_bounded(data, maximum)
        with tarfile.open(fileobj=io.BytesIO(raw_tar), mode="r:") as archive:
            maximum_members = (
                len(expected_directories) + len(expected_files) if package is not None else _MAX_ORPHAN_ARCHIVE_MEMBERS
            )
            members: list[tarfile.TarInfo] = []
            extracted: dict[str, bytes] = {}
            for member in archive:
                if len(members) >= maximum_members:
                    reader.issue(f"archive exceeds member limit: {relative}")
                    return
                members.append(member)
                expected_contents = expected_files.get(member.name)
                if member.isfile() and expected_contents is not None:
                    if member.size != len(expected_contents):
                        reader.issue(f"archive member size mismatch: {member.name}")
                        continue
                    stream = archive.extractfile(member)
                    if stream is None:
                        reader.issue(f"malformed archive member: {member.name}")
                    else:
                        extracted[member.name] = stream.read(len(expected_contents) + 1)
    except _ArchiveExpansionError:
        reader.issue(f"archive expands beyond expected size: {relative}")
        return
    except (
        OSError,
        EOFError,
        MemoryError,
        OverflowError,
        RecursionError,
        ValueError,
        tarfile.TarError,
        zlib.error,
    ) as error:
        reader.issue(f"malformed archive: {relative}: {error}")
        return

    safe_members = True
    for member in members:
        path = PurePosixPath(member.name)
        if path.is_absolute() or ".." in path.parts or not path.parts:
            reader.issue(f"unsafe archive member path: {member.name}")
            safe_members = False
        elif path.parts[0] != expected_root:
            reader.issue(f"archive member root mismatch: {member.name}")
            safe_members = False
        if not (member.isfile() or member.isdir()):
            reader.issue(f"archive member is not regular: {member.name}")
        expected_mode = 0o644 if member.isfile() else 0o755
        if (
            member.uid != 0
            or member.gid != 0
            or member.uname != ""
            or member.gname != ""
            or member.mtime != 0
            or member.mode != expected_mode
            or member.pax_headers
        ):
            reader.issue(f"archive member metadata mismatch: {member.name}")

    names = [member.name for member in members]
    if names != sorted(names) or len(names) != len(set(names)):
        reader.issue(f"archive member ordering mismatch: {relative}")
    actual_directories = {member.name for member in members if member.isdir()}
    actual_files = {member.name for member in members if member.isfile()}
    if (
        package is not None
        and safe_members
        and (actual_directories != expected_directories or actual_files != set(expected_files))
    ):
        reader.issue(f"archive member inventory mismatch: {relative}")
    for name in sorted(actual_files & set(expected_files)):
        if extracted.get(name) != expected_files[name]:
            reader.issue(f"archive member bytes mismatch: {name}")
    if expected_archive is not None and data != expected_archive:
        reader.issue(f"archive bytes are not deterministic or package-exact: {relative}")


def _validate_archives(
    export: DirectorySnapshot,
    specs: tuple[ModelSpec, ...],
    packages: dict[str, DirectorySnapshot],
    reader: _PackageReader,
) -> None:
    try:
        archives = export.subtree("archives")
    except SnapshotError:
        return
    expected_names = {f"{spec.slug}-{PACKAGE_VERSION}.tar.gz" for spec in specs}
    actual_files = {path.name for path in archives.files if len(path.parts) == 1}
    actual_directories = {path.name for path in archives.directories if len(path.parts) == 1}
    problem_names = {problem.path.name for problem in archives.problems if len(problem.path.parts) == 1}
    for name in sorted(expected_names - (actual_files | problem_names)):
        reader.issue(f"missing archive: archives/{name}")
    for name in sorted((actual_files | actual_directories | problem_names) - expected_names):
        reader.issue(f"unexpected archive: archives/{name}")
    for name in sorted(actual_directories & expected_names):
        reader.issue(f"archive must be a regular file: archives/{name}")
    for spec in specs:
        name = f"{spec.slug}-{PACKAGE_VERSION}.tar.gz"
        data = archives.files.get(PurePosixPath(name))
        package = packages.get(spec.key)
        if data is not None:
            _validate_archive(data, spec, package, reader)


def _deduplicate(issues: list[ValidationIssue] | tuple[ValidationIssue, ...]) -> tuple[ValidationIssue, ...]:
    return tuple(dict.fromkeys(issues))


def _validate_package_snapshot(
    snapshot: DirectorySnapshot,
    spec: ModelSpec,
    expected: _ExpectedData | None,
    approved: ThumbnailSet | None,
    issues: list[ValidationIssue],
) -> None:
    reader = _PackageReader(snapshot, spec.slug, issues)
    reader.report_snapshot_problems()
    reader.validate_root_inventory(
        expected_files=set(_REQUIRED_FILES),
        expected_directories={"meshes", "thumbnails"},
        label="package",
    )
    for relative in _REQUIRED_FILES:
        reader.read(relative)
    sdf_root = _parse_xml(reader, "model.sdf")
    expected_model = expected.model if expected is not None else None
    if sdf_root is not None and _element_depth_exceeds(sdf_root):
        reader.issue("model.sdf exceeds nesting limit")
        uris = None
    elif sdf_root is not None:
        try:
            uris = _validate_sdf(sdf_root, spec, expected_model, reader)
        except RecursionError:
            reader.issue("model.sdf exceeds nesting limit")
            uris = None
    else:
        uris = None
    _validate_meshes(uris, expected.mesh_digests if expected is not None else None, reader)
    _validate_thumbnail(spec, expected, approved, reader)
    _validate_manifest(spec, expected.sources if expected is not None else None, reader)
    _validate_legal(spec, reader)


def validate_package(path: Path, spec: ModelSpec) -> tuple[ValidationIssue, ...]:
    """Return every safely discoverable issue in one exported package."""
    issues: list[ValidationIssue] = []
    try:
        package_snapshot = capture_directory(Path(path))
    except SnapshotError as error:
        message = str(error)
        if "root must not be a symlink" in message:
            message = "package path must not be a symlink"
        elif message.startswith("snapshot entry must be a regular file or directory: "):
            relative = message.rsplit(": ", 1)[1]
            message = f"required file is not regular: {relative}"
        issues.append(ValidationIssue(spec.slug, f"package snapshot failed: {message}"))
        return _deduplicate(issues)
    reader = _PackageReader(package_snapshot, spec.slug, issues)
    specs = load_model_specs()
    expected_by_key = _build_expected_data(specs, reader)
    expected = expected_by_key.get(spec.key)
    approved = _capture_approved_thumbnails(reader, _expected_model_hashes(specs, expected_by_key))
    _validate_package_snapshot(package_snapshot, spec, expected, approved, issues)
    return _deduplicate(issues)


def _validate_export(
    path: Path,
    *,
    require_current_approved_thumbnails: bool,
) -> tuple[tuple[ModelSpec, ...], dict[str, _ExpectedData]]:
    try:
        export_snapshot = capture_directory(Path(path))
    except SnapshotError as error:
        message = str(error).replace("snapshot root", "export root")
        raise ValidationFailure((ValidationIssue("<export>", message),)) from None

    specs = load_model_specs()
    root_issues: list[ValidationIssue] = []
    root_reader = _PackageReader(export_snapshot, "<export>", root_issues)
    package_roots = {spec.slug for spec in specs}
    for problem in export_snapshot.problems:
        if not problem.path.parts or len(problem.path.parts) == 1 or problem.path.parts[0] not in package_roots:
            root_reader.issue(f"{problem.message}: {problem.path}")
    root_reader.validate_root_inventory(
        expected_files={".onerobotics-a1-gazebo-export.json"},
        expected_directories={"archives", *(spec.slug for spec in specs)},
        label="export",
    )
    marker = export_snapshot.files.get(PurePosixPath(EXPORT_MARKER))
    expected_marker = (json.dumps({"generator": "onerobotics_a1_gazebo.package", "schema_version": 1}) + "\n").encode()
    if marker is not None and marker != expected_marker:
        root_reader.issue("export marker mismatch")
    expected = _build_expected_data(specs, root_reader)
    approved = (
        _capture_approved_thumbnails(root_reader, _expected_model_hashes(specs, expected))
        if require_current_approved_thumbnails
        else None
    )
    packages: dict[str, DirectorySnapshot] = {}
    for spec in specs:
        try:
            package = export_snapshot.subtree(spec.slug)
        except SnapshotError:
            continue
        packages[spec.key] = package
        _validate_package_snapshot(package, spec, expected.get(spec.key), approved, root_issues)
    _validate_archives(export_snapshot, specs, packages, root_reader)
    issues = _deduplicate(root_issues)
    if issues:
        raise ValidationFailure(issues)
    return specs, expected


def validate_render_input(path: Path) -> None:
    """Validate render inputs without requiring thumbnails from the current render config."""
    _validate_export(path, require_current_approved_thumbnails=False)


def validate_all(path: Path) -> None:
    """Validate the exact three-model export tree and print a stable success summary."""
    specs, expected = _validate_export(path, require_current_approved_thumbnails=True)
    mesh_count = 0
    actuated_count = 0
    for spec in specs:
        link_count = len(spec.physical_links)
        joint_count = len(spec.actuated_joints)
        model_expected = expected[spec.key]
        mesh_count += len(model_expected.mesh_digests)
        actuated_count += joint_count
        print(
            f"VALID {spec.slug}: {link_count} links, {joint_count} actuated joints, "
            f"{len(model_expected.mesh_digests)} meshes"
        )
    print(f"VALIDATION_OK: {len(specs)} models, {mesh_count} meshes, {actuated_count} actuated joints")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", type=Path, help="export root containing the three Fuel model packages")
    arguments = parser.parse_args(argv)
    try:
        validate_all(arguments.path)
    except (ValidationFailure, ValueError, OSError) as error:
        if isinstance(error, ValidationFailure):
            for issue in error.issues:
                print(f"INVALID {issue.model}: {issue.message}", file=sys.stderr)
        else:
            print(f"VALIDATION_ERROR: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
