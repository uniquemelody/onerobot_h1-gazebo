"""Semantic validation for SDF parsed by Gazebo Harmonic's SDFormat library."""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from xml.etree import ElementTree
from xml.etree.ElementTree import Element

from onerobotics_a1_gazebo.spec import ModelSpec, load_model_specs
from onerobotics_a1_gazebo.validation_snapshot import SnapshotError, capture_directory

_LIMIT_FIELDS = ("lower", "upper", "effort", "velocity")
_MAX_SDF_BYTES = 8 * 1024 * 1024
_MAX_XML_DEPTH = 256


@dataclass(frozen=True)
class _JointSemantics:
    joint_type: str
    parent: str
    child: str
    limits: tuple[float, float, float, float] | None


@dataclass(frozen=True)
class _ModelSemantics:
    links: frozenset[str]
    joints: dict[str, _JointSemantics]


def _deduplicate(issues: list[str]) -> tuple[str, ...]:
    return tuple(dict.fromkeys(issues))


def snapshot_export(source: Path, destination: Path) -> tuple[str, ...]:
    """Capture and materialize one descriptor-safe immutable export snapshot."""
    source = Path(source)
    destination = Path(destination)
    try:
        snapshot = capture_directory(source)
    except SnapshotError as error:
        return (str(error),)
    if snapshot.problems:
        return tuple(f"{problem.message}: {problem.path}" for problem in snapshot.problems)
    if destination.exists() or destination.is_symlink():
        return (f"snapshot destination already exists: {destination}",)
    try:
        destination.mkdir(mode=0o700)
        for relative in sorted(snapshot.directories, key=lambda path: (len(path.parts), path.as_posix())):
            (destination / Path(*relative.parts)).mkdir(mode=0o700)
        for relative, data in sorted(snapshot.files.items(), key=lambda item: item[0].as_posix()):
            path = destination / Path(*relative.parts)
            with path.open("xb") as stream:
                stream.write(data)
    except (MemoryError, OSError, ValueError) as error:
        return (f"unable to materialize export snapshot: {error}",)
    return ()


def _depth_exceeds(root: Element) -> bool:
    pending = [(root, 1)]
    while pending:
        element, depth = pending.pop()
        if depth > _MAX_XML_DEPTH:
            return True
        pending.extend((child, depth + 1) for child in element)
    return False


def _parse_document(path: Path, label: str, issues: list[str]) -> Element | None:
    try:
        size = path.stat().st_size
        if size > _MAX_SDF_BYTES:
            issues.append(f"{label} SDF exceeds the {_MAX_SDF_BYTES}-byte limit")
            return None
        data = path.read_bytes()
        root = ElementTree.fromstring(data)
    except RecursionError:
        issues.append(f"{label} SDF exceeds the XML nesting limit")
        return None
    except (ElementTree.ParseError, LookupError):
        issues.append(f"malformed {label} SDF")
        return None
    except MemoryError:
        issues.append(f"{label} SDF exceeds the available parsing resources")
        return None
    except (OSError, OverflowError, ValueError) as error:
        issues.append(f"unable to read {label} SDF: {error}")
        return None
    if _depth_exceeds(root):
        issues.append(f"{label} SDF exceeds the XML nesting limit")
        return None
    if root.tag != "sdf":
        issues.append(f"{label} SDF root must be <sdf>")
        return None
    return root


def _direct_named(
    model: Element,
    tag: str,
    label: str,
    issues: list[str],
) -> tuple[list[str], dict[str, Element]]:
    names: list[str] = []
    by_name: dict[str, Element] = {}
    for element in model.findall(tag):
        name = element.attrib.get("name", "").strip()
        if not name:
            issues.append(f"{label} {tag} requires a name")
            continue
        names.append(name)
        if name in by_name:
            issues.append(f"{label} duplicate {tag} name: {name}")
        else:
            by_name[name] = element
    return names, by_name


def _single_text(parent: Element, tag: str, context: str, issues: list[str]) -> str | None:
    children = parent.findall(tag)
    if len(children) != 1 or children[0].text is None or not children[0].text.strip():
        issues.append(f"{context} must contain exactly one non-empty {tag}")
        return None
    return " ".join(children[0].text.split())


def _finite_limit(joint: Element, joint_name: str, field: str, label: str, issues: list[str]) -> float | None:
    axes = joint.findall("axis")
    if len(axes) != 1:
        issues.append(f"{label} joint {joint_name} must contain exactly one axis")
        return None
    limits = axes[0].findall("limit")
    if len(limits) != 1:
        issues.append(f"{label} joint {joint_name} must contain exactly one limit")
        return None
    text = _single_text(limits[0], field, f"{label} joint {joint_name} limit", issues)
    if text is None:
        return None
    try:
        value = float(text)
    except ValueError:
        value = math.nan
    if not math.isfinite(value):
        issues.append(f"{label} joint {joint_name} {field} must be a finite numeric value")
        return None
    return value


def _extract_semantics(root: Element, spec: ModelSpec, label: str, issues: list[str]) -> _ModelSemantics | None:
    models = root.findall("model")
    if len(models) != 1:
        issues.append(f"{label} SDF must contain exactly one direct model")
        return None
    model = models[0]
    nested_models = model.findall(".//model")
    if nested_models:
        issues.append(f"{label} SDF must not contain a nested model")
    if len(list(root.iter("model"))) != 1:
        issues.append(f"{label} SDF must contain exactly one model in the document")
    if len(root) != 1 or root[0] is not model:
        issues.append(f"{label} SDF must contain only direct model resource")
    if model.attrib.get("name") != spec.slug:
        issues.append(f"{label} model name must be {spec.slug}")
    if model.attrib.get("canonical_link") != "base_link":
        issues.append(f"{label} canonical link must be base_link")

    link_names, links = _direct_named(model, "link", label, issues)
    expected_links = set(spec.physical_links)
    if len(link_names) != len(expected_links) or set(link_names) != expected_links:
        missing = sorted(expected_links - set(link_names))
        extra = sorted(set(link_names) - expected_links)
        issues.append(f"{label} link inventory mismatch; missing={missing}, extra={extra}")

    joint_names, joint_elements = _direct_named(model, "joint", label, issues)
    expected_joint_names = {"world_to_base", *spec.fixed_joints, *spec.actuated_joints}
    if len(joint_names) != len(expected_joint_names) or set(joint_names) != expected_joint_names:
        missing = sorted(expected_joint_names - set(joint_names))
        extra = sorted(set(joint_names) - expected_joint_names)
        issues.append(f"{label} joint inventory mismatch; missing={missing}, extra={extra}")

    joints: dict[str, _JointSemantics] = {}
    for name, joint in joint_elements.items():
        joint_type = joint.attrib.get("type", "").strip()
        if not joint_type:
            issues.append(f"{label} joint {name} requires a joint type")
        parent = _single_text(joint, "parent", f"{label} joint {name}", issues)
        child = _single_text(joint, "child", f"{label} joint {name}", issues)
        limits: tuple[float, float, float, float] | None = None
        if name in spec.actuated_joints:
            values = tuple(_finite_limit(joint, name, field, label, issues) for field in _LIMIT_FIELDS)
            if all(value is not None for value in values):
                limits = (values[0], values[1], values[2], values[3])  # type: ignore[arg-type]
        if parent is not None and child is not None:
            joints[name] = _JointSemantics(joint_type, parent, child, limits)

    anchor = joints.get("world_to_base")
    if anchor is None or (anchor.joint_type, anchor.parent, anchor.child) != ("fixed", "world", "base_link"):
        issues.append("world anchor world_to_base must be fixed world -> base_link")
    for shoulder in spec.fixed_joints:
        semantics = joints.get(shoulder)
        if semantics is None or semantics.joint_type != "fixed":
            issues.append(f"fixed shoulder {shoulder} must remain fixed")

    return _ModelSemantics(frozenset(links), joints)


def validate_parsed_sdf(packaged_sdf: Path, parsed_sdf: Path, spec: ModelSpec) -> tuple[str, ...]:
    """Compare Harmonic-parsed SDF semantics with a pure-validated package SDF."""
    issues: list[str] = []
    expected_root = _parse_document(Path(packaged_sdf), "packaged", issues)
    parsed_root = _parse_document(Path(parsed_sdf), "parsed", issues)
    if expected_root is None or parsed_root is None:
        return _deduplicate(issues)

    expected = _extract_semantics(expected_root, spec, "packaged", issues)
    actual = _extract_semantics(parsed_root, spec, "parsed", issues)
    if expected is None or actual is None:
        return _deduplicate(issues)

    for name in sorted(set(expected.joints) & set(actual.joints)):
        expected_joint = expected.joints[name]
        actual_joint = actual.joints[name]
        if actual_joint.joint_type != expected_joint.joint_type:
            issues.append(
                f"joint type mismatch for {name}: expected {expected_joint.joint_type}, got {actual_joint.joint_type}"
            )
        if actual_joint.parent != expected_joint.parent:
            issues.append(
                f"joint parent mismatch for {name}: expected {expected_joint.parent}, got {actual_joint.parent}"
            )
        if actual_joint.child != expected_joint.child:
            issues.append(f"joint child mismatch for {name}: expected {expected_joint.child}, got {actual_joint.child}")
        if name not in spec.actuated_joints or expected_joint.limits is None or actual_joint.limits is None:
            continue
        for field, expected_value, actual_value in zip(
            _LIMIT_FIELDS,
            expected_joint.limits,
            actual_joint.limits,
            strict=True,
        ):
            if actual_value != expected_value:
                issues.append(f"joint {name} {field} mismatch: expected {expected_value:.17g}, got {actual_value:.17g}")
    return _deduplicate(issues)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot-export", action="store_true")
    parser.add_argument("packaged_sdf", type=Path)
    parser.add_argument("parsed_sdf", type=Path)
    parser.add_argument("slug", nargs="?")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.snapshot_export:
        if args.slug is not None:
            print("HARMONIC_SNAPSHOT_ERROR: snapshot mode accepts exactly a source and destination", file=sys.stderr)
            return 2
        issues = snapshot_export(args.packaged_sdf, args.parsed_sdf)
        for issue in issues:
            print(f"HARMONIC_SNAPSHOT_ERROR: {issue}", file=sys.stderr)
        return 1 if issues else 0
    if args.slug is None:
        print("HARMONIC_SEMANTIC_ERROR <unknown>: model slug is required", file=sys.stderr)
        return 2
    try:
        specs = {spec.slug: spec for spec in load_model_specs()}
    except ValueError as error:
        print(f"HARMONIC_SEMANTIC_ERROR {args.slug}: unable to load model specifications: {error}", file=sys.stderr)
        return 1
    spec = specs.get(args.slug)
    if spec is None:
        print(f"HARMONIC_SEMANTIC_ERROR {args.slug}: unknown model slug", file=sys.stderr)
        return 1
    issues = validate_parsed_sdf(args.packaged_sdf, args.parsed_sdf, spec)
    for issue in issues:
        print(f"HARMONIC_SEMANTIC_ERROR {spec.slug}: {issue}", file=sys.stderr)
    return 1 if issues else 0


if __name__ == "__main__":
    raise SystemExit(main())
