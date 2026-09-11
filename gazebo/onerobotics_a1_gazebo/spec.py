"""Validated model specifications derived from the public A1 source bundle."""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

SOURCE_REPOSITORY = "https://github.com/katazen/onerobot_h1"
SOURCE_COMMIT = "ecf530911284ba0e559f7a24dc222fd8e60d31ed"
SOURCE_ROOT_RELATIVE = Path("source/h1_reach/h1_reach/assets/urdf/A1_2026")
MOTOR_JOINT_INDICES = {"4340": (1, 2, 3), "4310": (4, 5, 6, 7)}
GROUP_JOINT_INDICES = {3: (1, 2, 3), 4: (4, 5, 6, 7)}
PUBLIC_MODEL_SCHEMA = (
    (
        "right_arm",
        "onerobotics_a1_right_arm",
        "OneRobotics A1 Right Arm",
        "a1_r.urdf",
        ("base_link", "Link1", "Link2", "Link3", "Link4", "Link5", "Link6", "Link7"),
        (),
        (
            ("joint1-a1_r", "joint2-a1_r", "joint3-a1_r"),
            ("joint4-a1_r", "joint5-a1_r", "joint6-a1_r", "joint7-a1_r"),
        ),
    ),
    (
        "left_arm",
        "onerobotics_a1_left_arm",
        "OneRobotics A1 Left Arm",
        "a1_l.urdf",
        ("base_link", "Link1", "Link2", "Link3", "Link4", "Link5", "Link6", "Link7"),
        (),
        (
            ("joint1-a1_l", "joint2-a1_l", "joint3-a1_l"),
            ("joint4-a1_l", "joint5-a1_l", "joint6-a1_l", "joint7-a1_l"),
        ),
    ),
    (
        "bimanual_stand",
        "onerobotics_a1_bimanual_stand",
        "OneRobotics A1 Bimanual Stand",
        "bimanual_stand/a1_bimanual_stand.urdf",
        (
            "base_link",
            "Link_r0",
            "Link_r1",
            "Link_r2",
            "Link_r3",
            "Link_r4",
            "Link_r5",
            "Link_r6",
            "Link_r7",
            "Link_l0",
            "Link_l1",
            "Link_l2",
            "Link_l3",
            "Link_l4",
            "Link_l5",
            "Link_l6",
            "Link_l7",
        ),
        ("joint_r0", "joint_l0"),
        (
            ("joint_r1", "joint_r2", "joint_r3"),
            ("joint_r4", "joint_r5", "joint_r6", "joint_r7"),
            ("joint_l1", "joint_l2", "joint_l3"),
            ("joint_l4", "joint_l5", "joint_l6", "joint_l7"),
        ),
    ),
)


@dataclass(frozen=True)
class JointLimitOverlay:
    effort: float
    velocity: float


@dataclass(frozen=True)
class ModelSpec:
    key: str
    slug: str
    display_name: str
    urdf: Path
    physical_links: tuple[str, ...]
    fixed_joints: tuple[str, ...]
    actuator_groups: tuple[tuple[str, ...], ...]

    @property
    def actuated_joints(self) -> tuple[str, ...]:
        return tuple(joint for group in self.actuator_groups for joint in group)


def _gazebo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _repository_root() -> Path:
    return _gazebo_root().parent


def source_root() -> Path:
    return _repository_root() / SOURCE_ROOT_RELATIVE


def _load_yaml_mapping(path: Path) -> dict[str, Any]:
    try:
        with path.open(encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
    except OSError as error:
        raise ValueError(f"Unable to read {path}") from error
    if not isinstance(document, dict):
        raise ValueError(f"{path} must contain a YAML mapping")
    return document


def _string_tuple(value: object, field: str, *, allow_empty: bool = False) -> tuple[str, ...]:
    if (
        not isinstance(value, list)
        or (not allow_empty and not value)
        or not all(isinstance(item, str) and item for item in value)
    ):
        qualifier = "a list of strings" if allow_empty else "a non-empty list of strings"
        raise ValueError(f"{field} must be {qualifier}")
    return tuple(value)


def _joint_index(joint: str) -> int:
    match = re.fullmatch(r"joint(?:(\d+)-a1_[rl]|_[rl](\d+))", joint)
    if match is None:
        raise ValueError(f"Unsupported actuator joint name: {joint}")
    return int(match.group(1) or match.group(2))


def _validate_model(spec: ModelSpec) -> None:
    if not spec.urdf.is_file():
        raise ValueError(f"Missing URDF source file: {spec.urdf}")
    if len(set(spec.physical_links)) != len(spec.physical_links):
        raise ValueError(f"Duplicate physical links in {spec.key}")
    if len(set(spec.fixed_joints)) != len(spec.fixed_joints):
        raise ValueError(f"Duplicate fixed joints in {spec.key}")
    if not spec.actuator_groups:
        raise ValueError(f"{spec.key} must declare actuator groups")

    actuated = spec.actuated_joints
    if len(set(actuated)) != len(actuated):
        raise ValueError(f"Duplicate actuated joints in {spec.key}")
    if set(actuated) & set(spec.fixed_joints):
        raise ValueError(f"Fixed and actuated joints overlap in {spec.key}")

    for group in spec.actuator_groups:
        expected_indices = GROUP_JOINT_INDICES.get(len(group))
        if expected_indices is None:
            raise ValueError(f"Invalid actuator group length in {spec.key}: {len(group)}")
        if tuple(_joint_index(joint) for joint in group) != expected_indices:
            raise ValueError(f"Actuator group does not match motor indices in {spec.key}: {group}")


def _expected_model_specs(assets_root: Path) -> tuple[ModelSpec, ...]:
    return tuple(
        ModelSpec(
            key=key,
            slug=slug,
            display_name=display_name,
            urdf=assets_root / urdf,
            physical_links=physical_links,
            fixed_joints=fixed_joints,
            actuator_groups=actuator_groups,
        )
        for key, slug, display_name, urdf, physical_links, fixed_joints, actuator_groups in PUBLIC_MODEL_SCHEMA
    )


def load_model_specs(config_path: Path | None = None) -> tuple[ModelSpec, ...]:
    """Load the three immutable, validated public A1 model specifications."""
    path = config_path or _gazebo_root() / "config/models.yaml"
    config = _load_yaml_mapping(path)
    if config.get("source_repository") != SOURCE_REPOSITORY:
        raise ValueError("Unexpected source repository")
    if config.get("source_commit") != SOURCE_COMMIT:
        raise ValueError("Unexpected source commit")
    if config.get("source_root") != SOURCE_ROOT_RELATIVE.as_posix():
        raise ValueError("Unexpected source root")

    models = config.get("models")
    if not isinstance(models, list) or not models:
        raise ValueError("models must be a non-empty list")

    assets_root = source_root().resolve()
    expected_specs = _expected_model_specs(assets_root)
    if len(models) != len(expected_specs):
        raise ValueError("models must contain exactly three public models")
    specs: list[ModelSpec] = []
    for model, expected_spec in zip(models, expected_specs, strict=True):
        if not isinstance(model, dict):
            raise ValueError("Each model must be a mapping")
        scalar_fields = ("key", "slug", "display_name", "urdf")
        if not all(isinstance(model.get(field), str) and model[field] for field in scalar_fields):
            raise ValueError("Each model needs non-empty key, slug, display_name, and urdf fields")
        relative_urdf = Path(model["urdf"])
        if relative_urdf.is_absolute() or relative_urdf.suffix != ".urdf":
            raise ValueError("Each model urdf must be a relative .urdf path")
        urdf = (assets_root / relative_urdf).resolve()
        try:
            urdf.relative_to(assets_root)
        except ValueError as error:
            raise ValueError("Each model urdf must remain under the public source root") from error
        groups = model.get("actuator_groups")
        if not isinstance(groups, list):
            raise ValueError(f"actuator_groups must be a list for {model['key']}")
        spec = ModelSpec(
            key=model["key"],
            slug=model["slug"],
            display_name=model["display_name"],
            urdf=urdf,
            physical_links=_string_tuple(model.get("physical_links"), "physical_links"),
            fixed_joints=_string_tuple(model.get("fixed_joints"), "fixed_joints", allow_empty=True),
            actuator_groups=tuple(_string_tuple(group, "actuator_groups") for group in groups),
        )
        _validate_model(spec)
        if spec != expected_spec:
            raise ValueError("Model does not match the required public model schema")
        specs.append(spec)

    if len({spec.key for spec in specs}) != len(specs):
        raise ValueError("Duplicate model keys")
    if len({spec.slug for spec in specs}) != len(specs):
        raise ValueError("Duplicate model slugs")
    if len({spec.urdf for spec in specs}) != len(specs):
        raise ValueError("Duplicate model URDFs")
    return tuple(specs)


def _positive_finite(value: object, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be a finite positive number")
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise ValueError(f"{field} must be a finite positive number")
    return result


def load_hardware_overlay(parameters_path: Path | None = None) -> dict[int, JointLimitOverlay]:
    """Load the verified 4340/4310 limits for every joint of one 7-DoF arm."""
    path = parameters_path or source_root() / "model_parameters.yaml"
    parameters = _load_yaml_mapping(path)
    motors = parameters.get("motors")
    if not isinstance(motors, dict):
        raise ValueError("motors must be a YAML mapping")

    overlay: dict[int, JointLimitOverlay] = {}
    for motor, indices in MOTOR_JOINT_INDICES.items():
        values = motors.get(motor)
        if not isinstance(values, dict):
            raise ValueError(f"Missing motor {motor}")
        for indices_field in ("joint_indices", "applies_to_each_arm_joint_indices"):
            if tuple(values.get(indices_field, ())) != indices:
                raise ValueError(f"Motor {motor} has unexpected {indices_field}")
        limit = JointLimitOverlay(
            effort=_positive_finite(values.get("peak_output_torque_nm"), f"{motor}.peak_output_torque_nm"),
            velocity=_positive_finite(values.get("rated_output_speed_rad_s"), f"{motor}.rated_output_speed_rad_s"),
        )
        for index in indices:
            if index in overlay:
                raise ValueError(f"Duplicate limit for joint {index}")
            overlay[index] = limit

    if set(overlay) != set(range(1, 8)):
        raise ValueError("Hardware overlay must contain exactly joints 1 through 7")
    return overlay
