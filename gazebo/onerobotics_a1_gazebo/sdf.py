"""Strict conversion of the public OneRobotics A1 URDFs to native SDF."""

from __future__ import annotations

import math
import re
from pathlib import Path, PurePosixPath
from xml.etree import ElementTree
from xml.etree.ElementTree import Element, SubElement

from onerobotics_a1_gazebo.spec import JointLimitOverlay, ModelSpec

_INERTIA_COMPONENTS = ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
_JOINT_TYPES = {"fixed", "revolute"}
_ROOT_CHILDREN = {"joint", "link", "mujoco"}
_LINK_CHILDREN = {"collision", "inertial", "visual"}
_KNOWN_MUJOCO_COMPILERS = (
    {"meshdir": ".", "balanceinertia": "true", "discardvisual": "false"},
    {"meshdir": "meshes", "strippath": "true", "balanceinertia": "true"},
)
_MUJOCO_COMPILER_ATTRIBUTES = frozenset().union(*_KNOWN_MUJOCO_COMPILERS)

Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]
Pose = tuple[Vector3, Quaternion]


def pose_text(origin: Element | None) -> str:
    xyz = "0 0 0" if origin is None else origin.attrib.get("xyz", "0 0 0")
    rpy = "0 0 0" if origin is None else origin.attrib.get("rpy", "0 0 0")
    return f"{' '.join(xyz.split())} {' '.join(rpy.split())}"


def _parse_vector(text: str, context: str) -> Vector3:
    tokens = text.split()
    if len(tokens) != 3:
        raise ValueError(f"{context} must contain exactly three finite numbers")
    try:
        values = tuple(float(token) for token in tokens)
    except ValueError as error:
        raise ValueError(f"{context} must contain exactly three finite numbers") from error
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"{context} must contain exactly three finite numbers")
    return values  # type: ignore[return-value]


def _quaternion_multiply(left: Quaternion, right: Quaternion) -> Quaternion:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _normalize_quaternion(quaternion: Quaternion) -> Quaternion:
    norm = math.sqrt(sum(component * component for component in quaternion))
    if not math.isfinite(norm) or norm == 0:
        raise ValueError("pose rotation produced an invalid quaternion")
    return tuple(component / norm for component in quaternion)  # type: ignore[return-value]


def _rpy_to_quaternion(rpy: Vector3) -> Quaternion:
    roll, pitch, yaw = rpy
    half_roll, half_pitch, half_yaw = roll / 2, pitch / 2, yaw / 2
    cr, sr = math.cos(half_roll), math.sin(half_roll)
    cp, sp = math.cos(half_pitch), math.sin(half_pitch)
    cy, sy = math.cos(half_yaw), math.sin(half_yaw)
    return _normalize_quaternion(
        (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )
    )


def _quaternion_to_rpy(quaternion: Quaternion) -> Vector3:
    x, y, z, w = _normalize_quaternion(quaternion)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sin_pitch = 2 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sin_pitch) if abs(sin_pitch) >= 1 else math.asin(sin_pitch)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def _rotate_vector(quaternion: Quaternion, vector: Vector3) -> Vector3:
    x, y, z, w = quaternion
    vx, vy, vz = vector
    cross_x = y * vz - z * vy
    cross_y = z * vx - x * vz
    cross_z = x * vy - y * vx
    double_cross_x = y * cross_z - z * cross_y
    double_cross_y = z * cross_x - x * cross_z
    double_cross_z = x * cross_y - y * cross_x
    return (
        vx + 2 * (w * cross_x + double_cross_x),
        vy + 2 * (w * cross_y + double_cross_y),
        vz + 2 * (w * cross_z + double_cross_z),
    )


def _origin_pose(origin: Element | None, context: str) -> Pose:
    if origin is None:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)
    xyz = _parse_vector(origin.attrib.get("xyz", "0 0 0"), f"{context} xyz")
    rpy = _parse_vector(origin.attrib.get("rpy", "0 0 0"), f"{context} rpy")
    return xyz, _rpy_to_quaternion(rpy)


def _compose_poses(parent: Pose, child: Pose) -> Pose:
    parent_translation, parent_rotation = parent
    child_translation, child_rotation = child
    rotated_child = _rotate_vector(parent_rotation, child_translation)
    translation = tuple(parent_translation[index] + rotated_child[index] for index in range(3))
    rotation = _normalize_quaternion(_quaternion_multiply(parent_rotation, child_rotation))
    return translation, rotation  # type: ignore[return-value]


def _format_pose_number(value: float) -> str:
    if not math.isfinite(value):
        raise ValueError(f"pose value must be finite: {value}")
    if abs(value) < 1e-15:
        value = 0.0
    return format(value, ".15g")


def _pose_to_text(pose: Pose) -> str:
    translation, quaternion = pose
    values = (*translation, *_quaternion_to_rpy(quaternion))
    return " ".join(_format_pose_number(value) for value in values)


def add_text(parent: Element, tag: str, text: str, **attributes: str) -> Element:
    child = SubElement(parent, tag, attributes)
    child.text = text
    return child


def source_joint_index(name: str) -> int:
    match = re.search(r"([1-7])(?:-a1_[rl])?$", name)
    if match is None:
        raise ValueError(f"cannot map actuated joint to motor index: {name}")
    return int(match.group(1))


def _format_number(value: float) -> str:
    if not math.isfinite(value):
        raise ValueError(f"hardware limit must be finite: {value}")
    return str(value)


def _normalized_attribute(element: Element, attribute: str, context: str) -> str:
    value = element.attrib.get(attribute)
    if value is None or not value.split():
        raise ValueError(f"{context} requires {attribute}")
    return " ".join(value.split())


def _single_child(parent: Element, tag: str, context: str, *, required: bool = True) -> Element | None:
    children = parent.findall(tag)
    if len(children) > 1 or (required and not children):
        qualifier = "exactly one" if required else "at most one"
        raise ValueError(f"{context} must contain {qualifier} {tag}")
    return children[0] if children else None


def _validate_element(
    element: Element,
    context: str,
    *,
    attributes: frozenset[str] = frozenset(),
    required_attributes: frozenset[str] = frozenset(),
    children: frozenset[str] = frozenset(),
) -> None:
    unknown_attributes = set(element.attrib) - attributes
    if unknown_attributes:
        raise ValueError(f"unsupported {context} attribute: {min(unknown_attributes)}")
    missing_attributes = required_attributes - set(element.attrib)
    if missing_attributes:
        raise ValueError(f"{context} requires attribute: {min(missing_attributes)}")
    unknown = [child.tag for child in element if child.tag not in children]
    if unknown:
        raise ValueError(f"unsupported {context} child: {unknown[0]}")
    if element.text is not None and element.text.strip():
        raise ValueError(f"unsupported {context} text")
    if any(child.tail is not None and child.tail.strip() for child in element):
        raise ValueError(f"unsupported {context} text")


def _validate_mujoco(root: Element) -> None:
    mujoco = _single_child(root, "mujoco", "robot")
    assert mujoco is not None
    _validate_element(mujoco, "mujoco", children=frozenset({"compiler"}))
    compiler = _single_child(mujoco, "compiler", "mujoco")
    assert compiler is not None
    _validate_element(
        compiler,
        "mujoco compiler",
        attributes=_MUJOCO_COMPILER_ATTRIBUTES,
    )
    if compiler.attrib not in _KNOWN_MUJOCO_COMPILERS:
        raise ValueError("unsupported mujoco compiler attributes or values")


def _read_urdf(path: Path) -> Element:
    try:
        root = ElementTree.parse(path).getroot()
    except (OSError, ElementTree.ParseError) as error:
        raise ValueError(f"unable to parse URDF: {path}") from error
    if root.tag != "robot":
        raise ValueError(f"URDF root must be <robot>, got <{root.tag}>")
    _validate_element(
        root,
        "robot",
        attributes=frozenset({"name"}),
        required_attributes=frozenset({"name"}),
        children=frozenset(_ROOT_CHILDREN),
    )
    _validate_mujoco(root)
    return root


def _named_elements(root: Element, tag: str) -> list[Element]:
    elements = root.findall(tag)
    names: set[str] = set()
    for element in elements:
        name = element.attrib.get("name")
        if not name:
            raise ValueError(f"every {tag} requires a name")
        if name in names:
            raise ValueError(f"duplicate {tag} name: {name}")
        names.add(name)
    return elements


def _copy_inertial(link_out: Element, inertial: Element) -> None:
    _validate_element(inertial, "inertial", children=frozenset({"origin", "mass", "inertia"}))
    origin = _single_child(inertial, "origin", "inertial", required=False)
    mass = _single_child(inertial, "mass", "inertial")
    inertia = _single_child(inertial, "inertia", "inertial")
    assert mass is not None and inertia is not None
    if origin is not None:
        _validate_element(origin, "origin", attributes=frozenset({"xyz", "rpy"}))
    _validate_element(
        mass,
        "mass",
        attributes=frozenset({"value"}),
        required_attributes=frozenset({"value"}),
    )
    _validate_element(
        inertia,
        "inertia",
        attributes=frozenset(_INERTIA_COMPONENTS),
        required_attributes=frozenset(_INERTIA_COMPONENTS),
    )

    inertial_out = SubElement(link_out, "inertial")
    add_text(inertial_out, "pose", pose_text(origin))
    add_text(inertial_out, "mass", _normalized_attribute(mass, "value", "mass"))
    inertia_out = SubElement(inertial_out, "inertia")
    for component in _INERTIA_COMPONENTS:
        add_text(
            inertia_out,
            component,
            _normalized_attribute(inertia, component, "inertia"),
        )


def _mesh_uri(filename: str, mesh_sources: dict[str, str]) -> str:
    normalized = filename.replace("\\", "/")
    basename = PurePosixPath(normalized).name
    if not basename:
        raise ValueError("mesh filename must have a basename")
    previous = mesh_sources.setdefault(basename, normalized)
    if previous != normalized:
        raise ValueError(f"mesh basename collision: {basename}")
    return f"meshes/{basename}"


def _copy_geometry(parent_out: Element, geometry: Element, mesh_sources: dict[str, str]) -> None:
    shapes = list(geometry)
    if len(shapes) != 1:
        raise ValueError("geometry must contain exactly one shape")
    shape = shapes[0]
    geometry_out = SubElement(parent_out, "geometry")

    if shape.tag == "mesh":
        _validate_element(
            shape,
            "mesh",
            attributes=frozenset({"filename", "scale"}),
            required_attributes=frozenset({"filename"}),
        )
        filename = _normalized_attribute(shape, "filename", "mesh")
        mesh_out = SubElement(geometry_out, "mesh")
        add_text(mesh_out, "uri", _mesh_uri(filename, mesh_sources))
        scale = shape.attrib.get("scale")
        if scale is not None:
            add_text(mesh_out, "scale", " ".join(scale.split()))
        return
    if shape.tag == "box":
        _validate_element(
            shape,
            "box",
            attributes=frozenset({"size"}),
            required_attributes=frozenset({"size"}),
        )
        box_out = SubElement(geometry_out, "box")
        add_text(box_out, "size", _normalized_attribute(shape, "size", "box"))
        return
    if shape.tag == "cylinder":
        _validate_element(
            shape,
            "cylinder",
            attributes=frozenset({"radius", "length"}),
            required_attributes=frozenset({"radius", "length"}),
        )
        cylinder_out = SubElement(geometry_out, "cylinder")
        add_text(cylinder_out, "radius", _normalized_attribute(shape, "radius", "cylinder"))
        add_text(cylinder_out, "length", _normalized_attribute(shape, "length", "cylinder"))
        return
    if shape.tag == "sphere":
        _validate_element(
            shape,
            "sphere",
            attributes=frozenset({"radius"}),
            required_attributes=frozenset({"radius"}),
        )
        sphere_out = SubElement(geometry_out, "sphere")
        add_text(sphere_out, "radius", _normalized_attribute(shape, "radius", "sphere"))
        return
    raise ValueError(f"unsupported geometry: {shape.tag}")


def _copy_material(visual_out: Element, material: Element) -> None:
    _validate_element(
        material,
        "material",
        attributes=frozenset({"name"}),
        children=frozenset({"color"}),
    )
    color = _single_child(material, "color", "material")
    assert color is not None
    _validate_element(
        color,
        "color",
        attributes=frozenset({"rgba"}),
        required_attributes=frozenset({"rgba"}),
    )
    rgba = _normalized_attribute(color, "rgba", "color")
    material_out = SubElement(visual_out, "material")
    add_text(material_out, "ambient", rgba)
    add_text(material_out, "diffuse", rgba)


def _copy_visual_or_collision(
    link_out: Element,
    source: Element,
    output_tag: str,
    name: str,
    mesh_sources: dict[str, str],
) -> None:
    allowed = {"geometry", "origin"}
    if output_tag == "visual":
        allowed.add("material")
    _validate_element(source, output_tag, children=frozenset(allowed))
    origin = _single_child(source, "origin", output_tag, required=False)
    geometry = _single_child(source, "geometry", output_tag)
    assert geometry is not None
    if origin is not None:
        _validate_element(origin, "origin", attributes=frozenset({"xyz", "rpy"}))
    _validate_element(geometry, "geometry", children=frozenset({"box", "cylinder", "mesh", "sphere"}))

    output = SubElement(link_out, output_tag, {"name": name})
    add_text(output, "pose", pose_text(origin))
    _copy_geometry(output, geometry, mesh_sources)
    material = _single_child(source, "material", output_tag, required=False)
    if material is not None:
        _copy_material(output, material)


def _joint_end(joint: Element, tag: str, name: str) -> str:
    end = _single_child(joint, tag, f"joint {name}")
    assert end is not None
    _validate_element(
        end,
        f"joint {name} {tag}",
        attributes=frozenset({"link"}),
        required_attributes=frozenset({"link"}),
    )
    return _normalized_attribute(end, "link", f"joint {name} {tag}")


def _source_graph(
    joints: list[Element], physical_names: set[str]
) -> tuple[dict[str, tuple[Element, str]], Element | None]:
    incoming: dict[str, tuple[Element, str]] = {}
    world_anchor: Element | None = None
    for joint in joints:
        name = joint.attrib["name"]
        parent = _joint_end(joint, "parent", name)
        child = _joint_end(joint, "child", name)
        if child == "world":
            raise ValueError("a joint child cannot be world")
        if child not in physical_names:
            raise ValueError(f"joint {name} has unknown child: {child}")
        if parent != "world" and parent not in physical_names:
            raise ValueError(f"joint {name} has unknown parent: {parent}")
        if child in incoming:
            raise ValueError(f"link has multiple parent joints: {child}")
        incoming[child] = (joint, parent)
        if parent == "world":
            if world_anchor is not None:
                raise ValueError("URDF has multiple world anchors")
            world_anchor = joint
    return incoming, world_anchor


def _validate_physical_tree(incoming: dict[str, tuple[Element, str]], physical_names: set[str]) -> None:
    children_by_parent: dict[str, list[str]] = {name: [] for name in physical_names}
    for child, (_, parent) in incoming.items():
        if parent != "world":
            children_by_parent[parent].append(child)

    visited: set[str] = set()
    pending = ["base_link"]
    while pending:
        link = pending.pop()
        if link in visited:
            raise ValueError("physical joint graph must be a connected acyclic tree rooted at base_link")
        visited.add(link)
        pending.extend(children_by_parent[link])
    if visited != physical_names:
        raise ValueError("physical joint graph must be a connected acyclic tree rooted at base_link")


def _absolute_link_poses(incoming: dict[str, tuple[Element, str]], physical_names: set[str]) -> dict[str, Pose]:
    """Resolve zero-position link poses for static viewers without frame semantics."""
    identity: Pose = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    root_data = incoming.get("base_link")
    if root_data is None:
        base_pose = identity
    else:
        root_joint, root_parent = root_data
        if root_parent != "world":
            raise ValueError("base_link parent must be world when a root joint is present")
        base_pose = _origin_pose(root_joint.find("origin"), f"joint {root_joint.attrib['name']} origin")

    absolute = {"base_link": base_pose}
    while len(absolute) < len(physical_names):
        previous_size = len(absolute)
        for child, (joint, parent) in incoming.items():
            if child in absolute or parent not in absolute:
                continue
            joint_pose = _origin_pose(joint.find("origin"), f"joint {joint.attrib['name']} origin")
            absolute[child] = _compose_poses(absolute[parent], joint_pose)
        if len(absolute) == previous_size:
            raise ValueError("cannot resolve absolute physical link poses")
    return absolute


def _validate_joint_types(joints: list[Element], spec: ModelSpec, world_anchor: Element | None) -> None:
    joint_by_name = {joint.attrib["name"]: joint for joint in joints}
    for name in spec.actuated_joints:
        if joint_by_name[name].attrib.get("type") != "revolute":
            raise ValueError(f"actuated joint {name} must be revolute")
    for name in spec.fixed_joints:
        if joint_by_name[name].attrib.get("type") != "fixed":
            raise ValueError(f"declared fixed joint {name} must be fixed")
    if world_anchor is not None and world_anchor.attrib.get("type") != "fixed":
        raise ValueError(f"world anchor {world_anchor.attrib['name']} must be fixed")


def _copy_link(
    model: Element,
    link: Element,
    absolute_pose: Pose,
    mesh_sources: dict[str, str],
) -> None:
    name = link.attrib["name"]
    _validate_element(
        link,
        f"link {name}",
        attributes=frozenset({"name"}),
        required_attributes=frozenset({"name"}),
        children=frozenset(_LINK_CHILDREN),
    )
    link_out = SubElement(model, "link", {"name": name})
    # Gazebo Fuel's browser viewer ignores SDF relative_to frame semantics.
    # A model-frame pose keeps its static preview equivalent to native Gazebo.
    add_text(link_out, "pose", _pose_to_text(absolute_pose))

    inertial = _single_child(link, "inertial", f"link {name}", required=False)
    if inertial is not None:
        _copy_inertial(link_out, inertial)
    for index, visual in enumerate(link.findall("visual")):
        _copy_visual_or_collision(
            link_out,
            visual,
            "visual",
            f"{name}_visual_{index}",
            mesh_sources,
        )
    for index, collision in enumerate(link.findall("collision")):
        _copy_visual_or_collision(
            link_out,
            collision,
            "collision",
            f"{name}_collision_{index}",
            mesh_sources,
        )


def _copy_joint(
    model: Element,
    joint: Element,
    overlay: dict[int, JointLimitOverlay],
    *,
    world_anchor: bool = False,
) -> None:
    name = joint.attrib["name"]
    joint_type = joint.attrib.get("type")
    if joint_type not in _JOINT_TYPES:
        raise ValueError(f"unsupported joint type for {name}: {joint_type}")
    allowed = {"axis", "child", "limit", "origin", "parent"}
    _validate_element(
        joint,
        f"joint {name}",
        attributes=frozenset({"name", "type"}),
        required_attributes=frozenset({"name", "type"}),
        children=frozenset(allowed),
    )
    parent_name = _joint_end(joint, "parent", name)
    child_name = _joint_end(joint, "child", name)
    origin = _single_child(joint, "origin", f"joint {name}", required=False)
    if origin is not None:
        _validate_element(origin, "origin", attributes=frozenset({"xyz", "rpy"}))

    if joint_type == "fixed":
        axis = _single_child(joint, "axis", f"fixed joint {name}", required=False)
        if axis is not None:
            if name not in {"joint_r0", "joint_l0"}:
                raise ValueError(f"fixed joint {name} cannot contain axis")
            _validate_element(
                axis,
                f"fixed joint {name} axis",
                attributes=frozenset({"xyz"}),
                required_attributes=frozenset({"xyz"}),
            )
            if " ".join(axis.attrib["xyz"].split()) != "0 0 0":
                raise ValueError(f"fixed joint {name} axis must be the known zero-axis exporter artifact")
        if joint.findall("limit"):
            raise ValueError(f"fixed joint {name} cannot contain limit")

    joint_out = SubElement(model, "joint", {"name": name, "type": joint_type})
    if world_anchor:
        add_text(joint_out, "pose", "0 0 0 0 0 0", relative_to=child_name)
    else:
        add_text(joint_out, "pose", pose_text(origin), relative_to=parent_name)
    add_text(joint_out, "parent", "world" if world_anchor else parent_name)
    add_text(joint_out, "child", child_name)

    if joint_type == "revolute":
        axis = _single_child(joint, "axis", f"joint {name}")
        limit = _single_child(joint, "limit", f"joint {name}")
        assert axis is not None and limit is not None
        _validate_element(
            axis,
            f"joint {name} axis",
            attributes=frozenset({"xyz"}),
            required_attributes=frozenset({"xyz"}),
        )
        _validate_element(
            limit,
            f"joint {name} limit",
            attributes=frozenset({"effort", "lower", "upper", "velocity"}),
            required_attributes=frozenset({"effort", "lower", "upper", "velocity"}),
        )
        axis_out = SubElement(joint_out, "axis")
        add_text(axis_out, "xyz", _normalized_attribute(axis, "xyz", f"joint {name} axis"))
        limit_out = SubElement(axis_out, "limit")
        add_text(limit_out, "lower", _normalized_attribute(limit, "lower", f"joint {name} limit"))
        add_text(limit_out, "upper", _normalized_attribute(limit, "upper", f"joint {name} limit"))
        try:
            limits = overlay[source_joint_index(name)]
        except KeyError as error:
            raise ValueError(f"missing hardware limits for joint {name}") from error
        add_text(limit_out, "effort", _format_number(limits.effort))
        add_text(limit_out, "velocity", _format_number(limits.velocity))


def _add_world_anchor(model: Element, child_name: str) -> None:
    anchor = SubElement(model, "joint", {"name": "world_to_base", "type": "fixed"})
    add_text(anchor, "pose", "0 0 0 0 0 0", relative_to=child_name)
    add_text(anchor, "parent", "world")
    add_text(anchor, "child", child_name)


def convert_urdf(spec: ModelSpec, overlay: dict[int, JointLimitOverlay]) -> ElementTree.ElementTree:
    """Convert one validated A1 model specification to an SDF 1.11 tree."""
    root = _read_urdf(spec.urdf)
    links = _named_elements(root, "link")
    joints = _named_elements(root, "joint")
    link_by_name = {link.attrib["name"]: link for link in links}
    physical_names = set(spec.physical_links)
    source_names = set(link_by_name)
    if source_names - physical_names not in (set(), {"world"}):
        raise ValueError("URDF contains links outside the physical model")
    if physical_names != source_names - {"world"}:
        raise ValueError("model specification does not match URDF physical links")
    if "world" in link_by_name:
        _validate_element(
            link_by_name["world"],
            "link world",
            attributes=frozenset({"name"}),
            required_attributes=frozenset({"name"}),
        )
    if len(set(link_by_name) | {joint.attrib["name"] for joint in joints}) != len(links) + len(joints):
        raise ValueError("link and joint names must be unique")

    incoming, world_anchor = _source_graph(joints, physical_names)
    root_links = [name for name in spec.physical_links if name not in incoming or incoming[name][1] == "world"]
    if root_links != ["base_link"]:
        raise ValueError(f"physical joint graph must have base_link as its only root: {root_links}")
    _validate_physical_tree(incoming, physical_names)

    expected_joints = set(spec.actuated_joints) | set(spec.fixed_joints)
    source_non_anchor = {joint.attrib["name"] for joint in joints if joint is not world_anchor}
    if source_non_anchor != expected_joints:
        raise ValueError("model specification does not match URDF joints")
    if world_anchor is not None and world_anchor.attrib["name"] != "world_to_base":
        raise ValueError("source world anchor must be named world_to_base")
    _validate_joint_types(joints, spec, world_anchor)

    sdf = Element("sdf", {"version": "1.11"})
    model = SubElement(sdf, "model", {"name": spec.slug, "canonical_link": "base_link"})
    mesh_sources: dict[str, str] = {}
    absolute_link_poses = _absolute_link_poses(incoming, physical_names)
    for name in spec.physical_links:
        _copy_link(model, link_by_name[name], absolute_link_poses[name], mesh_sources)

    if world_anchor is None:
        _add_world_anchor(model, "base_link")
    else:
        _copy_joint(model, world_anchor, overlay, world_anchor=True)
    for joint in joints:
        if joint is not world_anchor:
            _copy_joint(model, joint, overlay)
    return ElementTree.ElementTree(sdf)


def serialize_sdf(tree: ElementTree.ElementTree) -> bytes:
    """Serialize an SDF tree with stable whitespace and encoding."""
    ElementTree.indent(tree, space="    ")
    serialized = ElementTree.tostring(tree.getroot(), encoding="utf-8", xml_declaration=True)
    return serialized.rstrip(b"\n") + b"\n"
