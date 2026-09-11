import math
from dataclasses import replace
from pathlib import Path, PurePosixPath
from xml.etree import ElementTree
from xml.etree.ElementTree import SubElement

import pytest
from onerobotics_a1_gazebo.sdf import convert_urdf, serialize_sdf, source_joint_index
from onerobotics_a1_gazebo.spec import load_hardware_overlay, load_model_specs


def model_spec(key: str):
    return next(spec for spec in load_model_specs() if spec.key == key)


def converted_model(key: str):
    tree = convert_urdf(model_spec(key), load_hardware_overlay())
    return tree.getroot().find("model")


def mutated_spec(tmp_path: Path, key: str, mutate):
    spec = model_spec(key)
    tree = ElementTree.parse(spec.urdf)
    mutate(tree.getroot())
    urdf = tmp_path / f"{key}.urdf"
    tree.write(urdf, encoding="utf-8", xml_declaration=True)
    return replace(spec, urdf=urdf)


def convert_mutated(tmp_path: Path, key: str, mutate):
    return convert_urdf(mutated_spec(tmp_path, key, mutate), load_hardware_overlay())


def source_pose(origin):
    xyz = "0 0 0" if origin is None else origin.attrib.get("xyz", "0 0 0")
    rpy = "0 0 0" if origin is None else origin.attrib.get("rpy", "0 0 0")
    return f"{' '.join(xyz.split())} {' '.join(rpy.split())}"


def pose_transform(text: str):
    x, y, z, roll, pitch, yaw = (float(token) for token in text.split())
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation = (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )
    return ((x, y, z), rotation)


def compose_transforms(parent, child):
    parent_translation, parent_rotation = parent
    child_translation, child_rotation = child
    translation = tuple(
        parent_translation[row] + sum(parent_rotation[row][column] * child_translation[column] for column in range(3))
        for row in range(3)
    )
    rotation = tuple(
        tuple(
            sum(parent_rotation[row][inner] * child_rotation[inner][column] for inner in range(3))
            for column in range(3)
        )
        for row in range(3)
    )
    return translation, rotation


def source_absolute_link_transforms(spec):
    root = ElementTree.parse(spec.urdf).getroot()
    physical_names = set(spec.physical_links)
    incoming = {}
    for joint in root.findall("joint"):
        parent = joint.find("parent").attrib["link"]
        child = joint.find("child").attrib["link"]
        if child in physical_names:
            incoming[child] = (parent, pose_transform(source_pose(joint.find("origin"))))

    base_parent, base_origin = incoming.get("base_link", (None, pose_transform("0 0 0 0 0 0")))
    assert base_parent in (None, "world")
    absolute = {"base_link": base_origin}
    while len(absolute) < len(physical_names):
        before = len(absolute)
        for child, (parent, origin) in incoming.items():
            if parent in absolute and child not in absolute:
                absolute[child] = compose_transforms(absolute[parent], origin)
        assert len(absolute) > before
    return absolute


def test_right_arm_has_world_anchor_and_seven_revolute_joints():
    model = converted_model("right_arm")
    assert model.attrib["name"] == "onerobotics_a1_right_arm"
    assert model.attrib["canonical_link"] == "base_link"
    assert model.find("canonical_link") is None
    assert model.find("static") is None
    assert [link.attrib["name"] for link in model.findall("link")] == [
        "base_link",
        "Link1",
        "Link2",
        "Link3",
        "Link4",
        "Link5",
        "Link6",
        "Link7",
    ]
    anchor = model.find("joint[@name='world_to_base']")
    assert anchor is not None and anchor.attrib["type"] == "fixed"
    assert anchor.findtext("parent") == "world"
    assert anchor.findtext("child") == "base_link"
    assert len(model.findall("joint[@type='revolute']")) == 7


def test_left_arm_drops_only_virtual_world_link():
    model = converted_model("left_arm")
    assert model.find("link[@name='world']") is None
    assert len(model.findall("link")) == 8
    assert model.find("joint[@name='world_to_base']/parent").text == "world"


def test_bimanual_fixed_shoulders_are_not_lumped():
    model = converted_model("bimanual_stand")
    assert len(model.findall("link")) == 17
    assert len(model.findall("joint[@type='revolute']")) == 14
    assert {joint.attrib["name"] for joint in model.findall("joint[@type='fixed']")} == {
        "world_to_base",
        "joint_r0",
        "joint_l0",
    }
    assert model.find("link[@name='Link_r0']") is not None
    assert model.find("link[@name='Link_l0']") is not None


@pytest.mark.parametrize("key", ["right_arm", "left_arm", "bimanual_stand"])
def test_every_link_pose_is_explicitly_in_the_model_frame_for_static_viewers(key: str):
    model = converted_model(key)

    for link in model.findall("link"):
        pose = link.find("pose")
        assert pose is not None, link.attrib["name"]
        assert "relative_to" not in pose.attrib, link.attrib["name"]
        values = [float(token) for token in pose.text.split()]
        assert len(values) == 6, link.attrib["name"]
        assert all(math.isfinite(value) for value in values), link.attrib["name"]


def test_joint_pose_is_the_source_origin_and_child_has_absolute_model_pose():
    model = converted_model("right_arm")
    joint_pose = model.find("joint[@name='joint2-a1_r']/pose")
    assert joint_pose.attrib["relative_to"] == "Link1"
    assert joint_pose.text == "-0.0 0.0 0.12745 1.5708 1.5708 -0.0"
    child_pose = model.find("link[@name='Link2']/pose")
    assert "relative_to" not in child_pose.attrib
    assert [float(token) for token in child_pose.text.split()[:3]] == pytest.approx([0.0, 0.0, 0.13995])


@pytest.mark.parametrize("key", ["right_arm", "left_arm", "bimanual_stand"])
def test_absolute_link_poses_preserve_the_source_zero_configuration(key: str):
    spec = model_spec(key)
    model = converted_model(key)
    expected = source_absolute_link_transforms(spec)

    for link_name, (expected_translation, expected_rotation) in expected.items():
        pose = model.find(f"link[@name='{link_name}']/pose")
        assert pose is not None
        actual_translation, actual_rotation = pose_transform(pose.text)
        assert actual_translation == pytest.approx(expected_translation, abs=1e-12)
        for actual_row, expected_row in zip(actual_rotation, expected_rotation, strict=True):
            assert actual_row == pytest.approx(expected_row, abs=1e-9)


def test_mesh_and_color_are_portable_and_preserved():
    model = converted_model("right_arm")
    visual = model.find("link[@name='Link1']/visual[@name='Link1_visual_0']")
    assert visual.findtext("geometry/mesh/uri") == "meshes/Link1.STL"
    assert visual.findtext("material/ambient") == "0.898039215686275 0.917647058823529 0.929411764705882 1"
    assert visual.findtext("material/diffuse") == visual.findtext("material/ambient")


def test_unknown_leaf_attribute_is_rejected(tmp_path: Path):
    def add_attribute(root):
        root.find("link/inertial/mass").set("unknown", "value")

    with pytest.raises(ValueError, match="unsupported mass attribute"):
        convert_mutated(tmp_path, "right_arm", add_attribute)


def test_unknown_leaf_child_is_rejected(tmp_path: Path):
    def add_child(root):
        SubElement(root.find("link/inertial/mass"), "unknown")

    with pytest.raises(ValueError, match="unsupported mass child"):
        convert_mutated(tmp_path, "right_arm", add_child)


def test_nested_leaf_text_is_rejected(tmp_path: Path):
    def add_text(root):
        root.find("link/inertial/mass").text = "unexpected"

    with pytest.raises(ValueError, match="unsupported mass text"):
        convert_mutated(tmp_path, "right_arm", add_text)


@pytest.mark.parametrize("bad_axis", ["0 0 1", "0 0", "not numeric"])
def test_fixed_joint_rejects_nonzero_or_malformed_axis(tmp_path: Path, bad_axis: str):
    def alter_axis(root):
        root.find("joint[@name='joint_r0']/axis").set("xyz", bad_axis)

    with pytest.raises(ValueError, match="fixed joint joint_r0 axis"):
        convert_mutated(tmp_path, "bimanual_stand", alter_axis)


def test_fixed_joint_rejects_duplicate_axis(tmp_path: Path):
    def add_axis(root):
        SubElement(root.find("joint[@name='joint_r0']"), "axis", {"xyz": "0 0 0"})

    with pytest.raises(ValueError, match="at most one axis"):
        convert_mutated(tmp_path, "bimanual_stand", add_axis)


def test_fixed_joint_rejects_limit(tmp_path: Path):
    def add_limit(root):
        SubElement(
            root.find("joint[@name='joint_r0']"),
            "limit",
            {"lower": "0", "upper": "0", "effort": "0", "velocity": "0"},
        )

    with pytest.raises(ValueError, match="fixed joint joint_r0 cannot contain limit"):
        convert_mutated(tmp_path, "bimanual_stand", add_limit)


def test_zero_axis_exporter_exception_is_limited_to_bimanual_shoulders(tmp_path: Path):
    def add_axis(root):
        SubElement(root.find("joint[@name='world_to_base']"), "axis", {"xyz": "0 0 0"})

    with pytest.raises(ValueError, match="fixed joint world_to_base cannot contain axis"):
        convert_mutated(tmp_path, "left_arm", add_axis)


@pytest.mark.parametrize(
    "mutate_compiler",
    [
        lambda compiler: compiler.set("unknown", "true"),
        lambda compiler: compiler.set("meshdir", "unexpected"),
        lambda compiler: SubElement(compiler, "unknown"),
    ],
)
def test_unknown_mujoco_compiler_content_is_rejected(tmp_path: Path, mutate_compiler):
    def alter_compiler(root):
        mutate_compiler(root.find("mujoco/compiler"))

    with pytest.raises(ValueError, match="mujoco compiler"):
        convert_mutated(tmp_path, "right_arm", alter_compiler)


def test_dropped_virtual_world_link_is_still_strictly_validated(tmp_path: Path):
    def add_attribute(root):
        root.find("link[@name='world']").set("unknown", "value")

    with pytest.raises(ValueError, match="unsupported link world attribute"):
        convert_mutated(tmp_path, "left_arm", add_attribute)


def test_disconnected_joint_cycle_is_rejected(tmp_path: Path):
    def create_cycle(root):
        root.find("joint[@name='joint1-a1_r']/parent").set("link", "Link7")

    with pytest.raises(ValueError, match="connected acyclic tree"):
        convert_mutated(tmp_path, "right_arm", create_cycle)


def test_declared_actuated_joint_must_be_revolute(tmp_path: Path):
    def change_type(root):
        root.find("joint[@name='joint1-a1_r']").set("type", "fixed")

    with pytest.raises(ValueError, match="actuated joint joint1-a1_r must be revolute"):
        convert_mutated(tmp_path, "right_arm", change_type)


def test_declared_fixed_joint_must_be_fixed(tmp_path: Path):
    def change_type(root):
        root.find("joint[@name='joint_r0']").set("type", "revolute")

    with pytest.raises(ValueError, match="declared fixed joint joint_r0 must be fixed"):
        convert_mutated(tmp_path, "bimanual_stand", change_type)


def test_world_anchor_must_be_fixed(tmp_path: Path):
    def change_type(root):
        root.find("joint[@name='world_to_base']").set("type", "revolute")

    with pytest.raises(ValueError, match="world anchor world_to_base must be fixed"):
        convert_mutated(tmp_path, "left_arm", change_type)


@pytest.mark.parametrize("key", ["right_arm", "left_arm", "bimanual_stand"])
def test_all_source_link_dynamics_and_geometry_are_preserved(key: str):
    spec = model_spec(key)
    source = ElementTree.parse(spec.urdf).getroot()
    model = converted_model(key)

    for link_name in spec.physical_links:
        link_source = source.find(f"link[@name='{link_name}']")
        link_out = model.find(f"link[@name='{link_name}']")
        inertial_source = link_source.find("inertial")
        inertial_out = link_out.find("inertial")
        assert inertial_out.findtext("pose") == source_pose(inertial_source.find("origin"))
        assert inertial_out.findtext("mass") == inertial_source.find("mass").attrib["value"]
        for component in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
            assert inertial_out.findtext(f"inertia/{component}") == inertial_source.find("inertia").attrib[component]

        for output_tag in ("visual", "collision"):
            source_shapes = link_source.findall(output_tag)
            output_shapes = link_out.findall(output_tag)
            assert [shape.attrib["name"] for shape in output_shapes] == [
                f"{link_name}_{output_tag}_{index}" for index in range(len(source_shapes))
            ]
            for shape_source, shape_out in zip(source_shapes, output_shapes, strict=True):
                assert shape_out.findtext("pose") == source_pose(shape_source.find("origin"))
                mesh_source = shape_source.find("geometry/mesh")
                mesh_out = shape_out.find("geometry/mesh")
                basename = PurePosixPath(mesh_source.attrib["filename"].replace("\\", "/")).name
                assert mesh_out.findtext("uri") == f"meshes/{basename}"
                source_scale = mesh_source.attrib.get("scale")
                if source_scale is None:
                    assert mesh_out.find("scale") is None
                else:
                    assert mesh_out.findtext("scale") == " ".join(source_scale.split())
                if output_tag == "visual":
                    rgba = shape_source.find("material/color").attrib["rgba"]
                    assert shape_out.findtext("material/ambient") == rgba
                    assert shape_out.findtext("material/diffuse") == rgba


@pytest.mark.parametrize("key", ["right_arm", "left_arm", "bimanual_stand"])
def test_all_source_joint_edges_axes_positions_and_hardware_limits_are_preserved(key: str):
    spec = model_spec(key)
    source = ElementTree.parse(spec.urdf).getroot()
    model = converted_model(key)
    overlay = load_hardware_overlay()
    source_joints = {joint.attrib["name"]: joint for joint in source.findall("joint")}

    for joint_name in (*spec.fixed_joints, *spec.actuated_joints):
        joint_source = source_joints[joint_name]
        joint_out = model.find(f"joint[@name='{joint_name}']")
        parent = joint_source.find("parent").attrib["link"]
        child = joint_source.find("child").attrib["link"]
        assert joint_out.findtext("parent") == parent
        assert joint_out.findtext("child") == child
        assert joint_out.find("pose").attrib["relative_to"] == parent
        assert joint_out.findtext("pose") == source_pose(joint_source.find("origin"))
        if joint_name in spec.fixed_joints:
            assert joint_out.attrib["type"] == "fixed"
            assert joint_out.find("axis") is None
            continue

        assert joint_out.attrib["type"] == "revolute"
        assert joint_out.findtext("axis/xyz") == " ".join(joint_source.find("axis").attrib["xyz"].split())
        assert joint_out.findtext("axis/limit/lower") == joint_source.find("limit").attrib["lower"]
        assert joint_out.findtext("axis/limit/upper") == joint_source.find("limit").attrib["upper"]
        limits = overlay[source_joint_index(joint_name)]
        assert joint_out.findtext("axis/limit/effort") == str(limits.effort)
        assert joint_out.findtext("axis/limit/velocity") == str(limits.velocity)
        assert float(joint_out.findtext("axis/limit/effort")) > 0
        assert float(joint_out.findtext("axis/limit/velocity")) > 0


@pytest.mark.parametrize("key", ["right_arm", "left_arm", "bimanual_stand"])
def test_world_is_only_an_anchor_parent_and_never_a_pose_frame_or_child(key: str):
    model = converted_model(key)
    assert model.find("link[@name='world']") is None
    assert all(joint.findtext("child") != "world" for joint in model.findall("joint"))
    assert not model.findall(".//*[@relative_to='world']")
    anchors = [joint for joint in model.findall("joint") if joint.findtext("parent") == "world"]
    assert [anchor.attrib["name"] for anchor in anchors] == ["world_to_base"]
    assert anchors[0].findtext("child") == "base_link"
    assert anchors[0].find("pose").attrib["relative_to"] == "base_link"
    assert anchors[0].findtext("pose") == "0 0 0 0 0 0"


def test_nonidentity_left_world_anchor_is_folded_into_root_link_pose(tmp_path: Path):
    def move_anchor(root):
        origin = root.find("joint[@name='world_to_base']/origin")
        origin.set("xyz", "1.0  2.0 3.0")
        origin.set("rpy", "0.1 0.2  0.3")

    tree = convert_mutated(tmp_path, "left_arm", move_anchor)
    model = tree.getroot().find("model")
    base_pose = model.find("link[@name='base_link']/pose")
    anchor_pose = model.find("joint[@name='world_to_base']/pose")
    assert [float(token) for token in base_pose.text.split()] == pytest.approx([1.0, 2.0, 3.0, 0.1, 0.2, 0.3])
    assert "relative_to" not in base_pose.attrib
    assert anchor_pose.attrib["relative_to"] == "base_link"
    assert anchor_pose.text == "0 0 0 0 0 0"


def test_mesh_scale_is_preserved(tmp_path: Path):
    def add_scale(root):
        root.find("link[@name='Link1']/visual/geometry/mesh").set("scale", "1.0  2.0 3.0")

    tree = convert_mutated(tmp_path, "right_arm", add_scale)
    assert tree.findtext("model/link[@name='Link1']/visual/geometry/mesh/scale") == "1.0 2.0 3.0"


@pytest.mark.parametrize("key", ["right_arm", "left_arm", "bimanual_stand"])
def test_serialization_is_repeatable_utf8_four_space_lf_with_one_final_newline(key: str):
    spec = model_spec(key)
    overlay = load_hardware_overlay()
    tree = convert_urdf(spec, overlay)
    first = serialize_sdf(tree)
    assert first == serialize_sdf(tree)
    assert first == serialize_sdf(convert_urdf(spec, overlay))
    assert first.startswith(b"<?xml version='1.0' encoding='utf-8'?>\n")
    assert b"\r" not in first
    assert first.endswith(b"\n") and not first.endswith(b"\n\n")
    for line in first.splitlines()[1:]:
        indentation = len(line) - len(line.lstrip(b" "))
        assert indentation % 4 == 0


def test_malformed_root_is_rejected(tmp_path: Path):
    def change_root(root):
        root.tag = "not_robot"

    with pytest.raises(ValueError, match="URDF root must be <robot>"):
        convert_mutated(tmp_path, "right_arm", change_root)


@pytest.mark.parametrize(
    "xpath,replacement,duplicate_kind",
    [
        ("link[@name='Link1']", "base_link", "link"),
        ("joint[@name='joint2-a1_r']", "joint1-a1_r", "joint"),
    ],
)
def test_duplicate_names_are_rejected(tmp_path: Path, xpath: str, replacement: str, duplicate_kind: str):
    def duplicate_name(root):
        root.find(xpath).set("name", replacement)

    with pytest.raises(ValueError, match=rf"duplicate {duplicate_kind} name"):
        convert_mutated(tmp_path, "right_arm", duplicate_name)


def test_mesh_basename_collision_is_rejected(tmp_path: Path):
    def collide_mesh(root):
        root.find("link[@name='Link2']/visual/geometry/mesh").set("filename", "./different/Link1.STL")

    with pytest.raises(ValueError, match="mesh basename collision: Link1.STL"):
        convert_mutated(tmp_path, "right_arm", collide_mesh)


def test_unsupported_geometry_is_rejected(tmp_path: Path):
    def replace_geometry(root):
        root.find("link[@name='Link1']/visual/geometry/mesh").tag = "capsule"

    with pytest.raises(ValueError, match="unsupported geometry"):
        convert_mutated(tmp_path, "right_arm", replace_geometry)
