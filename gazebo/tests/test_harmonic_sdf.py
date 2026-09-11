from __future__ import annotations

import importlib
import math
from collections.abc import Callable
from pathlib import Path
from xml.etree import ElementTree
from xml.etree.ElementTree import Element, SubElement

import pytest
from onerobotics_a1_gazebo.package import export_models
from onerobotics_a1_gazebo.spec import ModelSpec, load_model_specs

Mutation = Callable[[Element], None]
ACTUATED_CASES = tuple((spec.key, joint_name) for spec in load_model_specs() for joint_name in spec.actuated_joints)


def _harmonic_module():
    return importlib.import_module("onerobotics_a1_gazebo.harmonic")


def _spec(key: str) -> ModelSpec:
    return next(spec for spec in load_model_specs() if spec.key == key)


@pytest.fixture(scope="module")
def export_root(tmp_path_factory: pytest.TempPathFactory) -> Path:
    root = tmp_path_factory.mktemp("harmonic-semantic") / "export"
    export_models(root)
    return root


def _documents(export_root: Path, tmp_path: Path, key: str = "bimanual_stand") -> tuple[Path, Path, ModelSpec]:
    spec = _spec(key)
    packaged = export_root / spec.slug / "model.sdf"
    parsed = tmp_path / "pretty parsed.sdf"
    parsed.write_bytes(packaged.read_bytes())
    return packaged, parsed, spec


def _mutate(path: Path, mutation: Mutation) -> None:
    root = ElementTree.parse(path).getroot()
    mutation(root)
    ElementTree.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def _model(root: Element) -> Element:
    model = root.find("model")
    assert model is not None
    return model


def _issues(packaged: Path, parsed: Path, spec: ModelSpec) -> tuple[str, ...]:
    return _harmonic_module().validate_parsed_sdf(packaged, parsed, spec)


def test_semantic_gate_accepts_numeric_equivalent_pretty_sdf(export_root: Path, tmp_path: Path) -> None:
    packaged, parsed, spec = _documents(export_root, tmp_path, "right_arm")

    def rewrite(root: Element) -> None:
        value = _model(root).find("joint[@name='joint1-a1_r']/axis/limit/lower")
        assert value is not None and value.text is not None
        value.text = f"{float(value.text):.17g}"

    _mutate(parsed, rewrite)

    assert _issues(packaged, parsed, spec) == ()


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        (lambda root: _model(root).set("name", "wrong-model"), "model name"),
        (lambda root: _model(root).set("canonical_link", "Link_r1"), "canonical link"),
        (lambda root: _model(root).remove(_model(root).find("link[@name='Link_r7']")), "link inventory"),
        (lambda root: SubElement(_model(root), "link", {"name": "extra_link"}), "link inventory"),
        (lambda root: _model(root).remove(_model(root).find("joint[@name='joint_r7']")), "joint inventory"),
        (
            lambda root: SubElement(_model(root), "joint", {"name": "extra_joint", "type": "fixed"}),
            "joint inventory",
        ),
        (lambda root: _model(root).find("joint[@name='joint_r1']").set("type", "prismatic"), "joint type"),
        (
            lambda root: setattr(_model(root).find("joint[@name='joint_r1']/parent"), "text", "base_link"),
            "joint parent",
        ),
        (
            lambda root: setattr(_model(root).find("joint[@name='joint_r1']/child"), "text", "Link_r2"),
            "joint child",
        ),
        (
            lambda root: setattr(_model(root).find("joint[@name='world_to_base']/parent"), "text", "Link_r0"),
            "world anchor",
        ),
        (lambda root: _model(root).find("joint[@name='world_to_base']").set("type", "revolute"), "world anchor"),
        (
            lambda root: setattr(_model(root).find("joint[@name='world_to_base']/child"), "text", "Link_r0"),
            "world anchor",
        ),
        (lambda root: _model(root).find("joint[@name='joint_r0']").set("type", "revolute"), "fixed shoulder"),
        (lambda root: _model(root).find("joint[@name='joint_l0']").set("type", "revolute"), "fixed shoulder"),
    ],
)
def test_semantic_gate_rejects_topology_mutations(
    export_root: Path,
    tmp_path: Path,
    mutation: Mutation,
    message: str,
) -> None:
    packaged, parsed, spec = _documents(export_root, tmp_path)
    _mutate(parsed, mutation)

    issues = _issues(packaged, parsed, spec)

    assert any(message in issue.lower() for issue in issues), issues


@pytest.mark.parametrize(("key", "joint_name"), ACTUATED_CASES)
@pytest.mark.parametrize("field", ["lower", "upper", "effort", "velocity"])
def test_semantic_gate_rejects_one_double_step_in_every_actuated_limit_field(
    export_root: Path,
    tmp_path: Path,
    key: str,
    joint_name: str,
    field: str,
) -> None:
    packaged, parsed, spec = _documents(export_root, tmp_path, key)

    def change_limit(root: Element) -> None:
        value = _model(root).find(f"joint[@name='{joint_name}']/axis/limit/{field}")
        assert value is not None and value.text is not None
        value.text = repr(math.nextafter(float(value.text), math.inf))

    _mutate(parsed, change_limit)

    issues = _issues(packaged, parsed, spec)

    assert any(field in issue and joint_name in issue for issue in issues), issues


def test_semantic_gate_requires_one_direct_model(export_root: Path, tmp_path: Path) -> None:
    packaged, parsed, spec = _documents(export_root, tmp_path, "right_arm")

    def nest_model(root: Element) -> None:
        model = root.find("model")
        assert model is not None
        root.remove(model)
        world = SubElement(root, "world", {"name": "default"})
        world.append(model)

    _mutate(parsed, nest_model)

    assert any("one direct" in issue.lower() for issue in _issues(packaged, parsed, spec))


def test_semantic_gate_rejects_nested_model(export_root: Path, tmp_path: Path) -> None:
    packaged, parsed, spec = _documents(export_root, tmp_path, "right_arm")

    def add_nested(root: Element) -> None:
        SubElement(_model(root), "model", {"name": "hidden"})

    _mutate(parsed, add_nested)

    assert any("nested model" in issue.lower() for issue in _issues(packaged, parsed, spec))


def test_semantic_gate_rejects_model_nested_under_top_level_world(export_root: Path, tmp_path: Path) -> None:
    packaged, parsed, spec = _documents(export_root, tmp_path, "right_arm")

    def add_world_model(root: Element) -> None:
        world = SubElement(root, "world", {"name": "hidden-world"})
        SubElement(world, "model", {"name": "hidden-model"})

    _mutate(parsed, add_world_model)

    assert any("exactly one model in the document" in issue.lower() for issue in _issues(packaged, parsed, spec))


def test_semantic_gate_rejects_other_top_level_sdf_resources(export_root: Path, tmp_path: Path) -> None:
    packaged, parsed, spec = _documents(export_root, tmp_path, "right_arm")

    def add_light(root: Element) -> None:
        SubElement(root, "light", {"name": "hidden-light", "type": "point"})

    _mutate(parsed, add_light)

    assert any("only direct model" in issue.lower() for issue in _issues(packaged, parsed, spec))


@pytest.mark.parametrize("value", ["nan", "inf", "-inf", "not-a-number"])
def test_semantic_gate_rejects_nonfinite_or_malformed_limit_as_controlled_issue(
    export_root: Path, tmp_path: Path, value: str
) -> None:
    packaged, parsed, spec = _documents(export_root, tmp_path, "right_arm")

    def change_limit(root: Element) -> None:
        lower = _model(root).find("joint[@name='joint1-a1_r']/axis/limit/lower")
        assert lower is not None
        lower.text = value

    _mutate(parsed, change_limit)

    issues = _issues(packaged, parsed, spec)

    assert issues
    assert any("finite numeric" in issue.lower() for issue in issues)


def test_semantic_gate_reports_malformed_xml_without_traceback(export_root: Path, tmp_path: Path) -> None:
    packaged, parsed, spec = _documents(export_root, tmp_path, "right_arm")
    parsed.write_text("<sdf><model>", encoding="utf-8")

    issues = _issues(packaged, parsed, spec)

    assert any("malformed" in issue.lower() for issue in issues)


def test_semantic_gate_rejects_excessive_nesting_as_controlled_issue(export_root: Path, tmp_path: Path) -> None:
    packaged, parsed, spec = _documents(export_root, tmp_path, "right_arm")
    parsed.write_text("<sdf>" + "<x>" * 300 + "</x>" * 300 + "</sdf>", encoding="utf-8")

    issues = _issues(packaged, parsed, spec)

    assert any("nesting" in issue.lower() for issue in issues)


def test_semantic_cli_success_is_quiet(export_root: Path, tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    packaged, parsed, spec = _documents(export_root, tmp_path, "right_arm")

    status = _harmonic_module().main([str(packaged), str(parsed), spec.slug])

    captured = capsys.readouterr()
    assert status == 0
    assert captured.out == ""
    assert captured.err == ""


def test_semantic_cli_failure_is_actionable_and_stable(
    export_root: Path, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    packaged, parsed, spec = _documents(export_root, tmp_path, "right_arm")
    parsed.write_text("not XML", encoding="utf-8")

    status = _harmonic_module().main([str(packaged), str(parsed), spec.slug])

    captured = capsys.readouterr()
    assert status == 1
    assert captured.out == ""
    assert captured.err.startswith(f"HARMONIC_SEMANTIC_ERROR {spec.slug}: ")
    assert "malformed" in captured.err.lower()
    assert "traceback" not in captured.err.lower()


def test_semantic_cli_rejects_unknown_slug_without_traceback(
    export_root: Path, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    packaged, parsed, _ = _documents(export_root, tmp_path, "right_arm")

    status = _harmonic_module().main([str(packaged), str(parsed), "unknown-model"])

    captured = capsys.readouterr()
    assert status == 1
    assert "unknown model slug" in captured.err.lower()
