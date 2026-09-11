from __future__ import annotations

import copy
import gzip
import io
import json
import os
import shutil
import tarfile
from collections.abc import Callable
from dataclasses import replace
from pathlib import Path
from xml.etree import ElementTree
from xml.etree.ElementTree import SubElement

import onerobotics_a1_gazebo.thumbnail as thumbnail_module
import onerobotics_a1_gazebo.validate as validation_module
import pytest
from onerobotics_a1_gazebo.package import export_models
from onerobotics_a1_gazebo.spec import ModelSpec, load_model_specs, source_root
from onerobotics_a1_gazebo.thumbnail import approved_assets_root, capture_thumbnail_set
from onerobotics_a1_gazebo.validate import ValidationFailure, main, validate_all, validate_package

Mutation = Callable[[Path], None]


def _spec(key: str) -> ModelSpec:
    return next(spec for spec in load_model_specs() if spec.key == key)


def _exported_package(tmp_path: Path, key: str = "bimanual_stand") -> tuple[Path, ModelSpec]:
    output = tmp_path / "export"
    export_models(output)
    spec = _spec(key)
    return output / spec.slug, spec


@pytest.fixture(scope="module")
def validator_export(tmp_path_factory: pytest.TempPathFactory) -> Path:
    output = tmp_path_factory.mktemp("validator-export") / "models"
    export_models(output)
    return output


def _edit_sdf(package: Path, mutate: Callable[[ElementTree.Element], None]) -> None:
    path = package / "model.sdf"
    tree = ElementTree.parse(path)
    mutate(tree.getroot())
    tree.write(path, encoding="utf-8", xml_declaration=True)


def remove_one_mesh(package: Path) -> None:
    (package / "meshes" / "Link_l0.STL").unlink()


def zero_bimanual_effort(package: Path) -> None:
    _edit_sdf(
        package,
        lambda root: setattr(
            root.find("model/joint[@name='joint_r1']/axis/limit/effort"),
            "text",
            "0",
        ),
    )


def remove_fixed_shoulder(package: Path) -> None:
    def mutate(root: ElementTree.Element) -> None:
        model = root.find("model")
        model.remove(model.find("joint[@name='joint_r0']"))

    _edit_sdf(package, mutate)


def make_inertia_non_positive(package: Path) -> None:
    _edit_sdf(
        package,
        lambda root: setattr(
            root.find("model/link[@name='Link_r1']/inertial/inertia/ixx"),
            "text",
            "0",
        ),
    )


def insert_absolute_uri(package: Path) -> None:
    _edit_sdf(
        package,
        lambda root: setattr(
            root.find(".//geometry/mesh/uri"),
            "text",
            "/tmp/attacker-controlled.stl",
        ),
    )


def remove_cc_by_metadata(package: Path) -> None:
    path = package / "metadata.pbtxt"
    path.write_text(
        path.read_text(encoding="utf-8").replace(
            'license: "Creative Commons Attribution 4.0 International"',
            'license: "unknown"',
        ),
        encoding="utf-8",
    )


def corrupt_manifest_digest(package: Path) -> None:
    path = package / "SOURCE_MANIFEST.json"
    manifest = json.loads(path.read_text(encoding="utf-8"))
    manifest["sources"][0]["sha256"] = "0" * 64
    path.write_text(json.dumps(manifest), encoding="utf-8")


@pytest.mark.parametrize(
    ("mutate", "message"),
    [
        (remove_one_mesh, "missing mesh"),
        (zero_bimanual_effort, "effort limit mismatch"),
        (remove_fixed_shoulder, "fixed-joint topology mismatch"),
        (make_inertia_non_positive, "inertia is not positive definite"),
        (insert_absolute_uri, "non-portable URI"),
        (remove_cc_by_metadata, "CC BY 4.0 metadata missing"),
        (corrupt_manifest_digest, "package source digest mismatch"),
    ],
)
def test_validator_rejects_corruption(tmp_path: Path, mutate: Mutation, message: str) -> None:
    package, spec = _exported_package(tmp_path)
    mutate(package)

    issues = validate_package(package, spec)

    assert any(message in issue.message for issue in issues)


def test_validator_accepts_all_fresh_exports_and_prints_stable_summary(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    output = tmp_path / "export"
    export_models(output)

    validate_all(output)

    assert capsys.readouterr().out == (
        "VALID onerobotics_a1_right_arm: 8 links, 7 actuated joints, 8 meshes\n"
        "VALID onerobotics_a1_left_arm: 8 links, 7 actuated joints, 8 meshes\n"
        "VALID onerobotics_a1_bimanual_stand: 17 links, 14 actuated joints, 17 meshes\n"
        "VALIDATION_OK: 3 models, 33 meshes, 28 actuated joints\n"
    )


def test_validator_reports_malformed_xml_instead_of_crashing(tmp_path: Path) -> None:
    package, spec = _exported_package(tmp_path, "right_arm")
    (package / "model.sdf").write_text("<sdf><model>", encoding="utf-8")

    issues = validate_package(package, spec)

    assert any("malformed model.sdf" in issue.message for issue in issues)


@pytest.mark.parametrize("encoding", ["no-such", "UTF-32"])
def test_cli_converts_unsupported_xml_encoding_to_failure(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
    encoding: str,
) -> None:
    output = tmp_path / "export"
    export_models(output)
    spec = _spec("right_arm")
    (output / spec.slug / "model.sdf").write_bytes(f'<?xml version="1.0" encoding="{encoding}"?><sdf/>'.encode())

    status = main([str(output)])
    captured = capsys.readouterr()

    assert status == 1
    assert "malformed model.sdf" in captured.err
    assert "VALIDATION_OK" not in captured.out


@pytest.mark.parametrize("value", ["nan", "inf", "-inf", "not-a-number"])
def test_validator_rejects_nonfinite_or_malformed_mass(tmp_path: Path, value: str) -> None:
    package, spec = _exported_package(tmp_path, "right_arm")
    _edit_sdf(
        package,
        lambda root: setattr(root.find("model/link/inertial/mass"), "text", value),
    )

    issues = validate_package(package, spec)

    assert any("mass must be a finite number" in issue.message for issue in issues)


def test_validator_rejects_parent_traversal_uri_without_reading_it(tmp_path: Path) -> None:
    package, spec = _exported_package(tmp_path, "right_arm")
    outside = package.parent / "outside.STL"
    outside.write_bytes(b"attacker controlled")
    _edit_sdf(
        package,
        lambda root: setattr(root.find(".//geometry/mesh/uri"), "text", "../outside.STL"),
    )

    issues = validate_package(package, spec)

    assert any("non-portable URI" in issue.message for issue in issues)


def test_validator_rejects_nonportable_uri_outside_mesh_geometry(tmp_path: Path) -> None:
    package, spec = _exported_package(tmp_path, "right_arm")

    def mutate(root: ElementTree.Element) -> None:
        include = SubElement(root.find("model"), "include")
        SubElement(include, "uri").text = "file:///tmp/attacker-controlled.sdf"

    _edit_sdf(package, mutate)

    issues = validate_package(package, spec)

    assert any("non-portable URI" in issue.message for issue in issues)


def test_validator_rejects_mesh_symlink_before_reading_target(tmp_path: Path) -> None:
    package, spec = _exported_package(tmp_path, "right_arm")
    mesh = package / "meshes" / "base_link.STL"
    target = tmp_path / "outside.STL"
    target.write_bytes(b"attacker controlled")
    mesh.unlink()
    mesh.symlink_to(target)

    issues = validate_package(package, spec)

    assert any("symlink" in issue.message for issue in issues)


def test_validator_rejects_symlinked_package_without_traversal(tmp_path: Path) -> None:
    package, spec = _exported_package(tmp_path, "right_arm")
    alias = tmp_path / "package-alias"
    alias.symlink_to(package, target_is_directory=True)

    issues = validate_package(alias, spec)

    assert any("package path must not be a symlink" in issue.message for issue in issues)


def test_validator_rejects_unreferenced_symlink_in_package_root(tmp_path: Path) -> None:
    package, spec = _exported_package(tmp_path, "right_arm")
    (package / "attacker-link").symlink_to("/etc/passwd")

    issues = validate_package(package, spec)

    assert any("symlink" in issue.message for issue in issues)


def test_validate_all_accumulates_unsafe_leaf_and_manifest_corruption(tmp_path: Path) -> None:
    output = tmp_path / "export"
    export_models(output)
    spec = _spec("right_arm")
    package = output / spec.slug
    (package / "attacker-link").symlink_to("/etc/passwd")
    corrupt_manifest_digest(package)

    with pytest.raises(ValidationFailure) as failure:
        validate_all(output)

    owned = [issue for issue in failure.value.issues if issue.model == spec.slug]
    assert sum("symlink" in issue.message for issue in owned) == 1
    assert any("package source digest mismatch" in issue.message for issue in owned)
    assert not any(issue.model == "<export>" and "symlink" in issue.message for issue in failure.value.issues)


def test_validator_rejects_plugins_static_models_sensors_and_ros_references(tmp_path: Path) -> None:
    package, spec = _exported_package(tmp_path, "right_arm")

    def mutate(root: ElementTree.Element) -> None:
        model = root.find("model")
        SubElement(model, "static").text = "true"
        SubElement(model, "plugin", {"name": "ros_control", "filename": "libgazebo_ros.so"})
        SubElement(model.find("link"), "sensor", {"name": "camera", "type": "camera"})

    _edit_sdf(package, mutate)

    messages = {issue.message for issue in validate_package(package, spec)}

    assert any("published model must not contain <static>" in message for message in messages)
    assert any("published model must not contain plugins" in message for message in messages)
    assert any("published model must not contain sensors" in message for message in messages)
    assert any("published model must not contain ROS dependencies" in message for message in messages)


def test_validator_rejects_forbidden_content_outside_model_element(tmp_path: Path) -> None:
    package, spec = _exported_package(tmp_path, "right_arm")

    def mutate(root: ElementTree.Element) -> None:
        SubElement(root, "plugin", {"name": "attacker", "filename": "libattacker.so"})

    _edit_sdf(package, mutate)

    issues = validate_package(package, spec)

    assert any("published model must not contain plugins" in issue.message for issue in issues)


def test_validator_reports_forbidden_content_when_model_count_is_wrong(tmp_path: Path) -> None:
    package, spec = _exported_package(tmp_path, "right_arm")

    def mutate(root: ElementTree.Element) -> None:
        extra = SubElement(root, "model", {"name": "attacker"})
        SubElement(extra, "plugin", {"name": "attacker", "filename": "libattacker.so"})
        include = SubElement(extra, "include")
        SubElement(include, "uri").text = "/tmp/attacker-controlled.sdf"

    _edit_sdf(package, mutate)

    messages = [issue.message for issue in validate_package(package, spec)]

    assert any("exactly one model" in message for message in messages)
    assert any("must not contain plugins" in message for message in messages)
    assert any("non-portable URI" in message for message in messages)
    assert not any("missing mesh reference" in message for message in messages)


def test_validator_rejects_source_physical_field_changes(tmp_path: Path) -> None:
    package, spec = _exported_package(tmp_path, "right_arm")
    _edit_sdf(
        package,
        lambda root: setattr(root.find("model/link/inertial/mass"), "text", "1.0"),
    )

    issues = validate_package(package, spec)

    assert any("mass mismatch" in issue.message for issue in issues)


def test_validator_rejects_corrupted_package_mesh_bytes(tmp_path: Path) -> None:
    package, spec = _exported_package(tmp_path, "right_arm")
    mesh = package / "meshes" / "base_link.STL"
    mesh.write_bytes(mesh.read_bytes() + b"corruption")

    issues = validate_package(package, spec)

    assert any("package mesh digest mismatch" in issue.message for issue in issues)


def test_validator_rejects_live_source_tree_that_no_longer_matches_lock(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    package, original_spec = _exported_package(tmp_path, "right_arm")
    copied_source = tmp_path / "source-copy"
    shutil.copytree(source_root(), copied_source)
    copied_urdf = copied_source / original_spec.urdf.relative_to(source_root())
    copied_urdf.write_text(
        copied_urdf.read_text(encoding="utf-8").replace(
            '<mass value="0.401650168942009" />',
            '<mass value="0.501650168942009" />',
            1,
        ),
        encoding="utf-8",
    )
    spec = replace(original_spec, urdf=copied_urdf)
    monkeypatch.setattr(validation_module, "source_root", lambda: copied_source)

    issues = validate_package(package, spec)

    assert any("immutable source lock mismatch" in issue.message for issue in issues)


@pytest.mark.parametrize("field", ["source_url", "source_commit", "changes"])
def test_validator_requires_exact_manifest_provenance(tmp_path: Path, field: str) -> None:
    package, spec = _exported_package(tmp_path, "right_arm")
    path = package / "SOURCE_MANIFEST.json"
    manifest = json.loads(path.read_text(encoding="utf-8"))
    manifest[field] = "attacker controlled"
    path.write_text(json.dumps(manifest), encoding="utf-8")

    issues = validate_package(package, spec)

    assert any(f"source manifest {field} mismatch" in issue.message for issue in issues)


@pytest.mark.parametrize(
    ("relative", "message"),
    [
        ("model.config", "model.config metadata mismatch"),
        ("README.md", "README provenance metadata mismatch"),
    ],
)
def test_validator_requires_exact_package_metadata(tmp_path: Path, relative: str, message: str) -> None:
    package, spec = _exported_package(tmp_path, "right_arm")
    path = package / relative
    path.write_text(path.read_text(encoding="utf-8") + "attacker controlled\n", encoding="utf-8")

    issues = validate_package(package, spec)

    assert any(message in issue.message for issue in issues)


def test_validator_rejects_manifest_path_traversal(tmp_path: Path) -> None:
    package, spec = _exported_package(tmp_path, "right_arm")
    path = package / "SOURCE_MANIFEST.json"
    manifest = json.loads(path.read_text(encoding="utf-8"))
    manifest["sources"][0]["path"] = "../outside"
    path.write_text(json.dumps(manifest), encoding="utf-8")

    issues = validate_package(package, spec)

    assert any("non-portable source manifest path" in issue.message for issue in issues)


def test_validator_reports_multiple_safely_discoverable_issues(tmp_path: Path) -> None:
    package, spec = _exported_package(tmp_path, "right_arm")
    (package / "model.sdf").write_text("<not-sdf/>", encoding="utf-8")
    (package / "metadata.pbtxt").write_text("invalid", encoding="utf-8")
    (package / "NOTICE").unlink()

    messages = [issue.message for issue in validate_package(package, spec)]

    assert any("SDF root mismatch" in message for message in messages)
    assert any("CC BY 4.0 metadata missing" in message for message in messages)
    assert any("missing required file: NOTICE" in message for message in messages)


def test_validator_rejects_non_regular_metadata_file(tmp_path: Path) -> None:
    package, spec = _exported_package(tmp_path, "right_arm")
    metadata = package / "metadata.pbtxt"
    metadata.unlink()
    os.mkfifo(metadata)

    issues = validate_package(package, spec)

    assert any("required file is not regular: metadata.pbtxt" in issue.message for issue in issues)


def test_validate_all_rejects_unexpected_export_root_entry(tmp_path: Path) -> None:
    output = tmp_path / "export"
    export_models(output)
    (output / "attacker-extra").mkdir()

    with pytest.raises(ValidationFailure) as failure:
        validate_all(output)

    assert any("unexpected export root entry" in issue.message for issue in failure.value.issues)


def test_validate_all_rejects_corrupted_export_marker(tmp_path: Path) -> None:
    output = tmp_path / "export"
    export_models(output)
    (output / ".onerobotics-a1-gazebo-export.json").write_text("attacker controlled\n", encoding="utf-8")

    with pytest.raises(ValidationFailure) as failure:
        validate_all(output)

    assert any("export marker mismatch" in issue.message for issue in failure.value.issues)


def _right_archive(output: Path) -> Path:
    return output / "archives/onerobotics_a1_right_arm-1.0.0.tar.gz"


@pytest.mark.parametrize(
    ("mutate", "message"),
    [
        (lambda output, tmp_path: _right_archive(output).unlink(), "missing archive"),
        (
            lambda output, tmp_path: (output / "archives/attacker-extra.tar.gz").write_bytes(b"attacker"),
            "unexpected archive",
        ),
        (
            lambda output, tmp_path: (
                _right_archive(output).unlink(),
                _right_archive(output).symlink_to(tmp_path / "outside.tar.gz"),
            ),
            "symlink",
        ),
        (
            lambda output, tmp_path: (
                _right_archive(output).unlink(),
                os.mkfifo(_right_archive(output)),
            ),
            "regular file",
        ),
    ],
)
def test_validate_all_rejects_bad_archive_inventory(
    tmp_path: Path,
    mutate: Callable[[Path, Path], object],
    message: str,
) -> None:
    output = tmp_path / "export"
    export_models(output)
    mutate(output, tmp_path)

    with pytest.raises(ValidationFailure) as failure:
        validate_all(output)

    messages = [issue.message for issue in failure.value.issues]
    assert any(message in issue for issue in messages)
    if message in {"symlink", "regular file"}:
        assert not any("missing archive" in issue for issue in messages)


def test_validate_all_reports_corrupt_archive_without_crashing(tmp_path: Path) -> None:
    output = tmp_path / "export"
    export_models(output)
    _right_archive(output).write_bytes(b"not a gzip archive")

    with pytest.raises(ValidationFailure) as failure:
        validate_all(output)

    assert any("malformed archive" in issue.message for issue in failure.value.issues)


def test_validate_all_bounds_orphan_archive_member_iteration(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    output = tmp_path / "export"
    export_models(output)
    spec = _spec("right_arm")
    shutil.rmtree(output / spec.slug)
    monkeypatch.setattr(validation_module, "_MAX_ORPHAN_ARCHIVE_MEMBERS", 2)
    buffer = io.BytesIO()
    with gzip.GzipFile(filename="", mode="wb", fileobj=buffer, mtime=0) as zipped:
        with tarfile.open(fileobj=zipped, mode="w", format=tarfile.PAX_FORMAT) as archive:
            for index in range(3):
                info = tarfile.TarInfo(f"{spec.slug}-1.0.0/extra-{index}")
                info.type = tarfile.DIRTYPE
                info.mode = 0o755
                info.mtime = 0
                archive.addfile(info)
    _right_archive(output).write_bytes(buffer.getvalue())

    with pytest.raises(ValidationFailure) as failure:
        validate_all(output)

    assert any("archive exceeds member limit" in issue.message for issue in failure.value.issues)


def test_validate_all_reports_corrupt_gzip_payload_without_crashing(tmp_path: Path) -> None:
    output = tmp_path / "export"
    export_models(output)
    archive = _right_archive(output)
    corrupted = bytearray(archive.read_bytes())
    # Corrupt the gzip CRC deterministically. Flipping a byte in the compressed
    # stream can still produce a valid stream whose tar member merely differs,
    # and the exact midpoint changes whenever model.sdf changes size.
    corrupted[-8] ^= 0xFF
    archive.write_bytes(corrupted)

    with pytest.raises(ValidationFailure) as failure:
        validate_all(output)

    assert any("malformed archive" in issue.message for issue in failure.value.issues)


def test_cli_converts_invalid_deflate_payload_to_stderr(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    output = tmp_path / "export"
    export_models(output)
    _right_archive(output).write_bytes(bytes.fromhex("1f8b08000000000002ff") + b"garbage")

    status = main([str(output)])
    captured = capsys.readouterr()

    assert status == 1
    assert "malformed archive" in captured.err
    assert "VALIDATION_OK" not in captured.out


def test_validate_all_rejects_gzip_expansion_bomb_before_unbounded_decode(tmp_path: Path) -> None:
    output = tmp_path / "export"
    export_models(output)
    archive = _right_archive(output)
    expected_size = len(gzip.decompress(archive.read_bytes()))
    archive.write_bytes(gzip.compress(b"x" * (expected_size + 1), mtime=0))

    with pytest.raises(ValidationFailure) as failure:
        validate_all(output)

    assert any("archive expands beyond expected size" in issue.message for issue in failure.value.issues)


def test_validate_all_rejects_archive_for_the_wrong_model(tmp_path: Path) -> None:
    output = tmp_path / "export"
    export_models(output)
    _right_archive(output).write_bytes((output / "archives/onerobotics_a1_left_arm-1.0.0.tar.gz").read_bytes())

    with pytest.raises(ValidationFailure) as failure:
        validate_all(output)

    assert any("archive member root mismatch" in issue.message for issue in failure.value.issues)


def test_validator_converts_deep_json_recursion_to_issue(tmp_path: Path) -> None:
    package, spec = _exported_package(tmp_path, "right_arm")
    (package / "SOURCE_MANIFEST.json").write_text("[" * 2000 + "0" + "]" * 2000, encoding="utf-8")

    issues = validate_package(package, spec)

    assert any("SOURCE_MANIFEST.json exceeds nesting limit" in issue.message for issue in issues)


def test_validator_converts_json_integer_limit_error_to_issue(tmp_path: Path) -> None:
    package, spec = _exported_package(tmp_path, "right_arm")
    (package / "SOURCE_MANIFEST.json").write_text('{"schema_version":' + "1" * 5000 + "}", encoding="utf-8")

    issues = validate_package(package, spec)

    assert any("malformed SOURCE_MANIFEST.json" in issue.message for issue in issues)


def _make_deep_model_sdf(package: Path, spec: ModelSpec) -> None:
    depth = 2000
    (package / "model.sdf").write_text(
        f'<sdf version="1.11"><model name="{spec.slug}" canonical_link="base_link">'
        + "<nested>" * depth
        + "</nested>" * depth
        + "</model></sdf>",
        encoding="utf-8",
    )


def test_validator_converts_deep_xml_recursion_to_issue(tmp_path: Path) -> None:
    package, spec = _exported_package(tmp_path, "right_arm")
    _make_deep_model_sdf(package, spec)

    issues = validate_package(package, spec)

    assert any("model.sdf exceeds nesting limit" in issue.message for issue in issues)


def test_cli_returns_one_and_stderr_for_deep_malformed_input(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    output = tmp_path / "export"
    export_models(output)
    spec = _spec("right_arm")
    _make_deep_model_sdf(output / spec.slug, spec)

    status = main([str(output)])
    captured = capsys.readouterr()

    assert status == 1
    assert "model.sdf exceeds nesting limit" in captured.err
    assert "VALIDATION_OK" not in captured.out


def test_multi_model_accumulates_global_and_expected_model_semantic_issues(tmp_path: Path) -> None:
    package, spec = _exported_package(tmp_path, "right_arm")

    def mutate(root: ElementTree.Element) -> None:
        expected = root.find(f"model[@name='{spec.slug}']")
        expected.find("link/inertial/mass").text = "nan"
        expected.find("link/inertial/inertia/ixx").text = "0"
        extra = SubElement(root, "model", {"name": "attacker"})
        SubElement(extra, "plugin", {"name": "attacker", "filename": "libattacker.so"})

    _edit_sdf(package, mutate)

    messages = [issue.message for issue in validate_package(package, spec)]

    assert any("exactly one model" in message for message in messages)
    assert any("must not contain plugins" in message for message in messages)
    assert any("mass must be a finite number" in message for message in messages)
    assert any("inertia is not positive definite" in message for message in messages)


def test_duplicate_expected_models_validate_local_physics_on_every_model(tmp_path: Path) -> None:
    package, spec = _exported_package(tmp_path, "right_arm")

    def mutate(root: ElementTree.Element) -> None:
        duplicate = copy.deepcopy(root.find(f"model[@name='{spec.slug}']"))
        duplicate.find("link/inertial/inertia/ixx").text = "0"
        limit = duplicate.find(f"joint[@name='{spec.actuated_joints[0]}']/axis/limit")
        limit.find("lower").text = "1"
        limit.find("upper").text = "-1"
        root.append(duplicate)

    _edit_sdf(package, mutate)

    messages = [issue.message for issue in validate_package(package, spec)]

    assert any("exactly one model" in message for message in messages)
    assert any("inertia is not positive definite" in message for message in messages)
    assert any("lower limit must be less than upper limit" in message for message in messages)
    assert any("position limit must contain zero" in message for message in messages)


def test_models_without_unique_expected_name_still_validate_local_physics(tmp_path: Path) -> None:
    package, spec = _exported_package(tmp_path, "right_arm")

    def mutate(root: ElementTree.Element) -> None:
        first = root.find("model")
        second = copy.deepcopy(first)
        first.attrib["name"] = "attacker-one"
        second.attrib["name"] = "attacker-two"
        second.find("link/inertial/mass").text = "-1"
        limit = second.find(f"joint[@name='{spec.actuated_joints[0]}']/axis/limit")
        limit.find("lower").text = "1"
        limit.find("upper").text = "2"
        second.find(".//geometry/mesh/uri").text = "meshes/attacker.STL"
        root.append(second)

    _edit_sdf(package, mutate)

    messages = [issue.message for issue in validate_package(package, spec)]

    assert any("exactly one model" in message for message in messages)
    assert any("mass must be positive" in message for message in messages)
    assert any("position limit must contain zero" in message for message in messages)
    assert any("unexpected mesh reference" in message for message in messages)


def test_nested_model_still_validates_local_physics(tmp_path: Path) -> None:
    package, spec = _exported_package(tmp_path, "right_arm")

    def mutate(root: ElementTree.Element) -> None:
        nested = SubElement(root.find("model"), "model", {"name": "attacker"})
        link = SubElement(nested, "link", {"name": "attacker-link"})
        inertial = SubElement(link, "inertial")
        SubElement(inertial, "mass").text = "-1"

    _edit_sdf(package, mutate)

    messages = [issue.message for issue in validate_package(package, spec)]

    assert any("mass must be positive" in message for message in messages)


def test_validator_rejects_overflowing_non_positive_inertia(tmp_path: Path) -> None:
    package, spec = _exported_package(tmp_path, "right_arm")

    def mutate(root: ElementTree.Element) -> None:
        inertia = root.find("model/link/inertial/inertia")
        for component, value in {
            "ixx": "1e308",
            "ixy": "1e308",
            "ixz": "0",
            "iyy": "1e308",
            "iyz": "0",
            "izz": "1e308",
        }.items():
            inertia.find(component).text = value

    _edit_sdf(package, mutate)

    issues = validate_package(package, spec)

    assert any("inertia is not positive definite" in issue.message for issue in issues)


@pytest.mark.parametrize("mutation", ["missing-dir", "missing-file", "extra", "symlink", "fifo"])
def test_validator_rejects_unsafe_or_nonexact_thumbnail_inventory(tmp_path: Path, mutation: str) -> None:
    package, spec = _exported_package(tmp_path, "right_arm")
    thumbnails = package / "thumbnails"
    image = thumbnails / "0.png"
    if mutation == "missing-dir":
        shutil.rmtree(thumbnails)
    elif mutation == "missing-file":
        image.unlink()
    elif mutation == "extra":
        (thumbnails / "extra.png").write_bytes(image.read_bytes())
    elif mutation == "symlink":
        image.unlink()
        image.symlink_to(approved_assets_root() / "onerobotics_a1_right_arm.png")
    else:
        image.unlink()
        os.mkfifo(image)

    messages = [issue.message for issue in validate_package(package, spec)]

    assert any("thumbnail" in message.lower() or "thumbnails" in message.lower() for message in messages)
    assert any(
        token in message.lower()
        for message in messages
        for token in ("missing", "unexpected", "symlink", "regular", "inventory")
    )


def test_validator_rejects_corrupt_and_wrong_model_thumbnail_bytes(tmp_path: Path) -> None:
    approved = capture_thumbnail_set(approved_assets_root())
    package, spec = _exported_package(tmp_path, "right_arm")
    image = package / "thumbnails/0.png"
    image.write_bytes(b"not a PNG")

    corrupt_messages = [issue.message for issue in validate_package(package, spec)]

    assert any("thumbnail" in message.lower() and "png" in message.lower() for message in corrupt_messages)

    image.write_bytes(approved.png_by_slug["onerobotics_a1_left_arm"])
    mismatch_messages = [issue.message for issue in validate_package(package, spec)]

    assert any(
        "approved thumbnail" in message.lower() and "mismatch" in message.lower() for message in mismatch_messages
    )


def test_validator_binds_package_model_sdf_raw_bytes_to_approved_manifest(tmp_path: Path) -> None:
    package, spec = _exported_package(tmp_path, "right_arm")
    model_sdf = package / "model.sdf"
    model_sdf.write_bytes(model_sdf.read_bytes() + b"\n")

    messages = [issue.message for issue in validate_package(package, spec)]

    assert any("thumbnail manifest model.sdf hash mismatch" in message.lower() for message in messages)


@pytest.mark.parametrize("operation", ["all", "package"])
@pytest.mark.parametrize(
    "mutation",
    ["missing-root", "extra", "symlink", "fifo", "corrupt-manifest", "corrupt-png"],
)
def test_validator_wiring_fails_closed_for_invalid_approved_asset_roots(
    validator_export: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    operation: str,
    mutation: str,
) -> None:
    approved = tmp_path / "approved"
    if mutation != "missing-root":
        shutil.copytree(approved_assets_root(), approved)
        right = approved / "onerobotics_a1_right_arm.png"
        if mutation == "extra":
            (approved / "extra.png").write_bytes(b"unexpected")
        elif mutation == "symlink":
            right.unlink()
            right.symlink_to(approved / "onerobotics_a1_left_arm.png")
        elif mutation == "fifo":
            right.unlink()
            os.mkfifo(right)
        elif mutation == "corrupt-manifest":
            (approved / "render-manifest.json").write_bytes(b"{not-json")
        elif mutation == "corrupt-png":
            right.write_bytes(b"not-a-png")
    monkeypatch.setattr(validation_module, "approved_assets_root", lambda: approved)

    if operation == "all":
        with pytest.raises(ValidationFailure) as caught:
            validate_all(validator_export)
        messages = [issue.message for issue in caught.value.issues]
    else:
        spec = _spec("right_arm")
        messages = [issue.message for issue in validate_package(validator_export / spec.slug, spec)]

    assert any("approved thumbnail snapshot invalid" in message.lower() for message in messages)


def test_validate_all_captures_approved_thumbnails_once(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    output = tmp_path / "export"
    export_models(output)
    real_capture = capture_thumbnail_set
    calls = 0

    def counted_capture(*args, **kwargs):
        nonlocal calls
        calls += 1
        return real_capture(*args, **kwargs)

    monkeypatch.setattr(validation_module, "capture_thumbnail_set", counted_capture)

    validate_all(output)

    assert calls == 1


@pytest.mark.parametrize("operation", ["all", "package"])
def test_validator_passes_one_source_snapshot_model_hashes_into_approved_thumbnail_capture(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    operation: str,
) -> None:
    output = tmp_path / "export"
    export_models(output)
    real_capture = capture_thumbnail_set
    captured_model_hashes: list[dict[str, str]] = []

    def capture_with_fixed_model_hashes(*args, **kwargs):
        model_hashes = kwargs.get("expected_model_sdf_sha256")
        assert model_hashes is not None
        assert set(model_hashes) == {spec.slug for spec in load_model_specs()}
        captured_model_hashes.append(dict(model_hashes))
        return real_capture(*args, **kwargs)

    def forbid_second_live_source_derivation():
        raise AssertionError("validator must not rederive model hashes from live source")

    monkeypatch.setattr(validation_module, "capture_thumbnail_set", capture_with_fixed_model_hashes)
    monkeypatch.setattr(thumbnail_module, "current_model_sdf_sha256", forbid_second_live_source_derivation)

    if operation == "all":
        validate_all(output)
    else:
        spec = _spec("right_arm")
        assert validate_package(output / spec.slug, spec) == ()

    assert len(captured_model_hashes) == 1
