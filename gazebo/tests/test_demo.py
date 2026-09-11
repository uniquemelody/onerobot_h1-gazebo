from __future__ import annotations

import hashlib
import os
import subprocess
from pathlib import Path
from types import SimpleNamespace
from xml.etree import ElementTree

import pytest
from onerobotics_a1_gazebo import demo
from onerobotics_a1_gazebo.demo import build_demo_world, generate_worlds, world_filename
from onerobotics_a1_gazebo.package import export_models
from onerobotics_a1_gazebo.sdf import serialize_sdf
from onerobotics_a1_gazebo.spec import ModelSpec, load_model_specs

EXPECTED_WORLD_FILES = {
    "onerobotics_a1_right_arm_demo.sdf",
    "onerobotics_a1_left_arm_demo.sdf",
    "onerobotics_a1_bimanual_stand_demo.sdf",
}
WORLD_SYSTEMS = [
    ("gz-sim-physics-system", "gz::sim::systems::Physics"),
    ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands"),
    ("gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster"),
]
GROUND_POSES = {
    "onerobotics_a1_right_arm": "0 0 -0.025 0 0 0",
    "onerobotics_a1_left_arm": "0 0 -0.025 0 0 0",
    "onerobotics_a1_bimanual_stand": "0 0 -0.25 0 0 0",
}


@pytest.fixture(scope="module")
def model_specs() -> tuple[ModelSpec, ...]:
    return load_model_specs()


@pytest.fixture()
def export_root(tmp_path: Path) -> Path:
    root = tmp_path / "models"
    export_models(root)
    return root


def _hashes(root: Path) -> dict[str, str]:
    return {
        path.relative_to(root).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(root.rglob("*"))
        if path.is_file()
    }


def _world_document(path: Path) -> tuple[ElementTree.Element, ElementTree.Element]:
    root = ElementTree.parse(path).getroot()
    world = root.find("world")
    assert world is not None
    return root, world


def test_generation_is_exact_deterministic_and_idempotent(
    export_root: Path,
    tmp_path: Path,
    model_specs: tuple[ModelSpec, ...],
) -> None:
    output = tmp_path / "worlds"

    first = generate_worlds(models_root=export_root, output_root=output)
    first_bytes = {path.name: path.read_bytes() for path in first}
    second = generate_worlds(models_root=export_root, output_root=output)

    assert {path.name for path in first} == EXPECTED_WORLD_FILES
    assert tuple(path.name for path in first) == tuple(world_filename(spec) for spec in model_specs)
    assert first_bytes == {path.name: path.read_bytes() for path in second}
    assert set(path.name for path in output.iterdir()) == EXPECTED_WORLD_FILES
    for spec in model_specs:
        assert (output / world_filename(spec)).read_bytes() == serialize_sdf(build_demo_world(spec))


@pytest.mark.parametrize("spec_index", range(3))
def test_world_has_exact_native_structure(
    export_root: Path,
    tmp_path: Path,
    model_specs: tuple[ModelSpec, ...],
    spec_index: int,
) -> None:
    spec = model_specs[spec_index]
    output = tmp_path / f"worlds-{spec_index}"
    generate_worlds(models_root=export_root, output_root=output)
    root, world = _world_document(output / world_filename(spec))

    assert root.tag == "sdf"
    assert root.attrib == {"version": "1.11"}
    assert list(root) == [world]
    assert world.attrib == {"name": "a1_demo"}
    assert world.findtext("gravity") == "0 0 -9.8"
    physics = world.find("physics")
    assert physics is not None
    assert physics.attrib == {"name": "one_millisecond", "type": "ignored"}
    assert physics.findtext("max_step_size") == "0.001"
    assert physics.findtext("real_time_update_rate") == "1000"

    systems = [(plugin.attrib.get("filename"), plugin.attrib.get("name")) for plugin in world.findall("plugin")]
    assert systems == WORLD_SYSTEMS
    assert world.find("plugin[@filename='gz-sim-physics-system']/engine/filename").text == ("gz-physics-dartsim-plugin")

    ground_models = world.findall("model")
    assert len(ground_models) == 1
    ground = ground_models[0]
    assert ground.attrib == {"name": "ground_plane"}
    assert ground.findtext("static") == "true"
    assert ground.findtext("pose") == GROUND_POSES[spec.slug]
    assert ground.findtext("link/collision/geometry/plane/normal") == "0 0 1"
    assert ground.findtext("link/collision/geometry/plane/size") == "100 100"
    assert ground.findtext("link/visual/geometry/plane/normal") == "0 0 1"
    assert ground.findtext("link/visual/geometry/plane/size") == "100 100"

    lights = world.findall("light")
    assert len(lights) == 1
    assert lights[0].attrib == {"name": "sun", "type": "directional"}
    assert lights[0].findtext("direction") == "-0.5 0.1 -0.9"

    includes = world.findall("include")
    assert len(includes) == 1
    include = includes[0]
    assert include.findtext("uri") == f"model://{spec.slug}"
    assert include.findtext("name") == spec.slug
    assert [element.text for element in root.iter("uri")] == [f"model://{spec.slug}"]


@pytest.mark.parametrize("spec_index", range(3))
def test_include_scopes_exact_controller_and_state_plugins(
    export_root: Path,
    tmp_path: Path,
    model_specs: tuple[ModelSpec, ...],
    spec_index: int,
) -> None:
    spec = model_specs[spec_index]
    output = tmp_path / f"worlds-{spec_index}"
    generate_worlds(models_root=export_root, output_root=output)
    _, world = _world_document(output / world_filename(spec))
    include = world.find("include")
    assert include is not None

    controllers = include.findall("plugin[@filename='gz-sim-joint-position-controller-system']")
    assert len(controllers) == len(spec.actuated_joints)
    assert [plugin.attrib.get("name") for plugin in controllers] == ["gz::sim::systems::JointPositionController"] * len(
        spec.actuated_joints
    )
    for plugin, joint in zip(controllers, spec.actuated_joints, strict=True):
        assert [(child.tag, child.text) for child in plugin] == [
            ("joint_name", joint),
            ("joint_index", "0"),
            ("topic", f"/a1_demo/{spec.slug}/{joint}/cmd_pos"),
            ("use_velocity_commands", "true"),
            ("cmd_max", "0.1"),
            ("cmd_min", "-0.1"),
            ("initial_position", "0"),
        ]

    publishers = include.findall("plugin[@filename='gz-sim-joint-state-publisher-system']")
    assert len(publishers) == 1
    publisher = publishers[0]
    assert publisher.attrib.get("name") == "gz::sim::systems::JointStatePublisher"
    assert [(child.tag, child.text) for child in publisher] == [("joint_name", joint) for joint in spec.actuated_joints]
    assert publisher.find("update_rate") is None
    assert all("pd" not in (text or "").lower() for text in root_texts(world))


def root_texts(root: ElementTree.Element) -> tuple[str | None, ...]:
    return tuple(element.text for element in root.iter())


def test_generation_validates_models_without_mutating_published_packages(export_root: Path, tmp_path: Path) -> None:
    before = _hashes(export_root)

    generate_worlds(models_root=export_root, output_root=tmp_path / "worlds")

    assert _hashes(export_root) == before
    for model_sdf in export_root.glob("onerobotics_a1_*/model.sdf"):
        assert ElementTree.parse(model_sdf).getroot().find("model/plugin") is None


def test_generation_rejects_invalid_package_before_creating_output(export_root: Path, tmp_path: Path) -> None:
    model_sdf = export_root / "onerobotics_a1_right_arm" / "model.sdf"
    model_sdf.write_text("<sdf/>", encoding="utf-8")
    output = tmp_path / "worlds"

    with pytest.raises(ValueError, match="model export validation failed"):
        generate_worlds(models_root=export_root, output_root=output)

    assert not output.exists()


def test_generation_rejects_symlink_model_root(export_root: Path, tmp_path: Path) -> None:
    linked = tmp_path / "linked-models"
    linked.symlink_to(export_root, target_is_directory=True)

    with pytest.raises(ValueError, match="snapshot|symlink"):
        generate_worlds(models_root=linked, output_root=tmp_path / "worlds")


def test_generation_rejects_output_root_symlink(export_root: Path, tmp_path: Path) -> None:
    target = tmp_path / "target"
    target.mkdir()
    output = tmp_path / "worlds"
    output.symlink_to(target, target_is_directory=True)

    with pytest.raises(ValueError, match="output.*symlink"):
        generate_worlds(models_root=export_root, output_root=output)


@pytest.mark.parametrize("entry_kind", ["symlink", "fifo"])
def test_generation_rejects_unsafe_existing_output_entry(
    export_root: Path,
    tmp_path: Path,
    entry_kind: str,
) -> None:
    output = tmp_path / "worlds"
    output.mkdir()
    entry = output / "onerobotics_a1_right_arm_demo.sdf"
    if entry_kind == "symlink":
        entry.symlink_to(tmp_path / "outside")
    else:
        os.mkfifo(entry)

    with pytest.raises(ValueError, match="output.*regular file"):
        generate_worlds(models_root=export_root, output_root=output)


def test_generation_rejects_extra_output_entry(export_root: Path, tmp_path: Path) -> None:
    output = tmp_path / "worlds"
    output.mkdir()
    (output / "unexpected.sdf").write_text("do not delete me", encoding="utf-8")

    with pytest.raises(ValueError, match="unexpected output entry"):
        generate_worlds(models_root=export_root, output_root=output)

    assert (output / "unexpected.sdf").read_text(encoding="utf-8") == "do not delete me"


def test_generation_does_not_follow_output_directory_swapped_to_symlink(
    export_root: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    output = tmp_path / "worlds"
    output.mkdir()
    displaced = tmp_path / "displaced-worlds"
    outside = tmp_path / "outside"
    outside.mkdir()
    real_replace = os.replace
    swapped = False

    def swap_root_then_replace(source: object, destination: object, *args: object, **kwargs: object) -> None:
        nonlocal swapped
        if not swapped:
            swapped = True
            output.rename(displaced)
            output.symlink_to(outside, target_is_directory=True)
        real_replace(source, destination, *args, **kwargs)

    monkeypatch.setattr(demo.os, "replace", swap_root_then_replace)

    with pytest.raises(ValueError, match="output directory.*(?:changed|replaced)"):
        generate_worlds(models_root=export_root, output_root=output)

    assert list(outside.iterdir()) == []


def test_generated_worlds_pass_real_sdformat_14(
    export_root: Path,
    tmp_path: Path,
) -> None:
    output = tmp_path / "worlds"
    generated = generate_worlds(models_root=export_root, output_root=output)
    helper = Path(__file__).resolve().parents[1] / "scripts/harmonic_env.sh"
    script = 'source "$1"; shift; a1_harmonic_run "$@"'

    for world in generated:
        result = subprocess.run(
            [
                "bash",
                "-c",
                script,
                "a1-harmonic-sdf-check",
                str(helper),
                "env",
                f"GZ_SIM_RESOURCE_PATH={export_root.resolve()}",
                f"SDF_PATH={export_root.resolve()}",
                "gz",
                "sdf",
                "--force-version",
                "14",
                "-k",
                str(world),
            ],
            check=False,
            capture_output=True,
            text=True,
            timeout=20,
            env={key: value for key, value in os.environ.items() if key != "PYTHONPATH"},
        )
        assert result.returncode == 0, result.stdout + result.stderr


def test_live_cases_cover_standalone_models_and_both_bimanual_halves() -> None:
    assert demo._LIVE_CASES == (
        ("onerobotics_a1_right_arm", "joint1-a1_r", 0.05),
        ("onerobotics_a1_left_arm", "joint1-a1_l", 0.05),
        ("onerobotics_a1_bimanual_stand", "joint_r1", 0.05),
        ("onerobotics_a1_bimanual_stand", "joint_l1", 0.05),
    )


def test_cache_smoke_cli_prints_only_after_all_motion_passes(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
    tmp_path: Path,
) -> None:
    calls: list[dict[str, Path]] = []

    def smoke_cache_all(**kwargs: Path) -> tuple[SimpleNamespace, ...]:
        calls.append(kwargs)
        return tuple(
            SimpleNamespace(
                model=model,
                joint=joint,
                initial_position=0.0,
                final_position=0.03,
                delta=0.03,
            )
            for model, joint, _ in demo._LIVE_CASES
        )

    monkeypatch.setattr(demo, "smoke_cache_all", smoke_cache_all, raising=False)
    trusted = tmp_path / "trusted"
    cached = tmp_path / "cached"
    worlds = tmp_path / "worlds"

    status = demo.main(
        [
            "smoke-cache-all",
            "--trusted-models",
            str(trusted),
            "--cache-models",
            str(cached),
            "--worlds",
            str(worlds),
        ]
    )

    captured = capsys.readouterr()
    assert status == 0
    assert calls == [{"trusted_models": trusted, "cache_models": cached, "worlds_root": worlds}]
    assert captured.out.count("CACHE_SMOKE_TEST_VALID") == 4
    assert "CACHE_SMOKE_TEST_OK: 4 cases across 3 models" in captured.out


def test_cache_smoke_cli_defers_every_success_line_until_all_motion_passes(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
    tmp_path: Path,
) -> None:
    monkeypatch.setattr(
        demo,
        "smoke_cache_all",
        lambda **kwargs: (_ for _ in ()).throw(RuntimeError("cache third motion failed")),
        raising=False,
    )

    status = demo.main(
        [
            "smoke-cache-all",
            "--trusted-models",
            str(tmp_path / "trusted"),
            "--cache-models",
            str(tmp_path / "cached"),
            "--worlds",
            str(tmp_path / "worlds"),
        ]
    )

    captured = capsys.readouterr()
    assert status == 1
    assert "CACHE_SMOKE_TEST_VALID" not in captured.out
    assert "CACHE_SMOKE_TEST_OK" not in captured.out
    assert "CACHE_SMOKE_TEST_ERROR: cache third motion failed" in captured.err
