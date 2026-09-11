from pathlib import Path

import pytest
import yaml
from onerobotics_a1_gazebo.spec import load_hardware_overlay, load_model_specs


def _write_modified_model_config(tmp_path: Path, modify) -> Path:
    source = Path(__file__).resolve().parents[1] / "config/models.yaml"
    config = yaml.safe_load(source.read_text(encoding="utf-8"))
    modify(config)
    destination = tmp_path / "models.yaml"
    destination.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
    return destination


def test_three_atomic_model_specs_are_complete():
    specs = load_model_specs()
    assert [spec.slug for spec in specs] == [
        "onerobotics_a1_right_arm",
        "onerobotics_a1_left_arm",
        "onerobotics_a1_bimanual_stand",
    ]
    assert [len(spec.physical_links) for spec in specs] == [8, 8, 17]
    assert [len(spec.actuated_joints) for spec in specs] == [7, 7, 14]
    assert specs[2].fixed_joints == ("joint_r0", "joint_l0")
    assert specs[0].actuator_groups == (
        ("joint1-a1_r", "joint2-a1_r", "joint3-a1_r"),
        ("joint4-a1_r", "joint5-a1_r", "joint6-a1_r", "joint7-a1_r"),
    )


def test_hardware_overlay_matches_public_yaml():
    overlay = load_hardware_overlay()
    assert overlay[1].effort == 26.859
    assert overlay[1].velocity == 2.6179938779914944
    assert overlay[4].effort == 5.975
    assert overlay[4].velocity == 12.566370614359172
    assert set(overlay) == set(range(1, 8))


def test_model_specs_reject_urdf_outside_the_public_source_root(tmp_path: Path):
    config = _write_modified_model_config(
        tmp_path,
        lambda config: config["models"][0].update(urdf="/etc/passwd"),
    )

    with pytest.raises(ValueError, match="relative .urdf path"):
        load_model_specs(config)


def test_model_specs_reject_missing_required_model(tmp_path: Path):
    config = _write_modified_model_config(tmp_path, lambda config: config["models"].pop())

    with pytest.raises(ValueError, match="exactly three public models"):
        load_model_specs(config)


def test_model_specs_reject_substituted_model_identity(tmp_path: Path):
    config = _write_modified_model_config(
        tmp_path,
        lambda config: config["models"][1].update(key="substituted_left_arm"),
    )

    with pytest.raises(ValueError, match="does not match the required public model schema"):
        load_model_specs(config)


def test_model_specs_reject_altered_bimanual_fixed_topology(tmp_path: Path):
    config = _write_modified_model_config(
        tmp_path,
        lambda config: config["models"][2].update(fixed_joints=["joint_r0"]),
    )

    with pytest.raises(ValueError, match="does not match the required public model schema"):
        load_model_specs(config)


def test_model_specs_reject_mixed_side_actuator_groups(tmp_path: Path):
    config = _write_modified_model_config(
        tmp_path,
        lambda config: config["models"][0].update(
            actuator_groups=[
                ["joint1-a1_l", "joint2-a1_l", "joint3-a1_l"],
                ["joint4-a1_l", "joint5-a1_l", "joint6-a1_l", "joint7-a1_l"],
            ]
        ),
    )

    with pytest.raises(ValueError, match="does not match the required public model schema"):
        load_model_specs(config)
