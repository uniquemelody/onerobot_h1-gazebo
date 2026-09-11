from __future__ import annotations

import json
import os
import queue
import shutil
import tarfile
import threading
from dataclasses import replace
from pathlib import Path

import pytest
from onerobotics_a1_gazebo import package as package_module
from onerobotics_a1_gazebo.package import create_reproducible_archive, export_models
from onerobotics_a1_gazebo.thumbnail import approved_assets_root, capture_thumbnail_set

REPO_ROOT = Path(__file__).resolve().parents[2]


def _tree_bytes(root: Path) -> dict[str, bytes]:
    return {path.relative_to(root).as_posix(): path.read_bytes() for path in sorted(root.rglob("*")) if path.is_file()}


def test_exported_packages_are_atomic_and_self_contained(tmp_path: Path) -> None:
    exported = export_models(tmp_path / "dist")

    assert [item.directory.name for item in exported] == [
        "onerobotics_a1_right_arm",
        "onerobotics_a1_left_arm",
        "onerobotics_a1_bimanual_stand",
    ]
    assert [len(list((item.directory / "meshes").glob("*.STL"))) for item in exported] == [8, 8, 17]
    for item in exported:
        assert {
            "model.config",
            "model.sdf",
            "metadata.pbtxt",
            "README.md",
            "LICENSE",
            "NOTICE",
            "SOURCE_MANIFEST.json",
        } <= {path.name for path in item.directory.iterdir()}
        assert not any(path.is_symlink() for path in item.directory.rglob("*"))
        sdf = (item.directory / "model.sdf").read_text(encoding="utf-8")
        assert str(REPO_ROOT) not in sdf
        assert "../" not in sdf


def test_package_provenance_and_license_are_explicit(tmp_path: Path) -> None:
    item = export_models(tmp_path / "dist")[0]
    notice = (item.directory / "NOTICE").read_text(encoding="utf-8")
    metadata = (item.directory / "metadata.pbtxt").read_text(encoding="utf-8")

    assert "Copyright © 2026 OneRobotics" in notice
    assert "CC BY 4.0" in notice
    assert "ecf530911284ba0e559f7a24dc222fd8e60d31ed" in notice
    assert "URDF-to-SDF conversion" in notice
    assert "Creative Commons Attribution 4.0" in metadata


def test_fuel_metadata_has_exact_required_fields(tmp_path: Path) -> None:
    item = export_models(tmp_path / "dist")[0]
    metadata = (item.directory / "metadata.pbtxt").read_text(encoding="utf-8")

    assert 'name: "OneRobotics A1 Right Arm"' in metadata
    assert not any(line.startswith("version:") for line in metadata.splitlines())
    assert 'authors {\n  name: "OneRobotics"\n}' in metadata
    assert 'file: "model.sdf"' in metadata
    assert "version { major: 1 minor: 11 }" in metadata
    assert 'name: "sdf"' in metadata
    assert 'copyright: "Copyright © 2026 OneRobotics"' in metadata
    assert 'license: "Creative Commons Attribution 4.0 International"' in metadata
    assert [line.strip() for line in metadata.splitlines() if line.strip().startswith("tags:")] == [
        'tags: "robot"',
        'tags: "manipulator"',
        'tags: "onerobotics"',
    ]
    assert "email" not in metadata


def test_source_manifest_records_only_inputs_copied_into_package(tmp_path: Path) -> None:
    item = export_models(tmp_path / "dist")[0]
    manifest = json.loads((item.directory / "SOURCE_MANIFEST.json").read_text(encoding="utf-8"))

    assert manifest["schema_version"] == 1
    assert manifest["source_url"] == "https://github.com/katazen/onerobot_h1"
    assert manifest["source_commit"] == "ecf530911284ba0e559f7a24dc222fd8e60d31ed"
    assert manifest["model_key"] == "right_arm"
    assert "native SDFormat 1.11" in manifest["changes"]
    paths = [source["path"] for source in manifest["sources"]]
    assert paths == sorted(paths)
    assert paths == [
        "a1_r.urdf",
        "meshes/a1_r/Link1.STL",
        "meshes/a1_r/Link2.STL",
        "meshes/a1_r/Link3.STL",
        "meshes/a1_r/Link4.STL",
        "meshes/a1_r/Link5.STL",
        "meshes/a1_r/Link6.STL",
        "meshes/a1_r/Link7.STL",
        "meshes/a1_r/base_link.STL",
        "model_parameters.yaml",
    ]
    assert all(len(source["sha256"]) == 64 for source in manifest["sources"])


def test_archives_are_byte_reproducible(tmp_path: Path) -> None:
    first = export_models(tmp_path / "first")
    second = export_models(tmp_path / "second")

    assert [item.archive_sha256 for item in first] == [item.archive_sha256 for item in second]
    assert [item.archive.read_bytes() for item in first] == [item.archive.read_bytes() for item in second]


def test_archive_has_normalized_safe_members(tmp_path: Path) -> None:
    item = export_models(tmp_path / "dist")[0]

    with tarfile.open(item.archive, "r:gz") as archive:
        members = archive.getmembers()
    assert members
    assert [member.name for member in members] == sorted(member.name for member in members)
    assert all(member.name.startswith(f"{item.directory.name}-1.0.0/") for member in members)
    assert all(not Path(member.name).is_absolute() and ".." not in Path(member.name).parts for member in members)
    assert all(member.uid == member.gid == member.mtime == 0 for member in members)
    assert all(member.uname == member.gname == "" for member in members)
    assert all(member.mode == (0o755 if member.isdir() else 0o644) for member in members)
    assert all(member.isfile() or member.isdir() for member in members)


@pytest.mark.parametrize("dangerous", [Path("/"), Path.home(), REPO_ROOT])
def test_export_refuses_dangerous_output_roots(dangerous: Path) -> None:
    with pytest.raises(ValueError, match="Refusing dangerous output root"):
        export_models(dangerous)


def test_export_preserves_an_unrecognized_existing_directory(tmp_path: Path) -> None:
    output_root = tmp_path / "dist"
    output_root.mkdir()
    sentinel = output_root / "user-data.txt"
    sentinel.write_text("keep me", encoding="utf-8")

    with pytest.raises(ValueError, match="not a generated export"):
        export_models(output_root)

    assert sentinel.read_text(encoding="utf-8") == "keep me"


def test_staging_install_failure_restores_previous_generated_tree(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    output_root = tmp_path / "dist"
    export_models(output_root)
    (output_root / "previous.txt").write_text("previous export", encoding="utf-8")
    previous = _tree_bytes(output_root)
    real_replace = os.replace

    def fail_staging_install(source: Path, destination: Path) -> None:
        if Path(source).name.startswith(f".{output_root.name}.stage-") and Path(destination) == output_root:
            raise OSError("injected staging install failure")
        real_replace(source, destination)

    monkeypatch.setattr(package_module.os, "replace", fail_staging_install)

    with pytest.raises(OSError, match="injected staging install failure"):
        export_models(output_root)

    assert _tree_bytes(output_root) == previous
    assert not list(tmp_path.glob(f".{output_root.name}.backup-*"))


def test_backup_cleanup_failure_leaves_new_export_authoritative_and_old_tree_recoverable(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    output_root = tmp_path / "dist"
    export_models(output_root)
    (output_root / "previous.txt").write_text("previous export", encoding="utf-8")
    real_rmtree = package_module.shutil.rmtree

    def fail_backup_cleanup(path: Path, *args, **kwargs) -> None:
        if Path(path).name.startswith(f".{output_root.name}.backup-"):
            raise OSError("injected backup cleanup failure")
        real_rmtree(path, *args, **kwargs)

    monkeypatch.setattr(package_module.shutil, "rmtree", fail_backup_cleanup)

    exported = export_models(output_root)

    assert len(exported) == 3
    assert not (output_root / "previous.txt").exists()
    backups = list(tmp_path.glob(f".{output_root.name}.backup-*"))
    assert len(backups) == 1
    assert (backups[0] / "previous.txt").read_text(encoding="utf-8") == "previous export"
    assert all(item.directory.is_dir() and item.archive.is_file() for item in exported)


def test_export_uses_one_locked_snapshot_after_initial_validation(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    baseline = export_models(tmp_path / "baseline")[0]
    expected_sdf = (baseline.directory / "model.sdf").read_bytes()
    expected_mesh = (baseline.directory / "meshes/Link1.STL").read_bytes()
    expected_manifest = (baseline.directory / "SOURCE_MANIFEST.json").read_bytes()

    public_root = package_module.source_root()
    mutable_source = tmp_path / "mutable-source"
    shutil.copytree(public_root, mutable_source)
    mutable_specs = tuple(
        replace(spec, urdf=mutable_source / spec.urdf.relative_to(public_root))
        for spec in package_module.load_model_specs()
    )
    real_validate = package_module.validate_source_lock
    real_load_overlay = package_module.load_hardware_overlay
    mutation_injected = False

    def validate_mutable_or_snapshot(*, lock=None, source_root=None) -> None:
        real_validate(lock=lock, source_root=source_root or mutable_source)

    def mutate_after_initial_check(parameters_path: Path | None = None):
        nonlocal mutation_injected
        if not mutation_injected:
            mutation_injected = True
            urdf = mutable_source / "a1_r.urdf"
            urdf.write_text(
                urdf.read_text(encoding="utf-8").replace(
                    '<mass value="0.401650168942009" />',
                    '<mass value="0.501650168942009" />',
                    1,
                ),
                encoding="utf-8",
            )
            mesh = mutable_source / "meshes/a1_r/Link1.STL"
            mesh.write_bytes(mesh.read_bytes() + b"injected mutation")
            parameters = mutable_source / "model_parameters.yaml"
            parameters.write_text(
                parameters.read_text(encoding="utf-8").replace(
                    "peak_output_torque_nm: 26.859", "peak_output_torque_nm: 25.0", 1
                ),
                encoding="utf-8",
            )
        return real_load_overlay(parameters_path or mutable_source / "model_parameters.yaml")

    monkeypatch.setattr(package_module, "source_root", lambda: mutable_source)
    monkeypatch.setattr(package_module, "load_model_specs", lambda: mutable_specs)
    monkeypatch.setattr(package_module, "validate_source_lock", validate_mutable_or_snapshot)
    monkeypatch.setattr(package_module, "load_hardware_overlay", mutate_after_initial_check)

    exported = export_models(tmp_path / "mutated")[0]

    assert mutation_injected
    assert "0.501650168942009" in (mutable_source / "a1_r.urdf").read_text(encoding="utf-8")
    assert (mutable_source / "meshes/a1_r/Link1.STL").read_bytes().endswith(b"injected mutation")
    assert "peak_output_torque_nm: 25.0" in (mutable_source / "model_parameters.yaml").read_text(encoding="utf-8")
    assert (exported.directory / "model.sdf").read_bytes() == expected_sdf
    assert (exported.directory / "meshes/Link1.STL").read_bytes() == expected_mesh
    assert (exported.directory / "SOURCE_MANIFEST.json").read_bytes() == expected_manifest


def test_standalone_archive_refuses_to_overwrite_model_tree(tmp_path: Path) -> None:
    model_dir = tmp_path / "model"
    model_dir.mkdir()
    (model_dir / "model.sdf").write_text("sdf", encoding="utf-8")

    with pytest.raises(ValueError, match="outside the model directory"):
        create_reproducible_archive(model_dir, model_dir / "archive.tar.gz")


def test_archive_rejects_symlink_entry_without_leaving_partial_output(tmp_path: Path) -> None:
    model_dir = tmp_path / "model"
    model_dir.mkdir()
    target = tmp_path / "outside.txt"
    target.write_text("outside", encoding="utf-8")
    (model_dir / "link.txt").symlink_to(target)
    archive = tmp_path / "model.tar.gz"

    with pytest.raises(ValueError, match="symlink"):
        create_reproducible_archive(model_dir, archive)

    assert not archive.exists()


def test_archive_rejects_fifo_without_blocking_or_partial_output(tmp_path: Path) -> None:
    model_dir = tmp_path / "model"
    model_dir.mkdir()
    os.mkfifo(model_dir / "pipe")
    archive = tmp_path / "model.tar.gz"

    with pytest.raises(ValueError, match="regular file or directory"):
        create_reproducible_archive(model_dir, archive)

    assert not archive.exists()


def test_archive_rejects_symlink_destination_without_touching_target(tmp_path: Path) -> None:
    model_dir = tmp_path / "model"
    model_dir.mkdir()
    (model_dir / "model.sdf").write_text("sdf", encoding="utf-8")
    victim = tmp_path / "victim"
    victim.write_bytes(b"keep me")
    archive = tmp_path / "model.tar.gz"
    archive.symlink_to(victim)

    with pytest.raises(ValueError, match="symlink archive destination"):
        create_reproducible_archive(model_dir, archive)

    assert victim.read_bytes() == b"keep me"
    assert archive.is_symlink()


def test_archive_rejects_entry_swapped_between_scan_and_open(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    model_dir = tmp_path / "model"
    model_dir.mkdir()
    payload = model_dir / "payload.txt"
    payload.write_bytes(b"expected")
    replacement = tmp_path / "replacement.txt"
    replacement.write_bytes(b"replacement")
    archive = tmp_path / "model.tar.gz"
    real_open = os.open
    swapped = False

    def swap_before_open(path, flags, mode=0o777, *, dir_fd=None):
        nonlocal swapped
        if path == "payload.txt" and dir_fd is not None and not swapped:
            swapped = True
            payload.rename(model_dir / "original.txt")
            payload.symlink_to(replacement)
        return real_open(path, flags, mode, dir_fd=dir_fd)

    monkeypatch.setattr(package_module.os, "open", swap_before_open)

    with pytest.raises(ValueError, match="changed during archive scan"):
        create_reproducible_archive(model_dir, archive)

    assert swapped
    assert not archive.exists()


def test_archive_rejects_regular_file_swapped_to_fifo_without_blocking(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    model_dir = tmp_path / "model"
    model_dir.mkdir()
    payload = model_dir / "payload.txt"
    payload.write_bytes(b"expected")
    archive = tmp_path / "model.tar.gz"
    real_open = os.open
    swapped = False
    outcome: queue.SimpleQueue[BaseException | None] = queue.SimpleQueue()

    def swap_to_fifo_before_open(path, flags, mode=0o777, *, dir_fd=None):
        nonlocal swapped
        if path == "payload.txt" and dir_fd is not None and not swapped:
            swapped = True
            payload.unlink()
            os.mkfifo(payload)
        return real_open(path, flags, mode, dir_fd=dir_fd)

    def create_archive() -> None:
        try:
            create_reproducible_archive(model_dir, archive)
        except BaseException as error:
            outcome.put(error)
        else:
            outcome.put(None)

    monkeypatch.setattr(package_module.os, "open", swap_to_fifo_before_open)
    worker = threading.Thread(target=create_archive, daemon=True)
    worker.start()
    worker.join(timeout=0.5)
    returned_before_timeout = not worker.is_alive()
    if worker.is_alive():
        writer = real_open(payload, os.O_WRONLY | os.O_NONBLOCK)
        os.close(writer)
        worker.join(timeout=2)

    assert returned_before_timeout, "archive creation blocked opening a swapped FIFO"
    assert not worker.is_alive()
    assert swapped
    assert isinstance(outcome.get_nowait(), ValueError)
    assert not archive.exists()


def test_exported_packages_and_archives_contain_exact_approved_thumbnail_bytes(tmp_path: Path) -> None:
    approved = capture_thumbnail_set(approved_assets_root())

    exported = export_models(tmp_path / "dist")

    for item in exported:
        expected = approved.png_by_slug[item.directory.name]
        thumbnails = item.directory / "thumbnails"
        assert [path.name for path in thumbnails.iterdir()] == ["0.png"]
        assert (thumbnails / "0.png").read_bytes() == expected
        with tarfile.open(item.archive, "r:gz") as archive:
            member = archive.extractfile(f"{item.directory.name}-1.0.0/thumbnails/0.png")
            assert member is not None
            assert member.read() == expected
        published = (item.directory / "model.sdf").read_text(encoding="utf-8")
        assert "<sensor" not in published
        assert "<plugin" not in published


def test_export_captures_the_approved_thumbnail_set_exactly_once(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    real_capture = capture_thumbnail_set
    calls = 0

    def counted_capture(*args, **kwargs):
        nonlocal calls
        calls += 1
        return real_capture(*args, **kwargs)

    monkeypatch.setattr(package_module, "capture_thumbnail_set", counted_capture)

    exported = export_models(tmp_path / "dist")

    assert len(exported) == 3
    assert calls == 1


def test_export_uses_only_approved_snapshot_bytes_after_capture(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    mutable_approved = tmp_path / "mutable-approved"
    shutil.copytree(approved_assets_root(), mutable_approved)
    original = capture_thumbnail_set(mutable_approved)
    capture_calls = 0

    def mutate_after_capture(*args, **kwargs):
        del args, kwargs
        nonlocal capture_calls
        capture_calls += 1
        snapshot = capture_thumbnail_set(mutable_approved)
        for slug in snapshot.png_by_slug:
            (mutable_approved / f"{slug}.png").write_bytes(b"changed after snapshot")
        return snapshot

    monkeypatch.setattr(package_module, "capture_thumbnail_set", mutate_after_capture)
    monkeypatch.setattr(package_module, "approved_assets_root", lambda: mutable_approved)

    exported = export_models(tmp_path / "dist")

    assert capture_calls == 1
    for item in exported:
        assert (item.directory / "thumbnails/0.png").read_bytes() == original.png_by_slug[item.directory.name]


def test_stale_approved_model_binding_fails_before_install_and_preserves_previous_export(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    output = tmp_path / "dist"
    export_models(output)
    previous = _tree_bytes(output)
    stale = tmp_path / "stale-approved"
    shutil.copytree(approved_assets_root(), stale)
    manifest_path = stale / "render-manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["models"][0]["model_sdf_sha256"] = "0" * 64
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    monkeypatch.setattr(package_module, "approved_assets_root", lambda: stale)

    with pytest.raises(ValueError, match="model SDF hash|canonical"):
        export_models(output)

    assert _tree_bytes(output) == previous
