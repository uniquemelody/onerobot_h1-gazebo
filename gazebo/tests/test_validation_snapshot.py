from __future__ import annotations

import os
import shutil
from dataclasses import replace
from pathlib import Path, PurePosixPath

import onerobotics_a1_gazebo.validate as validation_module
import onerobotics_a1_gazebo.validation_snapshot as snapshot_module
import pytest
from onerobotics_a1_gazebo.package import export_models
from onerobotics_a1_gazebo.spec import load_model_specs, source_root
from onerobotics_a1_gazebo.validate import ValidationFailure, validate_all, validate_package
from onerobotics_a1_gazebo.validation_snapshot import SnapshotError, capture_directory


def test_capture_directory_returns_one_coherent_recursive_snapshot(tmp_path: Path) -> None:
    root = tmp_path / "root"
    (root / "nested").mkdir(parents=True)
    (root / "first.txt").write_bytes(b"first")
    (root / "nested/second.txt").write_bytes(b"second")

    snapshot = capture_directory(root)

    assert snapshot.files == {
        PurePosixPath("first.txt"): b"first",
        PurePosixPath("nested/second.txt"): b"second",
    }
    assert snapshot.directories == frozenset({PurePosixPath("nested")})


@pytest.mark.parametrize("leaf_type", ["symlink", "fifo"])
def test_capture_records_unsafe_leaf_and_continues(
    tmp_path: Path,
    leaf_type: str,
) -> None:
    root = tmp_path / "root"
    root.mkdir()
    (root / "safe.txt").write_bytes(b"safe")
    unsafe = root / "unsafe"
    if leaf_type == "symlink":
        unsafe.symlink_to(tmp_path / "outside")
    else:
        os.mkfifo(unsafe)

    snapshot = capture_directory(root)

    assert snapshot.files == {PurePosixPath("safe.txt"): b"safe"}
    assert len(snapshot.problems) == 1
    assert snapshot.problems[0].path == PurePosixPath("unsafe")
    assert leaf_type in snapshot.problems[0].message


def test_subtree_filters_and_relocates_snapshot_problems(tmp_path: Path) -> None:
    root = tmp_path / "root"
    (root / "package").mkdir(parents=True)
    (root / "other").mkdir()
    (root / "package/unsafe").symlink_to(tmp_path / "outside")
    os.mkfifo(root / "other/unsafe")

    package = capture_directory(root).subtree("package")

    assert len(package.problems) == 1
    assert package.problems[0].path == PurePosixPath("unsafe")
    assert "symlink" in package.problems[0].message


def test_capture_skips_oversized_regular_file_before_reading(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    root = tmp_path / "root"
    root.mkdir()
    oversized = root / "oversized.bin"
    with oversized.open("wb") as stream:
        stream.truncate(snapshot_module._MAX_REGULAR_FILE_BYTES + 1)
    (root / "safe.txt").write_bytes(b"safe")
    real_read = snapshot_module._read_stable_file
    read_paths: list[PurePosixPath] = []

    def record_read(descriptor, opened, relative_path):
        read_paths.append(relative_path)
        return real_read(descriptor, opened, relative_path)

    monkeypatch.setattr(snapshot_module, "_read_stable_file", record_read)

    snapshot = capture_directory(root)

    assert snapshot.files == {PurePosixPath("safe.txt"): b"safe"}
    assert len(snapshot.problems) == 1
    assert snapshot.problems[0].path == PurePosixPath("oversized.bin")
    assert "per-file byte limit" in snapshot.problems[0].message
    assert read_paths == [PurePosixPath("safe.txt")]


def test_capture_rejects_total_byte_limit_before_reading(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    root = tmp_path / "root"
    root.mkdir()
    (root / "first.bin").write_bytes(b"123")
    (root / "second.bin").write_bytes(b"456")
    monkeypatch.setattr(snapshot_module, "_MAX_CAPTURED_BYTES", 5)

    with pytest.raises(SnapshotError, match="total captured byte limit"):
        capture_directory(root)


def test_capture_rejects_entry_limit_during_bounded_enumeration(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    root = tmp_path / "root"
    root.mkdir()
    for index in range(3):
        (root / f"{index}.txt").touch()
    monkeypatch.setattr(snapshot_module, "_MAX_SNAPSHOT_ENTRIES", 2)

    with pytest.raises(SnapshotError, match="entry limit"):
        capture_directory(root)


@pytest.mark.parametrize(
    ("failure", "message"),
    [
        ("total", "total captured byte limit"),
        ("entries", "entry limit"),
        ("memory", "snapshot capture ran out of memory"),
    ],
)
def test_cli_converts_snapshot_resource_failures_to_diagnostics(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
    failure: str,
    message: str,
) -> None:
    output = tmp_path / "export"
    export_models(output)
    if failure == "total":
        monkeypatch.setattr(snapshot_module, "_MAX_CAPTURED_BYTES", 1)
    elif failure == "entries":
        monkeypatch.setattr(snapshot_module, "_MAX_SNAPSHOT_ENTRIES", 1)
    else:

        def raise_memory_error(*args, **kwargs):
            raise MemoryError

        monkeypatch.setattr(snapshot_module, "_directory_inventory", raise_memory_error)

    status = validation_module.main([str(output)])
    captured = capsys.readouterr()

    assert status == 1
    assert message in captured.err
    assert "VALIDATION_OK" not in captured.out


def test_capture_rejects_entry_added_after_initial_inventory(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    root = tmp_path / "root"
    root.mkdir()
    (root / "first.txt").write_bytes(b"first")
    real_open = snapshot_module._open_scanned_entry
    injected = False

    def inject(directory_fd, name, relative_path, expected):
        nonlocal injected
        if not injected:
            injected = True
            (root / "attacker-extra").write_bytes(b"hidden")
        return real_open(directory_fd, name, relative_path, expected)

    monkeypatch.setattr(snapshot_module, "_open_scanned_entry", inject)

    with pytest.raises(SnapshotError, match="changed during snapshot capture"):
        capture_directory(root)


def test_capture_rejects_file_swapped_between_inventory_and_open(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    root = tmp_path / "root"
    root.mkdir()
    victim = root / "victim.txt"
    victim.write_bytes(b"trusted")
    outside = tmp_path / "outside.txt"
    outside.write_bytes(b"attacker controlled")
    real_open = snapshot_module._open_scanned_entry
    swapped = False

    def swap(directory_fd, name, relative_path, expected):
        nonlocal swapped
        if not swapped and relative_path == PurePosixPath("victim.txt"):
            swapped = True
            victim.unlink()
            victim.symlink_to(outside)
        return real_open(directory_fd, name, relative_path, expected)

    monkeypatch.setattr(snapshot_module, "_open_scanned_entry", swap)

    with pytest.raises(SnapshotError, match="changed during snapshot capture"):
        capture_directory(root)


def test_capture_rejects_source_file_changed_after_it_was_read(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    root = tmp_path / "root"
    root.mkdir()
    first = root / "a-first.txt"
    first.write_bytes(b"trusted")
    (root / "z-last.txt").write_bytes(b"last")
    real_open = snapshot_module._open_scanned_entry
    changed = False

    def change_after_first_read(directory_fd, name, relative_path, expected):
        nonlocal changed
        if not changed and relative_path == PurePosixPath("z-last.txt"):
            changed = True
            first.write_bytes(b"changed after capture")
        return real_open(directory_fd, name, relative_path, expected)

    monkeypatch.setattr(snapshot_module, "_open_scanned_entry", change_after_first_read)

    with pytest.raises(SnapshotError, match="changed during snapshot capture"):
        capture_directory(root)


def test_capture_rejects_nested_file_changed_after_subtree_was_read(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    root = tmp_path / "root"
    nested = root / "a-dir"
    nested.mkdir(parents=True)
    victim = nested / "victim.txt"
    victim.write_bytes(b"trusted")
    (root / "z-last.txt").write_bytes(b"last")
    real_open = snapshot_module._open_scanned_entry
    changed = False

    def change_nested_after_subtree(directory_fd, name, relative_path, expected):
        nonlocal changed
        if not changed and relative_path == PurePosixPath("z-last.txt"):
            changed = True
            victim.write_bytes(b"changed after nested capture")
        return real_open(directory_fd, name, relative_path, expected)

    monkeypatch.setattr(snapshot_module, "_open_scanned_entry", change_nested_after_subtree)

    with pytest.raises(SnapshotError, match="changed during snapshot capture"):
        capture_directory(root)


def _create_nested_directories(root: Path, depth: int) -> None:
    current = root
    for _ in range(depth):
        current /= "d"
        current.mkdir()


def test_capture_rejects_excessive_directory_nesting(tmp_path: Path) -> None:
    root = tmp_path / "root"
    root.mkdir()
    _create_nested_directories(root, 200)

    with pytest.raises(SnapshotError, match="snapshot exceeds directory nesting limit"):
        capture_directory(root)


def test_cli_converts_excessive_directory_nesting_to_failure(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    output = tmp_path / "export"
    export_models(output)
    _create_nested_directories(output / "onerobotics_a1_right_arm", 200)

    status = validation_module.main([str(output)])
    captured = capsys.readouterr()

    assert status == 1
    assert "snapshot exceeds directory nesting limit" in captured.err
    assert "VALIDATION_OK" not in captured.out


def test_validate_all_rejects_extra_entry_added_during_export_capture(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    output = tmp_path / "export"
    export_models(output)
    real_open = snapshot_module._open_scanned_entry
    injected = False

    def inject(directory_fd, name, relative_path, expected):
        nonlocal injected
        if not injected:
            injected = True
            (output / "attacker-extra").write_bytes(b"hidden")
        return real_open(directory_fd, name, relative_path, expected)

    monkeypatch.setattr(snapshot_module, "_open_scanned_entry", inject)

    with pytest.raises(ValidationFailure) as failure:
        validate_all(output)

    assert any("changed during snapshot capture" in issue.message for issue in failure.value.issues)


def test_validate_package_rejects_source_swap_between_inventory_and_read(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    output = tmp_path / "export"
    export_models(output)
    original_spec = next(spec for spec in load_model_specs() if spec.key == "right_arm")
    copied_source = tmp_path / "source-copy"
    shutil.copytree(source_root(), copied_source)
    copied_urdf = copied_source / original_spec.urdf.relative_to(source_root())
    spec = replace(original_spec, urdf=copied_urdf)
    monkeypatch.setattr(validation_module, "source_root", lambda: copied_source)
    real_open = snapshot_module._open_scanned_entry
    swapped = False

    def swap(directory_fd, name, relative_path, expected):
        nonlocal swapped
        if not swapped and relative_path == PurePosixPath("a1_r.urdf"):
            swapped = True
            copied_urdf.write_bytes(copied_urdf.read_bytes() + b"changed")
        return real_open(directory_fd, name, relative_path, expected)

    monkeypatch.setattr(snapshot_module, "_open_scanned_entry", swap)

    issues = validate_package(output / spec.slug, spec)

    assert any("immutable source snapshot failed" in issue.message for issue in issues)
