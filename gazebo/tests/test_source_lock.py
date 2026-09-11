from pathlib import Path

import pytest
from onerobotics_a1_gazebo.source_lock import build_source_lock, discover_source_files, validate_source_lock


def test_source_inventory_is_exactly_the_public_39_file_set():
    files = discover_source_files()
    assert len(files) == 39
    assert sum(path.suffix.lower() == ".stl" for path in files) == 33
    assert sum(path.suffix == ".urdf" for path in files) == 3
    assert {path.name for path in files} >= {"README.md", "SHA256SUMS", "model_parameters.yaml"}


def test_checked_in_lock_matches_source_bytes():
    validate_source_lock()


def test_lock_rejects_changed_bytes(tmp_path: Path):
    source = tmp_path / "source"
    source.mkdir()
    (source / "a.urdf").write_text("one", encoding="utf-8")
    lock = build_source_lock((source / "a.urdf",), relative_to=source)
    (source / "a.urdf").write_text("two", encoding="utf-8")
    with pytest.raises(ValueError, match="SHA-256 mismatch"):
        validate_source_lock(lock=lock, source_root=source)
