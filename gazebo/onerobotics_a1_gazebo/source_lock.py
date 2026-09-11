"""Generate and validate a byte-for-byte lock of the public A1 source tree."""

from __future__ import annotations

import argparse
import hashlib
import json
from collections.abc import Iterable
from pathlib import Path

from .spec import SOURCE_COMMIT, SOURCE_REPOSITORY
from .spec import source_root as public_source_root

SCHEMA_VERSION = 1


def _gazebo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _manifest_path() -> Path:
    return _gazebo_root() / "generated-manifest.json"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def discover_source_files(root: Path | None = None) -> tuple[Path, ...]:
    """Return every regular public source file, ordered by its POSIX-relative path."""
    directory = root or public_source_root()
    if not directory.is_dir():
        raise ValueError(f"Missing source root: {directory}")
    files = (path for path in directory.rglob("*") if path.is_file())
    return tuple(sorted(files, key=lambda path: path.relative_to(directory).as_posix()))


def build_source_lock(files: Iterable[Path], *, relative_to: Path) -> dict[str, object]:
    return {
        "schema_version": SCHEMA_VERSION,
        "source_repository": SOURCE_REPOSITORY,
        "source_commit": SOURCE_COMMIT,
        "files": {
            path.relative_to(relative_to).as_posix(): sha256_file(path)
            for path in sorted(files, key=lambda path: path.relative_to(relative_to).as_posix())
        },
    }


def _read_lock(path: Path) -> dict[str, object]:
    try:
        with path.open(encoding="utf-8") as stream:
            lock = json.load(stream)
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"Unable to read source lock: {path}") from error
    if not isinstance(lock, dict):
        raise ValueError("Source lock must be a JSON object")
    return lock


def validate_source_lock(*, lock: dict[str, object] | None = None, source_root: Path | None = None) -> None:
    """Raise ValueError unless a lock exactly matches its source file inventory and bytes."""
    root = source_root or public_source_root()
    expected = lock if lock is not None else _read_lock(_manifest_path())
    actual = build_source_lock(discover_source_files(root), relative_to=root)

    for field in ("schema_version", "source_repository", "source_commit"):
        if expected.get(field) != actual[field]:
            raise ValueError(f"Source lock {field} mismatch")

    expected_files = expected.get("files")
    actual_files = actual["files"]
    if not isinstance(expected_files, dict):
        raise ValueError("Source lock files must be a mapping")
    if set(expected_files) != set(actual_files):
        raise ValueError("Source lock file inventory mismatch")
    for relative_path, digest in actual_files.items():
        if expected_files[relative_path] != digest:
            raise ValueError(f"SHA-256 mismatch: {relative_path}")


def _inventory_summary(files: Iterable[Path]) -> str:
    source_files = tuple(files)
    stl_count = sum(path.suffix.lower() == ".stl" for path in source_files)
    urdf_count = sum(path.suffix == ".urdf" for path in source_files)
    metadata_count = len(source_files) - stl_count - urdf_count
    return (
        f"SOURCE_LOCK_OK: {len(source_files)} files ({stl_count} STL, "
        f"{urdf_count} URDF, {metadata_count} metadata/document files)"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--write", type=Path, metavar="PATH")
    action.add_argument("--check", type=Path, metavar="PATH")
    arguments = parser.parse_args()
    files = discover_source_files()
    if arguments.write is not None:
        lock = build_source_lock(files, relative_to=public_source_root())
        arguments.write.write_text(json.dumps(lock, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    else:
        validate_source_lock(lock=_read_lock(arguments.check), source_root=public_source_root())
    print(_inventory_summary(files))


if __name__ == "__main__":
    main()
