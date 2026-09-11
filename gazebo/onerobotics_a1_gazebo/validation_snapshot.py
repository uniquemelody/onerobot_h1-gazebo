"""Race-resistant immutable directory snapshots for package validation."""

from __future__ import annotations

import os
import stat
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from types import MappingProxyType


class SnapshotError(ValueError):
    """Raised when a directory cannot be captured as one coherent snapshot."""


_MAX_SNAPSHOT_DEPTH = 128
_MAX_SNAPSHOT_ENTRIES = 512
_MAX_REGULAR_FILE_BYTES = 16 * 1024 * 1024
_MAX_CAPTURED_BYTES = 128 * 1024 * 1024


@dataclass(frozen=True)
class SnapshotProblem:
    """One unsafe leaf skipped while the surrounding tree stayed coherent."""

    path: PurePosixPath
    message: str


@dataclass(frozen=True)
class DirectorySnapshot:
    """Immutable regular-file bytes and directory names relative to one root."""

    files: Mapping[PurePosixPath, bytes]
    directories: frozenset[PurePosixPath]
    problems: tuple[SnapshotProblem, ...] = ()

    def subtree(self, prefix: str | PurePosixPath) -> DirectorySnapshot:
        root = PurePosixPath(prefix)
        if root.is_absolute() or ".." in root.parts or root not in self.directories:
            raise SnapshotError(f"snapshot directory is missing: {root}")
        root_parts = len(root.parts)
        try:
            files = {
                PurePosixPath(*path.parts[root_parts:]): data
                for path, data in self.files.items()
                if path.parts[:root_parts] == root.parts and len(path.parts) > root_parts
            }
            directories = frozenset(
                PurePosixPath(*path.parts[root_parts:])
                for path in self.directories
                if path.parts[:root_parts] == root.parts and len(path.parts) > root_parts
            )
            problems = tuple(
                SnapshotProblem(PurePosixPath(*problem.path.parts[root_parts:]), problem.message)
                for problem in self.problems
                if problem.path.parts[:root_parts] == root.parts and len(problem.path.parts) > root_parts
            )
            return DirectorySnapshot(MappingProxyType(files), directories, problems)
        except MemoryError as error:
            raise SnapshotError("snapshot capture ran out of memory") from error


_Identity = tuple[int, int, int, int, int, int]


@dataclass(frozen=True)
class _HeldDirectory:
    descriptor: int
    opened: os.stat_result
    inventory: dict[str, _Identity]
    relative_path: PurePosixPath


@dataclass
class _EntryBudget:
    count: int = 0

    def consume(self, relative: PurePosixPath) -> None:
        self.count += 1
        if self.count > _MAX_SNAPSHOT_ENTRIES:
            raise SnapshotError(f"snapshot exceeds entry limit while inventorying: {relative or '.'}")


@dataclass
class _CaptureBudget:
    initial_entries: _EntryBudget
    repeated_entries: _EntryBudget
    captured_bytes: int = 0

    def reserve_file(self, size: int) -> None:
        if self.captured_bytes + size > _MAX_CAPTURED_BYTES:
            raise SnapshotError("snapshot exceeds total captured byte limit")
        self.captured_bytes += size


def _identity(details: os.stat_result) -> _Identity:
    return (
        details.st_dev,
        details.st_ino,
        stat.S_IFMT(details.st_mode),
        details.st_size,
        details.st_mtime_ns,
        details.st_ctime_ns,
    )


def _directory_inventory(
    directory_fd: int,
    relative: PurePosixPath,
    budget: _EntryBudget,
    maximum_entries: int | None = None,
) -> dict[str, _Identity]:
    try:
        before = os.fstat(directory_fd)
        inventory: dict[str, _Identity] = {}
        local_count = 0
        with os.scandir(directory_fd) as entries:
            for entry in entries:
                if maximum_entries is not None and local_count >= maximum_entries:
                    break
                budget.consume(relative)
                inventory[entry.name] = _identity(entry.stat(follow_symlinks=False))
                local_count += 1
        after = os.fstat(directory_fd)
        if _identity(before) != _identity(after):
            raise SnapshotError(f"directory changed during snapshot capture: {relative or '.'}")
        return dict(sorted(inventory.items()))
    except SnapshotError:
        raise
    except OSError as error:
        raise SnapshotError(f"unable to inventory snapshot directory: {relative or '.'}") from error


def _open_scanned_entry(
    directory_fd: int,
    name: str,
    relative_path: PurePosixPath,
    expected: _Identity,
) -> tuple[int, os.stat_result]:
    try:
        inspected = os.stat(name, dir_fd=directory_fd, follow_symlinks=False)
    except OSError as error:
        raise SnapshotError(f"entry changed during snapshot capture: {relative_path}") from error
    if _identity(inspected) != expected:
        raise SnapshotError(f"entry changed during snapshot capture: {relative_path}")
    if stat.S_ISLNK(inspected.st_mode):
        raise SnapshotError(f"snapshot must not contain symlinks: {relative_path}")
    if stat.S_ISDIR(inspected.st_mode):
        flags = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC
    elif stat.S_ISREG(inspected.st_mode):
        flags = os.O_RDONLY | os.O_NONBLOCK | os.O_NOFOLLOW | os.O_CLOEXEC
    else:
        raise SnapshotError(f"snapshot entry must be a regular file or directory: {relative_path}")
    try:
        descriptor = os.open(name, flags, dir_fd=directory_fd)
    except OSError as error:
        raise SnapshotError(f"entry changed during snapshot capture: {relative_path}") from error
    opened = os.fstat(descriptor)
    if _identity(opened) != expected:
        os.close(descriptor)
        raise SnapshotError(f"entry changed during snapshot capture: {relative_path}")
    return descriptor, opened


def _read_stable_file_contents(descriptor: int, opened: os.stat_result, relative_path: PurePosixPath) -> bytes:
    data = bytearray(opened.st_size)
    view = memoryview(data)
    size = 0
    try:
        while size < opened.st_size:
            read = os.readv(descriptor, [view[size : min(size + 1024 * 1024, opened.st_size)]])
            if read == 0:
                break
            size += read
        extra = os.read(descriptor, 1)
        after = os.fstat(descriptor)
    except SnapshotError:
        raise
    except OSError as error:
        raise SnapshotError(f"unable to read snapshot file: {relative_path}") from error
    finally:
        view.release()
    if _identity(after) != _identity(opened) or size != opened.st_size or extra:
        raise SnapshotError(f"file changed during snapshot capture: {relative_path}")
    return bytes(data)


def _read_stable_file(descriptor: int, opened: os.stat_result, relative_path: PurePosixPath) -> bytes:
    return _read_stable_file_contents(descriptor, opened, relative_path)


def _capture_directory(
    directory_fd: int,
    prefix: PurePosixPath,
    files: dict[PurePosixPath, bytes],
    directories: set[PurePosixPath],
    held_directories: list[_HeldDirectory],
    problems: list[SnapshotProblem],
    budget: _CaptureBudget,
    depth: int,
) -> None:
    before_details = os.fstat(directory_fd)
    held_index = len(held_directories)
    held_directories.append(_HeldDirectory(directory_fd, before_details, {}, prefix))
    if depth > _MAX_SNAPSHOT_DEPTH:
        raise SnapshotError(f"snapshot exceeds directory nesting limit: {prefix}")
    before = _directory_inventory(directory_fd, prefix, budget.initial_entries)
    held_directories[held_index] = _HeldDirectory(directory_fd, before_details, before, prefix)
    for name, expected in before.items():
        relative_path = prefix / name
        mode = expected[2]
        if stat.S_ISLNK(mode):
            problems.append(SnapshotProblem(relative_path, "snapshot must not contain symlinks"))
            continue
        if not (stat.S_ISDIR(mode) or stat.S_ISREG(mode)):
            leaf_type = "fifo" if stat.S_ISFIFO(mode) else "special file"
            problems.append(
                SnapshotProblem(
                    relative_path,
                    f"snapshot entry must be a regular file or directory ({leaf_type})",
                )
            )
            continue
        if stat.S_ISREG(mode) and expected[3] > _MAX_REGULAR_FILE_BYTES:
            problems.append(SnapshotProblem(relative_path, "snapshot regular file exceeds per-file byte limit"))
            continue
        if stat.S_ISREG(mode):
            budget.reserve_file(expected[3])
        descriptor, opened = _open_scanned_entry(directory_fd, name, relative_path, expected)
        if stat.S_ISDIR(opened.st_mode):
            directories.add(relative_path)
            _capture_directory(
                descriptor,
                relative_path,
                files,
                directories,
                held_directories,
                problems,
                budget,
                depth + 1,
            )
        else:
            try:
                files[relative_path] = _read_stable_file(descriptor, opened, relative_path)
            finally:
                os.close(descriptor)
    after_details = os.fstat(directory_fd)
    after = _directory_inventory(
        directory_fd,
        prefix,
        budget.repeated_entries,
        len(before) + 1,
    )
    if before != after or _identity(before_details) != _identity(after_details):
        raise SnapshotError(f"directory changed during snapshot capture: {prefix or '.'}")


def _verify_directory_inventories(held_directories: list[_HeldDirectory]) -> None:
    budget = _EntryBudget()
    for held in held_directories:
        if (
            _identity(os.fstat(held.descriptor)) != _identity(held.opened)
            or _directory_inventory(
                held.descriptor,
                held.relative_path,
                budget,
                len(held.inventory) + 1,
            )
            != held.inventory
        ):
            raise SnapshotError(f"directory changed during snapshot capture: {held.relative_path or '.'}")


def _verify_held_entries(
    held_directories: list[_HeldDirectory],
    files: dict[PurePosixPath, bytes],
) -> None:
    _verify_directory_inventories(held_directories)
    directories = {held.relative_path: held for held in held_directories}
    for relative_path, captured in sorted(files.items(), key=lambda item: item[0].as_posix()):
        parent = directories.get(relative_path.parent)
        if parent is None or relative_path.name not in parent.inventory:
            raise SnapshotError(f"file changed during snapshot capture: {relative_path}")
        expected = parent.inventory[relative_path.name]
        descriptor, opened = _open_scanned_entry(
            parent.descriptor,
            relative_path.name,
            relative_path,
            expected,
        )
        try:
            repeated = _read_stable_file_contents(descriptor, opened, relative_path)
        finally:
            os.close(descriptor)
        if repeated != captured:
            raise SnapshotError(f"file changed during snapshot capture: {relative_path}")
    _verify_directory_inventories(held_directories)


def _open_root(root: Path) -> int:
    if not root.name or root.name in {".", ".."}:
        raise SnapshotError("snapshot root must name a directory")
    try:
        inspected = os.stat(root, follow_symlinks=False)
    except OSError as error:
        raise SnapshotError(f"snapshot root is missing or inaccessible: {root}") from error
    if stat.S_ISLNK(inspected.st_mode):
        raise SnapshotError(f"snapshot root must not be a symlink: {root}")
    if not stat.S_ISDIR(inspected.st_mode):
        raise SnapshotError(f"snapshot root must be a regular directory: {root}")
    try:
        parent = root.parent.resolve(strict=True)
        parent_fd = os.open(parent, os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC)
        try:
            descriptor = os.open(
                root.name,
                os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC,
                dir_fd=parent_fd,
            )
        finally:
            os.close(parent_fd)
    except OSError as error:
        raise SnapshotError(f"snapshot root changed while being opened: {root}") from error
    if _identity(os.fstat(descriptor)) != _identity(inspected):
        os.close(descriptor)
        raise SnapshotError(f"snapshot root changed while being opened: {root}")
    return descriptor


def capture_directory(root: Path) -> DirectorySnapshot:
    """Capture a whole regular directory tree or fail if it changes during capture."""
    try:
        descriptor = _open_root(Path(root))
    except MemoryError as error:
        raise SnapshotError("snapshot capture ran out of memory") from error
    files: dict[PurePosixPath, bytes] = {}
    directories: set[PurePosixPath] = set()
    problems: list[SnapshotProblem] = []
    held_directories: list[_HeldDirectory] = []
    budget = _CaptureBudget(_EntryBudget(), _EntryBudget())
    try:
        _capture_directory(
            descriptor,
            PurePosixPath(),
            files,
            directories,
            held_directories,
            problems,
            budget,
            0,
        )
        _verify_held_entries(held_directories, files)
        return DirectorySnapshot(
            MappingProxyType(files),
            frozenset(directories),
            tuple(problems),
        )
    except RecursionError as error:
        raise SnapshotError("snapshot exceeds directory nesting limit") from error
    except MemoryError as error:
        raise SnapshotError("snapshot capture ran out of memory") from error
    finally:
        for held in reversed(held_directories):
            os.close(held.descriptor)
