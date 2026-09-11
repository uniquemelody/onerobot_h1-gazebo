"""Fail-closed local evidence and raw-ZIP checks for an authorized Fuel publication."""

from __future__ import annotations

import argparse
import ctypes
import errno
import hashlib
import io
import json
import os
import secrets
import stat
import struct
import sys
import zipfile
import zlib
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path, PurePosixPath

from onerobotics_a1_gazebo.validation_snapshot import DirectorySnapshot, SnapshotError, capture_directory

_SCHEMA_VERSION = 1
_MAX_MANIFEST_BYTES = 512 * 1024
_MAX_API_BYTES = 128 * 1024
_MAX_ZIP_BYTES = 256 * 1024 * 1024
_MAX_ENTRIES = 512
_MAX_FILE_BYTES = 16 * 1024 * 1024
_MAX_TOTAL_BYTES = 128 * 1024 * 1024
_DIGEST_LENGTH = 64
_RENAME_NOREPLACE = 1


class PublicationError(ValueError):
    """A local publication artifact failed a strict trust-boundary check."""


@dataclass(frozen=True)
class ManifestFile:
    path: PurePosixPath
    size: int
    sha256: str


@dataclass(frozen=True)
class PublicationManifest:
    directories: tuple[PurePosixPath, ...]
    files: tuple[ManifestFile, ...]


@dataclass(frozen=True)
class _HeldParent:
    path: Path
    descriptor: int
    identity: tuple[int, int]


def _add_exception_note(error: BaseException, note: str) -> None:
    """Preserve cleanup diagnostics without requiring Python 3.11."""
    add_note = getattr(error, "add_note", None)
    if callable(add_note):
        add_note(note)
        return
    notes = list(getattr(error, "__notes__", ()))
    notes.append(note)
    error.__notes__ = notes


def _identity(details: os.stat_result) -> tuple[int, int]:
    return details.st_dev, details.st_ino


def _stable_file_identity(details: os.stat_result) -> tuple[int, int, int, int, int, int]:
    return (
        details.st_dev,
        details.st_ino,
        stat.S_IFMT(details.st_mode),
        details.st_size,
        details.st_mtime_ns,
        details.st_ctime_ns,
    )


def _safe_relative(raw: str, *, directory: bool = False) -> PurePosixPath:
    if not isinstance(raw, str) or not raw:
        raise PublicationError("inventory path must be non-empty text")
    try:
        raw.encode("utf-8")
    except UnicodeEncodeError:
        raise PublicationError(f"unsafe inventory path: {raw!r}") from None
    if (
        any(ord(character) < 32 or ord(character) == 127 for character in raw)
        or "\\" in raw
        or "%" in raw
        or ":" in raw
    ):
        raise PublicationError(f"unsafe inventory path: {raw!r}")
    if raw.startswith("/") or raw.startswith("~"):
        raise PublicationError(f"unsafe inventory path: {raw!r}")
    candidate = raw[:-1] if directory and raw.endswith("/") else raw
    if not candidate or candidate.endswith("/") or "//" in candidate:
        raise PublicationError(f"unsafe inventory path: {raw!r}")
    parts = candidate.split("/")
    if any(part in {"", ".", ".."} for part in parts):
        raise PublicationError(f"unsafe inventory path: {raw!r}")
    path = PurePosixPath(*parts)
    if path.is_absolute():
        raise PublicationError(f"unsafe inventory path: {raw!r}")
    return path


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _manifest_document(manifest: PublicationManifest) -> dict[str, object]:
    return {
        "directories": [path.as_posix() for path in manifest.directories],
        "files": [{"path": item.path.as_posix(), "sha256": item.sha256, "size": item.size} for item in manifest.files],
        "schema_version": _SCHEMA_VERSION,
    }


def _manifest_bytes(manifest: PublicationManifest) -> bytes:
    return (
        json.dumps(
            _manifest_document(manifest),
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
        ).encode("utf-8")
        + b"\n"
    )


def _open_directory(path: Path, label: str) -> _HeldParent:
    path = Path(path)
    try:
        inspected = path.lstat()
        if stat.S_ISLNK(inspected.st_mode) or not stat.S_ISDIR(inspected.st_mode):
            raise PublicationError(f"{label} must be a regular non-symlink directory")
        descriptor = os.open(path, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC)
        opened = os.fstat(descriptor)
    except PublicationError:
        raise
    except OSError as error:
        raise PublicationError(f"unable to open {label}: {error}") from None
    if _identity(inspected) != _identity(opened):
        os.close(descriptor)
        raise PublicationError(f"{label} changed while opening")
    return _HeldParent(path.absolute(), descriptor, _identity(opened))


def _open_parent(output: Path) -> _HeldParent:
    output = Path(output)
    parent = output.parent if output.parent != Path("") else Path(".")
    if ".." in parent.parts:
        raise PublicationError(f"trusted output parent must not contain '..': {parent!r}")
    if not parent.is_absolute():
        parent = Path.cwd() / parent

    # A returned pathname cannot be protected from root or another process with
    # the same euid.  The publication boundary therefore treats those actors as
    # trusted and excludes every other identity: the leaf parent is owner-only,
    # every lexical component is opened without following links, and writable
    # ancestors are accepted only when sticky-directory rename rules protect an
    # entry owned by root or this euid (the normal /tmp + mktemp layout).
    effective_uid = os.geteuid()
    components = parent.parts
    descriptor = -1
    try:
        descriptor = os.open(
            components[0],
            os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC,
        )
        opened_path = Path(components[0])
        _require_trusted_ancestor(os.fstat(descriptor), opened_path, effective_uid)
        for component in components[1:]:
            opened_path /= component
            try:
                inspected = os.stat(component, dir_fd=descriptor, follow_symlinks=False)
                child = os.open(
                    component,
                    os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC,
                    dir_fd=descriptor,
                )
            except OSError as error:
                raise PublicationError(f"unable to open trusted output parent {opened_path!r}: {error}") from None
            try:
                opened = os.fstat(child)
                if stat.S_ISLNK(inspected.st_mode) or not stat.S_ISDIR(inspected.st_mode):
                    raise PublicationError(
                        f"trusted output ancestry contains a symlink or non-directory: {opened_path!r}"
                    )
                if _identity(inspected) != _identity(opened):
                    raise PublicationError(f"trusted output ancestry changed while opening: {opened_path!r}")
                _require_trusted_ancestor(opened, opened_path, effective_uid)
            except BaseException:
                os.close(child)
                raise
            os.close(descriptor)
            descriptor = child

        opened = os.fstat(descriptor)
        if opened.st_uid != effective_uid:
            raise PublicationError(f"trusted output parent must be owned by the effective user: {parent!r}")
        if stat.S_IMODE(opened.st_mode) & 0o077:
            raise PublicationError(f"trusted output parent must have private owner-only permissions: {parent!r}")
        return _HeldParent(parent, descriptor, _identity(opened))
    except PublicationError:
        if descriptor >= 0:
            os.close(descriptor)
        raise
    except OSError as error:
        if descriptor >= 0:
            os.close(descriptor)
        raise PublicationError(f"unable to open trusted output parent {parent!r}: {error}") from None


def _require_trusted_ancestor(details: os.stat_result, path: Path, effective_uid: int) -> None:
    if not stat.S_ISDIR(details.st_mode):
        raise PublicationError(f"trusted output ancestry is not a directory: {path!r}")
    if details.st_uid not in {0, effective_uid}:
        raise PublicationError(f"trusted output ancestry has an untrusted owner: {path!r}")
    mode = stat.S_IMODE(details.st_mode)
    if mode & 0o022 and not mode & stat.S_ISVTX:
        raise PublicationError(f"trusted output ancestry is writable without sticky protection: {path!r}")


def _descriptor_is_within(descriptor: int, ancestor: tuple[int, int]) -> bool:
    current = os.dup(descriptor)
    try:
        for _ in range(1024):
            current_identity = _identity(os.fstat(current))
            if current_identity == ancestor:
                return True
            parent = os.open(
                "..",
                os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC,
                dir_fd=current,
            )
            parent_identity = _identity(os.fstat(parent))
            if parent_identity == current_identity:
                os.close(parent)
                return False
            os.close(current)
            current = parent
    except OSError as error:
        raise PublicationError(f"unable to verify output ancestry: {error}") from None
    finally:
        os.close(current)
    raise PublicationError("output ancestry exceeds directory nesting limit")


def _verify_parent(parent: _HeldParent) -> None:
    try:
        path_details = parent.path.lstat()
        descriptor_details = os.fstat(parent.descriptor)
    except OSError as error:
        raise PublicationError(f"output parent changed: {error}") from None
    if (
        stat.S_ISLNK(path_details.st_mode)
        or not stat.S_ISDIR(path_details.st_mode)
        or _identity(path_details) != parent.identity
        or _identity(descriptor_details) != parent.identity
    ):
        raise PublicationError("output parent changed")
    effective_uid = os.geteuid()
    if descriptor_details.st_uid != effective_uid or stat.S_IMODE(descriptor_details.st_mode) & 0o077:
        raise PublicationError("trusted output parent permissions or owner changed")


def _require_absent(parent: _HeldParent, name: str) -> None:
    try:
        os.stat(name, dir_fd=parent.descriptor, follow_symlinks=False)
    except FileNotFoundError:
        return
    except OSError as error:
        raise PublicationError(f"unable to inspect output: {error}") from None
    raise PublicationError("output already exists or is a symlink")


def _rename_noreplace(parent: _HeldParent, source: str, destination: str) -> None:
    libc = ctypes.CDLL(None, use_errno=True)
    function = getattr(libc, "renameat2", None)
    if function is None:
        raise PublicationError("atomic no-replace installation is unavailable")
    function.argtypes = [
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_uint,
    ]
    function.restype = ctypes.c_int
    result = function(
        parent.descriptor,
        os.fsencode(source),
        parent.descriptor,
        os.fsencode(destination),
        _RENAME_NOREPLACE,
    )
    if result != 0:
        number = ctypes.get_errno()
        if number == errno.EEXIST:
            raise PublicationError("output already exists or appeared during installation")
        raise PublicationError(f"unable to install output atomically: {os.strerror(number)}")


def _link_descriptor_noreplace(parent: _HeldParent, descriptor: int, destination: str) -> None:
    """Install the held regular-file inode, never a replaceable staging name."""
    try:
        os.link(
            f"/proc/self/fd/{descriptor}",
            destination,
            dst_dir_fd=parent.descriptor,
            follow_symlinks=True,
        )
    except FileExistsError:
        raise PublicationError("output already exists or appeared during installation") from None
    except (NotImplementedError, OSError) as error:
        raise PublicationError(f"unable to install held output atomically: {error}") from None


def _regular_name_matches_descriptor(parent_fd: int, name: str, descriptor: int) -> bool:
    try:
        named = os.stat(name, dir_fd=parent_fd, follow_symlinks=False)
        opened = os.fstat(descriptor)
    except OSError:
        return False
    return stat.S_ISREG(named.st_mode) and stat.S_ISREG(opened.st_mode) and _identity(named) == _identity(opened)


def _reconcile_link_after_error(
    operation_error: BaseException,
    parent: _HeldParent,
    descriptor: int,
    staging_name: str,
    output_name: str,
) -> bool | None:
    """Report whether a failed/interrupted link call actually installed output."""
    try:
        opened = os.fstat(descriptor)
        staged = os.stat(staging_name, dir_fd=parent.descriptor, follow_symlinks=False)
        try:
            installed = os.stat(output_name, dir_fd=parent.descriptor, follow_symlinks=False)
        except FileNotFoundError:
            installed = None
    except BaseException as error:
        _add_exception_note(operation_error, f"output link outcome reconciliation failed: {error}")
        return None
    if (
        not stat.S_ISREG(opened.st_mode)
        or not stat.S_ISREG(staged.st_mode)
        or _identity(staged) != _identity(opened)
        or (
            installed is not None and (not stat.S_ISREG(installed.st_mode) or _identity(installed) != _identity(opened))
        )
    ):
        _add_exception_note(operation_error, "output link outcome could not be reconciled safely")
        return None
    return installed is not None


def _link_descriptor_noreplace_reconciled(
    parent: _HeldParent,
    descriptor: int,
    staging_name: str,
    output_name: str,
    record_outcome: Callable[[bool], None],
) -> None:
    try:
        _link_descriptor_noreplace(parent, descriptor, output_name)
    except BaseException as operation_error:
        outcome = _reconcile_link_after_error(
            operation_error,
            parent,
            descriptor,
            staging_name,
            output_name,
        )
        if outcome is not None:
            record_outcome(outcome)
        raise
    else:
        record_outcome(True)


def _atomic_new_file(
    output: Path,
    data: bytes,
    *,
    forbidden_ancestor: tuple[int, int] | None = None,
) -> None:
    output = Path(output)
    if not output.name or output.name in {".", ".."}:
        raise PublicationError("output must name one file")
    parent = _open_parent(output)
    temporary = f".{output.name}.{secrets.token_hex(16)}"
    descriptor = -1
    installed_name: str | None = None
    temporary_created = False
    success = False
    failure: BaseException | None = None
    cleanup_error: OSError | None = None

    def record_link_outcome(installed: bool) -> None:
        nonlocal installed_name
        installed_name = output.name if installed else None

    try:
        try:
            _verify_parent(parent)
            if forbidden_ancestor is not None and _descriptor_is_within(parent.descriptor, forbidden_ancestor):
                raise PublicationError("publication manifest output must remain outside the model tree")
            _require_absent(parent, output.name)
            descriptor = os.open(
                temporary,
                os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
                0o600,
                dir_fd=parent.descriptor,
            )
            temporary_created = True
            with os.fdopen(os.dup(descriptor), "wb", closefd=True) as stream:
                stream.write(data)
                stream.flush()
                os.fsync(stream.fileno())
            _verify_parent(parent)
            if not _regular_name_matches_descriptor(parent.descriptor, temporary, descriptor):
                raise PublicationError("staged output file changed before installation")
            _require_absent(parent, output.name)
            _link_descriptor_noreplace_reconciled(
                parent,
                descriptor,
                temporary,
                output.name,
                record_link_outcome,
            )
            if not _regular_name_matches_descriptor(parent.descriptor, output.name, descriptor):
                raise PublicationError("staged output file changed during installation")
            if not _regular_name_matches_descriptor(parent.descriptor, temporary, descriptor):
                raise PublicationError("staged output file changed during installation")
            os.unlink(temporary, dir_fd=parent.descriptor)
            temporary_created = False
            os.fsync(parent.descriptor)
            _verify_parent(parent)
            if not _regular_name_matches_descriptor(parent.descriptor, output.name, descriptor):
                raise PublicationError("installed output file changed after installation")
            success = True
        except PublicationError:
            raise
        except OSError as error:
            raise PublicationError(f"unable to write output safely: {error}") from None
    except BaseException as error:
        failure = error
    finally:
        if not success and descriptor >= 0:
            candidates = [name for name in (installed_name, temporary if temporary_created else None) if name]
            for candidate in candidates:
                if not _regular_name_matches_descriptor(parent.descriptor, candidate, descriptor):
                    continue
                try:
                    os.unlink(candidate, dir_fd=parent.descriptor)
                    os.fsync(parent.descriptor)
                except OSError as error:
                    if cleanup_error is None:
                        cleanup_error = error
        if descriptor >= 0:
            os.close(descriptor)
        os.close(parent.descriptor)
    if failure is not None:
        if cleanup_error is not None:
            _add_exception_note(failure, f"output cleanup failed: {cleanup_error}")
        raise failure
    if cleanup_error is not None:
        raise PublicationError(f"output cleanup failed: {cleanup_error}")


def _read_stable_file(path: Path, maximum: int, label: str) -> bytes:
    path = Path(path)
    descriptor = -1
    try:
        inspected = path.lstat()
        if stat.S_ISLNK(inspected.st_mode) or not stat.S_ISREG(inspected.st_mode):
            raise PublicationError(f"{label} must be a regular non-symlink file")
        if inspected.st_size > maximum:
            raise PublicationError(f"{label} exceeds size limit")
        descriptor = os.open(path, os.O_RDONLY | os.O_NONBLOCK | os.O_NOFOLLOW | os.O_CLOEXEC)
        opened = os.fstat(descriptor)
        if _stable_file_identity(inspected) != _stable_file_identity(opened):
            raise PublicationError(f"{label} changed while opening")
        chunks: list[bytes] = []
        remaining = opened.st_size
        while remaining:
            chunk = os.read(descriptor, min(1024 * 1024, remaining))
            if not chunk:
                break
            chunks.append(chunk)
            remaining -= len(chunk)
        extra = os.read(descriptor, 1)
        after = os.fstat(descriptor)
        if remaining or extra or _stable_file_identity(after) != _stable_file_identity(opened):
            raise PublicationError(f"{label} changed while reading")
        return b"".join(chunks)
    except PublicationError:
        raise
    except OSError as error:
        raise PublicationError(f"unable to read {label}: {error}") from None
    finally:
        if descriptor >= 0:
            os.close(descriptor)


def _no_duplicate_object(pairs: list[tuple[str, object]]) -> dict[str, object]:
    result: dict[str, object] = {}
    for key, value in pairs:
        if key in result:
            raise PublicationError(f"JSON has duplicate key: {key!r}")
        result[key] = value
    return result


def _parse_json(data: bytes, label: str) -> object:
    try:
        return json.loads(
            data.decode("utf-8"),
            object_pairs_hook=_no_duplicate_object,
            parse_constant=lambda value: (_ for _ in ()).throw(
                PublicationError(f"{label} contains a non-finite number: {value}")
            ),
        )
    except PublicationError:
        raise
    except (UnicodeError, json.JSONDecodeError, RecursionError, ValueError) as error:
        raise PublicationError(f"{label} is malformed: {error}") from None


def _capture_model_snapshot(model: Path) -> DirectorySnapshot:
    try:
        snapshot = capture_directory(Path(model))
    except SnapshotError as error:
        raise PublicationError(f"model snapshot failed: {error}") from None
    if snapshot.problems:
        detail = "; ".join(f"{problem.message}: {problem.path}" for problem in snapshot.problems)
        raise PublicationError(f"model snapshot has unsafe entries: {detail}")
    return snapshot


def _manifest_from_snapshot(snapshot: DirectorySnapshot) -> PublicationManifest:
    for path in snapshot.directories:
        _safe_relative(path.as_posix())
    for path in snapshot.files:
        _safe_relative(path.as_posix())
    directories = tuple(sorted(snapshot.directories, key=lambda path: path.as_posix()))
    files = tuple(
        ManifestFile(path, len(data), _sha256(data))
        for path, data in sorted(snapshot.files.items(), key=lambda item: item[0].as_posix())
    )
    if not files:
        raise PublicationError("model snapshot contains no regular files")
    return PublicationManifest(directories, files)


def _snapshot_manifest(model: Path) -> PublicationManifest:
    return _manifest_from_snapshot(_capture_model_snapshot(model))


def _path_is_lexically_within(path: Path, ancestor: Path) -> bool:
    absolute_path = Path(os.path.abspath(path))
    absolute_ancestor = Path(os.path.abspath(ancestor))
    return absolute_path == absolute_ancestor or absolute_ancestor in absolute_path.parents


def write_manifest(model: Path, output: Path) -> PublicationManifest:
    """Capture one model and atomically write evidence under a same-euid trust boundary."""
    model = Path(model)
    output = Path(output)
    if _path_is_lexically_within(output, model):
        raise PublicationError("publication manifest output must remain outside the model tree")
    held_model = _open_directory(model, "model directory")
    try:
        manifest = _snapshot_manifest(model)
        try:
            current = model.lstat()
            opened = os.fstat(held_model.descriptor)
        except OSError as error:
            raise PublicationError(f"model directory changed after snapshot: {error}") from None
        if _identity(current) != held_model.identity or _identity(opened) != held_model.identity:
            raise PublicationError("model directory changed after snapshot")
        _atomic_new_file(
            output,
            _manifest_bytes(manifest),
            forbidden_ancestor=held_model.identity,
        )
        return manifest
    finally:
        os.close(held_model.descriptor)


def _parse_manifest_bytes(data: bytes) -> PublicationManifest:
    document = _parse_json(data, "publication manifest")
    if not isinstance(document, dict) or set(document) != {"schema_version", "directories", "files"}:
        raise PublicationError("publication manifest root fields are not exact")
    if document["schema_version"] != _SCHEMA_VERSION:
        raise PublicationError("publication manifest schema mismatch")
    raw_directories = document["directories"]
    raw_files = document["files"]
    if not isinstance(raw_directories, list) or not isinstance(raw_files, list):
        raise PublicationError("publication manifest inventory is malformed")
    if len(raw_directories) + len(raw_files) > _MAX_ENTRIES:
        raise PublicationError("publication manifest exceeds entry limit")
    directories: list[PurePosixPath] = []
    seen_directories: set[PurePosixPath] = set()
    for raw in raw_directories:
        path_value = _safe_relative(raw) if isinstance(raw, str) else None
        if path_value is None or path_value in seen_directories:
            raise PublicationError("publication manifest has duplicate or malformed directory")
        for parent in path_value.parents:
            if parent != PurePosixPath(".") and parent not in seen_directories:
                raise PublicationError("publication manifest omits a parent directory")
        seen_directories.add(path_value)
        directories.append(path_value)
    files: list[ManifestFile] = []
    seen_files: set[PurePosixPath] = set()
    total = 0
    for raw in raw_files:
        if not isinstance(raw, dict) or set(raw) != {"path", "sha256", "size"}:
            raise PublicationError("publication manifest file record is malformed")
        path_value = _safe_relative(raw["path"]) if isinstance(raw["path"], str) else None
        size = raw["size"]
        digest = raw["sha256"]
        if (
            path_value is None
            or path_value in seen_files
            or path_value in seen_directories
            or isinstance(size, bool)
            or not isinstance(size, int)
            or size < 0
            or size > _MAX_FILE_BYTES
            or not isinstance(digest, str)
            or len(digest) != _DIGEST_LENGTH
            or any(character not in "0123456789abcdef" for character in digest)
        ):
            raise PublicationError("publication manifest file record is malformed")
        total += size
        if total > _MAX_TOTAL_BYTES:
            raise PublicationError("publication manifest exceeds total byte limit")
        for parent in path_value.parents:
            if parent != PurePosixPath(".") and parent not in seen_directories:
                raise PublicationError("publication manifest omits a parent directory")
        seen_files.add(path_value)
        files.append(ManifestFile(path_value, size, digest))
    manifest = PublicationManifest(tuple(directories), tuple(files))
    if not files or directories != sorted(directories, key=lambda path: path.as_posix()):
        raise PublicationError("publication manifest inventory order is not canonical")
    if files != sorted(files, key=lambda item: item.path.as_posix()):
        raise PublicationError("publication manifest file order is not canonical")
    if data != _manifest_bytes(manifest):
        raise PublicationError("publication manifest bytes are not canonical")
    return manifest


def _read_approved_manifest(path: Path, approved_sha256: str) -> tuple[bytes, PublicationManifest]:
    if (
        not isinstance(approved_sha256, str)
        or len(approved_sha256) != _DIGEST_LENGTH
        or any(character not in "0123456789abcdef" for character in approved_sha256)
    ):
        raise PublicationError("approved manifest SHA-256 must be exactly 64 lowercase hexadecimal characters")
    data = _read_stable_file(path, _MAX_MANIFEST_BYTES, "publication manifest")
    if not secrets.compare_digest(_sha256(data), approved_sha256):
        raise PublicationError("approved manifest SHA-256 digest mismatch")
    return data, _parse_manifest_bytes(data)


def verify_api_response(
    response: Path,
    *,
    owner: str,
    name: str,
    license_name: str,
    visibility: str,
    version_output: Path,
) -> int:
    """Validate the exact post-upload resource identity and persist its version."""
    if visibility not in {"private", "public"}:
        raise PublicationError("expected visibility must be private or public")
    if not all(isinstance(value, str) and value for value in (owner, name, license_name)):
        raise PublicationError("expected API identity fields must be non-empty text")
    document = _parse_json(_read_stable_file(response, _MAX_API_BYTES, "Fuel API response"), "Fuel API response")
    if not isinstance(document, dict):
        raise PublicationError("Fuel API response root must be an object")
    if document.get("owner") != owner:
        raise PublicationError("Fuel API owner mismatch")
    if document.get("name") != name:
        raise PublicationError("Fuel API resource name mismatch")
    if document.get("license_name") != license_name:
        raise PublicationError("Fuel API license mismatch")
    expected_private = visibility == "private"
    if document.get("private") is not expected_private:
        raise PublicationError("Fuel API visibility mismatch")
    version = document.get("version")
    if isinstance(version, bool) or not isinstance(version, int) or version <= 0:
        raise PublicationError("Fuel API version must be a positive integer")
    _atomic_new_file(Path(version_output), f"{version}\n".encode())
    return version


def verify_license_response(response: Path, license_name: str) -> None:
    """Require one exact license name in the typed Fuel license response."""
    if (
        not isinstance(license_name, str)
        or not license_name
        or license_name != license_name.strip()
        or any(ord(character) < 32 or ord(character) == 127 for character in license_name)
    ):
        raise PublicationError("expected license name must be exact non-empty text")
    document = _parse_json(
        _read_stable_file(response, _MAX_API_BYTES, "Fuel license API response"),
        "Fuel license API response",
    )
    if not isinstance(document, list) or not document or len(document) > 128:
        raise PublicationError("Fuel license API response must be one bounded non-empty list")
    names: list[str] = []
    for entry in document:
        if not isinstance(entry, dict):
            raise PublicationError("Fuel license API response has a malformed license record")
        name = entry.get("name")
        if (
            not isinstance(name, str)
            or not name
            or name != name.strip()
            or any(ord(character) < 32 or ord(character) == 127 for character in name)
        ):
            raise PublicationError("Fuel license API response has a malformed license name")
        names.append(name)
    if len(names) != len(set(names)):
        raise PublicationError("Fuel license API response has a duplicate license name")
    if names.count(license_name) != 1:
        raise PublicationError("Fuel license API response is missing the exact requested license")


def _zip_member_path(info: zipfile.ZipInfo) -> tuple[PurePosixPath, bool]:
    raw = info.filename
    is_directory = info.is_dir()
    if is_directory and not raw.endswith("/"):
        raise PublicationError(f"ZIP directory path is malformed: {raw!r}")
    path = _safe_relative(raw, directory=is_directory)
    mode = (info.external_attr >> 16) & 0xFFFF
    kind = stat.S_IFMT(mode)
    if is_directory:
        if kind not in {0, stat.S_IFDIR}:
            raise PublicationError(f"ZIP directory is a link or special entry: {raw}")
        if info.file_size != 0:
            raise PublicationError(f"ZIP directory has a payload: {raw}")
    elif kind not in {0, stat.S_IFREG}:
        raise PublicationError(f"ZIP member is a link or non-regular entry: {raw}")
    if info.flag_bits & 0x1:
        raise PublicationError(f"ZIP member is encrypted: {raw}")
    if info.compress_type not in {zipfile.ZIP_STORED, zipfile.ZIP_DEFLATED}:
        raise PublicationError(f"ZIP member uses an unsupported compression method: {raw}")
    return path, is_directory


def _read_zip_payload(
    archive: zipfile.ZipFile,
    info: zipfile.ZipInfo,
    expected: ManifestFile,
) -> bytes:
    if info.file_size != expected.size or info.file_size > _MAX_FILE_BYTES:
        raise PublicationError(f"ZIP member size mismatch: {expected.path}")
    try:
        with archive.open(info, "r") as stream:
            chunks: list[bytes] = []
            remaining = expected.size
            while remaining:
                chunk = stream.read(min(1024 * 1024, remaining))
                if not chunk:
                    break
                chunks.append(chunk)
                remaining -= len(chunk)
            extra = stream.read(1)
    except (
        EOFError,
        OSError,
        OverflowError,
        RuntimeError,
        UnicodeError,
        ValueError,
        struct.error,
        zipfile.BadZipFile,
        zlib.error,
    ) as error:
        raise PublicationError(f"unable to read ZIP member {expected.path}: {error}") from None
    if remaining or extra:
        raise PublicationError(f"ZIP member size mismatch: {expected.path}")
    data = b"".join(chunks)
    if _sha256(data) != expected.sha256:
        raise PublicationError(f"ZIP member hash mismatch: {expected.path}")
    return data


def _read_zip_directory(archive: zipfile.ZipFile, info: zipfile.ZipInfo, path: PurePosixPath) -> None:
    try:
        with archive.open(info, "r") as stream:
            payload = stream.read(1)
    except (
        EOFError,
        OSError,
        OverflowError,
        RuntimeError,
        UnicodeError,
        ValueError,
        struct.error,
        zipfile.BadZipFile,
        zlib.error,
    ) as error:
        raise PublicationError(f"unable to read ZIP directory {path}: {error}") from None
    if payload:
        raise PublicationError(f"ZIP directory has a payload: {path}")


def _directory_matches_fd(parent_fd: int, name: str, directory_fd: int) -> bool:
    try:
        child = os.stat(name, dir_fd=parent_fd, follow_symlinks=False)
        opened = os.fstat(directory_fd)
    except OSError:
        return False
    return stat.S_ISDIR(child.st_mode) and _identity(child) == _identity(opened)


def _reconcile_rename_after_error(
    operation_error: BaseException,
    parent: _HeldParent,
    directory_fd: int,
    source_name: str,
    destination_name: str,
) -> bool | None:
    """Report whether a failed/interrupted no-replace rename reached its destination."""
    try:
        opened = os.fstat(directory_fd)
        matches: list[str] = []
        for name in (source_name, destination_name):
            try:
                named = os.stat(name, dir_fd=parent.descriptor, follow_symlinks=False)
            except FileNotFoundError:
                continue
            if stat.S_ISDIR(named.st_mode) and _identity(named) == _identity(opened):
                matches.append(name)
    except BaseException as error:
        _add_exception_note(operation_error, f"output rename outcome reconciliation failed: {error}")
        return None
    if len(matches) != 1:
        _add_exception_note(
            operation_error,
            f"output rename outcome could not be reconciled safely: found {matches!r}",
        )
        return None
    return matches[0] == destination_name


def _rename_noreplace_reconciled(
    parent: _HeldParent,
    directory_fd: int,
    source_name: str,
    destination_name: str,
    record_outcome: Callable[[bool], None],
) -> None:
    try:
        _rename_noreplace(parent, source_name, destination_name)
    except BaseException as operation_error:
        outcome = _reconcile_rename_after_error(
            operation_error,
            parent,
            directory_fd,
            source_name,
            destination_name,
        )
        if outcome is not None:
            record_outcome(outcome)
        raise
    else:
        record_outcome(True)


def _clear_directory_descriptor(directory_fd: int) -> None:
    try:
        with os.scandir(directory_fd) as entries:
            names = [entry.name for entry in entries]
        for name in names:
            inspected = os.stat(name, dir_fd=directory_fd, follow_symlinks=False)
            if stat.S_ISDIR(inspected.st_mode):
                child_fd = os.open(
                    name,
                    os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC,
                    dir_fd=directory_fd,
                )
                try:
                    if _identity(os.fstat(child_fd)) != _identity(inspected):
                        raise PublicationError("cleanup directory changed while opening")
                    _clear_directory_descriptor(child_fd)
                    if not _directory_matches_fd(directory_fd, name, child_fd):
                        raise PublicationError("cleanup directory changed")
                finally:
                    os.close(child_fd)
                os.rmdir(name, dir_fd=directory_fd)
            else:
                os.unlink(name, dir_fd=directory_fd)
        os.fsync(directory_fd)
    except PublicationError:
        raise
    except OSError as error:
        raise PublicationError(f"unable to clean private output tree: {error}") from None


def _remove_private_tree(parent: _HeldParent, name: str, directory_fd: int) -> None:
    if not _directory_matches_fd(parent.descriptor, name, directory_fd):
        raise PublicationError("private output directory changed before cleanup")
    _clear_directory_descriptor(directory_fd)
    if not _directory_matches_fd(parent.descriptor, name, directory_fd):
        raise PublicationError("private output directory changed during cleanup")
    try:
        os.rmdir(name, dir_fd=parent.descriptor)
        os.fsync(parent.descriptor)
    except OSError as error:
        raise PublicationError(f"unable to remove private output directory: {error}") from None


def _verify_materialized_descriptors(
    directory_fds: dict[PurePosixPath, int],
    file_identities: dict[PurePosixPath, tuple[int, int, int, int, int, int]],
) -> None:
    root = PurePosixPath(".")
    for path, descriptor in directory_fds.items():
        if path == root:
            continue
        parent_fd = directory_fds[path.parent]
        if not _directory_matches_fd(parent_fd, path.name, descriptor):
            raise PublicationError(f"staged output directory changed: {path}")
    for path, expected_identity in file_identities.items():
        try:
            named = os.stat(path.name, dir_fd=directory_fds[path.parent], follow_symlinks=False)
        except OSError as error:
            raise PublicationError(f"staged output file changed: {path}: {error}") from None
        if not stat.S_ISREG(named.st_mode) or _stable_file_identity(named) != expected_identity:
            raise PublicationError(f"staged output file changed: {path}")


def _verify_materialized_content(
    directory_fds: dict[PurePosixPath, int],
    file_identities: dict[PurePosixPath, tuple[int, int, int, int, int, int]],
    manifest: PublicationManifest,
) -> None:
    root = PurePosixPath(".")
    expected_children: dict[PurePosixPath, dict[str, bool]] = {path: {} for path in (root, *manifest.directories)}
    for path in manifest.directories:
        expected_children[path.parent][path.name] = True
    expected_files = {item.path: item for item in manifest.files}
    for item in manifest.files:
        expected_children[item.path.parent][item.path.name] = False

    try:
        for path, descriptor in directory_fds.items():
            before = _stable_file_identity(os.fstat(descriptor))
            with os.scandir(descriptor) as entries:
                actual = {entry.name: stat.S_ISDIR(entry.stat(follow_symlinks=False).st_mode) for entry in entries}
            after = _stable_file_identity(os.fstat(descriptor))
            if before != after or actual != expected_children[path]:
                raise PublicationError(f"staged output inventory changed: {path}")

        for path, item in expected_files.items():
            descriptor = os.open(
                path.name,
                os.O_RDONLY | os.O_NONBLOCK | os.O_NOFOLLOW | os.O_CLOEXEC,
                dir_fd=directory_fds[path.parent],
            )
            try:
                opened = os.fstat(descriptor)
                if _stable_file_identity(opened) != file_identities[path]:
                    raise PublicationError(f"staged output file changed: {path}")
                chunks: list[bytes] = []
                remaining = item.size
                while remaining:
                    chunk = os.read(descriptor, min(1024 * 1024, remaining))
                    if not chunk:
                        break
                    chunks.append(chunk)
                    remaining -= len(chunk)
                extra = os.read(descriptor, 1)
                after = os.fstat(descriptor)
                if remaining or extra or _stable_file_identity(after) != file_identities[path]:
                    raise PublicationError(f"staged output file changed: {path}")
                if _sha256(b"".join(chunks)) != item.sha256:
                    raise PublicationError(f"staged output file hash mismatch: {path}")
            finally:
                os.close(descriptor)
    except PublicationError:
        raise
    except OSError as error:
        raise PublicationError(f"unable to verify materialized output: {error}") from None


def _make_materialized_read_only(
    directory_fds: dict[PurePosixPath, int],
    file_identities: dict[PurePosixPath, tuple[int, int, int, int, int, int]],
) -> None:
    try:
        for path, expected_identity in tuple(file_identities.items()):
            descriptor = os.open(
                path.name,
                os.O_RDONLY | os.O_NONBLOCK | os.O_NOFOLLOW | os.O_CLOEXEC,
                dir_fd=directory_fds[path.parent],
            )
            try:
                if _stable_file_identity(os.fstat(descriptor)) != expected_identity:
                    raise PublicationError(f"staged output file changed before read-only lock: {path}")
                os.fchmod(descriptor, 0o444)
                os.fsync(descriptor)
                locked = os.fstat(descriptor)
                if stat.S_IMODE(locked.st_mode) != 0o444:
                    raise PublicationError(f"unable to make staged output file read-only: {path}")
                file_identities[path] = _stable_file_identity(locked)
            finally:
                os.close(descriptor)
        for descriptor in reversed(tuple(directory_fds.values())):
            os.fchmod(descriptor, 0o555)
            os.fsync(descriptor)
            if stat.S_IMODE(os.fstat(descriptor).st_mode) != 0o555:
                raise PublicationError("unable to make staged output directory read-only")
    except PublicationError:
        raise
    except OSError as error:
        raise PublicationError(f"unable to make staged output read-only: {error}") from None


def _restore_private_directory_permissions(directory_fds: dict[PurePosixPath, int]) -> None:
    failures: list[OSError] = []
    for descriptor in directory_fds.values():
        try:
            os.fchmod(descriptor, 0o700)
        except OSError as error:
            failures.append(error)
    if failures:
        raise PublicationError(f"unable to restore private output permissions: {failures[0]}")


def _materialize_verified(
    output: Path,
    manifest: PublicationManifest,
    payloads: dict[PurePosixPath, bytes],
    *,
    forbidden_ancestor: tuple[int, int] | None = None,
    read_only: bool = False,
) -> None:
    output = Path(output)
    if not output.name or output.name in {".", ".."}:
        raise PublicationError("output must name one directory")
    parent = _open_parent(output)
    staging_name = f".{output.name}.{secrets.token_hex(16)}"
    staging_fd = -1
    staging_created = False
    installed_name: str | None = None
    directory_fds: dict[PurePosixPath, int] = {}
    file_identities: dict[PurePosixPath, tuple[int, int, int, int, int, int]] = {}
    success = False
    failure: BaseException | None = None
    cleanup_error: BaseException | None = None

    def record_rename_outcome(installed: bool) -> None:
        nonlocal installed_name
        installed_name = output.name if installed else None

    try:
        try:
            _verify_parent(parent)
            if forbidden_ancestor is not None and _descriptor_is_within(parent.descriptor, forbidden_ancestor):
                raise PublicationError("prepared upload output must remain outside the model tree")
            _require_absent(parent, output.name)
            os.mkdir(staging_name, mode=0o700, dir_fd=parent.descriptor)
            staging_created = True
            staging_fd = os.open(
                staging_name,
                os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC,
                dir_fd=parent.descriptor,
            )
            directory_fds[PurePosixPath(".")] = staging_fd
            for directory in sorted(manifest.directories, key=lambda path: (len(path.parts), path.as_posix())):
                parent_fd = directory_fds[directory.parent]
                os.mkdir(directory.name, mode=0o755, dir_fd=parent_fd)
                child_fd = os.open(
                    directory.name,
                    os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC,
                    dir_fd=parent_fd,
                )
                if not _directory_matches_fd(parent_fd, directory.name, child_fd):
                    os.close(child_fd)
                    raise PublicationError(f"staged output directory changed: {directory}")
                directory_fds[directory] = child_fd
            for item in manifest.files:
                descriptor = os.open(
                    item.path.name,
                    os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
                    0o600,
                    dir_fd=directory_fds[item.path.parent],
                )
                try:
                    with os.fdopen(os.dup(descriptor), "wb", closefd=True) as stream:
                        stream.write(payloads[item.path])
                        stream.flush()
                        os.fsync(stream.fileno())
                    identity = _stable_file_identity(os.fstat(descriptor))
                    named = os.stat(
                        item.path.name,
                        dir_fd=directory_fds[item.path.parent],
                        follow_symlinks=False,
                    )
                    if not stat.S_ISREG(named.st_mode) or _stable_file_identity(named) != identity:
                        raise PublicationError(f"staged output file changed: {item.path}")
                    file_identities[item.path] = identity
                finally:
                    if descriptor >= 0:
                        os.close(descriptor)
            for descriptor in reversed(tuple(directory_fds.values())):
                os.fsync(descriptor)
            _verify_parent(parent)
            _verify_materialized_descriptors(directory_fds, file_identities)
            if not _directory_matches_fd(parent.descriptor, staging_name, staging_fd):
                raise PublicationError("staged output directory changed")
            _require_absent(parent, output.name)
            _rename_noreplace_reconciled(
                parent,
                staging_fd,
                staging_name,
                output.name,
                record_rename_outcome,
            )
            os.fsync(parent.descriptor)
            _verify_parent(parent)
            if not _directory_matches_fd(parent.descriptor, output.name, staging_fd):
                raise PublicationError("installed output directory changed")
            _verify_materialized_descriptors(directory_fds, file_identities)
            _verify_materialized_content(directory_fds, file_identities, manifest)
            _verify_parent(parent)
            if not _directory_matches_fd(parent.descriptor, output.name, staging_fd):
                raise PublicationError("installed output directory changed after validation")
            _verify_materialized_descriptors(directory_fds, file_identities)
            _verify_materialized_content(directory_fds, file_identities, manifest)
            if read_only:
                _make_materialized_read_only(directory_fds, file_identities)
                _verify_parent(parent)
                if not _directory_matches_fd(parent.descriptor, output.name, staging_fd):
                    raise PublicationError("read-only output directory changed")
                _verify_materialized_descriptors(directory_fds, file_identities)
                _verify_materialized_content(directory_fds, file_identities, manifest)
            success = True
        except PublicationError:
            raise
        except OSError as error:
            raise PublicationError(f"unable to materialize verified ZIP safely: {error}") from None
    except BaseException as error:
        failure = error
    finally:
        if not success:
            permission_error: BaseException | None = None
            if directory_fds:
                try:
                    _restore_private_directory_permissions(directory_fds)
                except BaseException as error:
                    permission_error = error
            try:
                if staging_fd >= 0:
                    _remove_private_tree(parent, installed_name or staging_name, staging_fd)
                elif staging_created:
                    os.rmdir(staging_name, dir_fd=parent.descriptor)
                    os.fsync(parent.descriptor)
            except BaseException as error:
                cleanup_error = error
            if permission_error is not None:
                if cleanup_error is not None:
                    _add_exception_note(permission_error, f"private output removal also failed: {cleanup_error}")
                cleanup_error = permission_error
        for path, descriptor in reversed(tuple(directory_fds.items())):
            del path
            os.close(descriptor)
        os.close(parent.descriptor)
    if failure is not None:
        if cleanup_error is not None:
            _add_exception_note(failure, f"verified ZIP cleanup failed: {cleanup_error}")
        raise failure
    if cleanup_error is not None:
        raise PublicationError(f"verified ZIP cleanup failed: {cleanup_error}")


def prepare_upload(
    model: Path,
    manifest_path: Path,
    output: Path,
    *,
    manifest_sha256: str,
) -> PublicationManifest:
    """Create a read-only upload tree; root and same-euid processes remain trusted."""
    model = Path(model)
    manifest_path = Path(manifest_path)
    output = Path(output)
    if _path_is_lexically_within(manifest_path, model):
        raise PublicationError("publication manifest must remain outside the model tree")
    if _path_is_lexically_within(output, model):
        raise PublicationError("prepared upload output must remain outside the model tree")
    _, expected = _read_approved_manifest(manifest_path, manifest_sha256)
    held_model = _open_directory(model, "model directory")
    try:
        snapshot = _capture_model_snapshot(model)
        observed = _manifest_from_snapshot(snapshot)
        try:
            current = model.lstat()
            opened = os.fstat(held_model.descriptor)
        except OSError as error:
            raise PublicationError(f"model directory changed while preparing upload: {error}") from None
        if _identity(current) != held_model.identity or _identity(opened) != held_model.identity:
            raise PublicationError("model directory changed while preparing upload")
        if observed != expected:
            raise PublicationError("model no longer matches the approved publication manifest")
        _materialize_verified(
            output,
            expected,
            dict(snapshot.files),
            forbidden_ancestor=held_model.identity,
            read_only=True,
        )
        return expected
    finally:
        os.close(held_model.descriptor)


def verify_raw_zip(
    zip_path: Path,
    manifest_path: Path,
    output: Path,
    *,
    manifest_sha256: str,
) -> None:
    """Verify an exact raw Fuel ZIP against external evidence, then materialize it."""
    _, manifest = _read_approved_manifest(Path(manifest_path), manifest_sha256)
    zip_data = _read_stable_file(Path(zip_path), _MAX_ZIP_BYTES, "raw Fuel ZIP")
    expected_files = {item.path: item for item in manifest.files}
    expected_directories = set(manifest.directories)
    payloads: dict[PurePosixPath, bytes] = {}
    explicit_directories: set[PurePosixPath] = set()
    seen: set[PurePosixPath] = set()
    derived_directories: set[PurePosixPath] = set()
    try:
        with zipfile.ZipFile(io.BytesIO(zip_data), "r") as archive:
            infos = archive.infolist()
            if len(infos) > _MAX_ENTRIES:
                raise PublicationError("ZIP exceeds entry limit")
            checked_infos: list[tuple[zipfile.ZipInfo, PurePosixPath, bool]] = []
            for info in infos:
                path, is_directory = _zip_member_path(info)
                if path in seen:
                    raise PublicationError(f"ZIP has duplicate member: {path}")
                seen.add(path)
                checked_infos.append((info, path, is_directory))
            for info, path, is_directory in checked_infos:
                if is_directory:
                    _read_zip_directory(archive, info, path)
                    explicit_directories.add(path)
                    continue
                expected = expected_files.get(path)
                if expected is None:
                    raise PublicationError(f"ZIP inventory has unexpected file: {path}")
                for parent_path in path.parents:
                    if parent_path != PurePosixPath("."):
                        derived_directories.add(parent_path)
                payloads[path] = _read_zip_payload(archive, info, expected)
    except PublicationError:
        raise
    except (
        EOFError,
        OSError,
        OverflowError,
        RuntimeError,
        UnicodeError,
        ValueError,
        struct.error,
        zipfile.BadZipFile,
        zipfile.LargeZipFile,
        zlib.error,
    ) as error:
        raise PublicationError(f"raw Fuel ZIP is malformed: {error}") from None
    if set(payloads) != set(expected_files):
        raise PublicationError("ZIP inventory is missing expected files")
    if not explicit_directories.issubset(expected_directories):
        raise PublicationError("ZIP inventory has an unexpected directory")
    if derived_directories | explicit_directories != expected_directories:
        raise PublicationError("ZIP directory inventory mismatch")
    _materialize_verified(Path(output), manifest, payloads)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    manifest = commands.add_parser("manifest", help="write a typed external model manifest")
    manifest.add_argument("--model", type=Path, required=True)
    manifest.add_argument("--output", type=Path, required=True)
    prepare = commands.add_parser("prepare-upload", help="create an exact independent read-only upload tree")
    prepare.add_argument("--model", type=Path, required=True)
    prepare.add_argument("--manifest", type=Path, required=True)
    prepare.add_argument("--manifest-sha256", required=True)
    prepare.add_argument("--output", type=Path, required=True)
    licenses = commands.add_parser("verify-licenses", help="verify one exact name in a Fuel license API response")
    licenses.add_argument("--response", type=Path, required=True)
    licenses.add_argument("--license", dest="license_name", required=True)
    api = commands.add_parser("verify-api", help="verify a captured Fuel resource API response")
    api.add_argument("--response", type=Path, required=True)
    api.add_argument("--owner", required=True)
    api.add_argument("--name", required=True)
    api.add_argument("--license", dest="license_name", required=True)
    api.add_argument("--visibility", choices=("private", "public"), required=True)
    api.add_argument("--version-output", type=Path, required=True)
    raw_zip = commands.add_parser("verify-zip", help="verify and safely materialize a raw Fuel ZIP")
    raw_zip.add_argument("--zip", dest="zip_path", type=Path, required=True)
    raw_zip.add_argument("--manifest", type=Path, required=True)
    raw_zip.add_argument("--manifest-sha256", required=True)
    raw_zip.add_argument("--output", type=Path, required=True)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.command == "manifest":
            manifest = write_manifest(args.model, args.output)
            print(f"PUBLICATION_MANIFEST_OK: {len(manifest.files)} files")
        elif args.command == "prepare-upload":
            manifest = prepare_upload(
                args.model,
                args.manifest,
                args.output,
                manifest_sha256=args.manifest_sha256,
            )
            print(f"UPLOAD_SNAPSHOT_OK: {len(manifest.files)} files, read-only")
        elif args.command == "verify-licenses":
            verify_license_response(args.response, args.license_name)
            print("FUEL_LICENSE_OK")
        elif args.command == "verify-api":
            version = verify_api_response(
                args.response,
                owner=args.owner,
                name=args.name,
                license_name=args.license_name,
                visibility=args.visibility,
                version_output=args.version_output,
            )
            print(f"FUEL_API_OK: version {version}")
        else:
            verify_raw_zip(
                args.zip_path,
                args.manifest,
                args.output,
                manifest_sha256=args.manifest_sha256,
            )
            print("RAW_ZIP_OK")
    except (MemoryError, PublicationError) as error:
        print(f"PUBLICATION_ERROR: {str(error)!r}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
