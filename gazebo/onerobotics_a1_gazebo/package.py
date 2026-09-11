"""Export self-contained Gazebo Fuel packages and reproducible review archives."""

from __future__ import annotations

import argparse
import errno
import gzip
import hashlib
import io
import json
import os
import shutil
import stat
import tarfile
import tempfile
import uuid
from contextlib import suppress
from dataclasses import dataclass, replace
from pathlib import Path, PurePosixPath
from string import Template
from xml.etree import ElementTree

from onerobotics_a1_gazebo.sdf import convert_urdf, serialize_sdf
from onerobotics_a1_gazebo.source_lock import discover_source_files, sha256_file, validate_source_lock
from onerobotics_a1_gazebo.spec import (
    SOURCE_COMMIT,
    SOURCE_REPOSITORY,
    ModelSpec,
    load_hardware_overlay,
    load_model_specs,
    source_root,
)
from onerobotics_a1_gazebo.thumbnail import approved_assets_root, capture_thumbnail_set

PACKAGE_VERSION = "1.0.0"
MANIFEST_SCHEMA_VERSION = 1
EXPORT_MARKER = ".onerobotics-a1-gazebo-export.json"
CHANGES_STATEMENT = (
    "Changes: Converted the published URDF description to native SDFormat 1.11; "
    "anchored the fixed-base mechanism to the SDF world frame; rewrote mesh paths "
    "for a self-contained Fuel package; overlaid the published peak effort and rated "
    "velocity limits from model_parameters.yaml. Geometry, mass, inertia, joint "
    "poses, axes, and position limits were not changed."
)

_DESCRIPTIONS = {
    "right_arm": "Native SDFormat 1.11 model of the OneRobotics A1 right arm.",
    "left_arm": "Native SDFormat 1.11 model of the OneRobotics A1 left arm.",
    "bimanual_stand": "Native SDFormat 1.11 model of the OneRobotics A1 bimanual stand.",
}

_EXPORTER_ATTRIBUTIONS = {
    "right_arm": "The published right-arm URDF contains no exporter attribution.",
    "left_arm": (
        "SolidWorks to URDF Exporter attribution retained from the source URDF: "
        "originally created by Stephen Brawner; commit version 1.6.0-4-g7f85cfe, "
        "build version 1.6.7995.38578. See http://wiki.ros.org/sw_urdf_exporter."
    ),
    "bimanual_stand": (
        "SolidWorks to URDF Exporter attribution retained from the source URDF: "
        "originally created by Stephen Brawner; commit version 1.6.0-4-g7f85cfe, "
        "build version 1.6.7995.38578. See http://wiki.ros.org/sw_urdf_exporter."
    ),
}


@dataclass(frozen=True)
class ExportedModel:
    directory: Path
    archive: Path
    archive_sha256: str


@dataclass(frozen=True)
class _ArchiveEntry:
    relative_path: PurePosixPath
    data: bytes | None


def _repository_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _templates_root() -> Path:
    return Path(__file__).resolve().parents[1] / "templates"


def _render_template(name: str, spec: ModelSpec) -> str:
    template_path = _templates_root() / name
    try:
        template = Template(template_path.read_text(encoding="utf-8"))
    except OSError as error:
        raise ValueError(f"Unable to read package template: {template_path}") from error
    return template.substitute(
        display_name=spec.display_name,
        description=_DESCRIPTIONS[spec.key],
        exporter_attribution=_EXPORTER_ATTRIBUTIONS[spec.key],
    )


def _source_meshes(spec: ModelSpec, assets_root: Path) -> tuple[Path, ...]:
    try:
        urdf = ElementTree.parse(spec.urdf).getroot()
    except (OSError, ElementTree.ParseError) as error:
        raise ValueError(f"Unable to read source URDF: {spec.urdf}") from error

    meshes: dict[str, Path] = {}
    for mesh in urdf.findall(".//mesh"):
        filename = mesh.attrib.get("filename")
        if not filename:
            raise ValueError(f"Source URDF mesh is missing filename: {spec.urdf}")
        relative = PurePosixPath(filename.replace("\\", "/"))
        if relative.is_absolute() or ".." in relative.parts:
            raise ValueError(f"Source URDF has non-portable mesh path: {filename}")
        source = (spec.urdf.parent / Path(*relative.parts)).resolve()
        try:
            source.relative_to(assets_root)
        except ValueError as error:
            raise ValueError(f"Source mesh escapes the public source root: {filename}") from error
        if not source.is_file():
            raise ValueError(f"Missing source mesh: {source}")
        previous = meshes.setdefault(source.name, source)
        if previous != source:
            raise ValueError(f"Source mesh basename collision: {source.name}")
    return tuple(meshes[name] for name in sorted(meshes))


def _locked_source_digests() -> dict[str, str]:
    lock_path = Path(__file__).resolve().parents[1] / "generated-manifest.json"
    try:
        lock = json.loads(lock_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"Unable to read source lock: {lock_path}") from error
    files = lock.get("files") if isinstance(lock, dict) else None
    if not isinstance(files, dict) or not all(
        isinstance(path, str) and isinstance(digest, str) for path, digest in files.items()
    ):
        raise ValueError("Source lock files must map paths to SHA-256 strings")
    return files


def _source_digests(
    spec: ModelSpec,
    meshes: tuple[Path, ...],
    assets_root: Path,
    locked_digests: dict[str, str],
) -> dict[Path, str]:
    inputs = (spec.urdf, assets_root / "model_parameters.yaml", *meshes)
    digests: dict[Path, str] = {}
    for path in inputs:
        relative_path = path.relative_to(assets_root).as_posix()
        expected = locked_digests.get(relative_path)
        if expected is None or sha256_file(path) != expected:
            raise ValueError(f"Locked snapshot digest mismatch: {relative_path}")
        digests[path] = expected
    return digests


def _write_source_manifest(
    package_dir: Path,
    spec: ModelSpec,
    source_digests: dict[Path, str],
    assets_root: Path,
) -> None:
    sources = [
        {
            "path": path.resolve().relative_to(assets_root).as_posix(),
            "sha256": digest,
        }
        for path, digest in source_digests.items()
    ]
    sources.sort(key=lambda source: source["path"])
    manifest = {
        "schema_version": MANIFEST_SCHEMA_VERSION,
        "source_url": SOURCE_REPOSITORY,
        "source_commit": SOURCE_COMMIT,
        "model_key": spec.key,
        "changes": CHANGES_STATEMENT,
        "sources": sources,
    }
    (package_dir / "SOURCE_MANIFEST.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


def _export_one(
    package_dir: Path,
    spec: ModelSpec,
    assets_root: Path,
    locked_digests: dict[str, str],
    sdf_data: bytes,
    thumbnail_data: bytes,
) -> None:
    package_dir.mkdir()
    meshes_dir = package_dir / "meshes"
    meshes_dir.mkdir()
    thumbnails_dir = package_dir / "thumbnails"
    thumbnails_dir.mkdir()

    (package_dir / "model.sdf").write_bytes(sdf_data)
    (thumbnails_dir / "0.png").write_bytes(thumbnail_data)
    for template_name, output_name in (
        ("model.config.xml", "model.config"),
        ("metadata.pbtxt", "metadata.pbtxt"),
        ("README.md", "README.md"),
        ("NOTICE", "NOTICE"),
    ):
        (package_dir / output_name).write_text(
            _render_template(template_name, spec),
            encoding="utf-8",
        )

    meshes = _source_meshes(spec, assets_root)
    source_digests = _source_digests(spec, meshes, assets_root, locked_digests)
    for source in meshes:
        destination = meshes_dir / source.name
        shutil.copyfile(source, destination)
        if sha256_file(destination) != source_digests[source]:
            raise ValueError(f"Copied mesh digest mismatch: {source.name}")
    shutil.copyfile(_repository_root() / "LICENSES/CC-BY-4.0.txt", package_dir / "LICENSE")
    _write_source_manifest(package_dir, spec, source_digests, assets_root)


def _copy_source_snapshot(public_root: Path, snapshot_root: Path) -> None:
    for source in discover_source_files(public_root):
        destination = snapshot_root / source.relative_to(public_root)
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, destination)


def _snapshot_specs(specs: tuple[ModelSpec, ...], public_root: Path, snapshot_root: Path) -> tuple[ModelSpec, ...]:
    return tuple(replace(spec, urdf=snapshot_root / spec.urdf.relative_to(public_root)) for spec in specs)


def _normalized_tar_info(entry: _ArchiveEntry, archive_name: PurePosixPath) -> tarfile.TarInfo:
    info = tarfile.TarInfo(archive_name.as_posix())
    info.uid = 0
    info.gid = 0
    info.uname = ""
    info.gname = ""
    info.mtime = 0
    if entry.data is None:
        info.type = tarfile.DIRTYPE
        info.mode = 0o755
    else:
        info.type = tarfile.REGTYPE
        info.mode = 0o644
        info.size = len(entry.data)
    return info


def _add_normalized_tar_entry(
    archive: tarfile.TarFile,
    entry: _ArchiveEntry,
    archive_name: PurePosixPath,
) -> None:
    info = _normalized_tar_info(entry, archive_name)
    if info.isfile():
        assert entry.data is not None
        archive.addfile(info, io.BytesIO(entry.data))
    else:
        archive.addfile(info)


def _file_identity(details: os.stat_result) -> tuple[int, int, int]:
    return details.st_dev, details.st_ino, stat.S_IFMT(details.st_mode)


def _directory_names(directory_fd: int) -> tuple[str, ...]:
    with os.scandir(directory_fd) as iterator:
        return tuple(sorted(entry.name for entry in iterator))


def _open_scanned_entry(
    directory_fd: int,
    name: str,
    relative_path: PurePosixPath,
) -> tuple[int, os.stat_result]:
    try:
        scanned = os.stat(name, dir_fd=directory_fd, follow_symlinks=False)
    except OSError as error:
        raise ValueError(f"Archive input changed during archive scan: {relative_path}") from error
    if stat.S_ISLNK(scanned.st_mode):
        raise ValueError(f"Archive input must not contain symlinks: {relative_path}")
    if stat.S_ISDIR(scanned.st_mode):
        flags = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC
    elif stat.S_ISREG(scanned.st_mode):
        flags = os.O_RDONLY | os.O_NONBLOCK | os.O_NOFOLLOW | os.O_CLOEXEC
    else:
        raise ValueError(f"Archive input must be a regular file or directory: {relative_path}")
    try:
        descriptor = os.open(name, flags, dir_fd=directory_fd)
    except OSError as error:
        if error.errno in {errno.ELOOP, errno.ENOENT, errno.ENOTDIR}:
            raise ValueError(f"Archive input changed during archive scan: {relative_path}") from error
        raise ValueError(f"Unable to open archive input: {relative_path}") from error
    opened = os.fstat(descriptor)
    if _file_identity(opened) != _file_identity(scanned):
        os.close(descriptor)
        raise ValueError(f"Archive input changed during archive scan: {relative_path}")
    return descriptor, opened


def _read_stable_file(
    descriptor: int,
    opened: os.stat_result,
    relative_path: PurePosixPath,
) -> bytes:
    chunks: list[bytes] = []
    while chunk := os.read(descriptor, 1024 * 1024):
        chunks.append(chunk)
    after = os.fstat(descriptor)
    if (
        _file_identity(after) != _file_identity(opened)
        or after.st_size != opened.st_size
        or after.st_mtime_ns != opened.st_mtime_ns
        or after.st_ctime_ns != opened.st_ctime_ns
    ):
        raise ValueError(f"Archive input changed while being read: {relative_path}")
    data = b"".join(chunks)
    if len(data) != opened.st_size:
        raise ValueError(f"Archive input changed while being read: {relative_path}")
    return data


def _scan_archive_directory(
    directory_fd: int,
    prefix: PurePosixPath = PurePosixPath(),
) -> list[_ArchiveEntry]:
    before = os.fstat(directory_fd)
    names = _directory_names(directory_fd)
    entries: list[_ArchiveEntry] = []
    for name in names:
        relative_path = prefix / name
        descriptor, opened = _open_scanned_entry(directory_fd, name, relative_path)
        try:
            if stat.S_ISDIR(opened.st_mode):
                entries.append(_ArchiveEntry(relative_path, None))
                entries.extend(_scan_archive_directory(descriptor, relative_path))
            else:
                entries.append(_ArchiveEntry(relative_path, _read_stable_file(descriptor, opened, relative_path)))
        finally:
            os.close(descriptor)
    after = os.fstat(directory_fd)
    if (
        names != _directory_names(directory_fd)
        or _file_identity(after) != _file_identity(before)
        or after.st_mtime_ns != before.st_mtime_ns
        or after.st_ctime_ns != before.st_ctime_ns
    ):
        raise ValueError(f"Archive input changed during archive scan: {prefix or '.'}")
    return entries


def _open_model_directory(model_dir: Path) -> tuple[Path, int]:
    if not model_dir.name:
        raise ValueError(f"Model directory must be a named regular directory: {model_dir}")
    try:
        parent = model_dir.parent.resolve(strict=True)
        parent_fd = os.open(parent, os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC)
    except OSError as error:
        raise ValueError(f"Model directory must be a regular directory: {model_dir}") from error
    try:
        descriptor, opened = _open_scanned_entry(parent_fd, model_dir.name, PurePosixPath(model_dir.name))
    finally:
        os.close(parent_fd)
    if not stat.S_ISDIR(opened.st_mode):
        os.close(descriptor)
        raise ValueError(f"Model directory must be a regular directory: {model_dir}")
    return parent / model_dir.name, descriptor


def _canonical_archive_destination(archive_path: Path) -> Path:
    if not archive_path.name:
        raise ValueError("Archive destination must have a file name")
    try:
        parent = archive_path.parent.resolve(strict=True)
    except OSError as error:
        raise ValueError(f"Archive parent directory does not exist: {archive_path.parent}") from error
    destination = parent / archive_path.name
    try:
        details = os.stat(destination, follow_symlinks=False)
    except FileNotFoundError:
        return destination
    except OSError as error:
        raise ValueError(f"Unable to inspect archive destination: {destination}") from error
    if stat.S_ISLNK(details.st_mode):
        raise ValueError(f"Refusing symlink archive destination: {destination}")
    if not stat.S_ISREG(details.st_mode):
        raise ValueError(f"Archive destination must be a regular file: {destination}")
    return destination


def _sha256_descriptor(descriptor: int) -> str:
    digest = hashlib.sha256()
    os.lseek(descriptor, 0, os.SEEK_SET)
    while chunk := os.read(descriptor, 1024 * 1024):
        digest.update(chunk)
    return digest.hexdigest()


def _same_file(path: Path, expected: os.stat_result) -> bool:
    try:
        actual = os.stat(path, follow_symlinks=False)
    except FileNotFoundError:
        return False
    return _file_identity(actual) == _file_identity(expected)


def create_reproducible_archive(model_dir: Path, archive_path: Path) -> str:
    """Create a normalized tar.gz archive and return its SHA-256 digest."""
    model_root, model_fd = _open_model_directory(model_dir)
    try:
        entries = _scan_archive_directory(model_fd)
    finally:
        os.close(model_fd)
    destination = _canonical_archive_destination(archive_path)
    if destination == model_root or model_root in destination.parents:
        raise ValueError("Archive path must be outside the model directory")

    archive_root = PurePosixPath(f"{model_root.name}-{PACKAGE_VERSION}")
    temporary_fd, temporary_name = tempfile.mkstemp(prefix=f".{destination.name}.tmp-", dir=destination.parent)
    temporary = Path(temporary_name)
    temporary_identity = os.fstat(temporary_fd)
    installed = False
    try:
        with os.fdopen(temporary_fd, "wb", closefd=False) as raw:
            with gzip.GzipFile(filename="", mode="wb", fileobj=raw, mtime=0) as zipped:
                with tarfile.open(fileobj=zipped, mode="w", format=tarfile.PAX_FORMAT) as archive:
                    for entry in entries:
                        _add_normalized_tar_entry(archive, entry, archive_root / entry.relative_path)
        digest = _sha256_descriptor(temporary_fd)
        if not _same_file(temporary, temporary_identity):
            raise ValueError("Temporary archive destination changed before installation")
        os.close(temporary_fd)
        temporary_fd = -1
        os.replace(temporary, destination)
        installed = True
        return digest
    finally:
        if temporary_fd >= 0:
            os.close(temporary_fd)
        if not installed and _same_file(temporary, temporary_identity):
            temporary.unlink()


def _validate_output_root(output_root: Path) -> Path:
    resolved = output_root.resolve()
    dangerous = {Path("/"), Path.home().resolve(), _repository_root().resolve()}
    if resolved in dangerous:
        raise ValueError(f"Refusing dangerous output root: {resolved}")
    if output_root.is_symlink():
        raise ValueError(f"Refusing symlink output root: {output_root}")
    if output_root.exists() and not output_root.is_dir():
        raise ValueError(f"Existing output root is not a generated export: {output_root}")
    return resolved


def _is_generated_export(output_root: Path) -> bool:
    marker = output_root / EXPORT_MARKER
    if marker.is_symlink() or not marker.is_file():
        return False
    try:
        value = json.loads(marker.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return False
    return value == {"generator": "onerobotics_a1_gazebo.package", "schema_version": 1}


def _install_staging_tree(staging: Path, output_root: Path) -> None:
    if not output_root.exists():
        os.replace(staging, output_root)
        return
    if not _is_generated_export(output_root):
        raise ValueError(f"Existing output root is not a generated export: {output_root}")

    backup = output_root.parent / f".{output_root.name}.backup-{uuid.uuid4().hex}"
    os.replace(output_root, backup)
    try:
        os.replace(staging, output_root)
    except BaseException:
        os.replace(backup, output_root)
        raise
    # The new tree is already authoritative. Keep the old tree recoverable
    # rather than reporting a failed export or attempting an impossible rollback.
    with suppress(OSError):
        shutil.rmtree(backup)


def export_models(output_root: Path) -> tuple[ExportedModel, ...]:
    """Atomically export all three public A1 Fuel packages below ``output_root``."""
    destination = _validate_output_root(output_root)
    if destination.exists() and not _is_generated_export(destination):
        raise ValueError(f"Existing output root is not a generated export: {destination}")

    public_root = source_root().resolve()
    validate_source_lock(source_root=public_root)
    specs = load_model_specs()
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(
        prefix=f".{destination.name}.source-snapshot-", dir=destination.parent
    ) as snapshot_name:
        snapshot_root = Path(snapshot_name)
        _copy_source_snapshot(public_root, snapshot_root)
        validate_source_lock(source_root=snapshot_root)
        locked_digests = _locked_source_digests()
        snapshot_specs = _snapshot_specs(specs, public_root, snapshot_root)
        overlay = load_hardware_overlay(snapshot_root / "model_parameters.yaml")
        sdf_by_slug = {spec.slug: serialize_sdf(convert_urdf(spec, overlay)) for spec in snapshot_specs}
        model_sdf_hashes = {slug: hashlib.sha256(data).hexdigest() for slug, data in sdf_by_slug.items()}
        approved = capture_thumbnail_set(
            approved_assets_root(),
            expected_model_sdf_sha256=model_sdf_hashes,
        )

        staging = Path(tempfile.mkdtemp(prefix=f".{destination.name}.stage-", dir=destination.parent))
        try:
            archives_dir = staging / "archives"
            archives_dir.mkdir()
            archive_hashes: list[str] = []
            for spec in snapshot_specs:
                package_dir = staging / spec.slug
                _export_one(
                    package_dir,
                    spec,
                    snapshot_root,
                    locked_digests,
                    sdf_by_slug[spec.slug],
                    approved.png_by_slug[spec.slug],
                )
                archive_hashes.append(
                    create_reproducible_archive(
                        package_dir,
                        archives_dir / f"{spec.slug}-{PACKAGE_VERSION}.tar.gz",
                    )
                )
            (staging / EXPORT_MARKER).write_text(
                json.dumps({"generator": "onerobotics_a1_gazebo.package", "schema_version": 1}) + "\n",
                encoding="utf-8",
            )
            _install_staging_tree(staging, destination)
        except BaseException:
            if staging.exists():
                shutil.rmtree(staging)
            raise

    return tuple(
        ExportedModel(
            directory=destination / spec.slug,
            archive=destination / "archives" / f"{spec.slug}-{PACKAGE_VERSION}.tar.gz",
            archive_sha256=archive_hash,
        )
        for spec, archive_hash in zip(specs, archive_hashes, strict=True)
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="Generated Fuel package root")
    arguments = parser.parse_args()
    exported = export_models(arguments.output)
    for item in exported:
        print(f"EXPORTED {item.directory.name}: {item.archive_sha256}")
    print(f"EXPORT_OK: {len(exported)} models")


if __name__ == "__main__":
    main()
