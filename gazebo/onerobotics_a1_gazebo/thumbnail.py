"""Strict, immutable thumbnail and render-manifest handling."""

from __future__ import annotations

import binascii
import hashlib
import json
import os
import re
import stat
import struct
import tempfile
import zlib
from collections.abc import Mapping
from dataclasses import dataclass, replace
from pathlib import Path, PurePosixPath
from types import MappingProxyType

import onerobotics_a1_gazebo.spec as _spec
from onerobotics_a1_gazebo.sdf import convert_urdf, serialize_sdf
from onerobotics_a1_gazebo.source_lock import validate_source_lock
from onerobotics_a1_gazebo.spec import load_hardware_overlay, load_model_specs
from onerobotics_a1_gazebo.validation_snapshot import DirectorySnapshot, SnapshotError, capture_directory

PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"
THUMBNAIL_WIDTH = 512
THUMBNAIL_HEIGHT = 512
MANIFEST_NAME = "render-manifest.json"
MANIFEST_SCHEMA_VERSION = 1
RENDER_ENGINE = "ogre2"
RENDER_BACKEND = "software"
_MAX_PNG_BYTES = 4 * 1024 * 1024
_MAX_MANIFEST_BYTES = 64 * 1024
_MAX_RENDER_CONFIG_BYTES = 1024 * 1024
_EXPECTED_SCANLINE_BYTES = THUMBNAIL_HEIGHT * (1 + THUMBNAIL_WIDTH * 3)
_SHA256 = re.compile(r"[0-9a-f]{64}")


@dataclass(frozen=True)
class PngInfo:
    width: int
    height: int
    bit_depth: int
    color_type: int


@dataclass(frozen=True)
class ThumbnailRecord:
    slug: str
    file: str
    png_sha256: str
    width: int
    height: int
    model_sdf_sha256: str


@dataclass(frozen=True)
class ThumbnailSet:
    png_by_slug: Mapping[str, bytes]
    records_by_slug: Mapping[str, ThumbnailRecord]
    manifest_bytes: bytes
    render_config_sha256: str


def thumbnail_slugs() -> tuple[str, ...]:
    return tuple(spec.slug for spec in load_model_specs())


def render_config_path() -> Path:
    return Path(__file__).resolve().parents[1] / "config/render.yaml"


def approved_assets_root() -> Path:
    return Path(__file__).resolve().parents[1] / "assets/thumbnails"


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def capture_render_config_bytes(path: Path | None = None) -> bytes:
    """Read one stable regular config file through a retained no-follow parent fd."""
    config_path = Path(path) if path is not None else render_config_path()
    if not config_path.is_absolute() or not config_path.name or config_path.name in {".", ".."}:
        raise ValueError("trusted render config path must be one absolute file path")
    parent = config_path.parent
    try:
        parent_inspected = parent.lstat()
        if stat.S_ISLNK(parent_inspected.st_mode) or not stat.S_ISDIR(parent_inspected.st_mode):
            raise ValueError("trusted render config parent must be a regular non-symlink directory")
        parent_fd = os.open(parent, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC)
    except ValueError:
        raise
    except OSError as error:
        raise ValueError(f"unable to open trusted render config parent: {error}") from None
    descriptor = -1
    try:
        parent_opened = os.fstat(parent_fd)
        if (parent_inspected.st_dev, parent_inspected.st_ino) != (parent_opened.st_dev, parent_opened.st_ino):
            raise ValueError("trusted render config parent changed while opening")
        inspected = os.stat(config_path.name, dir_fd=parent_fd, follow_symlinks=False)
        if stat.S_ISLNK(inspected.st_mode) or not stat.S_ISREG(inspected.st_mode):
            raise ValueError("trusted render config must be a regular non-symlink file")
        if inspected.st_size > _MAX_RENDER_CONFIG_BYTES:
            raise ValueError("trusted render config exceeds size limit")
        descriptor = os.open(
            config_path.name,
            os.O_RDONLY | os.O_NONBLOCK | os.O_NOFOLLOW | os.O_CLOEXEC,
            dir_fd=parent_fd,
        )
        opened = os.fstat(descriptor)
        identity = (inspected.st_dev, inspected.st_ino, inspected.st_size, inspected.st_mtime_ns, inspected.st_ctime_ns)
        if identity != (opened.st_dev, opened.st_ino, opened.st_size, opened.st_mtime_ns, opened.st_ctime_ns):
            raise ValueError("trusted render config changed while opening")
        chunks: list[bytes] = []
        remaining = opened.st_size
        while remaining:
            chunk = os.read(descriptor, min(remaining, 64 * 1024))
            if not chunk:
                break
            chunks.append(chunk)
            remaining -= len(chunk)
        if remaining or os.read(descriptor, 1):
            raise ValueError("trusted render config changed while reading")
        after = os.fstat(descriptor)
        at_name = os.stat(config_path.name, dir_fd=parent_fd, follow_symlinks=False)
        final_identity = (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns, after.st_ctime_ns)
        named_identity = (
            at_name.st_dev,
            at_name.st_ino,
            at_name.st_size,
            at_name.st_mtime_ns,
            at_name.st_ctime_ns,
        )
        if final_identity != identity or named_identity != identity:
            raise ValueError("trusted render config changed while reading")
        return b"".join(chunks)
    except ValueError:
        raise
    except OSError as error:
        raise ValueError(f"unable to read trusted render config: {error}") from None
    finally:
        if descriptor >= 0:
            os.close(descriptor)
        os.close(parent_fd)


def current_render_config_sha256() -> str:
    return sha256_bytes(capture_render_config_bytes())


def _materialize_source_snapshot(snapshot: DirectorySnapshot, destination: Path) -> None:
    for relative in sorted(snapshot.directories, key=lambda path: (len(path.parts), path.as_posix())):
        (destination / Path(*relative.parts)).mkdir()
    for relative, data in sorted(snapshot.files.items(), key=lambda item: item[0].as_posix()):
        path = destination / Path(*relative.parts)
        with path.open("xb") as stream:
            stream.write(data)


def current_model_sdf_sha256() -> Mapping[str, str]:
    """Derive all model hashes from one immutable, source-lock-validated snapshot."""
    specs = load_model_specs()
    try:
        source_base = _spec.source_root().resolve(strict=True)
        snapshot = capture_directory(source_base)
    except (OSError, SnapshotError) as error:
        raise ValueError(f"immutable source snapshot failed: {error}") from None
    if snapshot.problems:
        details = "; ".join(f"{problem.message}: {problem.path}" for problem in snapshot.problems)
        raise ValueError(f"immutable source snapshot failed: {details}")
    try:
        with tempfile.TemporaryDirectory(prefix="onerobotics-a1-thumbnail-source-") as temporary:
            snapshot_root = Path(temporary)
            _materialize_source_snapshot(snapshot, snapshot_root)
            validate_source_lock(source_root=snapshot_root)
            overlay = load_hardware_overlay(snapshot_root / "model_parameters.yaml")
            values: dict[str, str] = {}
            for spec in specs:
                relative_urdf = spec.urdf.relative_to(source_base)
                snapshot_spec = replace(spec, urdf=snapshot_root / relative_urdf)
                values[spec.slug] = sha256_bytes(serialize_sdf(convert_urdf(snapshot_spec, overlay)))
    except (OSError, ValueError) as error:
        raise ValueError(f"unable to derive locked model hashes: {error}") from None
    return MappingProxyType(values)


def _decompress_scanlines(idat: bytes) -> bytes:
    try:
        decompressor = zlib.decompressobj()
        raw = decompressor.decompress(idat, _EXPECTED_SCANLINE_BYTES + 1)
        if decompressor.unconsumed_tail or len(raw) > _EXPECTED_SCANLINE_BYTES:
            raise ValueError("PNG decompressed scanline data exceeds the exact RGB size")
        raw += decompressor.flush(_EXPECTED_SCANLINE_BYTES + 1 - len(raw))
    except zlib.error as error:
        raise ValueError(f"PNG IDAT stream is malformed: {error}") from None
    if (
        len(raw) != _EXPECTED_SCANLINE_BYTES
        or not decompressor.eof
        or decompressor.unused_data
        or decompressor.unconsumed_tail
    ):
        raise ValueError("PNG decompressed scanline data has the wrong size")
    stride = 1 + THUMBNAIL_WIDTH * 3
    for offset in range(0, len(raw), stride):
        if raw[offset] > 4:
            raise ValueError("PNG scanline uses an invalid filter byte")
    return raw


def _validate_plte_chunk(*, saw_ihdr: bool, saw_plte: bool, saw_idat: bool, length: int) -> None:
    if not saw_ihdr:
        raise ValueError("PNG PLTE must follow IHDR")
    if saw_plte:
        raise ValueError("PNG PLTE must contain at most one chunk")
    if saw_idat:
        raise ValueError("PNG PLTE must appear before IDAT")
    if length < 3 or length > 768 or length % 3:
        raise ValueError("PNG PLTE must contain 1 to 256 RGB palette entries")


def validate_thumbnail_bytes(data: bytes) -> PngInfo:
    """Validate one complete 512x512 non-interlaced 8-bit RGB PNG."""
    if not isinstance(data, bytes):
        raise ValueError("PNG payload must be bytes")
    if len(data) > _MAX_PNG_BYTES:
        raise ValueError("PNG file size exceeds the thumbnail limit")
    if not data.startswith(PNG_SIGNATURE):
        raise ValueError("PNG signature mismatch")

    offset = len(PNG_SIGNATURE)
    chunk_index = 0
    saw_ihdr = False
    saw_plte = False
    saw_idat = False
    saw_iend = False
    idat_finished = False
    idat_parts: list[bytes] = []
    info: PngInfo | None = None
    while offset < len(data):
        if saw_iend:
            raise ValueError("PNG has trailing data after IEND")
        if len(data) - offset < 12:
            raise ValueError("PNG chunk is truncated")
        length = struct.unpack(">I", data[offset : offset + 4])[0]
        kind = data[offset + 4 : offset + 8]
        end = offset + 12 + length
        if end > len(data):
            raise ValueError("PNG chunk bounds exceed the file")
        payload = data[offset + 8 : offset + 8 + length]
        expected_crc = struct.unpack(">I", data[offset + 8 + length : end])[0]
        actual_crc = binascii.crc32(kind + payload) & 0xFFFFFFFF
        if actual_crc != expected_crc:
            raise ValueError(f"PNG {kind.decode('ascii', 'replace')} chunk CRC mismatch")
        if len(kind) != 4 or not all(65 <= byte <= 90 or 97 <= byte <= 122 for byte in kind):
            raise ValueError("PNG chunk type is malformed")
        if not 65 <= kind[2] <= 90:
            raise ValueError("PNG chunk type has an invalid reserved bit")
        if 65 <= kind[0] <= 90 and kind not in {b"IHDR", b"PLTE", b"IDAT", b"IEND"}:
            raise ValueError("PNG contains an unknown critical chunk")
        if chunk_index == 0 and kind != b"IHDR":
            raise ValueError("PNG IHDR must be the first chunk")
        if kind == b"IHDR":
            if saw_ihdr or chunk_index != 0 or length != 13:
                raise ValueError("PNG must contain one 13-byte leading IHDR")
            width, height, bit_depth, color_type, compression, filtering, interlace = struct.unpack(">IIBBBBB", payload)
            if (width, height) != (THUMBNAIL_WIDTH, THUMBNAIL_HEIGHT):
                raise ValueError("PNG must be exactly 512x512")
            if bit_depth != 8:
                raise ValueError("PNG must use 8-bit samples")
            if color_type != 2:
                raise ValueError("PNG must use truecolor RGB")
            if (compression, filtering, interlace) != (0, 0, 0):
                raise ValueError("PNG must use standard compression, filtering, and no interlace")
            info = PngInfo(width, height, bit_depth, color_type)
            saw_ihdr = True
        elif kind == b"PLTE":
            _validate_plte_chunk(
                saw_ihdr=saw_ihdr,
                saw_plte=saw_plte,
                saw_idat=saw_idat,
                length=length,
            )
            saw_plte = True
        elif kind == b"IDAT":
            if not saw_ihdr or idat_finished:
                raise ValueError("PNG IDAT chunks must be consecutive after IHDR")
            saw_idat = True
            idat_parts.append(payload)
        elif kind == b"IEND":
            if not saw_ihdr or not saw_idat or saw_iend or length != 0:
                raise ValueError("PNG must contain one empty terminal IEND after IDAT")
            saw_iend = True
        elif saw_idat:
            idat_finished = True
        offset = end
        chunk_index += 1

    if not saw_ihdr or info is None:
        raise ValueError("PNG is missing IHDR")
    if not saw_idat:
        raise ValueError("PNG is missing IDAT")
    if not saw_iend:
        raise ValueError("PNG is missing terminal IEND")
    _decompress_scanlines(b"".join(idat_parts))
    return info


def _require_digest(value: object, label: str) -> str:
    if not isinstance(value, str) or _SHA256.fullmatch(value) is None:
        raise ValueError(f"{label} must be one lowercase SHA-256 digest")
    return value


def _require_exact_slugs(mapping: Mapping[str, object], label: str) -> tuple[str, ...]:
    slugs = thumbnail_slugs()
    if set(mapping) != set(slugs):
        raise ValueError(f"{label} must contain exactly the three locked model slugs")
    return slugs


def build_render_manifest(
    png_by_slug: Mapping[str, bytes],
    *,
    render_config_sha256: str,
    model_sdf_sha256: Mapping[str, str],
) -> bytes:
    """Build the canonical manifest binding config, models, and PNG bytes."""
    slugs = _require_exact_slugs(png_by_slug, "PNG mapping")
    _require_exact_slugs(model_sdf_sha256, "model SDF mapping")
    config_digest = _require_digest(render_config_sha256, "render config hash")
    records = []
    for slug in slugs:
        data = png_by_slug[slug]
        info = validate_thumbnail_bytes(data)
        records.append(
            {
                "file": f"{slug}.png",
                "height": info.height,
                "model_sdf_sha256": _require_digest(model_sdf_sha256[slug], f"{slug} model SDF hash"),
                "png_sha256": sha256_bytes(data),
                "slug": slug,
                "width": info.width,
            }
        )
    manifest = {
        "backend": RENDER_BACKEND,
        "models": records,
        "render_config_sha256": config_digest,
        "render_engine": RENDER_ENGINE,
        "schema_version": MANIFEST_SCHEMA_VERSION,
    }
    return (json.dumps(manifest, indent=2, sort_keys=True, ensure_ascii=False) + "\n").encode("utf-8")


def _reject_duplicate_pairs(pairs: list[tuple[str, object]]) -> dict[str, object]:
    result: dict[str, object] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"render manifest has duplicate key: {key}")
        result[key] = value
    return result


def _parse_manifest(data: bytes) -> tuple[str, tuple[ThumbnailRecord, ...]]:
    if len(data) > _MAX_MANIFEST_BYTES:
        raise ValueError("render manifest exceeds size limit")
    try:
        text = data.decode("utf-8")
        manifest = json.loads(
            text,
            object_pairs_hook=_reject_duplicate_pairs,
            parse_constant=lambda token: (_ for _ in ()).throw(ValueError(f"non-finite JSON value: {token}")),
        )
    except (UnicodeError, json.JSONDecodeError, RecursionError) as error:
        raise ValueError(f"render manifest is malformed: {error}") from None
    if not isinstance(manifest, dict) or set(manifest) != {
        "schema_version",
        "render_engine",
        "backend",
        "render_config_sha256",
        "models",
    }:
        raise ValueError("render manifest root fields are not exact")
    if manifest["schema_version"] != MANIFEST_SCHEMA_VERSION:
        raise ValueError("render manifest schema mismatch")
    if manifest["render_engine"] != RENDER_ENGINE or manifest["backend"] != RENDER_BACKEND:
        raise ValueError("render manifest engine/backend mismatch")
    config_digest = _require_digest(manifest["render_config_sha256"], "render manifest config hash")
    models = manifest["models"]
    slugs = thumbnail_slugs()
    if not isinstance(models, list) or len(models) != len(slugs):
        raise ValueError("render manifest models must contain exactly three records")
    records: list[ThumbnailRecord] = []
    for expected_slug, value in zip(slugs, models, strict=True):
        if not isinstance(value, dict) or set(value) != {
            "slug",
            "file",
            "png_sha256",
            "width",
            "height",
            "model_sdf_sha256",
        }:
            raise ValueError("render manifest model record fields are not exact")
        if value["slug"] != expected_slug or value["file"] != f"{expected_slug}.png":
            raise ValueError("render manifest model order, slug, or file mismatch")
        if isinstance(value["width"], bool) or isinstance(value["height"], bool):
            raise ValueError("render manifest dimensions are malformed")
        if (value["width"], value["height"]) != (THUMBNAIL_WIDTH, THUMBNAIL_HEIGHT):
            raise ValueError("render manifest dimensions must be 512x512")
        records.append(
            ThumbnailRecord(
                slug=expected_slug,
                file=f"{expected_slug}.png",
                png_sha256=_require_digest(value["png_sha256"], f"{expected_slug} PNG hash"),
                width=THUMBNAIL_WIDTH,
                height=THUMBNAIL_HEIGHT,
                model_sdf_sha256=_require_digest(value["model_sdf_sha256"], f"{expected_slug} model SDF hash"),
            )
        )
    return config_digest, tuple(records)


def capture_thumbnail_set(
    root: Path,
    *,
    expected_render_config_sha256: str | None = None,
    expected_model_sdf_sha256: Mapping[str, str] | None = None,
) -> ThumbnailSet:
    """Capture and validate an exact four-file thumbnail set without reopening paths."""
    try:
        snapshot = capture_directory(Path(root))
    except SnapshotError as error:
        raise ValueError(f"thumbnail snapshot failed: {error}") from None
    return validate_thumbnail_snapshot(
        snapshot,
        expected_render_config_sha256=expected_render_config_sha256,
        expected_model_sdf_sha256=expected_model_sdf_sha256,
    )


def validate_thumbnail_snapshot(
    snapshot: DirectorySnapshot,
    *,
    expected_render_config_sha256: str | None = None,
    expected_model_sdf_sha256: Mapping[str, str] | None = None,
) -> ThumbnailSet:
    """Validate thumbnail bytes already captured through a trusted descriptor."""
    if snapshot.problems:
        details = "; ".join(f"{problem.message}: {problem.path}" for problem in snapshot.problems)
        raise ValueError(f"thumbnail snapshot failed: {details}")
    if snapshot.directories:
        raise ValueError("thumbnail snapshot has unexpected directories")
    slugs = thumbnail_slugs()
    expected_files = {PurePosixPath(MANIFEST_NAME), *(PurePosixPath(f"{slug}.png") for slug in slugs)}
    actual_files = set(snapshot.files)
    if actual_files != expected_files:
        missing = sorted(path.as_posix() for path in expected_files - actual_files)
        extra = sorted(path.as_posix() for path in actual_files - expected_files)
        raise ValueError(f"thumbnail inventory mismatch: missing={missing}, extra={extra}")

    manifest_bytes = snapshot.files[PurePosixPath(MANIFEST_NAME)]
    config_digest, records = _parse_manifest(manifest_bytes)
    expected_config = expected_render_config_sha256 or current_render_config_sha256()
    if config_digest != _require_digest(expected_config, "expected render config hash"):
        raise ValueError("render manifest config hash mismatch")
    expected_models = expected_model_sdf_sha256 or current_model_sdf_sha256()
    _require_exact_slugs(expected_models, "expected model SDF mapping")

    pngs: dict[str, bytes] = {}
    records_by_slug: dict[str, ThumbnailRecord] = {}
    for record in records:
        data = snapshot.files[PurePosixPath(record.file)]
        info = validate_thumbnail_bytes(data)
        if (info.width, info.height) != (record.width, record.height):
            raise ValueError(f"render manifest PNG dimensions mismatch: {record.slug}")
        if sha256_bytes(data) != record.png_sha256:
            raise ValueError(f"render manifest PNG hash mismatch: {record.slug}")
        if record.model_sdf_sha256 != _require_digest(
            expected_models[record.slug], f"expected {record.slug} model SDF hash"
        ):
            raise ValueError(f"render manifest model SDF hash mismatch: {record.slug}")
        pngs[record.slug] = data
        records_by_slug[record.slug] = record

    canonical = build_render_manifest(
        pngs,
        render_config_sha256=config_digest,
        model_sdf_sha256={slug: records_by_slug[slug].model_sdf_sha256 for slug in slugs},
    )
    if manifest_bytes != canonical:
        raise ValueError("render manifest bytes are not canonical")
    return ThumbnailSet(
        png_by_slug=MappingProxyType(pngs),
        records_by_slug=MappingProxyType(records_by_slug),
        manifest_bytes=manifest_bytes,
        render_config_sha256=config_digest,
    )
