"""Fail-closed validation for post-download Gazebo Fuel cache snapshots."""

from __future__ import annotations

import os
import re
import secrets
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from types import MappingProxyType
from urllib.parse import quote, unquote_to_bytes, urlsplit
from xml.etree import ElementTree
from xml.etree.ElementTree import Element

from onerobotics_a1_gazebo.harmonic import validate_parsed_sdf
from onerobotics_a1_gazebo.spec import ModelSpec, load_model_specs
from onerobotics_a1_gazebo.validation_snapshot import DirectorySnapshot, SnapshotError, capture_directory

_MODEL_CONFIG = PurePosixPath("model.config")
_MODEL_SDF = PurePosixPath("model.sdf")
_MAX_XML_BYTES = 8 * 1024 * 1024
_MAX_XML_DEPTH = 256
_MAX_XML_ELEMENTS = 20_000
_MAX_XML_ATTRIBUTES = 20_000
_MAX_NATIVE_OUTPUT = 16 * 1024 * 1024
_NATIVE_TIMEOUT = 20.0
_NATIVE_TOTAL_TIMEOUT = 60.0
_FUEL_HOST = "fuel.gazebosim.org"


@dataclass(frozen=True)
class ValidatedCacheSnapshots:
    """Trusted exact export and safe cache bytes captured for one motion run."""

    trusted_export: DirectorySnapshot
    normalized_models: DirectorySnapshot


def _capture_trusted_export(path: Path) -> DirectorySnapshot:
    # The import stays local so demo.py can call this module without an import cycle.
    from onerobotics_a1_gazebo.demo import _validated_export_snapshot

    return _validated_export_snapshot(Path(path))


def _run_harmonic_command(**kwargs: object):
    # Reuse the runtime's bounded process-group owner instead of subprocess.run.
    from onerobotics_a1_gazebo.demo import _run_harmonic

    return _run_harmonic(**kwargs)


def _capture_cache(path: Path) -> DirectorySnapshot:
    try:
        snapshot = capture_directory(Path(path))
    except SnapshotError as error:
        raise ValueError(f"cache snapshot failed: {error}") from None
    if snapshot.problems:
        details = "; ".join(f"{problem.message}: {problem.path}" for problem in snapshot.problems)
        raise ValueError(f"cache snapshot failed: {details}")
    return snapshot


def _parse_xml(data: bytes, label: str) -> Element:
    if len(data) > _MAX_XML_BYTES:
        raise ValueError(f"{label} exceeds the XML byte limit")
    try:
        text = data.decode("utf-8-sig")
    except UnicodeDecodeError:
        raise ValueError(f"{label} must use UTF-8 XML encoding") from None
    if re.search(r"<!\s*(?:DOCTYPE|ENTITY)\b", text, flags=re.IGNORECASE):
        raise ValueError(f"{label} must not contain a document type or entity declaration")
    parser = ElementTree.XMLPullParser(events=("start", "end", "start-ns"))
    root: Element | None = None
    depth = 0
    element_count = 0
    attribute_count = 0

    def consume_events() -> None:
        nonlocal root, depth, element_count, attribute_count
        for event, element in parser.read_events():
            if event == "start-ns":
                attribute_count += 1
                if attribute_count > _MAX_XML_ATTRIBUTES:
                    raise ValueError(f"{label} exceeds the XML attribute limit")
            elif event == "start":
                if root is None:
                    root = element
                depth += 1
                element_count += 1
                attribute_count += len(element.attrib)
                if depth > _MAX_XML_DEPTH:
                    raise ValueError(f"{label} exceeds the XML nesting limit")
                if element_count > _MAX_XML_ELEMENTS:
                    raise ValueError(f"{label} exceeds the XML element limit")
                if attribute_count > _MAX_XML_ATTRIBUTES:
                    raise ValueError(f"{label} exceeds the XML attribute limit")
            else:
                depth -= 1

    try:
        for offset in range(0, len(text), 64 * 1024):
            parser.feed(text[offset : offset + 64 * 1024])
            consume_events()
        parser.close()
        consume_events()
    except RecursionError:
        raise ValueError(f"{label} exceeds the XML nesting limit") from None
    except (ElementTree.ParseError, LookupError, ValueError) as error:
        if isinstance(error, ValueError) and str(error).startswith(label):
            raise
        raise ValueError(f"malformed {label}: {error}") from None
    if root is None or depth != 0:
        raise ValueError(f"malformed {label}: XML document has no complete root")
    return root


def _xml_signature(element: Element) -> tuple[object, ...]:
    text = element.text or ""
    tail = element.tail or ""
    return (
        element.tag,
        tuple(sorted(element.attrib.items())),
        "" if not text.strip() else text,
        tuple(_xml_signature(child) for child in element),
        "" if not tail.strip() else tail,
    )


def _serialize_xml(root: Element) -> bytes:
    return ElementTree.tostring(root, encoding="utf-8", xml_declaration=True)


def _trusted_mesh_uris(root: Element, spec: ModelSpec, label: str) -> tuple[str, ...]:
    mesh_elements = root.findall(".//geometry/mesh/uri")
    all_uris = root.findall(".//uri")
    if len(mesh_elements) != len(all_uris):
        raise ValueError(f"{label} contains a non-mesh URI")
    trusted: list[str] = []
    for element in mesh_elements:
        value = (element.text or "").strip()
        path = PurePosixPath(value)
        if (
            len(path.parts) != 2
            or path.parts[0] != "meshes"
            or any(part in {"", ".", ".."} for part in path.parts)
            or "\\" in value
            or "%" in value
        ):
            raise ValueError(f"{label} has an invalid trusted mesh URI: {value or '<empty>'}")
        trusted.append(value)
    if not trusted:
        raise ValueError(f"{label} has no trusted mesh URI")
    return tuple(trusted)


def _safe_rewritten_mesh_target(value: str, expected: str, spec: ModelSpec, label: str) -> str:
    raw = value
    if (
        not raw
        or raw != raw.strip()
        or any(ord(character) < 32 or ord(character) == 127 for character in raw)
        or "\\" in raw
        or "?" in raw
        or "#" in raw
    ):
        raise ValueError(f"{label} mesh URI is unsafe: {raw or '<empty>'}")
    expected_path = PurePosixPath(expected)
    try:
        parsed = urlsplit(raw)
    except ValueError as error:
        raise ValueError(f"{label} mesh URI is malformed: {error}") from None
    if parsed.query or parsed.fragment:
        raise ValueError(f"{label} mesh URI contains a query or fragment: {raw}")

    if parsed.scheme:
        if parsed.scheme in {"file", "model"} and "%" in raw:
            raise ValueError(f"{label} mesh URI contains encoded path data: {raw}")
        if parsed.scheme == "file":
            if parsed.netloc or not parsed.path.startswith("/"):
                raise ValueError(f"{label} mesh URI is not a local absolute file URI: {raw}")
            path_text = parsed.path
        elif parsed.scheme == "model":
            if parsed.netloc != spec.slug or not parsed.path.startswith("/"):
                raise ValueError(f"{label} mesh URI targets another model: {raw}")
            path_text = parsed.path
        elif parsed.scheme == "https":
            if parsed.netloc != _FUEL_HOST:
                raise ValueError(f"{label} mesh URI targets a non-Fuel HTTPS authority: {raw}")
            route = parsed.path.split("/")
            if route and route[0] == "":
                route = route[1:]
            if (
                len(route) != 8
                or any(segment in {"", ".", ".."} for segment in route)
                or route[0] != "1.0"
                or re.fullmatch(r"[A-Za-z0-9_.-]+", route[1]) is None
                or route[2] != "models"
                or re.fullmatch(r"[1-9][0-9]*", route[4]) is None
                or route[5] != "files"
                or route[6:] != list(expected_path.parts)
            ):
                raise ValueError(f"{label} mesh URI is not one versioned Fuel file route: {raw}")
            encoded_name = route[3]
            try:
                decoded_name = unquote_to_bytes(encoded_name).decode("utf-8")
            except UnicodeDecodeError:
                raise ValueError(f"{label} mesh URI has a malformed Fuel resource name: {raw}") from None
            if (
                not decoded_name
                or decoded_name != decoded_name.strip()
                or any(ord(character) < 32 or ord(character) == 127 for character in decoded_name)
                or any(character in decoded_name for character in ("/", "\\", "%"))
                or quote(decoded_name, safe="-._~") != encoded_name
                or decoded_name.casefold() != spec.display_name.casefold()
            ):
                raise ValueError(f"{label} mesh URI has an unexpected Fuel resource name: {raw}")
            return expected
        else:
            raise ValueError(f"{label} mesh URI uses a forbidden scheme: {raw}")
    else:
        if "%" in raw:
            raise ValueError(f"{label} mesh URI contains encoded path data: {raw}")
        path_text = raw

    path_segments = path_text.split("/")
    if path_text.startswith("/"):
        path_segments = path_segments[1:]
    if not path_segments or any(segment in {"", ".", ".."} for segment in path_segments):
        raise ValueError(f"{label} mesh URI contains an unsafe path segment: {raw}")
    path = PurePosixPath(path_text)
    if not path.is_absolute() and path != expected_path:
        raise ValueError(f"{label} mesh URI does not map to its trusted occurrence: {raw}")
    if len(path.parts) < 2 or path.parts[-2:] != expected_path.parts:
        raise ValueError(f"{label} mesh URI does not map to its trusted occurrence: {raw}")
    return expected


def _normalize_model_sdf(
    data: bytes,
    spec: ModelSpec,
    expected_uris: tuple[str, ...],
    label: str,
) -> tuple[bytes, Element]:
    root = _parse_xml(data, label)
    mesh_uris = root.findall(".//geometry/mesh/uri")
    all_uris = root.findall(".//uri")
    if len(mesh_uris) != len(all_uris):
        raise ValueError(f"{label} contains a non-mesh URI")
    if len(mesh_uris) != len(expected_uris):
        raise ValueError(f"{label} mesh URI inventory mismatch: expected {len(expected_uris)}, got {len(mesh_uris)}")
    for index, (element, expected) in enumerate(zip(mesh_uris, expected_uris, strict=True)):
        element.text = _safe_rewritten_mesh_target(
            element.text or "",
            expected,
            spec,
            f"{label} occurrence {index}",
        )
    return _serialize_xml(root), root


def _validate_root_inventory(snapshot: DirectorySnapshot, specs: tuple[ModelSpec, ...]) -> None:
    expected = {PurePosixPath(spec.slug) for spec in specs}
    actual_directories = {path for path in snapshot.directories if len(path.parts) == 1}
    root_files = {path for path in snapshot.files if len(path.parts) == 1}
    if actual_directories != expected or root_files:
        raise ValueError(
            "cache root inventory mismatch: "
            f"missing={sorted(str(path) for path in expected - actual_directories)}, "
            f"extra={sorted(str(path) for path in actual_directories - expected | root_files)}"
        )


def _validate_package_inventory(
    trusted: DirectorySnapshot,
    cached: DirectorySnapshot,
    spec: ModelSpec,
) -> None:
    if trusted.directories != cached.directories or set(trusted.files) != set(cached.files):
        raise ValueError(
            f"cache package inventory mismatch for {spec.slug}: "
            f"missing_files={sorted(str(path) for path in set(trusted.files) - set(cached.files))}, "
            f"extra_files={sorted(str(path) for path in set(cached.files) - set(trusted.files))}, "
            f"missing_directories={sorted(str(path) for path in trusted.directories - cached.directories)}, "
            f"extra_directories={sorted(str(path) for path in cached.directories - trusted.directories)}"
        )


def _normalized_cache_snapshot(
    trusted_export: DirectorySnapshot,
    cache_snapshot: DirectorySnapshot,
    specs: tuple[ModelSpec, ...],
) -> tuple[DirectorySnapshot, dict[str, tuple[str, ...]]]:
    _validate_root_inventory(cache_snapshot, specs)
    files = dict(cache_snapshot.files)
    expected_uris: dict[str, tuple[str, ...]] = {}
    for spec in specs:
        trusted = trusted_export.subtree(spec.slug)
        cached = cache_snapshot.subtree(spec.slug)
        _validate_package_inventory(trusted, cached, spec)
        for relative, trusted_data in trusted.files.items():
            cached_data = cached.files[relative]
            if relative == _MODEL_CONFIG:
                trusted_config = _parse_xml(trusted_data, f"trusted {spec.slug} model.config")
                cached_config = _parse_xml(cached_data, f"cache {spec.slug} model.config")
                if _xml_signature(cached_config) != _xml_signature(trusted_config):
                    raise ValueError(f"cache model.config semantic mismatch for {spec.slug}")
                files[PurePosixPath(spec.slug) / relative] = _serialize_xml(cached_config)
                continue
            if relative == _MODEL_SDF:
                trusted_root = _parse_xml(trusted_data, f"trusted {spec.slug} model.sdf")
                model_uris = _trusted_mesh_uris(trusted_root, spec, f"trusted {spec.slug} model.sdf")
                normalized, _ = _normalize_model_sdf(
                    cached_data,
                    spec,
                    model_uris,
                    f"cache {spec.slug} model.sdf",
                )
                files[PurePosixPath(spec.slug) / relative] = normalized
                expected_uris[spec.slug] = model_uris
                continue
            if cached_data != trusted_data:
                raise ValueError(f"cache byte mismatch for {spec.slug}/{relative}")
    normalized_snapshot = DirectorySnapshot(
        files=MappingProxyType(dict(sorted(files.items(), key=lambda item: item[0].as_posix()))),
        directories=cache_snapshot.directories,
        problems=(),
    )
    return normalized_snapshot, expected_uris


def _materialize_snapshot(snapshot: DirectorySnapshot, destination: Path) -> None:
    destination.mkdir(mode=0o700)
    for relative in sorted(snapshot.directories, key=lambda path: (len(path.parts), path.as_posix())):
        (destination / Path(*relative.parts)).mkdir(mode=0o700)
    for relative, data in sorted(snapshot.files.items(), key=lambda item: item[0].as_posix()):
        path = destination / Path(*relative.parts)
        with path.open("xb") as stream:
            stream.write(data)


def _require_native(result: object, label: str, *, require_output: bool = False) -> str:
    returncode = getattr(result, "returncode", None)
    stdout = getattr(result, "stdout", "")
    stderr = getattr(result, "stderr", "")
    if not isinstance(returncode, int) or not isinstance(stdout, str) or not isinstance(stderr, str):
        raise RuntimeError(f"native {label} returned an invalid command result")
    try:
        output_size = len(stdout.encode("utf-8")) + len(stderr.encode("utf-8"))
    except UnicodeEncodeError as error:
        raise RuntimeError(f"native {label} returned non-UTF-8-compatible text") from error
    if output_size > _MAX_NATIVE_OUTPUT:
        raise RuntimeError(f"native {label} output exceeds the byte limit")
    if returncode != 0:
        detail = (stdout[-4096:] + stderr[-4096:])[-8192:]
        raise RuntimeError(f"native {label} failed with exit {returncode}: {detail}")
    if require_output and not stdout.strip():
        raise RuntimeError(f"native {label} returned empty output")
    return stdout


def _native_command(
    *,
    partition: str,
    private_export: Path,
    command: list[str],
    label: str,
    deadline: float,
    require_output: bool = False,
) -> str:
    result = _run_harmonic_command(
        partition=partition,
        private_export=private_export,
        command=command,
        timeout=_NATIVE_TIMEOUT,
        timeout_message=f"native {label} timeout",
        deadline=deadline,
        output_limit_bytes=_MAX_NATIVE_OUTPUT,
    )
    return _require_native(result, label, require_output=require_output)


def _validate_native_models(
    trusted_export: DirectorySnapshot,
    normalized_cache: DirectorySnapshot,
    specs: tuple[ModelSpec, ...],
    expected_uris: dict[str, tuple[str, ...]],
) -> None:
    partition = f"onerobotics-a1-cache-{os.getpid()}-{secrets.token_hex(8)}"
    deadline = time.monotonic() + _NATIVE_TOTAL_TIMEOUT
    with tempfile.TemporaryDirectory(prefix="onerobotics-a1-cache-validation-") as temporary:
        root = Path(temporary).resolve()
        trusted_root = root / "trusted"
        cache_root = root / "cache"
        _materialize_snapshot(trusted_export, trusted_root)
        _materialize_snapshot(normalized_cache, cache_root)
        for spec in specs:
            trusted_sdf = trusted_root / spec.slug / "model.sdf"
            cache_sdf = cache_root / spec.slug / "model.sdf"
            for label, path in (("trusted", trusted_sdf), ("cache", cache_sdf)):
                _native_command(
                    partition=partition,
                    private_export=cache_root,
                    command=["sdf", "--force-version", "14", "-k", os.fspath(path)],
                    label=f"{label} SDF check for {spec.slug}",
                    deadline=deadline,
                )
            trusted_pretty = _native_command(
                partition=partition,
                private_export=cache_root,
                command=["sdf", "--force-version", "14", "-p", os.fspath(trusted_sdf)],
                label=f"trusted SDF parse for {spec.slug}",
                deadline=deadline,
                require_output=True,
            ).encode("utf-8")
            cache_pretty_text = _native_command(
                partition=partition,
                private_export=cache_root,
                command=["sdf", "--force-version", "14", "-p", os.fspath(cache_sdf)],
                label=f"cache SDF parse for {spec.slug}",
                deadline=deadline,
                require_output=True,
            )
            cache_pretty = cache_pretty_text.encode("utf-8")
            _, trusted_parsed_root = _normalize_model_sdf(
                trusted_pretty,
                spec,
                expected_uris[spec.slug],
                f"native trusted parsed {spec.slug} SDF",
            )
            _, cache_parsed_root = _normalize_model_sdf(
                cache_pretty,
                spec,
                expected_uris[spec.slug],
                f"native cache parsed {spec.slug} SDF",
            )
            if _xml_signature(cache_parsed_root) != _xml_signature(trusted_parsed_root):
                raise ValueError(f"native parsed SDF semantic mismatch for {spec.slug}")

            cache_pretty_path = root / f"{spec.slug}.cache.pretty.sdf"
            cache_pretty_path.write_text(cache_pretty_text, encoding="utf-8")
            backstop = validate_parsed_sdf(trusted_sdf, cache_pretty_path, spec)
            if backstop:
                raise ValueError(f"native parsed SDF topology/limit mismatch for {spec.slug}: {'; '.join(backstop)}")

            metadata = cache_root / spec.slug / "metadata.pbtxt"
            _native_command(
                partition=partition,
                private_export=cache_root,
                command=[
                    "fuel",
                    "--force-version",
                    "9",
                    "meta",
                    "--pbtxt2config",
                    os.fspath(metadata),
                ],
                label=f"Fuel metadata parse for {spec.slug}",
                deadline=deadline,
                require_output=True,
            )


def validate_cache_models(*, trusted_models: Path, cache_models: Path) -> ValidatedCacheSnapshots:
    """Capture, normalize, and semantically validate exactly three Fuel cache models."""
    specs = load_model_specs()
    trusted_export = _capture_trusted_export(Path(trusted_models))
    cache_snapshot = _capture_cache(Path(cache_models))
    normalized, expected_uris = _normalized_cache_snapshot(trusted_export, cache_snapshot, specs)
    _validate_native_models(trusted_export, normalized, specs, expected_uris)
    return ValidatedCacheSnapshots(trusted_export=trusted_export, normalized_models=normalized)
