from __future__ import annotations

import binascii
import hashlib
import io
import json
import os
import shutil
import signal
import struct
import subprocess
import zlib
from pathlib import Path
from types import MappingProxyType
from xml.etree import ElementTree

import onerobotics_a1_gazebo.spec as spec_module
import onerobotics_a1_gazebo.thumbnail as thumbnail_module
import pytest
from onerobotics_a1_gazebo import demo, render
from onerobotics_a1_gazebo.package import export_models
from onerobotics_a1_gazebo.spec import load_model_specs
from onerobotics_a1_gazebo.thumbnail import (
    PngInfo,
    build_render_manifest,
    capture_thumbnail_set,
    current_model_sdf_sha256,
    validate_thumbnail_bytes,
)

SLUGS = tuple(spec.slug for spec in load_model_specs())
FRAME_PREFIX = "camera_rig::link::render_camera_"
CAMERA_TOPIC = "/world/a1_render/model/camera_rig/link/link/sensor/render_camera/image"


def _chunk(kind: bytes, payload: bytes, *, corrupt_crc: bool = False) -> bytes:
    crc = binascii.crc32(kind + payload) & 0xFFFFFFFF
    if corrupt_crc:
        crc ^= 1
    return struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", crc)


def _png(
    *,
    width: int = 512,
    height: int = 512,
    bit_depth: int = 8,
    color_type: int = 2,
    filter_byte: int = 0,
    raw: bytes | None = None,
    include_idat: bool = True,
    include_iend: bool = True,
    corrupt_idat_crc: bool = False,
    trailing: bytes = b"",
) -> bytes:
    channels = 3 if color_type == 2 else 4
    scanlines = raw
    if scanlines is None:
        scanlines = (bytes([filter_byte]) + bytes(width * channels)) * height
    chunks = [_chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, bit_depth, color_type, 0, 0, 0))]
    if include_idat:
        chunks.append(_chunk(b"IDAT", zlib.compress(scanlines), corrupt_crc=corrupt_idat_crc))
    if include_iend:
        chunks.append(_chunk(b"IEND", b""))
    return b"\x89PNG\r\n\x1a\n" + b"".join(chunks) + trailing


def _manifest(
    pngs: dict[str, bytes],
    *,
    config_digest: str = "a" * 64,
    model_digests: dict[str, str] | None = None,
) -> bytes:
    return build_render_manifest(
        pngs,
        render_config_sha256=config_digest,
        model_sdf_sha256=model_digests or {slug: hashlib.sha256(slug.encode()).hexdigest() for slug in SLUGS},
    )


def _write_thumbnail_set(
    root: Path,
    *,
    pngs: dict[str, bytes] | None = None,
    config_digest: str = "a" * 64,
    model_digests: dict[str, str] | None = None,
) -> tuple[dict[str, bytes], dict[str, str]]:
    root.mkdir()
    payloads = pngs or {slug: _png() for slug in SLUGS}
    digests = model_digests or {slug: hashlib.sha256(slug.encode()).hexdigest() for slug in SLUGS}
    for slug, data in payloads.items():
        (root / f"{slug}.png").write_bytes(data)
    (root / "render-manifest.json").write_bytes(_manifest(payloads, config_digest=config_digest, model_digests=digests))
    return payloads, digests


def test_strict_png_validator_accepts_complete_512_rgb() -> None:
    assert validate_thumbnail_bytes(_png()) == PngInfo(width=512, height=512, bit_depth=8, color_type=2)


@pytest.mark.parametrize(
    ("data", "message"),
    [
        (b"not-png", "signature"),
        (_png(width=511), "512"),
        (_png(height=513), "512"),
        (_png(bit_depth=16), "8-bit"),
        (_png(color_type=6), "truecolor"),
        (_png(include_idat=False), "IDAT"),
        (_png(include_iend=False), "IEND"),
        (_png(corrupt_idat_crc=True), "CRC"),
        (_png(trailing=b"attacker"), "trailing"),
        (_png(filter_byte=5), "filter"),
        (_png(raw=b"\x00"), "scanline"),
    ],
)
def test_strict_png_validator_rejects_malformed_images(data: bytes, message: str) -> None:
    with pytest.raises(ValueError, match=message):
        validate_thumbnail_bytes(data)


def test_strict_png_validator_rejects_duplicate_ihdr_and_iend() -> None:
    valid = _png()
    ihdr = valid[8 : 8 + 25]
    duplicate_ihdr = valid[: 8 + 25] + ihdr + valid[8 + 25 :]
    duplicate_iend = valid + _chunk(b"IEND", b"")

    with pytest.raises(ValueError, match="IHDR"):
        validate_thumbnail_bytes(duplicate_ihdr)
    with pytest.raises(ValueError, match="trailing|IEND"):
        validate_thumbnail_bytes(duplicate_iend)


@pytest.mark.parametrize(
    ("kind", "message"),
    [(b"ABCD", "critical"), (b"abcd", "reserved")],
)
def test_strict_png_validator_rejects_unknown_critical_and_invalid_reserved_chunks(
    kind: bytes,
    message: str,
) -> None:
    valid = _png()
    after_ihdr = 8 + 12 + 13
    mutated = valid[:after_ihdr] + _chunk(kind, b"") + valid[after_ihdr:]

    with pytest.raises(ValueError, match=message):
        validate_thumbnail_bytes(mutated)


@pytest.mark.parametrize(
    ("palette_chunks", "message"),
    [
        ((b"\x00\x00\x00", b"\xff\xff\xff"), "PLTE.*one|duplicate"),
        ((b"",), "PLTE.*non-empty|palette"),
        ((b"\x00\x00\x00\x00",), "PLTE.*multiple|palette"),
        ((bytes(771),), "PLTE.*768|palette"),
    ],
)
def test_strict_truecolor_png_validator_enforces_plte_inventory_and_size(
    palette_chunks: tuple[bytes, ...],
    message: str,
) -> None:
    valid = _png()
    after_ihdr = 8 + 12 + 13
    mutated = valid[:after_ihdr] + b"".join(_chunk(b"PLTE", payload) for payload in palette_chunks) + valid[after_ihdr:]

    with pytest.raises(ValueError, match=message):
        validate_thumbnail_bytes(mutated)


def test_strict_truecolor_png_validator_rejects_plte_after_idat() -> None:
    valid = _png()
    before_iend = len(valid) - 12
    mutated = valid[:before_iend] + _chunk(b"PLTE", b"\x00\x00\x00") + valid[before_iend:]

    with pytest.raises(ValueError, match="PLTE.*before.*IDAT|after.*IDAT"):
        validate_thumbnail_bytes(mutated)


def test_strict_png_validator_bounds_compressed_and_decompressed_data() -> None:
    oversized_scanlines = (b"\x00" + bytes(512 * 3)) * 512 + b"extra"
    oversized_file = _png() + bytes(5 * 1024 * 1024)

    with pytest.raises(ValueError, match="scanline"):
        validate_thumbnail_bytes(_png(raw=oversized_scanlines))
    with pytest.raises(ValueError, match="size"):
        validate_thumbnail_bytes(oversized_file)


def test_render_config_is_exact_and_finite() -> None:
    config = render.load_render_config()

    assert tuple(config) == SLUGS
    for slug, settings in config.items():
        assert settings.slug == slug
        assert len(settings.camera_pose) == 6
        assert len(settings.ground_pose) == 6
        assert len(settings.lights) == 2
        assert {light.name for light in settings.lights} == {"key", "fill"}
        for light in settings.lights:
            assert len(light.pose) == 6
            assert len(light.diffuse) == 4
            assert len(light.specular) == 4
            assert len(light.direction) == 3


@pytest.mark.parametrize(
    ("yaml_text", "message"),
    [
        ("schema_version: 1\nmodels: {}\n", "exactly"),
        (
            "schema_version: 1\nschema_version: 1\nmodels: {}\n",
            "duplicate",
        ),
        (
            "schema_version: 1\nmodels:\n  unexpected: {}\n",
            "exactly",
        ),
    ],
)
def test_render_config_rejects_wrong_inventory_and_duplicate_keys(
    tmp_path: Path,
    yaml_text: str,
    message: str,
) -> None:
    path = tmp_path / "render.yaml"
    path.write_text(yaml_text, encoding="utf-8")

    with pytest.raises(ValueError, match=message):
        render.load_render_config(path)


@pytest.mark.parametrize("invalid", ["true", ".nan", ".inf", "1e309", "null", "text"])
def test_render_config_rejects_nonfinite_non_numeric_and_boolean_values(
    tmp_path: Path,
    invalid: str,
) -> None:
    text = render.render_config_path().read_text(encoding="utf-8")
    text = text.replace("camera_pose: [", f"camera_pose: [{invalid}, ", 1)
    path = tmp_path / "render.yaml"
    path.write_text(text, encoding="utf-8")

    with pytest.raises(ValueError, match="camera_pose|finite|length"):
        render.load_render_config(path)


def test_render_candidates_capture_one_immutable_config_for_every_model_and_manifest(
    render_export: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    config_path = tmp_path / "render.yaml"
    original = render.render_config_path().read_bytes()
    config_path.write_bytes(original)
    original_digest = hashlib.sha256(original).hexdigest()
    changed = original.replace(b"-0.26432", b"-0.36432", 1)
    seen_poses: list[tuple[float, ...]] = []

    def fake_capture(*, render_settings: render.RenderSettings, **kwargs: object) -> bytes:
        del kwargs
        seen_poses.append(render_settings.camera_pose)
        if len(seen_poses) == 1:
            config_path.write_bytes(changed)
        return _png(raw=(b"\x00" + bytes([len(seen_poses)]) * (512 * 3)) * 512)

    monkeypatch.setattr(render, "render_config_path", lambda: config_path)
    monkeypatch.setattr(render, "capture_model_thumbnail", fake_capture)
    candidates = tmp_path / "candidates"

    render.render_candidates(render_export, candidates)

    manifest = json.loads((candidates / "render-manifest.json").read_text(encoding="utf-8"))
    assert manifest["render_config_sha256"] == original_digest
    assert seen_poses == [
        (-0.26432, -1.1, 0.32, 0.0, 0.22, 1.570796326795),
        (0.26432, -1.1, 0.32, 0.0, 0.22, 1.570796326795),
        (1.45, 0.0, 0.22, 0.0, 0.1, 3.14159265359),
    ]


def test_render_iteration_accepts_a_new_camera_config_before_approved_thumbnails_are_replaced(
    render_export: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    config_path = tmp_path / "render.yaml"
    changed = render.render_config_path().read_bytes().replace(b"-0.26432", b"-0.36432", 1)
    config_path.write_bytes(changed)
    capture_calls: list[str] = []

    def fake_capture(*, model_dir: Path, **kwargs: object) -> bytes:
        del kwargs
        capture_calls.append(model_dir.name)
        return _png(raw=(b"\x00" + bytes([len(capture_calls) + 50]) * (512 * 3)) * 512)

    monkeypatch.setattr(render, "render_config_path", lambda: config_path)
    monkeypatch.setattr(thumbnail_module, "render_config_path", lambda: config_path)
    monkeypatch.setattr(render, "capture_model_thumbnail", fake_capture)
    candidates = tmp_path / "candidates"

    render.render_candidates(render_export, candidates)

    assert capture_calls == list(SLUGS)
    manifest = json.loads((candidates / "render-manifest.json").read_text(encoding="utf-8"))
    assert manifest["render_config_sha256"] == hashlib.sha256(changed).hexdigest()


def test_render_input_validation_still_rejects_mutated_model_bytes_before_renderer_runs(
    render_export: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    mutated_export = tmp_path / "models"
    shutil.copytree(render_export, mutated_export)
    model_sdf = mutated_export / SLUGS[0] / "model.sdf"
    original = model_sdf.read_bytes()
    changed = original.replace(b"<static>false</static>", b"<static>true</static>", 1)
    if changed == original:
        changed = original + b"\n<!-- mutation -->\n"
    model_sdf.write_bytes(changed)
    capture_calls: list[str] = []

    def forbidden_capture(**kwargs: object) -> bytes:
        capture_calls.append(str(kwargs.get("model_dir")))
        raise AssertionError("renderer must not receive an invalid model export")

    monkeypatch.setattr(render, "capture_model_thumbnail", forbidden_capture)

    with pytest.raises(ValueError, match="validation|model|digest|mismatch"):
        render.render_candidates(mutated_export, tmp_path / "candidates")

    assert capture_calls == []


@pytest.mark.parametrize("spec_index", range(3))
def test_render_world_has_exact_camera_lights_ground_and_controller_free_include(
    tmp_path: Path,
    spec_index: int,
) -> None:
    spec = load_model_specs()[spec_index]
    model_dir = tmp_path / "models" / spec.slug
    model_dir.mkdir(parents=True)
    original_model = b'<sdf version="1.11"><model name="model"/></sdf>\n'
    (model_dir / "model.sdf").write_bytes(original_model)
    frames = (tmp_path / "frames").resolve()
    frames.mkdir()
    output = tmp_path / "world.sdf"

    assert render.build_render_world(model_dir, output, frames) == output

    root = ElementTree.parse(output).getroot()
    world = root.find("world")
    assert root.attrib == {"version": "1.11"}
    assert world is not None and world.attrib == {"name": "a1_render"}
    assert world.findtext("gravity") == "0 0 0"
    assert [include.findtext("uri") for include in world.findall("include")] == [f"model://{spec.slug}"]
    assert world.find("include/plugin") is None
    assert len(world.findall("light")) == 2
    assert len(world.findall("model[@name='ground_plane']")) == 1
    sensors = world.findall(".//sensor[@type='camera']")
    assert len(sensors) == 1
    camera = sensors[0].find("camera")
    assert camera is not None
    assert camera.findtext("horizontal_fov") == "0.9"
    assert camera.findtext("image/width") == "512"
    assert camera.findtext("image/height") == "512"
    assert camera.findtext("image/format") == "R8G8B8"
    assert camera.find("save").attrib == {"enabled": "true"}
    assert camera.findtext("save/path") == os.fspath(frames)
    plugins = world.findall("plugin")
    assert len([plugin for plugin in plugins if plugin.attrib.get("filename") == "gz-sim-sensors-system"]) == 1
    sensor_plugin = next(plugin for plugin in plugins if plugin.attrib.get("filename") == "gz-sim-sensors-system")
    assert sensor_plugin.attrib["name"] == "gz::sim::systems::Sensors"
    assert sensor_plugin.findtext("render_engine") == "ogre2"
    assert not any("controller" in plugin.attrib.get("filename", "").lower() for plugin in root.iter("plugin"))
    assert (model_dir / "model.sdf").read_bytes() == original_model


def test_render_world_rejects_unsafe_or_non_absolute_paths(tmp_path: Path) -> None:
    spec = load_model_specs()[0]
    real_model = tmp_path / "real" / spec.slug
    real_model.mkdir(parents=True)
    (real_model / "model.sdf").write_text("<sdf/>", encoding="utf-8")
    linked_model = tmp_path / "models" / spec.slug
    linked_model.parent.mkdir()
    linked_model.symlink_to(real_model, target_is_directory=True)

    with pytest.raises(ValueError, match="symlink"):
        render.build_render_world(linked_model, tmp_path / "world.sdf", tmp_path.resolve())
    with pytest.raises(ValueError, match="absolute"):
        render.build_render_world(real_model, tmp_path / "other.sdf", Path("relative-frames"))


def test_select_frame_uses_greatest_numeric_suffix_not_mtime_or_lexicographic(tmp_path: Path) -> None:
    frames = tmp_path / "frames"
    frames.mkdir()
    for suffix in (10, 2, 9):
        path = frames / f"{FRAME_PREFIX}{suffix}.png"
        path.write_bytes(_png())
        os.utime(path, ns=(suffix, suffix))

    selected = render.select_frame(frames)

    assert selected.name == f"{FRAME_PREFIX}10.png"
    assert render.capture_selected_frame(frames).data == (frames / selected.name).read_bytes()


def test_select_frame_requires_three_complete_strict_numeric_regular_files(tmp_path: Path) -> None:
    frames = tmp_path / "frames"
    frames.mkdir()
    (frames / f"{FRAME_PREFIX}0.png").write_bytes(_png())
    (frames / f"{FRAME_PREFIX}1.png").write_bytes(_png())

    with pytest.raises(ValueError, match="three"):
        render.select_frame(frames)

    (frames / f"{FRAME_PREFIX}2.png").write_bytes(b"")
    with pytest.raises(ValueError, match="complete|PNG|signature"):
        render.select_frame(frames)

    (frames / f"{FRAME_PREFIX}2.png").write_bytes(_png())
    (frames / "attacker.png").write_bytes(_png())
    with pytest.raises(ValueError, match="unexpected"):
        render.select_frame(frames)


@pytest.mark.parametrize("entry_kind", ["symlink", "fifo"])
def test_select_frame_rejects_unsafe_entries(tmp_path: Path, entry_kind: str) -> None:
    frames = tmp_path / "frames"
    frames.mkdir()
    for suffix in range(3):
        (frames / f"{FRAME_PREFIX}{suffix}.png").write_bytes(_png())
    unsafe = frames / f"{FRAME_PREFIX}3.png"
    if entry_kind == "symlink":
        unsafe.symlink_to(frames / f"{FRAME_PREFIX}2.png")
    else:
        os.mkfifo(unsafe)

    with pytest.raises(ValueError, match="symlink|regular|snapshot"):
        render.select_frame(frames)


def test_manifest_and_thumbnail_snapshot_are_deterministic_and_exact(tmp_path: Path) -> None:
    root = tmp_path / "candidates"
    pngs, model_digests = _write_thumbnail_set(root)

    first = capture_thumbnail_set(
        root,
        expected_render_config_sha256="a" * 64,
        expected_model_sdf_sha256=model_digests,
    )
    second = capture_thumbnail_set(
        root,
        expected_render_config_sha256="a" * 64,
        expected_model_sdf_sha256=model_digests,
    )

    assert first == second
    assert dict(first.png_by_slug) == pngs
    assert first.manifest_bytes == _manifest(pngs, model_digests=model_digests)
    parsed = json.loads(first.manifest_bytes)
    assert set(parsed) == {"schema_version", "render_engine", "backend", "render_config_sha256", "models"}
    assert parsed["render_engine"] == "ogre2"
    assert parsed["backend"] == "software"
    assert "time" not in first.manifest_bytes.decode().lower()
    assert os.fspath(tmp_path) not in first.manifest_bytes.decode()


@pytest.mark.parametrize("mutation", ["missing", "extra", "hash", "config", "model"])
def test_thumbnail_snapshot_rejects_wrong_inventory_and_hash_drift(tmp_path: Path, mutation: str) -> None:
    root = tmp_path / "candidates"
    _, model_digests = _write_thumbnail_set(root)
    if mutation == "missing":
        (root / f"{SLUGS[0]}.png").unlink()
    elif mutation == "extra":
        (root / "extra.txt").write_text("attacker", encoding="utf-8")
    else:
        manifest_path = root / "render-manifest.json"
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if mutation == "hash":
            manifest["models"][0]["png_sha256"] = "0" * 64
        elif mutation == "config":
            manifest["render_config_sha256"] = "0" * 64
        else:
            manifest["models"][0]["model_sdf_sha256"] = "0" * 64
        manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

    with pytest.raises(ValueError, match="missing|extra|inventory|hash|digest|config|model"):
        capture_thumbnail_set(
            root,
            expected_render_config_sha256="a" * 64,
            expected_model_sdf_sha256=model_digests,
        )


@pytest.mark.parametrize("entry_kind", ["symlink", "fifo"])
def test_thumbnail_snapshot_rejects_symlink_and_fifo(tmp_path: Path, entry_kind: str) -> None:
    root = tmp_path / "candidates"
    _, model_digests = _write_thumbnail_set(root)
    victim = root / f"{SLUGS[0]}.png"
    victim.unlink()
    if entry_kind == "symlink":
        victim.symlink_to(root / f"{SLUGS[1]}.png")
    else:
        os.mkfifo(victim)

    with pytest.raises(ValueError, match="symlink|regular|snapshot"):
        capture_thumbnail_set(
            root,
            expected_render_config_sha256="a" * 64,
            expected_model_sdf_sha256=model_digests,
        )


def test_build_manifest_rejects_non_exact_slug_mappings() -> None:
    pngs = {slug: _png() for slug in SLUGS[:-1]}

    with pytest.raises(ValueError, match="exactly"):
        build_render_manifest(
            MappingProxyType(pngs),
            render_config_sha256="a" * 64,
            model_sdf_sha256={slug: "b" * 64 for slug in SLUGS},
        )


def test_approval_installs_the_exact_captured_bytes_without_starting_a_process(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    candidates = tmp_path / "candidates"
    pngs, model_digests = _write_thumbnail_set(candidates)
    approved = tmp_path / "approved"

    def forbidden_popen(*args: object, **kwargs: object) -> None:
        raise AssertionError("approval must never start a subprocess")

    monkeypatch.setattr(render.subprocess, "Popen", forbidden_popen)
    installed = render.approve_candidates(
        candidates,
        approved_root=approved,
        expected_render_config_sha256="a" * 64,
        expected_model_sdf_sha256=model_digests,
    )

    assert installed == approved
    assert {path.name for path in approved.iterdir()} == {
        "render-manifest.json",
        *(f"{slug}.png" for slug in SLUGS),
    }
    assert {slug: (approved / f"{slug}.png").read_bytes() for slug in SLUGS} == pngs
    assert (approved / "render-manifest.json").read_bytes() == _manifest(pngs, model_digests=model_digests)


def test_approval_creates_one_missing_direct_parent_for_first_install(tmp_path: Path) -> None:
    candidates = tmp_path / "candidates"
    pngs, model_digests = _write_thumbnail_set(candidates)
    approved = tmp_path / "new-assets-parent" / "thumbnails"

    render.approve_candidates(
        candidates,
        approved_root=approved,
        expected_render_config_sha256="a" * 64,
        expected_model_sdf_sha256=model_digests,
    )

    assert approved.is_dir()
    assert {slug: (approved / f"{slug}.png").read_bytes() for slug in SLUGS} == pngs


def test_approval_uses_one_candidate_snapshot_and_never_reopens_candidate_paths(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    candidates = tmp_path / "candidates"
    pngs, model_digests = _write_thumbnail_set(candidates)
    approved = tmp_path / "approved"
    real_capture = render.capture_thumbnail_set
    capture_calls = 0

    def capture_once(*args: object, **kwargs: object):
        nonlocal capture_calls
        capture_calls += 1
        snapshot = real_capture(*args, **kwargs)
        if Path(args[0]) == candidates:
            for slug in SLUGS:
                (candidates / f"{slug}.png").write_bytes(b"changed after snapshot")
        return snapshot

    monkeypatch.setattr(render, "capture_thumbnail_set", capture_once)

    render.approve_candidates(
        candidates,
        approved_root=approved,
        expected_render_config_sha256="a" * 64,
        expected_model_sdf_sha256=model_digests,
    )

    assert capture_calls == 1
    assert {slug: (approved / f"{slug}.png").read_bytes() for slug in SLUGS} == pngs


def test_approval_install_failure_restores_previous_complete_set(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    candidates = tmp_path / "candidates"
    _, model_digests = _write_thumbnail_set(candidates)
    approved = tmp_path / "approved"
    previous_pngs = {
        slug: _png(raw=(b"\x00" + bytes([index + 1]) * (512 * 3)) * 512) for index, slug in enumerate(SLUGS)
    }
    _write_thumbnail_set(approved, pngs=previous_pngs, model_digests=model_digests)
    previous = {path.name: path.read_bytes() for path in approved.iterdir()}
    real_replace = render.os.replace

    def fail_stage_install(source: object, destination: object, *args: object, **kwargs: object) -> None:
        if Path(source).name.startswith(".approved.stage-") and Path(destination).name == approved.name:
            raise OSError("injected approval install failure")
        real_replace(source, destination, *args, **kwargs)

    monkeypatch.setattr(render.os, "replace", fail_stage_install)

    with pytest.raises(OSError, match="injected approval install failure"):
        render.approve_candidates(
            candidates,
            approved_root=approved,
            expected_render_config_sha256="a" * 64,
            expected_model_sdf_sha256=model_digests,
        )

    assert {path.name: path.read_bytes() for path in approved.iterdir()} == previous
    assert not list(tmp_path.glob(".approved.stage-*"))
    assert not list(tmp_path.glob(".approved.backup-*"))


@pytest.mark.parametrize("interrupted_transition", ["previous-to-backup", "staging-to-approved"])
def test_approval_reconciles_completed_rename_before_interrupt_and_restores_previous_set(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    interrupted_transition: str,
) -> None:
    candidates = tmp_path / "candidates"
    candidate_pngs = {
        slug: _png(raw=(b"\x00" + bytes([index + 61]) * (512 * 3)) * 512) for index, slug in enumerate(SLUGS)
    }
    _, model_digests = _write_thumbnail_set(candidates, pngs=candidate_pngs)
    approved = tmp_path / "approved"
    previous_pngs = {
        slug: _png(raw=(b"\x00" + bytes([index + 71]) * (512 * 3)) * 512) for index, slug in enumerate(SLUGS)
    }
    _write_thumbnail_set(approved, pngs=previous_pngs, model_digests=model_digests)
    previous = {path.name: path.read_bytes() for path in approved.iterdir()}
    original_replace = render.os.replace
    interrupted = False

    def interrupt_after_real_rename(source: object, destination: object, *args: object, **kwargs: object) -> None:
        nonlocal interrupted
        source_name = os.fspath(source)
        destination_name = os.fspath(destination)
        transition = (
            "previous-to-backup"
            if source_name == "approved" and destination_name.startswith(".approved.backup-")
            else "staging-to-approved"
            if source_name.startswith(".approved.stage-") and destination_name == "approved"
            else None
        )
        original_replace(source, destination, *args, **kwargs)
        if not interrupted and transition == interrupted_transition:
            interrupted = True
            raise KeyboardInterrupt(f"interrupted after {interrupted_transition} rename")

    monkeypatch.setattr(render.os, "replace", interrupt_after_real_rename)

    with pytest.raises(KeyboardInterrupt, match=interrupted_transition):
        render.approve_candidates(
            candidates,
            approved_root=approved,
            expected_render_config_sha256="a" * 64,
            expected_model_sdf_sha256=model_digests,
        )

    assert interrupted
    assert {path.name: path.read_bytes() for path in approved.iterdir()} == previous
    assert not list(tmp_path.glob(".approved.stage-*"))
    assert not list(tmp_path.glob(".approved.backup-*"))


def test_approval_rejects_symlink_target_without_touching_outside(tmp_path: Path) -> None:
    candidates = tmp_path / "candidates"
    _, model_digests = _write_thumbnail_set(candidates)
    outside = tmp_path / "outside"
    outside.mkdir()
    sentinel = outside / "sentinel"
    sentinel.write_bytes(b"keep")
    approved = tmp_path / "approved"
    approved.symlink_to(outside, target_is_directory=True)

    with pytest.raises(ValueError, match="symlink"):
        render.approve_candidates(
            candidates,
            approved_root=approved,
            expected_render_config_sha256="a" * 64,
            expected_model_sdf_sha256=model_digests,
        )

    assert sentinel.read_bytes() == b"keep"
    assert approved.is_symlink()


def test_approval_rejects_symlink_direct_parent_without_touching_target(tmp_path: Path) -> None:
    candidates = tmp_path / "candidates"
    _, model_digests = _write_thumbnail_set(candidates)
    outside = tmp_path / "outside"
    outside.mkdir()
    sentinel = outside / "sentinel"
    sentinel.write_bytes(b"keep")
    linked_parent = tmp_path / "linked-parent"
    linked_parent.symlink_to(outside, target_is_directory=True)

    with pytest.raises(ValueError, match="parent.*symlink|symlink.*parent"):
        render.approve_candidates(
            candidates,
            approved_root=linked_parent / "thumbnails",
            expected_render_config_sha256="a" * 64,
            expected_model_sdf_sha256=model_digests,
        )

    assert sentinel.read_bytes() == b"keep"
    assert not (outside / "thumbnails").exists()


def test_approval_defaults_capture_trusted_expectations_once_before_candidate_validation(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    candidates = tmp_path / "candidates"
    model_digests = dict(current_model_sdf_sha256())
    config_digest = hashlib.sha256(render.render_config_path().read_bytes()).hexdigest()
    _write_thumbnail_set(
        candidates,
        config_digest=config_digest,
        model_digests=model_digests,
    )
    config_calls = 0
    model_calls = 0

    def changing_config() -> str:
        nonlocal config_calls
        config_calls += 1
        return config_digest if config_calls == 1 else "f" * 64

    def changing_models():
        nonlocal model_calls
        model_calls += 1
        if model_calls == 1:
            return MappingProxyType(model_digests)
        return MappingProxyType({slug: "f" * 64 for slug in SLUGS})

    monkeypatch.setattr(render, "current_render_config_sha256", changing_config)
    monkeypatch.setattr(render, "current_model_sdf_sha256", changing_models, raising=False)
    monkeypatch.setattr(thumbnail_module, "current_render_config_sha256", changing_config)
    monkeypatch.setattr(thumbnail_module, "current_model_sdf_sha256", changing_models)
    approved = tmp_path / "approved"

    render.approve_candidates(candidates, approved_root=approved)

    assert config_calls == 1
    assert model_calls == 1
    assert approved.is_dir()


def test_current_model_hashes_come_from_one_immutable_public_source_snapshot(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    copied_source = tmp_path / "A1_2026"
    shutil.copytree(spec_module.source_root(), copied_source)
    monkeypatch.setattr(spec_module, "source_root", lambda: copied_source)
    expected = dict(thumbnail_module.current_model_sdf_sha256())
    original_convert = thumbnail_module.convert_urdf
    conversion_count = 0

    def mutate_live_source_after_first_conversion(*args: object, **kwargs: object):
        nonlocal conversion_count
        tree = original_convert(*args, **kwargs)
        conversion_count += 1
        if conversion_count == 1:
            left_urdf = copied_source / "a1_l.urdf"
            original = left_urdf.read_bytes()
            changed = original.replace(
                b'value="0.401650168942009"',
                b'value="0.501650168942009"',
                1,
            )
            assert changed != original
            left_urdf.write_bytes(changed)
        return tree

    monkeypatch.setattr(thumbnail_module, "convert_urdf", mutate_live_source_after_first_conversion)

    assert dict(thumbnail_module.current_model_sdf_sha256()) == expected
    assert conversion_count == len(SLUGS)


@pytest.mark.parametrize(
    "backup_cleanup_failure",
    [KeyboardInterrupt("backup cleanup interrupted"), ValueError("backup cleanup failed")],
    ids=["interrupt", "error"],
)
def test_approval_commit_point_never_rolls_back_installed_bytes_when_backup_cleanup_fails(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    backup_cleanup_failure: BaseException,
) -> None:
    candidates = tmp_path / "candidates"
    candidate_pngs = {
        slug: _png(raw=(b"\x00" + bytes([index + 11]) * (512 * 3)) * 512) for index, slug in enumerate(SLUGS)
    }
    _, model_digests = _write_thumbnail_set(candidates, pngs=candidate_pngs)
    approved = tmp_path / "approved"
    previous_pngs = {
        slug: _png(raw=(b"\x00" + bytes([index + 21]) * (512 * 3)) * 512) for index, slug in enumerate(SLUGS)
    }
    _write_thumbnail_set(approved, pngs=previous_pngs, model_digests=model_digests)
    original_remove = render._remove_held_child_at

    def fail_backup_cleanup(*args: object, **kwargs: object) -> None:
        label = args[3] if len(args) > 3 else kwargs.get("label")
        if label == "approved root backup":
            raise backup_cleanup_failure
        original_remove(*args, **kwargs)

    monkeypatch.setattr(render, "_remove_held_child_at", fail_backup_cleanup)

    with pytest.raises(type(backup_cleanup_failure), match="backup cleanup"):
        render.approve_candidates(
            candidates,
            approved_root=approved,
            expected_render_config_sha256="a" * 64,
            expected_model_sdf_sha256=model_digests,
        )

    assert {slug: (approved / f"{slug}.png").read_bytes() for slug in SLUGS} == candidate_pngs


def test_approval_post_backup_cleanup_verification_failure_keeps_committed_install(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    candidates = tmp_path / "candidates"
    candidate_pngs = {
        slug: _png(raw=(b"\x00" + bytes([index + 31]) * (512 * 3)) * 512) for index, slug in enumerate(SLUGS)
    }
    _, model_digests = _write_thumbnail_set(candidates, pngs=candidate_pngs)
    approved = tmp_path / "approved"
    previous_pngs = {
        slug: _png(raw=(b"\x00" + bytes([index + 41]) * (512 * 3)) * 512) for index, slug in enumerate(SLUGS)
    }
    _write_thumbnail_set(approved, pngs=previous_pngs, model_digests=model_digests)
    original_remove = render._remove_held_child_at
    original_verify = render._verify_held_directory
    backup_disposed = False

    def record_backup_disposal(*args: object, **kwargs: object) -> None:
        nonlocal backup_disposed
        original_remove(*args, **kwargs)
        label = args[3] if len(args) > 3 else kwargs.get("label")
        if label == "approved root backup":
            backup_disposed = True

    def fail_verification_after_disposal(*args: object, **kwargs: object) -> None:
        label = args[1] if len(args) > 1 else kwargs.get("label")
        if backup_disposed and label == "approved root parent":
            raise ValueError("post-cleanup verification failed")
        original_verify(*args, **kwargs)

    monkeypatch.setattr(render, "_remove_held_child_at", record_backup_disposal)
    monkeypatch.setattr(render, "_verify_held_directory", fail_verification_after_disposal)

    with pytest.raises(ValueError, match="post-cleanup verification"):
        render.approve_candidates(
            candidates,
            approved_root=approved,
            expected_render_config_sha256="a" * 64,
            expected_model_sdf_sha256=model_digests,
        )

    assert backup_disposed
    assert {slug: (approved / f"{slug}.png").read_bytes() for slug in SLUGS} == candidate_pngs


def test_approval_rejects_candidate_root_replaced_after_snapshot(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    candidates = tmp_path / "candidates"
    _, model_digests = _write_thumbnail_set(candidates)
    displaced = tmp_path / "displaced"
    attacker = tmp_path / "attacker"
    attacker.mkdir()
    approved = tmp_path / "approved"
    real_capture = render.capture_thumbnail_set

    def replace_after_capture(*args: object, **kwargs: object):
        result = real_capture(*args, **kwargs)
        if Path(args[0]) == candidates:
            candidates.rename(displaced)
            candidates.symlink_to(attacker, target_is_directory=True)
        return result

    monkeypatch.setattr(render, "capture_thumbnail_set", replace_after_capture)

    with pytest.raises(ValueError, match="changed|replaced"):
        render.approve_candidates(
            candidates,
            approved_root=approved,
            expected_render_config_sha256="a" * 64,
            expected_model_sdf_sha256=model_digests,
        )

    assert not approved.exists()
    assert list(attacker.iterdir()) == []


def test_approval_parent_swap_is_detected_and_rolled_back_via_retained_directory(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    candidates = tmp_path / "candidates"
    _, model_digests = _write_thumbnail_set(candidates)
    parent = tmp_path / "assets"
    parent.mkdir()
    approved = parent / "thumbnails"
    previous_pngs = {
        slug: _png(raw=(b"\x00" + bytes([index + 4]) * (512 * 3)) * 512) for index, slug in enumerate(SLUGS)
    }
    _write_thumbnail_set(approved, pngs=previous_pngs, model_digests=model_digests)
    previous = {path.name: path.read_bytes() for path in approved.iterdir()}
    displaced = tmp_path / "assets-displaced"
    real_replace = render.os.replace
    swapped = False

    def swap_parent_then_replace(source: object, destination: object, *args: object, **kwargs: object) -> None:
        nonlocal swapped
        source_name = Path(source).name if isinstance(source, (str, os.PathLike)) else ""
        if not swapped and source_name == "thumbnails":
            parent.rename(displaced)
            parent.mkdir()
            (parent / "attacker-sentinel").write_bytes(b"keep")
            swapped = True
        real_replace(source, destination, *args, **kwargs)

    monkeypatch.setattr(render.os, "replace", swap_parent_then_replace)

    with pytest.raises(ValueError, match="parent.*changed|changed.*parent"):
        render.approve_candidates(
            candidates,
            approved_root=approved,
            expected_render_config_sha256="a" * 64,
            expected_model_sdf_sha256=model_digests,
        )

    restored = displaced / "thumbnails"
    assert {path.name: path.read_bytes() for path in restored.iterdir()} == previous
    assert (parent / "attacker-sentinel").read_bytes() == b"keep"
    assert {path.name for path in parent.iterdir()} == {"attacker-sentinel"}


class _RenderProcess:
    def __init__(
        self,
        harness: _RenderHarness,
        *,
        pid: int,
        kind: str,
        stdout: object,
        stderr: object,
    ) -> None:
        self.harness = harness
        self.pid = pid
        self.kind = kind
        self.returncode = harness.server_returncode if kind == "server" else harness.subscriber_returncode
        self.stdout = io.StringIO(harness.server_stdout) if kind == "server" and stdout == subprocess.PIPE else None
        self.stderr = io.StringIO(harness.server_stderr) if kind == "server" and stderr == subprocess.PIPE else None

    def poll(self) -> int | None:
        return self.returncode

    def wait(self, timeout: float | None = None) -> int:
        del timeout
        self.harness.events.append(f"wait:{self.kind}")
        self.returncode = 0
        self.harness.alive_groups.discard(self.pid)
        return 0


class _RenderHarness:
    def __init__(
        self,
        *,
        publisher_types: tuple[str, ...] = ("gz.msgs.Image",),
        topic_list: str = f"{CAMERA_TOPIC}\n",
        subscriber_visible: bool = True,
        unpause_stdout: str = "data: true\n",
        pause_stdout: str = "data: true\n",
        frame_mode: str = "complete",
        server_stdout: str = "Gazebo render server ready\n",
        server_stderr: str = "",
        subscriber_start_error: bool = False,
        server_returncode: int | None = None,
        subscriber_returncode: int | None = None,
        subscriber_exit_on_unpause: int | None = None,
        subscriber_row: str = "node, google.protobuf.Message",
        preexisting_subscriber: bool = False,
        publisher_section: str | None = None,
    ) -> None:
        self.publisher_types = publisher_types
        self.topic_list = topic_list
        self.subscriber_visible = subscriber_visible
        self.unpause_stdout = unpause_stdout
        self.pause_stdout = pause_stdout
        self.frame_mode = frame_mode
        self.server_stdout = server_stdout
        self.server_stderr = server_stderr
        self.subscriber_start_error = subscriber_start_error
        self.server_returncode = server_returncode
        self.subscriber_returncode = subscriber_returncode
        self.subscriber_exit_on_unpause = subscriber_exit_on_unpause
        self.subscriber_row = subscriber_row
        self.preexisting_subscriber = preexisting_subscriber
        self.publisher_section = publisher_section
        self.events: list[str] = []
        self.alive_groups: set[int] = set()
        self.server: _RenderProcess | None = None
        self.subscriber: _RenderProcess | None = None
        self.server_argv: list[str] = []
        self.subscriber_argv: list[str] = []
        self.frame_dir: Path | None = None
        self.incomplete_frame: Path | None = None
        self.command_environments: list[tuple[str, ...]] = []

    @staticmethod
    def inner(argv: list[str]) -> list[str]:
        return argv[argv.index("gz") :]

    def popen(self, argv: list[str], **kwargs: object) -> _RenderProcess:
        inner = self.inner(argv)
        assert kwargs.get("start_new_session") is True
        if inner[:2] == ["gz", "sim"]:
            self.events.append("popen:server-paused")
            self.server_argv = argv
            world_path = Path(inner[-1])
            save_path = ElementTree.parse(world_path).getroot().findtext(".//sensor/camera/save/path")
            assert save_path is not None
            self.frame_dir = Path(save_path)
            process = _RenderProcess(
                self,
                pid=61001,
                kind="server",
                stdout=kwargs.get("stdout"),
                stderr=kwargs.get("stderr"),
            )
            self.server = process
        elif inner[:5] == ["gz", "topic", "--force-version", "13", "-e"]:
            self.events.append("popen:subscriber")
            if self.subscriber_start_error:
                raise OSError("subscriber start failed")
            self.subscriber_argv = argv
            process = _RenderProcess(
                self,
                pid=61002,
                kind="subscriber",
                stdout=kwargs.get("stdout"),
                stderr=kwargs.get("stderr"),
            )
            self.subscriber = process
        else:
            raise AssertionError(f"unexpected Popen command: {inner}")
        self.alive_groups.add(process.pid)
        return process

    def _write_frames(self) -> None:
        assert self.frame_dir is not None
        if self.frame_mode == "none":
            return
        for suffix in range(2):
            (self.frame_dir / f"{FRAME_PREFIX}{suffix}.png").write_bytes(_png())
        third = self.frame_dir / f"{FRAME_PREFIX}2.png"
        if self.frame_mode == "incomplete_then_complete":
            third.write_bytes(b"")
            self.incomplete_frame = third
        else:
            third.write_bytes(_png())

    def sleep(self, duration: float) -> None:
        del duration
        if self.incomplete_frame is not None:
            self.incomplete_frame.write_bytes(_png())
            self.incomplete_frame = None

    def run_harmonic(self, **kwargs: object) -> demo._CommandResult:
        self.command_environments.append(tuple(kwargs.get("extra_environment", ())))
        command = list(kwargs["command"])
        if command[:5] == ["topic", "--force-version", "13", "-l"]:
            self.events.append("topic:list")
            return demo._CommandResult(0, self.topic_list, "")
        if command[:4] == ["topic", "--force-version", "13", "-i"]:
            subscriber = self.subscriber is not None and self.subscriber_visible
            self.events.append("topic:info-with-subscriber" if subscriber else "topic:info-publisher")
            body = self.publisher_section or (
                "Publishers [Address, Message Type]:\n"
                + "".join(
                    f"node-{index}, {publisher_type}\n" for index, publisher_type in enumerate(self.publisher_types)
                )
            )
            if subscriber or self.preexisting_subscriber:
                body += f"Subscribers [Address, Message Type]:\n{self.subscriber_row}\n"
            return demo._CommandResult(0, body, "")
        if command[:2] == ["service", "--force-version"]:
            service = command[command.index("-s") + 1]
            request = command[command.index("--req") + 1]
            if service == "/world/a1_render/control" and request == "pause: false":
                self.events.append("service:unpause")
                if self.unpause_stdout.strip() == "data: true":
                    self._write_frames()
                if self.subscriber is not None and self.subscriber_exit_on_unpause is not None:
                    self.subscriber.returncode = self.subscriber_exit_on_unpause
                return demo._CommandResult(0, self.unpause_stdout, "")
            if service == "/world/a1_render/control" and request == "pause: true":
                self.events.append("service:pause")
                if self.frame_mode == "post_pause_incomplete_then_complete":
                    assert self.frame_dir is not None
                    fourth = self.frame_dir / f"{FRAME_PREFIX}3.png"
                    fourth.write_bytes(b"")
                    self.incomplete_frame = fourth
                return demo._CommandResult(0, self.pause_stdout, "")
            if service == "/server_control":
                self.events.append("service:server-stop")
                return demo._CommandResult(0, "data: true\n", "")
        raise AssertionError(f"unexpected harmonic command: {command}")

    def killpg(self, pid: int, sent_signal: signal.Signals | int) -> None:
        if sent_signal == 0:
            if pid not in self.alive_groups:
                raise ProcessLookupError(pid)
            return
        self.events.append(f"kill:{pid}:{int(sent_signal)}")
        self.alive_groups.discard(pid)
        process = self.server if pid == 61001 else self.subscriber
        if process is not None:
            process.returncode = -int(sent_signal)


@pytest.fixture(scope="module")
def render_export(tmp_path_factory: pytest.TempPathFactory) -> Path:
    root = tmp_path_factory.mktemp("render-export") / "models"
    export_models(root)
    return root


def _install_render_harness(monkeypatch: pytest.MonkeyPatch, harness: _RenderHarness) -> None:
    monkeypatch.setattr(render.subprocess, "Popen", harness.popen)
    monkeypatch.setattr(render, "_run_harmonic", harness.run_harmonic, raising=False)
    monkeypatch.setattr(demo, "_run_harmonic", harness.run_harmonic)
    monkeypatch.setattr(render.os, "killpg", harness.killpg)
    monkeypatch.setattr(render.time, "sleep", harness.sleep, raising=False)


def test_capture_starts_paused_connects_image_subscriber_before_unpause_and_cleans_groups(
    render_export: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    harness = _RenderHarness()
    _install_render_harness(monkeypatch, harness)
    model = render_export / SLUGS[0]
    run_root = tmp_path.resolve() / "run"
    run_root.mkdir()

    data = render.capture_model_thumbnail(
        model_dir=model,
        private_export=render_export,
        run_root=run_root,
    )

    validate_thumbnail_bytes(data)
    server_inner = harness.inner(harness.server_argv)
    assert server_inner[:8] == ["gz", "sim", "--force-version", "8", "-s", "--headless-rendering", "-v", "4"]
    assert "-r" not in server_inner
    assert {
        "QT_QPA_PLATFORM=offscreen",
        "LIBGL_ALWAYS_SOFTWARE=1",
        f"HOME={run_root / 'home'}",
        f"XDG_CACHE_HOME={run_root / 'cache'}",
    } <= set(harness.server_argv)
    assert not any(value.startswith("MESA_LOADER_DRIVER_OVERRIDE=") for value in harness.server_argv)
    assert harness.inner(harness.subscriber_argv) == [
        "gz",
        "topic",
        "--force-version",
        "13",
        "-e",
        "-n",
        "3",
        "--json-output",
        "-t",
        CAMERA_TOPIC,
    ]
    assert harness.events.index("popen:subscriber") < harness.events.index("topic:info-with-subscriber")
    assert harness.events.index("topic:info-with-subscriber") < harness.events.index("service:unpause")
    assert harness.events.index("service:unpause") < harness.events.index("service:pause")
    assert harness.events.index("service:pause") < harness.events.index("service:server-stop")
    fixed_environment = {
        "QT_QPA_PLATFORM=offscreen",
        "LIBGL_ALWAYS_SOFTWARE=1",
        f"HOME={run_root / 'home'}",
        f"XDG_CACHE_HOME={run_root / 'cache'}",
    }
    assert harness.command_environments
    assert all(fixed_environment <= set(environment) for environment in harness.command_environments)
    assert harness.alive_groups == set()


def test_capture_records_selected_suffix_timing_and_hash_only_after_success(
    render_export: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    harness = _RenderHarness()
    _install_render_harness(monkeypatch, harness)
    run_root = tmp_path.resolve() / "run"
    run_root.mkdir()
    observations: list[render.RenderObservation] = []

    data = render.capture_model_thumbnail(
        model_dir=render_export / SLUGS[0],
        private_export=render_export,
        run_root=run_root,
        observations=observations,
    )

    assert observations == [
        render.RenderObservation(
            slug=SLUGS[0],
            elapsed_seconds=observations[0].elapsed_seconds,
            selected_frame_suffix=2,
            png_sha256=hashlib.sha256(data).hexdigest(),
        )
    ]
    assert observations[0].elapsed_seconds >= 0


def test_capture_waits_past_a_transient_zero_byte_third_frame(
    render_export: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    harness = _RenderHarness(frame_mode="incomplete_then_complete")
    _install_render_harness(monkeypatch, harness)
    run_root = tmp_path.resolve() / "run"
    run_root.mkdir()

    data = render.capture_model_thumbnail(
        model_dir=render_export / SLUGS[0],
        private_export=render_export,
        run_root=run_root,
    )

    validate_thumbnail_bytes(data)
    assert harness.incomplete_frame is None
    assert harness.alive_groups == set()


def test_capture_retries_a_transient_incomplete_frame_created_after_pause(
    render_export: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    harness = _RenderHarness(frame_mode="post_pause_incomplete_then_complete")
    _install_render_harness(monkeypatch, harness)
    run_root = tmp_path.resolve() / "run"
    run_root.mkdir()

    data = render.capture_model_thumbnail(
        model_dir=render_export / SLUGS[0],
        private_export=render_export,
        run_root=run_root,
    )

    validate_thumbnail_bytes(data)
    assert harness.incomplete_frame is None
    assert hashlib.sha256(data).hexdigest() == hashlib.sha256(_png()).hexdigest()
    assert harness.alive_groups == set()


@pytest.mark.parametrize(
    ("harness", "message"),
    [
        (_RenderHarness(publisher_types=("gz.msgs.String",)), "gz.msgs.Image"),
        (_RenderHarness(unpause_stdout="data: false\n"), "unpause"),
        (_RenderHarness(pause_stdout="Service call timed out\n"), "pause"),
        (_RenderHarness(subscriber_start_error=True), "subscriber"),
    ],
)
def test_capture_fails_closed_and_cleans_every_group(
    render_export: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    harness: _RenderHarness,
    message: str,
) -> None:
    _install_render_harness(monkeypatch, harness)
    run_root = tmp_path.resolve() / "run"
    run_root.mkdir()

    with pytest.raises(RuntimeError, match=message):
        render.capture_model_thumbnail(
            model_dir=render_export / SLUGS[0],
            private_export=render_export,
            run_root=run_root,
            timeout=0.05,
        )

    assert harness.alive_groups == set()


@pytest.mark.parametrize(
    ("harness", "message"),
    [
        (_RenderHarness(server_returncode=23), "server exited early.*23"),
        (_RenderHarness(subscriber_returncode=24), "subscriber exited.*24"),
        (_RenderHarness(topic_list="/unrelated\n"), "topic discovery timeout"),
        (_RenderHarness(publisher_types=()), "gz.msgs.Image"),
        (_RenderHarness(publisher_types=("gz.msgs.Image", "gz.msgs.String")), "exactly one.*gz.msgs.Image"),
        (_RenderHarness(unpause_stdout="prefix data: true suffix\n"), "unpause"),
        (_RenderHarness(unpause_stdout="data: truefalse\n"), "unpause"),
    ],
)
def test_capture_rejects_early_exit_discovery_ambiguity_and_nonsemantic_service_text(
    render_export: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    harness: _RenderHarness,
    message: str,
) -> None:
    _install_render_harness(monkeypatch, harness)
    run_root = tmp_path.resolve() / "run"
    run_root.mkdir()

    with pytest.raises(RuntimeError, match=message):
        render.capture_model_thumbnail(
            model_dir=render_export / SLUGS[0],
            private_export=render_export,
            run_root=run_root,
            timeout=0.01,
        )

    assert harness.alive_groups == set()


@pytest.mark.parametrize(
    ("publisher_section", "message"),
    [
        (
            "Publishers [Address, Message Type]:\nnode-good, gz.msgs.Image\nmalformed-junk-row\n",
            "publisher.*malformed",
        ),
        (
            "Publishers [Address, Message Type]:\nnode-good, gz.msgs.Image\nnode-junk, gz.msgs.String, extra\n",
            "publisher.*malformed",
        ),
        ("Publishers:\nnode-good, gz.msgs.Image\n", "publisher.*header|publisher.*section"),
        (
            "Publishers [Address, Message Type]:\nnode-good, gz.msgs.Image\n"
            "Publishers [Address, Message Type]:\nnode-other, gz.msgs.Image\n",
            "publisher.*exactly once|publisher.*section",
        ),
    ],
)
def test_capture_fails_closed_on_malformed_or_ambiguous_publisher_information(
    render_export: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    publisher_section: str,
    message: str,
) -> None:
    harness = _RenderHarness(publisher_section=publisher_section)
    _install_render_harness(monkeypatch, harness)
    run_root = tmp_path.resolve() / "run"
    run_root.mkdir()

    with pytest.raises(RuntimeError, match=message):
        render.capture_model_thumbnail(
            model_dir=render_export / SLUGS[0],
            private_export=render_export,
            run_root=run_root,
            timeout=0.01,
        )

    assert "service:unpause" not in harness.events
    assert harness.alive_groups == set()


def test_publisher_parser_accepts_exact_real_harmonic_no_subscriber_terminator() -> None:
    information = (
        "Publishers [Address, Message Type]:\n"
        "  tcp://172.17.0.1:36805, gz.msgs.Image\n"
        f"No subscribers on topic [{render._CAMERA_TOPIC}]\n"
    )

    assert render._publisher_message_types(information) == ("gz.msgs.Image",)


@pytest.mark.parametrize(
    "terminator",
    [
        "No subscribers on topic [/world/wrong/image]",
        f"No subscribers on topic {render._CAMERA_TOPIC}",
        (f"No subscribers on topic [{render._CAMERA_TOPIC}]\nNo subscribers on topic [{render._CAMERA_TOPIC}]"),
        f"No subscribers on topic [{render._CAMERA_TOPIC}]\nmalformed-junk-row",
    ],
)
def test_publisher_parser_rejects_nonexact_or_nonterminal_no_subscriber_text(terminator: str) -> None:
    information = f"Publishers [Address, Message Type]:\n  tcp://172.17.0.1:36805, gz.msgs.Image\n{terminator}\n"

    with pytest.raises(ValueError, match="publisher"):
        render._publisher_message_types(information)


def test_capture_rejects_fatal_sensor_log_seen_during_lifecycle_and_cleans_groups(
    render_export: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    harness = _RenderHarness(server_stderr="[Err] Failed to initialize Sensors system with Ogre2\n")
    _install_render_harness(monkeypatch, harness)
    run_root = tmp_path.resolve() / "run"
    run_root.mkdir()

    with pytest.raises(RuntimeError, match="Sensors|Ogre2"):
        render.capture_model_thumbnail(
            model_dir=render_export / SLUGS[0],
            private_export=render_export,
            run_root=run_root,
        )

    assert harness.alive_groups == set()


@pytest.mark.parametrize("returncode", [24, -9])
def test_capture_rejects_subscriber_nonzero_exit_after_connection(
    render_export: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    returncode: int,
) -> None:
    harness = _RenderHarness(subscriber_exit_on_unpause=returncode)
    _install_render_harness(monkeypatch, harness)
    run_root = tmp_path.resolve() / "run"
    run_root.mkdir()

    with pytest.raises(RuntimeError, match=rf"subscriber exited.*{returncode}"):
        render.capture_model_thumbnail(
            model_dir=render_export / SLUGS[0],
            private_export=render_export,
            run_root=run_root,
        )

    assert harness.alive_groups == set()


def test_capture_allows_subscriber_zero_exit_after_three_messages(
    render_export: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    harness = _RenderHarness(subscriber_exit_on_unpause=0)
    _install_render_harness(monkeypatch, harness)
    run_root = tmp_path.resolve() / "run"
    run_root.mkdir()

    validate_thumbnail_bytes(
        render.capture_model_thumbnail(
            model_dir=render_export / SLUGS[0],
            private_export=render_export,
            run_root=run_root,
        )
    )

    assert harness.alive_groups == set()


@pytest.mark.parametrize(
    ("subscriber_row", "message"),
    [
        ("node, gz.msgs.String", "subscriber.*type"),
        ("node, google.protobuf.Message, extra", "subscriber.*malformed"),
    ],
)
def test_capture_rejects_wrong_type_and_malformed_subscriber_rows(
    render_export: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    subscriber_row: str,
    message: str,
) -> None:
    harness = _RenderHarness(subscriber_row=subscriber_row)
    _install_render_harness(monkeypatch, harness)
    run_root = tmp_path.resolve() / "run"
    run_root.mkdir()

    with pytest.raises(RuntimeError, match=message):
        render.capture_model_thumbnail(
            model_dir=render_export / SLUGS[0],
            private_export=render_export,
            run_root=run_root,
            timeout=0.01,
        )

    assert harness.alive_groups == set()


def test_capture_requires_subscriber_count_to_increase_over_preconnection_baseline(
    render_export: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    harness = _RenderHarness(preexisting_subscriber=True, subscriber_visible=False)
    _install_render_harness(monkeypatch, harness)
    run_root = tmp_path.resolve() / "run"
    run_root.mkdir()

    with pytest.raises(RuntimeError, match="subscriber connection timeout|camera topic inspection timeout"):
        render.capture_model_thumbnail(
            model_dir=render_export / SLUGS[0],
            private_export=render_export,
            run_root=run_root,
            timeout=0.01,
        )

    assert "service:unpause" not in harness.events
    assert harness.alive_groups == set()


@pytest.mark.parametrize(
    "first_cleanup",
    ["raised_interrupt", "deferred_interrupt", "survived_group"],
)
def test_connection_failure_retries_subscriber_cleanup_and_leaves_no_orphan_group(
    render_export: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    first_cleanup: str,
) -> None:
    harness = _RenderHarness(subscriber_visible=False)
    _install_render_harness(monkeypatch, harness)
    run_root = tmp_path.resolve() / "run"
    run_root.mkdir()
    original_kill = demo._kill_child_group
    subscriber_cleanup_calls = 0
    interruption = KeyboardInterrupt(f"{first_cleanup} during subscriber cleanup")

    def fail_first_subscriber_cleanup(process: _RenderProcess):
        nonlocal subscriber_cleanup_calls
        if process.kind != "subscriber":
            return original_kill(process)
        subscriber_cleanup_calls += 1
        if subscriber_cleanup_calls > 1:
            return original_kill(process)
        if first_cleanup == "raised_interrupt":
            raise interruption
        if first_cleanup == "deferred_interrupt":
            return demo._GroupCleanupResult(False, interruption)
        return demo._GroupCleanupResult(False, None)

    monkeypatch.setattr(demo, "_kill_child_group", fail_first_subscriber_cleanup)

    with pytest.raises(RuntimeError, match="subscriber connection timeout|camera topic inspection timeout") as caught:
        render.capture_model_thumbnail(
            model_dir=render_export / SLUGS[0],
            private_export=render_export,
            run_root=run_root,
            timeout=0.01,
        )

    assert subscriber_cleanup_calls >= 2
    assert harness.alive_groups == set()
    if first_cleanup != "survived_group":
        assert "subscriber cleanup" in "\n".join(getattr(caught.value, "__notes__", ()))


def test_connection_failure_force_kills_subscriber_when_cleanup_helper_always_interrupts(
    render_export: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    harness = _RenderHarness(subscriber_visible=False)
    _install_render_harness(monkeypatch, harness)
    run_root = tmp_path.resolve() / "run"
    run_root.mkdir()
    original_kill = demo._kill_child_group
    interruption = KeyboardInterrupt("persistent subscriber cleanup interruption")
    subscriber_cleanup_calls = 0

    def always_interrupt_subscriber_cleanup(process: _RenderProcess):
        nonlocal subscriber_cleanup_calls
        if process.kind == "subscriber":
            subscriber_cleanup_calls += 1
            raise interruption
        return original_kill(process)

    monkeypatch.setattr(demo, "_kill_child_group", always_interrupt_subscriber_cleanup)

    with pytest.raises(RuntimeError, match="subscriber connection timeout|camera topic inspection timeout") as caught:
        render.capture_model_thumbnail(
            model_dir=render_export / SLUGS[0],
            private_export=render_export,
            run_root=run_root,
            timeout=0.01,
        )

    assert subscriber_cleanup_calls >= 2
    assert f"kill:61002:{int(signal.SIGKILL)}" in harness.events
    assert "service:server-stop" in harness.events
    assert harness.alive_groups == set()
    assert "persistent subscriber cleanup interruption" in "\n".join(getattr(caught.value, "__notes__", ()))


def test_cleanup_attempts_server_and_log_after_subscriber_cleanup_raises_interrupt(
    render_export: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    harness = _RenderHarness()
    _install_render_harness(monkeypatch, harness)
    run_root = tmp_path.resolve() / "run"
    run_root.mkdir()
    real_kill = demo._kill_child_group
    interruption = KeyboardInterrupt("subscriber cleanup interrupted")

    def interrupted_kill(process: _RenderProcess):
        if process.kind == "subscriber":
            harness.alive_groups.discard(process.pid)
            raise interruption
        return real_kill(process)

    monkeypatch.setattr(demo, "_kill_child_group", interrupted_kill)

    with pytest.raises(KeyboardInterrupt) as caught:
        render.capture_model_thumbnail(
            model_dir=render_export / SLUGS[0],
            private_export=render_export,
            run_root=run_root,
        )

    assert caught.value is interruption
    assert "service:server-stop" in harness.events
    assert harness.alive_groups == set()


def test_cleanup_preserves_deferred_interrupt_object_and_notes(
    render_export: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    harness = _RenderHarness()
    _install_render_harness(monkeypatch, harness)
    run_root = tmp_path.resolve() / "run"
    run_root.mkdir()
    real_kill = demo._kill_child_group
    interruption = KeyboardInterrupt("deferred subscriber interruption")
    interruption.add_note("nested cleanup evidence")

    def deferred_kill(process: _RenderProcess):
        if process.kind == "subscriber":
            harness.alive_groups.discard(process.pid)
            return demo._GroupCleanupResult(True, interruption)
        return real_kill(process)

    monkeypatch.setattr(demo, "_kill_child_group", deferred_kill)

    with pytest.raises(KeyboardInterrupt) as caught:
        render.capture_model_thumbnail(
            model_dir=render_export / SLUGS[0],
            private_export=render_export,
            run_root=run_root,
        )

    assert caught.value is interruption
    assert "nested cleanup evidence" in getattr(caught.value, "__notes__", ())
    assert "service:server-stop" in harness.events
    assert harness.alive_groups == set()


def test_capture_timeout_has_bounded_diagnostics_and_cleans_process_groups(
    render_export: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    harness = _RenderHarness(frame_mode="none")
    _install_render_harness(monkeypatch, harness)
    run_root = tmp_path.resolve() / "run"
    run_root.mkdir()

    with pytest.raises(RuntimeError, match="frame capture timeout") as caught:
        render.capture_model_thumbnail(
            model_dir=render_export / SLUGS[0],
            private_export=render_export,
            run_root=run_root,
            timeout=0.01,
        )

    assert "server log tail" in str(caught.value)
    assert len(str(caught.value)) < 70 * 1024
    assert harness.alive_groups == set()


@pytest.mark.parametrize(
    "log",
    [
        "[Err] Failed to initialize Sensors system",
        "[Error] Ogre2 render engine failed",
        "[Err] Unable to load mesh model://missing.stl",
        "Failed while initializing Ogre2 render engine",
        "Failed while initializing Sensors system",
        "Could not load rendering mesh model://asset.stl",
    ],
)
def test_render_fatal_log_classifier_catches_sensor_ogre_and_mesh_errors(log: str) -> None:
    assert render._render_critical_error(log) is not None


def test_render_log_classifier_is_sticky_across_chunks_and_tail_rollover() -> None:
    tail = demo._BoundedLogTail(maximum=128, classifier=render._render_critical_error)
    tail._append("stderr", "Sensors system fai")
    tail._append("stderr", "led to initialize Ogre2\n")
    tail._append("stderr", "x" * 4096)

    assert tail.critical_error() is not None
    assert "Sensors" not in tail.text()


def test_render_log_classifier_is_sticky_for_reverse_word_order_across_chunks_and_rollover() -> None:
    tail = demo._BoundedLogTail(maximum=128, classifier=render._render_critical_error)
    tail._append("stderr", "Could not load render")
    tail._append("stderr", "ing mesh model://asset.stl\n")
    tail._append("stderr", "x" * 4096)

    assert tail.critical_error() is not None
    assert "Could not load" not in tail.text()


def test_render_candidates_is_atomic_and_does_not_touch_approved_assets(
    render_export: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    approved = tmp_path / "approved"
    approved.mkdir()
    sentinel = approved / "sentinel"
    sentinel.write_bytes(b"reviewed")
    candidates = tmp_path / "candidates"
    calls: list[str] = []

    def fake_capture(*, model_dir: Path, **kwargs: object) -> bytes:
        del kwargs
        calls.append(model_dir.name)
        return _png(raw=(b"\x00" + bytes([len(calls)]) * (512 * 3)) * 512)

    monkeypatch.setattr(render, "capture_model_thumbnail", fake_capture)
    monkeypatch.setattr(render, "approved_assets_root", lambda: approved)

    result = render.render_candidates(render_export, candidates)

    assert result == candidates
    assert calls == list(SLUGS)
    assert sentinel.read_bytes() == b"reviewed"
    assert {path.name for path in candidates.iterdir()} == {
        "render-manifest.json",
        *(f"{slug}.png" for slug in SLUGS),
    }
    capture_thumbnail_set(candidates)


def test_render_candidates_leaves_no_partial_output_when_third_model_fails(
    render_export: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    candidates = tmp_path / "candidates"
    calls = 0

    def fail_third(**kwargs: object) -> bytes:
        del kwargs
        nonlocal calls
        calls += 1
        if calls == 3:
            raise RuntimeError("third render failed")
        return _png()

    monkeypatch.setattr(render, "capture_model_thumbnail", fail_third)

    with pytest.raises(RuntimeError, match="third render failed"):
        render.render_candidates(render_export, candidates)

    assert calls == 3
    assert not candidates.exists()
    assert not list(tmp_path.glob(".candidates.stage-*"))


def test_render_candidates_rejects_symlink_direct_parent_without_touching_target(
    render_export: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    outside = tmp_path / "outside"
    outside.mkdir()
    sentinel = outside / "sentinel"
    sentinel.write_bytes(b"keep")
    linked_parent = tmp_path / "linked-parent"
    linked_parent.symlink_to(outside, target_is_directory=True)
    monkeypatch.setattr(render, "capture_model_thumbnail", lambda **kwargs: _png())

    with pytest.raises(ValueError, match="parent.*symlink|symlink.*parent"):
        render.render_candidates(render_export, linked_parent / "candidates")

    assert sentinel.read_bytes() == b"keep"
    assert not (outside / "candidates").exists()


def test_render_candidate_parent_swap_is_detected_and_installed_output_is_removed(
    render_export: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    parent = tmp_path / "candidate-parent"
    parent.mkdir()
    destination = parent / "candidates"
    displaced = tmp_path / "candidate-parent-displaced"
    monkeypatch.setattr(render, "capture_model_thumbnail", lambda **kwargs: _png())
    real_replace = render.os.replace
    swapped = False

    def swap_parent_then_replace(source: object, target: object, *args: object, **kwargs: object) -> None:
        nonlocal swapped
        target_name = Path(target).name if isinstance(target, (str, os.PathLike)) else ""
        if not swapped and target_name == "candidates":
            parent.rename(displaced)
            parent.mkdir()
            (parent / "attacker-sentinel").write_bytes(b"keep")
            swapped = True
        real_replace(source, target, *args, **kwargs)

    monkeypatch.setattr(render.os, "replace", swap_parent_then_replace)

    with pytest.raises(ValueError, match="parent.*changed|changed.*parent"):
        render.render_candidates(render_export, destination)

    assert (parent / "attacker-sentinel").read_bytes() == b"keep"
    assert {path.name for path in parent.iterdir()} == {"attacker-sentinel"}
    assert not (displaced / "candidates").exists()
    assert not list(displaced.glob(".candidates.stage-*"))


def test_render_candidate_postinstall_validation_failure_rolls_back_output(
    render_export: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    destination = tmp_path / "candidates"
    monkeypatch.setattr(render, "capture_model_thumbnail", lambda **kwargs: _png())
    calls = 0

    def fail_postinstall(*args: object, **kwargs: object):
        del args, kwargs
        nonlocal calls
        calls += 1
        if calls == 2:
            raise ValueError("injected candidate postinstall validation failure")

    monkeypatch.setattr(render, "_post_write_thumbnail_check", fail_postinstall, raising=False)

    with pytest.raises(ValueError, match="postinstall"):
        render.render_candidates(render_export, destination)

    assert not destination.exists()
    assert not list(tmp_path.glob(".candidates.stage-*"))


def test_candidate_reconciles_completed_install_rename_before_interrupt_and_removes_output(
    render_export: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    destination = tmp_path / "candidates"
    monkeypatch.setattr(render, "capture_model_thumbnail", lambda **kwargs: _png())
    original_replace = render.os.replace
    interrupted = False

    def interrupt_after_real_install(source: object, target: object, *args: object, **kwargs: object) -> None:
        nonlocal interrupted
        source_name = os.fspath(source)
        target_name = os.fspath(target)
        original_replace(source, target, *args, **kwargs)
        if not interrupted and source_name.startswith(".candidates.stage-") and target_name == "candidates":
            interrupted = True
            raise KeyboardInterrupt("interrupted after candidate install rename")

    monkeypatch.setattr(render.os, "replace", interrupt_after_real_install)

    with pytest.raises(KeyboardInterrupt, match="candidate install rename"):
        render.render_candidates(render_export, destination)

    assert interrupted
    assert not destination.exists()
    assert not list(tmp_path.glob(".candidates.stage-*"))


@pytest.mark.parametrize(
    "cleanup_failure",
    [ValueError("staging cleanup failed"), KeyboardInterrupt("staging cleanup interrupted")],
    ids=["value-error", "interrupt"],
)
def test_candidate_finally_cleanup_failure_does_not_mask_primary_install_error(
    render_export: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    cleanup_failure: BaseException,
) -> None:
    destination = tmp_path / "candidates"
    monkeypatch.setattr(render, "capture_model_thumbnail", lambda **kwargs: _png())
    monkeypatch.setattr(
        render,
        "_post_write_thumbnail_check",
        lambda *args, **kwargs: (_ for _ in ()).throw(RuntimeError("primary staging validation failed")),
        raising=False,
    )
    original_remove = render._remove_held_child_at

    def fail_staging_cleanup(*args: object, **kwargs: object) -> None:
        label = args[3] if len(args) > 3 else kwargs.get("label")
        if label == "candidate staging directory":
            raise cleanup_failure
        original_remove(*args, **kwargs)

    monkeypatch.setattr(render, "_remove_held_child_at", fail_staging_cleanup)

    try:
        render.render_candidates(render_export, destination)
    except BaseException as error:
        caught = error
    else:
        raise AssertionError("candidate rendering unexpectedly succeeded")

    assert isinstance(caught, RuntimeError)
    assert "primary staging validation failed" in str(caught)
    assert "staging cleanup" in "\n".join(getattr(caught, "__notes__", ()))


def test_candidate_descriptor_close_failure_does_not_mask_primary_install_error(
    render_export: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    destination = tmp_path / "candidates"
    monkeypatch.setattr(render, "capture_model_thumbnail", lambda **kwargs: _png())
    monkeypatch.setattr(
        render,
        "_post_write_thumbnail_check",
        lambda *args, **kwargs: (_ for _ in ()).throw(RuntimeError("primary candidate validation failed")),
        raising=False,
    )
    original_create = render._create_child_directory
    original_close = render.os.close
    staging_descriptor: int | None = None
    close_failure_raised = False

    def remember_staging_descriptor(*args: object, **kwargs: object):
        nonlocal staging_descriptor
        child = original_create(*args, **kwargs)
        prefix = args[1] if len(args) > 1 else kwargs.get("prefix")
        if isinstance(prefix, str) and prefix.startswith(".candidates.stage-"):
            staging_descriptor = child.descriptor
        return child

    def fail_after_closing_staging(descriptor: int) -> None:
        nonlocal close_failure_raised
        if descriptor == staging_descriptor and not close_failure_raised:
            close_failure_raised = True
            original_close(descriptor)
            raise ValueError("candidate staging descriptor close failed")
        original_close(descriptor)

    monkeypatch.setattr(render, "_create_child_directory", remember_staging_descriptor)
    monkeypatch.setattr(render.os, "close", fail_after_closing_staging)

    with pytest.raises(RuntimeError, match="primary candidate validation failed") as caught:
        render.render_candidates(render_export, destination)

    assert close_failure_raised
    assert "descriptor close" in "\n".join(getattr(caught.value, "__notes__", ()))


def test_render_cli_prints_success_only_after_all_candidates_and_approval_never_renders(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    candidates = tmp_path / "candidates"

    def failed_render(*args: object, **kwargs: object) -> Path:
        del args, kwargs
        raise RuntimeError("third render failed")

    monkeypatch.setattr(render, "render_candidates", failed_render)
    status = render.main(["render", "--models", str(tmp_path / "models"), "--candidates", str(candidates)])
    captured = capsys.readouterr()
    assert status == 1
    assert "RENDER_OK" not in captured.out
    assert "RENDER_MODEL_OK" not in captured.out
    assert "third render failed" in captured.err

    approved = tmp_path / "approved"
    monkeypatch.setattr(render, "approve_candidates", lambda *args, **kwargs: approved)
    monkeypatch.setattr(render, "render_candidates", lambda *args, **kwargs: (_ for _ in ()).throw(AssertionError))
    status = render.main(["approve", "--candidates", str(candidates)])
    assert status == 0
    assert capsys.readouterr().out == f"APPROVAL_OK: {approved}\n"


def test_render_cli_buffers_per_model_observations_until_complete_success(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    candidates = tmp_path / "candidates"

    def successful_render(*args: object, **kwargs: object) -> Path:
        del args
        observations = kwargs["observations"]
        for index, slug in enumerate(SLUGS):
            observations.append(
                render.RenderObservation(
                    slug=slug,
                    elapsed_seconds=index + 0.125,
                    selected_frame_suffix=index + 2,
                    png_sha256=str(index) * 64,
                )
            )
        return candidates

    monkeypatch.setattr(render, "render_candidates", successful_render)

    assert render.main(["render", "--models", str(tmp_path / "models")]) == 0

    assert capsys.readouterr().out.splitlines() == [
        f"RENDER_MODEL_OK: {SLUGS[0]} elapsed=0.125s selected_frame=2 sha256={'0' * 64}",
        f"RENDER_MODEL_OK: {SLUGS[1]} elapsed=1.125s selected_frame=3 sha256={'1' * 64}",
        f"RENDER_MODEL_OK: {SLUGS[2]} elapsed=2.125s selected_frame=4 sha256={'2' * 64}",
        f"RENDER_OK: {candidates}",
    ]


def test_render_shell_wrapper_is_cwd_independent_and_has_no_false_success_footer(tmp_path: Path) -> None:
    script = Path(__file__).resolve().parents[1] / "scripts/render_thumbnails.sh"
    text = script.read_text(encoding="utf-8")

    assert text.startswith("#!/usr/bin/env bash\n")
    assert "set -euo pipefail" in text
    assert "BASH_SOURCE[0]" in text
    assert os.access(script, os.X_OK)
    assert ".venv/bin/python" in text
    assert "env -u PYTHONPATH" in text
    assert "RENDER_OK" not in text
    assert "APPROVAL_OK" not in text

    result = subprocess.run(
        [script, "--help"],
        cwd=tmp_path,
        env={**os.environ, "PYTHONPATH": "/attacker"},
        check=False,
        capture_output=True,
        text=True,
        timeout=10,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "{render,approve}" in result.stdout
