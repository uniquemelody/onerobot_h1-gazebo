from __future__ import annotations

import importlib.util
import os
import shutil
from pathlib import Path, PurePosixPath
from types import SimpleNamespace
from urllib.parse import quote
from xml.etree import ElementTree

import pytest
from onerobotics_a1_gazebo import cache, demo
from onerobotics_a1_gazebo.package import export_models
from onerobotics_a1_gazebo.spec import load_model_specs
from onerobotics_a1_gazebo.validation_snapshot import DirectorySnapshot


def test_cache_validation_module_is_available() -> None:
    assert importlib.util.find_spec("onerobotics_a1_gazebo.cache") is not None


@pytest.fixture(scope="module")
def trusted_export(tmp_path_factory: pytest.TempPathFactory) -> Path:
    root = tmp_path_factory.mktemp("trusted-cache-export") / "models"
    export_models(root)
    return root


@pytest.fixture(scope="module")
def trusted_snapshot(trusted_export: Path) -> DirectorySnapshot:
    return demo._validated_export_snapshot(trusted_export)


@pytest.fixture()
def cache_models(trusted_export: Path, tmp_path: Path) -> Path:
    root = tmp_path / "cache models"
    root.mkdir()
    for spec in load_model_specs():
        shutil.copytree(trusted_export / spec.slug, root / spec.slug)
    return root


def _identity_harmonic_runner(calls: list[dict[str, object]]):
    def run(**kwargs: object) -> SimpleNamespace:
        calls.append(dict(kwargs))
        command = kwargs["command"]
        assert isinstance(command, list)
        stdout = ""
        if command[:4] == ["sdf", "--force-version", "14", "-p"]:
            stdout = Path(command[-1]).read_text(encoding="utf-8")
        elif command[:4] == ["fuel", "--force-version", "9", "meta"]:
            stdout = "<model><name>native metadata</name></model>\n"
        return SimpleNamespace(returncode=0, stdout=stdout, stderr="")

    return run


def _validate(
    monkeypatch: pytest.MonkeyPatch,
    trusted_export: Path,
    trusted_snapshot: DirectorySnapshot,
    cache_models: Path,
) -> tuple[cache.ValidatedCacheSnapshots, list[dict[str, object]]]:
    calls: list[dict[str, object]] = []
    monkeypatch.setattr(cache, "_capture_trusted_export", lambda path: trusted_snapshot)
    monkeypatch.setattr(cache, "_run_harmonic_command", _identity_harmonic_runner(calls))
    result = cache.validate_cache_models(
        trusted_models=trusted_export,
        cache_models=cache_models,
    )
    return result, calls


def _write_xml(path: Path, root: ElementTree.Element) -> None:
    ElementTree.indent(root, space="    ")
    path.write_bytes(ElementTree.tostring(root, encoding="utf-8", xml_declaration=True))


def _right_sdf(cache_models: Path) -> Path:
    return cache_models / "onerobotics_a1_right_arm" / "model.sdf"


def test_exact_cache_is_captured_and_native_checked(
    trusted_export: Path,
    trusted_snapshot: DirectorySnapshot,
    cache_models: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    result, calls = _validate(monkeypatch, trusted_export, trusted_snapshot, cache_models)

    expected_slugs = {spec.slug for spec in load_model_specs()}
    assert result.trusted_export is trusted_snapshot
    assert {path.parts[0] for path in result.normalized_models.directories if len(path.parts) == 1} == expected_slugs
    assert len(calls) == 15
    commands = [call["command"] for call in calls]
    assert sum(command[:4] == ["sdf", "--force-version", "14", "-k"] for command in commands) == 6
    assert sum(command[:4] == ["sdf", "--force-version", "14", "-p"] for command in commands) == 6
    assert sum(command[:4] == ["fuel", "--force-version", "9", "meta"] for command in commands) == 3
    assert all(Path(call["private_export"]).is_absolute() for call in calls)
    assert all(str(cache_models) not in " ".join(call["command"]) for call in calls)
    assert all(call["output_limit_bytes"] == cache._MAX_NATIVE_OUTPUT for call in calls)
    assert len({call["deadline"] for call in calls}) == 1


def test_trusted_export_still_uses_exact_prepublication_validator(
    trusted_export: Path,
    cache_models: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    exact_calls: list[Path] = []
    real = demo._validated_export_snapshot

    def exact(path: Path) -> DirectorySnapshot:
        exact_calls.append(Path(path))
        return real(path)

    monkeypatch.setattr(cache, "_capture_trusted_export", exact)
    monkeypatch.setattr(cache, "_run_harmonic_command", _identity_harmonic_runner([]))

    result = cache.validate_cache_models(trusted_models=trusted_export, cache_models=cache_models)

    assert exact_calls == [trusted_export]
    assert result.trusted_export.files


@pytest.mark.parametrize("change", ["missing", "extra_file", "extra_directory"])
def test_cache_root_must_contain_exactly_three_model_directories(
    trusted_export: Path,
    trusted_snapshot: DirectorySnapshot,
    cache_models: Path,
    monkeypatch: pytest.MonkeyPatch,
    change: str,
) -> None:
    if change == "missing":
        shutil.rmtree(cache_models / "onerobotics_a1_left_arm")
    elif change == "extra_file":
        (cache_models / "unexpected.txt").write_text("unexpected", encoding="utf-8")
    else:
        (cache_models / "unexpected").mkdir()

    with pytest.raises(ValueError, match="cache root inventory"):
        _validate(monkeypatch, trusted_export, trusted_snapshot, cache_models)


@pytest.mark.parametrize("change", ["missing", "extra"])
def test_each_cache_package_requires_exact_trusted_inventory(
    trusted_export: Path,
    trusted_snapshot: DirectorySnapshot,
    cache_models: Path,
    monkeypatch: pytest.MonkeyPatch,
    change: str,
) -> None:
    package = cache_models / "onerobotics_a1_right_arm"
    if change == "missing":
        (package / "NOTICE").unlink()
    else:
        (package / "unexpected.txt").write_text("unexpected", encoding="utf-8")

    with pytest.raises(ValueError, match="package inventory"):
        _validate(monkeypatch, trusted_export, trusted_snapshot, cache_models)


@pytest.mark.parametrize("entry_kind", ["symlink", "fifo"])
def test_cache_capture_rejects_links_and_special_files(
    trusted_export: Path,
    trusted_snapshot: DirectorySnapshot,
    cache_models: Path,
    monkeypatch: pytest.MonkeyPatch,
    entry_kind: str,
) -> None:
    entry = cache_models / "onerobotics_a1_right_arm" / "NOTICE"
    entry.unlink()
    if entry_kind == "symlink":
        entry.symlink_to(trusted_export / "onerobotics_a1_right_arm" / "NOTICE")
    else:
        os.mkfifo(entry)

    with pytest.raises(ValueError, match="cache snapshot"):
        _validate(monkeypatch, trusted_export, trusted_snapshot, cache_models)


@pytest.mark.parametrize(
    "relative",
    [
        "metadata.pbtxt",
        "LICENSE",
        "NOTICE",
        "SOURCE_MANIFEST.json",
        "thumbnails/0.png",
        "meshes/base_link.STL",
    ],
)
def test_every_non_xml_cache_byte_must_match_the_trusted_package(
    trusted_export: Path,
    trusted_snapshot: DirectorySnapshot,
    cache_models: Path,
    monkeypatch: pytest.MonkeyPatch,
    relative: str,
) -> None:
    path = cache_models / "onerobotics_a1_right_arm" / relative
    if not path.exists() and relative.startswith("meshes/"):
        path = next((cache_models / "onerobotics_a1_right_arm" / "meshes").iterdir())
    path.write_bytes(path.read_bytes() + b"changed")

    with pytest.raises(ValueError, match="byte mismatch"):
        _validate(monkeypatch, trusted_export, trusted_snapshot, cache_models)


def test_model_config_allows_only_complete_xml_semantic_equivalence(
    trusted_export: Path,
    trusted_snapshot: DirectorySnapshot,
    cache_models: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    config = cache_models / "onerobotics_a1_right_arm" / "model.config"
    root = ElementTree.parse(config).getroot()
    _write_xml(config, root)

    _validate(monkeypatch, trusted_export, trusted_snapshot, cache_models)

    root.find("name").text = "Different model"
    _write_xml(config, root)
    with pytest.raises(ValueError, match="model.config semantic mismatch"):
        _validate(monkeypatch, trusted_export, trusted_snapshot, cache_models)


def test_xml_declarations_are_rejected_even_after_a_long_prefix(
    trusted_export: Path,
    trusted_snapshot: DirectorySnapshot,
    cache_models: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    config = cache_models / "onerobotics_a1_right_arm" / "model.config"
    root = ElementTree.parse(config).getroot()
    serialized = ElementTree.tostring(root, encoding="utf-8")
    config.write_bytes(b'<?xml version="1.0"?>\n' + (b" " * 5000) + b"<!DOCTYPE model>\n" + serialized)
    native_calls: list[dict[str, object]] = []
    monkeypatch.setattr(cache, "_capture_trusted_export", lambda path: trusted_snapshot)
    monkeypatch.setattr(cache, "_run_harmonic_command", _identity_harmonic_runner(native_calls))

    with pytest.raises(ValueError, match="document type or entity"):
        cache.validate_cache_models(trusted_models=trusted_export, cache_models=cache_models)

    assert native_calls == []


def test_utf16_doctype_is_rejected_before_native_parsing(
    trusted_export: Path,
    trusted_snapshot: DirectorySnapshot,
    cache_models: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    config = cache_models / "onerobotics_a1_right_arm" / "model.config"
    root = ElementTree.parse(config).getroot()
    body = ElementTree.tostring(root, encoding="unicode")
    config.write_bytes(
        (f'<?xml version="1.0" encoding="utf-16"?>\n<!DOCTYPE model SYSTEM "file:///etc/passwd">\n{body}').encode(
            "utf-16"
        )
    )
    native_calls: list[dict[str, object]] = []
    monkeypatch.setattr(cache, "_capture_trusted_export", lambda path: trusted_snapshot)
    monkeypatch.setattr(cache, "_run_harmonic_command", _identity_harmonic_runner(native_calls))

    with pytest.raises(ValueError, match="UTF-8"):
        cache.validate_cache_models(trusted_models=trusted_export, cache_models=cache_models)

    assert native_calls == []


def test_ignored_model_config_markup_is_not_forwarded_to_gazebo(
    trusted_export: Path,
    trusted_snapshot: DirectorySnapshot,
    cache_models: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    config = cache_models / "onerobotics_a1_right_arm" / "model.config"
    original = config.read_text(encoding="utf-8")
    body = original.split("\n", 1)[1]
    config.write_text(
        '<?xml version="1.0"?>\n<?xml-stylesheet href="file:///etc/passwd"?>\n'
        + body.replace("<model>", "<model><!-- cache-only ignored markup -->", 1),
        encoding="utf-8",
    )

    result, _ = _validate(monkeypatch, trusted_export, trusted_snapshot, cache_models)
    normalized = result.normalized_models.files[PurePosixPath("onerobotics_a1_right_arm/model.config")]

    assert b"xml-stylesheet" not in normalized
    assert b"cache-only ignored markup" not in normalized


@pytest.mark.parametrize(
    ("injected", "message"),
    [
        ("<extra/>" * 20001, "element limit"),
        ("<extra " + " ".join(f'a{index}="x"' for index in range(20001)) + "/>", "attribute limit"),
    ],
    ids=("elements", "attributes"),
)
def test_xml_width_limits_reject_untrusted_trees_before_native_parsing(
    trusted_export: Path,
    trusted_snapshot: DirectorySnapshot,
    cache_models: Path,
    monkeypatch: pytest.MonkeyPatch,
    injected: str,
    message: str,
) -> None:
    config = cache_models / "onerobotics_a1_right_arm" / "model.config"
    document = config.read_text(encoding="utf-8").replace("</model>", f"{injected}</model>")
    config.write_text(document, encoding="utf-8")
    native_calls: list[dict[str, object]] = []
    monkeypatch.setattr(cache, "_capture_trusted_export", lambda path: trusted_snapshot)
    monkeypatch.setattr(cache, "_run_harmonic_command", _identity_harmonic_runner(native_calls))

    with pytest.raises(ValueError, match=message):
        cache.validate_cache_models(trusted_models=trusted_export, cache_models=cache_models)

    assert native_calls == []


def test_xml_namespace_declarations_count_toward_the_attribute_limit(
    trusted_export: Path,
    trusted_snapshot: DirectorySnapshot,
    cache_models: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    config = cache_models / "onerobotics_a1_right_arm" / "model.config"
    declarations = " ".join(f'xmlns:n{index}="urn:n{index}"' for index in range(20_001))
    document = config.read_text(encoding="utf-8").replace("<model>", f"<model {declarations}>", 1)
    config.write_text(document, encoding="utf-8")
    native_calls: list[dict[str, object]] = []
    monkeypatch.setattr(cache, "_capture_trusted_export", lambda path: trusted_snapshot)
    monkeypatch.setattr(cache, "_run_harmonic_command", _identity_harmonic_runner(native_calls))

    with pytest.raises(ValueError, match="attribute limit"):
        cache.validate_cache_models(trusted_models=trusted_export, cache_models=cache_models)

    assert native_calls == []


@pytest.mark.parametrize("mutation", ["content_whitespace", "tail_text"])
def test_model_config_comparison_preserves_all_non_formatting_text(
    trusted_export: Path,
    trusted_snapshot: DirectorySnapshot,
    cache_models: Path,
    monkeypatch: pytest.MonkeyPatch,
    mutation: str,
) -> None:
    config = cache_models / "onerobotics_a1_right_arm" / "model.config"
    root = ElementTree.parse(config).getroot()
    name = root.find("name")
    assert name is not None
    if mutation == "content_whitespace":
        name.text = (name.text or "").replace("OneRobotics A1", "OneRobotics  A1")
    else:
        name.tail = "unexpected character data"
    _write_xml(config, root)

    with pytest.raises(ValueError, match="model.config semantic mismatch"):
        _validate(monkeypatch, trusted_export, trusted_snapshot, cache_models)


def test_safe_absolute_file_mesh_uris_are_normalized_in_memory(
    trusted_export: Path,
    trusted_snapshot: DirectorySnapshot,
    cache_models: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    model_dir = cache_models / "onerobotics_a1_right_arm"
    sdf = model_dir / "model.sdf"
    root = ElementTree.parse(sdf).getroot()
    for uri in root.findall(".//geometry/mesh/uri"):
        name = PurePosixPath(uri.text or "").name
        uri.text = f"file://{model_dir.resolve().as_posix()}/meshes/{name}"
    _write_xml(sdf, root)

    result, _ = _validate(monkeypatch, trusted_export, trusted_snapshot, cache_models)
    normalized = result.normalized_models.files[PurePosixPath("onerobotics_a1_right_arm/model.sdf")]
    normalized_root = ElementTree.fromstring(normalized)

    assert all((uri.text or "").startswith("meshes/") for uri in normalized_root.findall(".//geometry/mesh/uri"))
    assert all(str(cache_models) not in (uri.text or "") for uri in normalized_root.findall(".//uri"))


def test_strict_versioned_fuel_https_mesh_uris_are_normalized_in_memory(
    trusted_export: Path,
    trusted_snapshot: DirectorySnapshot,
    cache_models: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    sdf = _right_sdf(cache_models)
    root = ElementTree.parse(sdf).getroot()
    for uri in root.findall(".//geometry/mesh/uri"):
        name = PurePosixPath(uri.text or "").name
        uri.text = (
            f"https://fuel.gazebosim.org/1.0/onerobotics/models/OneRobotics%20A1%20Right%20Arm/3/files/meshes/{name}"
        )
    _write_xml(sdf, root)

    result, _ = _validate(monkeypatch, trusted_export, trusted_snapshot, cache_models)
    normalized = result.normalized_models.files[PurePosixPath("onerobotics_a1_right_arm/model.sdf")]
    normalized_root = ElementTree.fromstring(normalized)

    assert all((uri.text or "").startswith("meshes/") for uri in normalized_root.findall(".//geometry/mesh/uri"))
    assert b"https://" not in normalized


def test_safe_rewritten_cache_passes_the_real_pinned_native_parsers(
    trusted_export: Path,
    cache_models: Path,
) -> None:
    for spec in load_model_specs():
        model_dir = cache_models / spec.slug
        sdf = model_dir / "model.sdf"
        root = ElementTree.parse(sdf).getroot()
        for uri in root.findall(".//geometry/mesh/uri"):
            name = PurePosixPath(uri.text or "").name
            uri.text = f"file://{model_dir.resolve().as_posix()}/meshes/{name}"
        _write_xml(sdf, root)

    result = cache.validate_cache_models(trusted_models=trusted_export, cache_models=cache_models)

    assert result.normalized_models.files


def test_versioned_fuel_https_cache_passes_the_real_pinned_native_parsers(
    trusted_export: Path,
    cache_models: Path,
) -> None:
    for spec in load_model_specs():
        sdf = cache_models / spec.slug / "model.sdf"
        root = ElementTree.parse(sdf).getroot()
        encoded_name = quote(spec.display_name, safe="-._~")
        for uri in root.findall(".//geometry/mesh/uri"):
            name = PurePosixPath(uri.text or "").name
            uri.text = f"https://fuel.gazebosim.org/1.0/onerobotics/models/{encoded_name}/3/files/meshes/{name}"
        _write_xml(sdf, root)

    result = cache.validate_cache_models(trusted_models=trusted_export, cache_models=cache_models)

    assert result.normalized_models.files


@pytest.mark.parametrize(
    "attack",
    [
        "https://fuel.gazebosim.org/files/meshes/{name}",
        "https://evil.example/1.0/onerobotics/models/OneRobotics%20A1%20Right%20Arm/3/files/meshes/{name}",
        "https://user@fuel.gazebosim.org/1.0/onerobotics/models/OneRobotics%20A1%20Right%20Arm/3/files/meshes/{name}",
        "https://fuel.gazebosim.org:443/1.0/onerobotics/models/OneRobotics%20A1%20Right%20Arm/3/files/meshes/{name}",
        "https://fuel.gazebosim.org/1.0/./models/OneRobotics%20A1%20Right%20Arm/3/files/meshes/{name}",
        "https://fuel.gazebosim.org/1.0/../models/OneRobotics%20A1%20Right%20Arm/3/files/meshes/{name}",
        "https://fuel.gazebosim.org/1.0/onerobotics/models/OneRobotics%20A1%20Right%20Arm/0/files/meshes/{name}",
        "https://fuel.gazebosim.org/1.0/onerobotics/models/%2e%2e/3/files/meshes/{name}",
        "https://fuel.gazebosim.org/1.0/onerobotics/models/OneRobotics%2FA1%20Right%20Arm/3/files/meshes/{name}",
        "meshes/../{name}",
        "meshes/%2e%2e/{name}",
        "file:///safe/meshes/{name}?download=1",
        "file:///safe/meshes/{name}#fragment",
        "file://remote-host/safe/meshes/{name}",
        "meshes\\{name}",
        " meshes/{name}",
        "file:///safe/./meshes/{name}",
        "file:///safe//meshes/{name}",
        "meshes/not-a-trusted-mesh.STL",
    ],
)
def test_mesh_uri_attacks_are_rejected_before_native_parsing(
    trusted_export: Path,
    trusted_snapshot: DirectorySnapshot,
    cache_models: Path,
    monkeypatch: pytest.MonkeyPatch,
    attack: str,
) -> None:
    sdf = _right_sdf(cache_models)
    root = ElementTree.parse(sdf).getroot()
    uri = root.find(".//geometry/mesh/uri")
    assert uri is not None
    name = PurePosixPath(uri.text or "").name
    uri.text = attack.format(name=name)
    _write_xml(sdf, root)
    native_calls: list[dict[str, object]] = []
    monkeypatch.setattr(cache, "_capture_trusted_export", lambda path: trusted_snapshot)
    monkeypatch.setattr(cache, "_run_harmonic_command", _identity_harmonic_runner(native_calls))

    with pytest.raises(ValueError, match="mesh URI"):
        cache.validate_cache_models(trusted_models=trusted_export, cache_models=cache_models)

    assert native_calls == []


def test_duplicate_uri_target_cannot_replace_an_expected_mesh_occurrence(
    trusted_export: Path,
    trusted_snapshot: DirectorySnapshot,
    cache_models: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    sdf = _right_sdf(cache_models)
    root = ElementTree.parse(sdf).getroot()
    uris = root.findall(".//geometry/mesh/uri")
    first_name = PurePosixPath(uris[0].text or "").name
    replacement = next(
        PurePosixPath(uri.text or "").name for uri in uris if PurePosixPath(uri.text or "").name != first_name
    )
    uris[0].text = f"meshes/{replacement}"
    _write_xml(sdf, root)

    with pytest.raises(ValueError, match="mesh URI.*map"):
        _validate(monkeypatch, trusted_export, trusted_snapshot, cache_models)


@pytest.mark.parametrize(
    ("selector", "value"),
    [
        (".//link/pose", "1 0 0 0 0 0"),
        (".//inertial/inertia/ixx", "999"),
        (".//joint/axis/xyz", "0 1 0"),
        (".//joint/child", "Link7"),
        (".//joint/axis/limit/upper", "0.001"),
    ],
)
def test_native_complete_tree_comparison_rejects_physical_mutations(
    trusted_export: Path,
    trusted_snapshot: DirectorySnapshot,
    cache_models: Path,
    monkeypatch: pytest.MonkeyPatch,
    selector: str,
    value: str,
) -> None:
    sdf = _right_sdf(cache_models)
    root = ElementTree.parse(sdf).getroot()
    element = root.find(selector)
    assert element is not None
    element.text = value
    _write_xml(sdf, root)

    with pytest.raises(ValueError, match="native parsed SDF semantic mismatch"):
        _validate(monkeypatch, trusted_export, trusted_snapshot, cache_models)


@pytest.mark.parametrize("tag", ["plugin", "sensor"])
def test_native_complete_tree_comparison_rejects_injected_runtime_elements(
    trusted_export: Path,
    trusted_snapshot: DirectorySnapshot,
    cache_models: Path,
    monkeypatch: pytest.MonkeyPatch,
    tag: str,
) -> None:
    sdf = _right_sdf(cache_models)
    root = ElementTree.parse(sdf).getroot()
    link = root.find("model/link")
    assert link is not None
    ElementTree.SubElement(link, tag, {"name": "injected"})
    _write_xml(sdf, root)

    with pytest.raises(ValueError, match="native parsed SDF semantic mismatch"):
        _validate(monkeypatch, trusted_export, trusted_snapshot, cache_models)


def test_cache_path_is_captured_once_and_never_read_after_validation_starts(
    trusted_export: Path,
    trusted_snapshot: DirectorySnapshot,
    cache_models: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    real_capture = cache.capture_directory
    cache_captures = 0

    def capture_then_replace(path: Path) -> DirectorySnapshot:
        nonlocal cache_captures
        snapshot = real_capture(path)
        if Path(path) == cache_models:
            cache_captures += 1
            _right_sdf(cache_models).write_text("<sdf>live path attack</sdf>", encoding="utf-8")
        return snapshot

    monkeypatch.setattr(cache, "capture_directory", capture_then_replace)
    result, _ = _validate(monkeypatch, trusted_export, trusted_snapshot, cache_models)

    assert cache_captures == 1
    captured = result.normalized_models.files[PurePosixPath("onerobotics_a1_right_arm/model.sdf")]
    assert b"live path attack" not in captured


def test_native_command_failure_rejects_the_whole_cache(
    trusted_export: Path,
    trusted_snapshot: DirectorySnapshot,
    cache_models: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls = 0

    def fail_first(**kwargs: object) -> SimpleNamespace:
        nonlocal calls
        calls += 1
        return SimpleNamespace(returncode=23, stdout="", stderr="native failure")

    monkeypatch.setattr(cache, "_capture_trusted_export", lambda path: trusted_snapshot)
    monkeypatch.setattr(cache, "_run_harmonic_command", fail_first)

    with pytest.raises(RuntimeError, match="native.*failed.*23"):
        cache.validate_cache_models(trusted_models=trusted_export, cache_models=cache_models)

    assert calls == 1


def test_native_result_rejects_large_success_stderr(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(cache, "_MAX_NATIVE_OUTPUT", 32)
    result = SimpleNamespace(returncode=0, stdout="ok", stderr="x" * 31)

    with pytest.raises(RuntimeError, match="output exceeds"):
        cache._require_native(result, "test command")
