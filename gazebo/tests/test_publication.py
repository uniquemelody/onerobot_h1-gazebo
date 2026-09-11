from __future__ import annotations

import hashlib
import json
import mmap
import os
import resource
import stat
import zlib
from pathlib import Path, PurePosixPath
from zipfile import ZIP_DEFLATED, ZipFile, ZipInfo

import pytest
from onerobotics_a1_gazebo import publication
from onerobotics_a1_gazebo import validation_snapshot as snapshot_module
from onerobotics_a1_gazebo.publication import (
    PublicationError,
    prepare_upload,
    verify_api_response,
    verify_license_response,
    verify_raw_zip,
    write_manifest,
)


def _model(root: Path) -> Path:
    model = root / "model"
    (model / "meshes").mkdir(parents=True)
    (model / "thumbnails").mkdir()
    (model / "model.sdf").write_text('<sdf version="1.11"/>\n', encoding="utf-8")
    (model / "metadata.pbtxt").write_text('name: "OneRobotics A1 Right Arm"\n', encoding="utf-8")
    (model / "meshes" / "Link1.STL").write_bytes(b"mesh-bytes")
    (model / "thumbnails" / "0.png").write_bytes(b"png-bytes")
    return model


def _manifest(tmp_path: Path) -> tuple[Path, Path]:
    model = _model(tmp_path)
    output = tmp_path / "evidence.json"
    write_manifest(model, output)
    return model, output


def _manifest_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _zip_from_model(model: Path, destination: Path, *, explicit_directories: bool = True) -> None:
    with ZipFile(destination, "w", compression=ZIP_DEFLATED) as archive:
        if explicit_directories:
            for directory in ("meshes/", "thumbnails/"):
                archive.writestr(directory, b"")
        for path in sorted(item for item in model.rglob("*") if item.is_file()):
            archive.write(path, path.relative_to(model).as_posix())


def _verify_zip(archive: Path, manifest: Path, output: Path) -> None:
    verify_raw_zip(
        archive,
        manifest,
        output,
        manifest_sha256=_manifest_sha256(manifest),
    )


def test_manifest_is_canonical_and_records_typed_complete_inventory(tmp_path: Path) -> None:
    model, output = _manifest(tmp_path)
    data = output.read_bytes()
    document = json.loads(data)

    assert data.endswith(b"\n")
    assert data == (json.dumps(document, ensure_ascii=False, indent=2, sort_keys=True).encode("utf-8") + b"\n")
    assert document["schema_version"] == 1
    assert document["directories"] == ["meshes", "thumbnails"]
    assert [item["path"] for item in document["files"]] == [
        "meshes/Link1.STL",
        "metadata.pbtxt",
        "model.sdf",
        "thumbnails/0.png",
    ]
    expected = (model / "meshes" / "Link1.STL").read_bytes()
    record = document["files"][0]
    assert record["size"] == len(expected)
    assert record["sha256"] == hashlib.sha256(expected).hexdigest()


@pytest.mark.parametrize("leaf_kind", ["symlink", "fifo"])
def test_manifest_rejects_links_and_special_files(tmp_path: Path, leaf_kind: str) -> None:
    model = _model(tmp_path)
    unsafe = model / "unsafe"
    if leaf_kind == "symlink":
        unsafe.symlink_to(model / "model.sdf")
    else:
        os.mkfifo(unsafe)

    with pytest.raises(PublicationError, match="snapshot"):
        write_manifest(model, tmp_path / "evidence.json")

    assert not (tmp_path / "evidence.json").exists()


@pytest.mark.parametrize("unsafe_name", ["percent%name", "line\nbreak", "control\x1bname"])
def test_manifest_rejects_unsafe_inventory_names(tmp_path: Path, unsafe_name: str) -> None:
    model = _model(tmp_path)
    (model / unsafe_name).write_bytes(b"unsafe")
    output = tmp_path / "evidence.json"

    with pytest.raises(PublicationError, match="unsafe inventory path"):
        write_manifest(model, output)

    assert not output.exists()


def test_manifest_refuses_a_symlink_or_existing_output(tmp_path: Path) -> None:
    model = _model(tmp_path)
    target = tmp_path / "target"
    target.write_text("keep", encoding="utf-8")
    output = tmp_path / "evidence.json"
    output.symlink_to(target)

    with pytest.raises(PublicationError, match="output"):
        write_manifest(model, output)

    assert target.read_text(encoding="utf-8") == "keep"


def test_manifest_must_be_written_outside_the_model_tree(tmp_path: Path) -> None:
    model = _model(tmp_path)
    output = model / "evidence.json"

    with pytest.raises(PublicationError, match="outside.*model"):
        write_manifest(model, output)

    assert not output.exists()


def test_manifest_output_cannot_escape_the_model_ancestry_check_via_rename(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    model = _model(tmp_path)
    outside = tmp_path / "outside"
    outside.mkdir(mode=0o700)
    nested = model / "evidence"
    output = outside / "manifest.json"
    real_check = publication._descriptor_is_within
    checked = False

    def check_while_moved(descriptor: int, ancestor: tuple[int, int]) -> bool:
        nonlocal checked
        checked = True
        outside.rename(nested)
        try:
            return real_check(descriptor, ancestor)
        finally:
            nested.rename(outside)

    monkeypatch.setattr(publication, "_descriptor_is_within", check_while_moved)

    with pytest.raises(PublicationError, match="outside.*model"):
        write_manifest(model, output)

    assert checked
    assert not output.exists()


@pytest.mark.parametrize("unsafe_level", ["parent", "ancestor"])
def test_manifest_rejects_an_output_parent_reachable_through_untrusted_permissions(
    tmp_path: Path,
    unsafe_level: str,
) -> None:
    model = _model(tmp_path)
    unsafe = tmp_path / "untrusted"
    unsafe.mkdir(mode=0o700)
    unsafe.chmod(0o777)
    if unsafe_level == "parent":
        parent = unsafe
    else:
        parent = unsafe / "private-child"
        parent.mkdir(mode=0o700)
    output = parent / "manifest.json"

    with pytest.raises(PublicationError, match="trusted|private|permissions|writable"):
        write_manifest(model, output)

    assert not output.exists()


def test_manifest_rejects_a_symlink_in_the_output_parent_ancestry(tmp_path: Path) -> None:
    model = _model(tmp_path)
    actual = tmp_path / "actual"
    private = actual / "private"
    private.mkdir(parents=True, mode=0o700)
    alias = tmp_path / "alias"
    alias.symlink_to(actual, target_is_directory=True)
    output = alias / "private" / "manifest.json"

    with pytest.raises(PublicationError, match="trusted|symlink|parent"):
        write_manifest(model, output)

    assert not output.exists()


def test_manifest_rejects_an_output_parent_not_owned_by_the_effective_user(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    model = _model(tmp_path)
    output = tmp_path / "manifest.json"
    real_euid = os.geteuid()
    monkeypatch.setattr(publication.os, "geteuid", lambda: real_euid + 1)

    with pytest.raises(PublicationError, match="trusted|owner|owned"):
        write_manifest(model, output)

    assert not output.exists()


def test_trusted_output_parent_descriptor_detects_a_parent_rename(tmp_path: Path) -> None:
    parent_path = tmp_path / "private-parent"
    parent_path.mkdir(mode=0o700)
    moved = tmp_path / "moved-parent"
    held = publication._open_parent(parent_path / "manifest.json")
    parent_path.rename(moved)
    try:
        with pytest.raises(PublicationError, match="parent changed"):
            publication._verify_parent(held)
    finally:
        os.close(held.descriptor)
        moved.rename(parent_path)


def test_manifest_accepts_an_owner_only_mktemp_style_output_parent(tmp_path: Path) -> None:
    model = _model(tmp_path)
    private_parent = tmp_path / "tmp.random-suffix"
    private_parent.mkdir(mode=0o700)
    output = private_parent / "manifest.json"

    write_manifest(model, output)

    assert stat.S_IMODE(private_parent.stat().st_mode) == 0o700
    assert output.is_file()


def test_stable_input_read_detects_same_inode_same_size_mutation(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source = tmp_path / "input.json"
    source.write_bytes(b"abcdef")
    real_read = publication.os.read
    mutated = False

    def mutate_after_read(descriptor: int, size: int) -> bytes:
        nonlocal mutated
        data = real_read(descriptor, size)
        if data and not mutated:
            mutated = True
            source.write_bytes(b"ABCDEF")
            os.utime(source, ns=(1_000_000_000, 1_000_000_000))
        return data

    monkeypatch.setattr(publication.os, "read", mutate_after_read)

    with pytest.raises(PublicationError, match="changed while reading"):
        publication._read_stable_file(source, 1024, "test input")


def test_stable_input_open_uses_nonblocking_mode(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source = tmp_path / "input.json"
    source.write_bytes(b"{}")
    flags_seen: list[int] = []

    def reject_open(path: Path, flags: int, *args: object, **kwargs: object) -> int:
        del path, args, kwargs
        flags_seen.append(flags)
        raise OSError("injected open failure")

    monkeypatch.setattr(publication.os, "open", reject_open)

    with pytest.raises(PublicationError, match="unable to read"):
        publication._read_stable_file(source, 1024, "test input")

    assert flags_seen and flags_seen[0] & os.O_NONBLOCK


def test_atomic_file_detects_a_replaced_staging_name(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    output = tmp_path / "evidence.json"
    real_link = publication._link_descriptor_noreplace

    def replace_then_link(parent: object, held_file: int, destination: str) -> None:
        parent_fd = parent.descriptor
        with os.scandir(parent_fd) as entries:
            source = next(entry.name for entry in entries if entry.name.startswith(".evidence.json."))
        os.unlink(source, dir_fd=parent_fd)
        attacker = os.open(
            source,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_CLOEXEC,
            0o600,
            dir_fd=parent_fd,
        )
        try:
            os.write(attacker, b"attacker bytes")
        finally:
            os.close(attacker)
        real_link(parent, held_file, destination)

    monkeypatch.setattr(publication, "_link_descriptor_noreplace", replace_then_link)

    with pytest.raises(PublicationError, match="changed|staged|held output"):
        publication._atomic_new_file(output, b"approved bytes")

    assert not output.exists()


def test_atomic_file_removes_its_output_when_parent_fsync_fails(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    output = tmp_path / "evidence.json"
    real_fsync = publication.os.fsync
    calls = 0

    def fail_parent_fsync(descriptor: int) -> None:
        nonlocal calls
        calls += 1
        if calls == 2:
            raise OSError("injected parent fsync failure")
        real_fsync(descriptor)

    monkeypatch.setattr(publication.os, "fsync", fail_parent_fsync)

    with pytest.raises(PublicationError, match="write output safely"):
        publication._atomic_new_file(output, b"approved bytes")

    assert not output.exists()


def test_atomic_file_reconciles_a_completed_link_before_interrupt(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    output = tmp_path / "evidence.json"
    real_link = publication._link_descriptor_noreplace

    def interrupt_after_link(parent: object, descriptor: int, destination: str) -> None:
        real_link(parent, descriptor, destination)
        raise KeyboardInterrupt("interrupted after evidence link")

    monkeypatch.setattr(publication, "_link_descriptor_noreplace", interrupt_after_link)

    with pytest.raises(KeyboardInterrupt, match="evidence link"):
        publication._atomic_new_file(output, b"approved bytes")

    assert not output.exists()
    assert not list(tmp_path.glob(".evidence.json.*"))


def test_api_verification_writes_only_the_exact_positive_version(tmp_path: Path) -> None:
    response = tmp_path / "response.json"
    response.write_text(
        json.dumps(
            {
                "owner": "OneRobotics",
                "name": "OneRobotics A1 Right Arm",
                "version": 3,
                "private": True,
                "license_name": "Creative Commons Attribution 4.0 International",
            }
        ),
        encoding="utf-8",
    )
    version = tmp_path / "version"

    result = verify_api_response(
        response,
        owner="OneRobotics",
        name="OneRobotics A1 Right Arm",
        license_name="Creative Commons Attribution 4.0 International",
        visibility="private",
        version_output=version,
    )

    assert result == 3
    assert version.read_bytes() == b"3\n"


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("owner", "Other", "owner"),
        ("name", "Other", "name"),
        ("version", 0, "version"),
        ("version", True, "version"),
        ("private", False, "visibility"),
        ("license_name", "Other", "license"),
    ],
)
def test_api_verification_fails_closed_on_mismatch(
    tmp_path: Path,
    field: str,
    value: object,
    message: str,
) -> None:
    document: dict[str, object] = {
        "owner": "OneRobotics",
        "name": "OneRobotics A1 Right Arm",
        "version": 3,
        "private": True,
        "license_name": "Creative Commons Attribution 4.0 International",
    }
    document[field] = value
    response = tmp_path / "response.json"
    response.write_text(json.dumps(document), encoding="utf-8")
    version = tmp_path / "version"

    with pytest.raises(PublicationError, match=message):
        verify_api_response(
            response,
            owner="OneRobotics",
            name="OneRobotics A1 Right Arm",
            license_name="Creative Commons Attribution 4.0 International",
            visibility="private",
            version_output=version,
        )

    assert not version.exists()


def test_api_verification_wraps_oversized_json_integer_as_publication_error(tmp_path: Path) -> None:
    response = tmp_path / "response.json"
    response.write_text('{"version": ' + "9" * 10000 + "}", encoding="utf-8")

    with pytest.raises(PublicationError, match="malformed"):
        verify_api_response(
            response,
            owner="OneRobotics",
            name="OneRobotics A1 Right Arm",
            license_name="Creative Commons Attribution 4.0 International",
            visibility="private",
            version_output=tmp_path / "version",
        )


def test_license_verification_requires_one_exact_typed_name(tmp_path: Path) -> None:
    response = tmp_path / "licenses.json"
    response.write_text(
        json.dumps(
            [
                {"name": "Creative Commons Zero v1.0 Universal", "url": "https://example.invalid/zero"},
                {
                    "name": "Creative Commons Attribution 4.0 International",
                    "url": "https://example.invalid/by",
                },
            ]
        ),
        encoding="utf-8",
    )

    verify_license_response(response, "Creative Commons Attribution 4.0 International")


@pytest.mark.parametrize(
    "document",
    [
        {"name": "Creative Commons Attribution 4.0 International"},
        [{"description": "Creative Commons Attribution 4.0 International"}],
        [{"name": "prefix Creative Commons Attribution 4.0 International suffix"}],
        [
            {"name": "Creative Commons Attribution 4.0 International"},
            {"name": "Creative Commons Attribution 4.0 International"},
        ],
    ],
)
def test_license_verification_rejects_malformed_substring_or_duplicate_results(
    tmp_path: Path,
    document: object,
) -> None:
    response = tmp_path / "licenses.json"
    response.write_text(json.dumps(document), encoding="utf-8")

    with pytest.raises(PublicationError, match="license"):
        verify_license_response(response, "Creative Commons Attribution 4.0 International")


def test_duplicate_json_key_is_escaped_in_the_error_message() -> None:
    with pytest.raises(PublicationError) as captured:
        publication._parse_json(b'{"line\\nkey": 1, "line\\nkey": 2}', "test JSON")

    assert "line\nkey" not in str(captured.value)
    assert r"line\nkey" in str(captured.value)


def test_cleanup_notes_do_not_require_python_311_add_note() -> None:
    class LegacyError(Exception):
        def __getattribute__(self, name: str) -> object:
            if name == "add_note":
                raise AttributeError(name)
            return super().__getattribute__(name)

    error = LegacyError("primary")

    publication._add_exception_note(error, "cleanup failed")

    assert error.__notes__ == ["cleanup failed"]


def test_cli_escapes_newlines_from_unsafe_snapshot_paths(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    model = _model(tmp_path)
    (model / "unsafe\nlink").symlink_to(model / "model.sdf")

    result = publication.main(
        [
            "manifest",
            "--model",
            str(model),
            "--output",
            str(tmp_path / "manifest.json"),
        ]
    )

    captured = capsys.readouterr()
    assert result == 1
    assert captured.err.count("\n") == 1
    assert r"unsafe\nlink" in captured.err


def test_prepare_upload_cli_requires_the_approved_manifest_sha256(tmp_path: Path) -> None:
    model, manifest = _manifest(tmp_path)

    with pytest.raises(SystemExit):
        publication._parser().parse_args(
            [
                "prepare-upload",
                "--model",
                str(model),
                "--manifest",
                str(manifest),
                "--output",
                str(tmp_path / "upload-stage"),
            ]
        )


@pytest.mark.parametrize(
    "approved_digest",
    [
        "0" * 64,
        "A" * 64,
        "0" * 63,
        "0" * 64 + "\n",
    ],
)
def test_prepare_upload_rejects_a_wrong_or_noncanonical_approved_manifest_digest(
    tmp_path: Path,
    approved_digest: str,
) -> None:
    model, manifest = _manifest(tmp_path)
    output = tmp_path / "upload-stage"

    with pytest.raises(PublicationError, match="manifest.*SHA-256|digest"):
        prepare_upload(
            model,
            manifest,
            output,
            manifest_sha256=approved_digest,
        )

    assert not output.exists()


def test_prepare_upload_parses_the_same_stable_manifest_bytes_bound_by_sha256(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    model, manifest = _manifest(tmp_path)
    approved_digest = _manifest_sha256(manifest)
    real_read = publication._read_stable_file
    manifest_reads = 0

    def replace_name_after_read(path: Path, maximum: int, label: str) -> bytes:
        nonlocal manifest_reads
        data = real_read(path, maximum, label)
        if label == "publication manifest":
            manifest_reads += 1
            manifest.write_bytes(b"{}\n")
        return data

    monkeypatch.setattr(publication, "_read_stable_file", replace_name_after_read)

    prepare_upload(
        model,
        manifest,
        tmp_path / "upload-stage",
        manifest_sha256=approved_digest,
    )

    assert manifest_reads == 1


def test_prepare_upload_materializes_only_the_external_manifest_snapshot_read_only(tmp_path: Path) -> None:
    model, manifest = _manifest(tmp_path)
    output = tmp_path / "upload-stage"

    prepared = prepare_upload(
        model,
        manifest,
        output,
        manifest_sha256=_manifest_sha256(manifest),
    )

    assert prepared.files
    assert not any(path.stat().st_mode & 0o222 for path in (output, *output.rglob("*")))
    assert {path.relative_to(output).as_posix(): path.read_bytes() for path in output.rglob("*") if path.is_file()} == {
        path.relative_to(model).as_posix(): path.read_bytes() for path in model.rglob("*") if path.is_file()
    }


def test_prepare_upload_reconciles_a_completed_install_before_interrupt(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    model, manifest = _manifest(tmp_path)
    output = tmp_path / "upload-stage"
    real_rename = publication._rename_noreplace

    def interrupt_after_rename(parent: object, source: str, destination: str) -> None:
        real_rename(parent, source, destination)
        raise KeyboardInterrupt("interrupted after upload-stage rename")

    monkeypatch.setattr(publication, "_rename_noreplace", interrupt_after_rename)

    with pytest.raises(KeyboardInterrupt, match="upload-stage rename"):
        prepare_upload(
            model,
            manifest,
            output,
            manifest_sha256=_manifest_sha256(manifest),
        )

    assert not output.exists()
    assert not list(tmp_path.glob(".upload-stage.*"))


def test_prepare_upload_rejects_an_untrusted_output_parent(tmp_path: Path) -> None:
    model, manifest = _manifest(tmp_path)
    unsafe_parent = tmp_path / "untrusted"
    unsafe_parent.mkdir(mode=0o700)
    unsafe_parent.chmod(0o777)
    output = unsafe_parent / "upload-stage"

    with pytest.raises(PublicationError, match="trusted|private|permissions|writable"):
        prepare_upload(
            model,
            manifest,
            output,
            manifest_sha256=_manifest_sha256(manifest),
        )

    assert not output.exists()


def test_prepare_upload_rejects_model_drift_before_creating_a_stage(tmp_path: Path) -> None:
    model, manifest = _manifest(tmp_path)
    (model / "model.sdf").write_text('<sdf version="1.10"/>\n', encoding="utf-8")
    output = tmp_path / "upload-stage"

    with pytest.raises(PublicationError, match="manifest|changed|mismatch"):
        prepare_upload(
            model,
            manifest,
            output,
            manifest_sha256=_manifest_sha256(manifest),
        )

    assert not output.exists()


def test_prepare_upload_rejects_a_metadata_invisible_dirty_mmap_change_after_read(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    model, manifest = _manifest(tmp_path)
    source = model / "model.sdf"
    output = tmp_path / "upload-stage"
    real_read = snapshot_module._read_stable_file
    changed = False

    with source.open("r+b") as stream:
        mapping = mmap.mmap(stream.fileno(), 0, access=mmap.ACCESS_WRITE)
        try:
            mapping[0:1] = mapping[0:1]

            def mutate_after_read(
                descriptor: int,
                opened: os.stat_result,
                relative_path: PurePosixPath,
            ) -> bytes:
                nonlocal changed
                data = real_read(descriptor, opened, relative_path)
                if not changed and relative_path == PurePosixPath("model.sdf"):
                    changed = True
                    mapping[0:1] = b"X"
                return data

            monkeypatch.setattr(snapshot_module, "_read_stable_file", mutate_after_read)

            with pytest.raises(PublicationError, match="snapshot|changed"):
                prepare_upload(
                    model,
                    manifest,
                    output,
                    manifest_sha256=_manifest_sha256(manifest),
                )
        finally:
            mapping.close()

    assert changed
    assert source.read_bytes().startswith(b"X")
    assert not output.exists()


def test_verify_zip_cli_requires_the_approved_manifest_sha256(tmp_path: Path) -> None:
    _, manifest = _manifest(tmp_path)

    with pytest.raises(SystemExit):
        publication._parser().parse_args(
            [
                "verify-zip",
                "--zip",
                str(tmp_path / "raw.zip"),
                "--manifest",
                str(manifest),
                "--output",
                str(tmp_path / "extracted"),
            ]
        )


def test_verify_raw_zip_rejects_a_wrong_approved_manifest_digest(tmp_path: Path) -> None:
    model, manifest = _manifest(tmp_path)
    archive = tmp_path / "raw.zip"
    _zip_from_model(model, archive)
    output = tmp_path / "extracted"

    with pytest.raises(PublicationError, match="manifest.*SHA-256|digest"):
        verify_raw_zip(
            archive,
            manifest,
            output,
            manifest_sha256="0" * 64,
        )

    assert not output.exists()


def test_verify_raw_zip_parses_the_same_stable_manifest_bytes_bound_by_sha256(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    model, manifest = _manifest(tmp_path)
    approved_digest = _manifest_sha256(manifest)
    archive = tmp_path / "raw.zip"
    _zip_from_model(model, archive)
    real_read = publication._read_stable_file
    manifest_reads = 0

    def replace_name_after_read(path: Path, maximum: int, label: str) -> bytes:
        nonlocal manifest_reads
        data = real_read(path, maximum, label)
        if label == "publication manifest":
            manifest_reads += 1
            manifest.write_bytes(b"{}\n")
        return data

    monkeypatch.setattr(publication, "_read_stable_file", replace_name_after_read)

    verify_raw_zip(
        archive,
        manifest,
        tmp_path / "extracted",
        manifest_sha256=approved_digest,
    )

    assert manifest_reads == 1


def test_valid_zip_without_explicit_parent_entries_is_verified_and_materialized(tmp_path: Path) -> None:
    model, manifest = _manifest(tmp_path)
    archive = tmp_path / "raw.zip"
    _zip_from_model(model, archive, explicit_directories=False)
    output = tmp_path / "extracted"

    _verify_zip(archive, manifest, output)

    assert {path.relative_to(output).as_posix() for path in output.rglob("*") if path.is_file()} == {
        "meshes/Link1.STL",
        "metadata.pbtxt",
        "model.sdf",
        "thumbnails/0.png",
    }
    assert (output / "meshes" / "Link1.STL").read_bytes() == b"mesh-bytes"


@pytest.mark.parametrize(
    "unsafe_name",
    [
        "/absolute",
        "../escape",
        "meshes/../escape",
        "./model.sdf",
        "meshes\\escape",
        "meshes/%2e%2e/escape",
        "C:/escape",
        "meshes/line\nbreak",
        "meshes/control\x1bescape",
    ],
)
def test_zip_rejects_unsafe_member_paths(tmp_path: Path, unsafe_name: str) -> None:
    _, manifest = _manifest(tmp_path)
    archive = tmp_path / "raw.zip"
    with ZipFile(archive, "w") as raw:
        raw.writestr(unsafe_name, b"bad")

    with pytest.raises(PublicationError, match="path"):
        _verify_zip(archive, manifest, tmp_path / "extracted")

    assert not (tmp_path / "extracted").exists()


def test_manifest_rejects_directory_without_declared_parent_before_reading_zip(tmp_path: Path) -> None:
    _, manifest = _manifest(tmp_path)
    document = json.loads(manifest.read_text(encoding="utf-8"))
    document["directories"].append("orphan/nested")
    document["directories"].sort()
    manifest.write_text(
        json.dumps(document, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    with pytest.raises(PublicationError, match="parent directory"):
        _verify_zip(tmp_path / "does-not-exist.zip", manifest, tmp_path / "extracted")


def test_manifest_rejects_unpaired_surrogate_as_publication_error(tmp_path: Path) -> None:
    _, manifest = _manifest(tmp_path)
    document = json.loads(manifest.read_text(encoding="utf-8"))
    document["directories"].append("\ud800")
    document["directories"].sort()
    manifest.write_text(
        json.dumps(document, ensure_ascii=True, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    with pytest.raises(PublicationError, match="unsafe|malformed"):
        _verify_zip(tmp_path / "does-not-exist.zip", manifest, tmp_path / "extracted")


def test_zip_rejects_directory_entry_with_a_payload(tmp_path: Path) -> None:
    model, manifest = _manifest(tmp_path)
    archive = tmp_path / "raw.zip"
    with ZipFile(archive, "w", compression=ZIP_DEFLATED) as raw:
        raw.writestr("meshes/", b"hidden-data")
        raw.writestr("thumbnails/", b"")
        for path in sorted(item for item in model.rglob("*") if item.is_file()):
            raw.write(path, path.relative_to(model).as_posix())

    with pytest.raises(PublicationError, match="directory.*payload"):
        _verify_zip(archive, manifest, tmp_path / "extracted")

    assert not (tmp_path / "extracted").exists()


def test_zip_validates_each_explicit_directory_local_header(tmp_path: Path) -> None:
    model, manifest = _manifest(tmp_path)
    archive = tmp_path / "raw.zip"
    _zip_from_model(model, archive)
    damaged = bytearray(archive.read_bytes())
    first_local_header = damaged.index(b"PK\x03\x04")
    damaged[first_local_header : first_local_header + 4] = b"BAD!"
    archive.write_bytes(damaged)

    with pytest.raises(PublicationError, match="directory|malformed|header"):
        _verify_zip(archive, manifest, tmp_path / "extracted")

    assert not (tmp_path / "extracted").exists()


def test_zip_rejects_duplicate_members(tmp_path: Path) -> None:
    _, manifest = _manifest(tmp_path)
    archive = tmp_path / "raw.zip"
    with ZipFile(archive, "w") as raw:
        raw.writestr("model.sdf", b"first")
        with pytest.warns(UserWarning, match="Duplicate name"):
            raw.writestr("model.sdf", b"second")

    with pytest.raises(PublicationError, match="duplicate"):
        _verify_zip(archive, manifest, tmp_path / "extracted")


def test_zip_wraps_malformed_member_name_as_publication_error(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    model, manifest = _manifest(tmp_path)
    archive = tmp_path / "raw.zip"
    _zip_from_model(model, archive)

    def malformed_inventory(self: ZipFile) -> list[ZipInfo]:
        del self
        raise UnicodeDecodeError("utf-8", b"\xff", 0, 1, "injected malformed name")

    monkeypatch.setattr(ZipFile, "infolist", malformed_inventory)

    with pytest.raises(PublicationError, match="malformed"):
        _verify_zip(archive, manifest, tmp_path / "extracted")


def test_zip_wraps_decompression_failure_as_publication_error(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    model, manifest = _manifest(tmp_path)
    archive = tmp_path / "raw.zip"
    _zip_from_model(model, archive)

    def broken_member(self: ZipFile, *args: object, **kwargs: object) -> object:
        del self, args, kwargs
        raise zlib.error("injected decompression failure")

    monkeypatch.setattr(ZipFile, "open", broken_member)

    with pytest.raises(PublicationError, match=r"unable to read ZIP (?:member|directory)"):
        _verify_zip(archive, manifest, tmp_path / "extracted")


def test_zip_rejects_symlink_members(tmp_path: Path) -> None:
    _, manifest = _manifest(tmp_path)
    archive = tmp_path / "raw.zip"
    link = ZipInfo("model.sdf")
    link.create_system = 3
    link.external_attr = (stat.S_IFLNK | 0o777) << 16
    with ZipFile(archive, "w") as raw:
        raw.writestr(link, "outside")

    with pytest.raises(PublicationError, match="link|regular"):
        _verify_zip(archive, manifest, tmp_path / "extracted")


def test_zip_rejects_missing_extra_empty_directory_and_mutated_bytes(tmp_path: Path) -> None:
    model, manifest = _manifest(tmp_path)
    archive = tmp_path / "raw.zip"
    _zip_from_model(model, archive)
    with ZipFile(archive, "a") as raw:
        raw.writestr("extra/", b"")

    with pytest.raises(PublicationError, match="inventory"):
        _verify_zip(archive, manifest, tmp_path / "extra-output")

    archive = tmp_path / "mutated.zip"
    _zip_from_model(model, archive)
    with ZipFile(archive, "a") as raw:
        with pytest.warns(UserWarning, match="Duplicate name"):
            raw.writestr("model.sdf", b"mutated")
    with pytest.raises(PublicationError, match="duplicate|hash|size"):
        _verify_zip(archive, manifest, tmp_path / "mutated-output")


def test_zip_refuses_existing_or_symlink_output_without_mutation(tmp_path: Path) -> None:
    model, manifest = _manifest(tmp_path)
    archive = tmp_path / "raw.zip"
    _zip_from_model(model, archive)
    target = tmp_path / "target"
    target.mkdir()
    sentinel = target / "sentinel"
    sentinel.write_text("keep", encoding="utf-8")
    output = tmp_path / "extracted"
    output.symlink_to(target, target_is_directory=True)

    with pytest.raises(PublicationError, match="output"):
        _verify_zip(archive, manifest, output)

    assert sentinel.read_text(encoding="utf-8") == "keep"


def test_zip_cleans_an_empty_staging_directory_when_open_fails(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    model, manifest = _manifest(tmp_path)
    archive = tmp_path / "raw.zip"
    _zip_from_model(model, archive)
    real_open = publication.os.open
    failed = False

    def fail_staging_open(path: object, flags: int, *args: object, **kwargs: object) -> int:
        nonlocal failed
        if not failed and isinstance(path, str) and path.startswith(".extracted.") and flags & os.O_DIRECTORY:
            failed = True
            raise OSError("injected staging open failure")
        return real_open(path, flags, *args, **kwargs)

    monkeypatch.setattr(publication.os, "open", fail_staging_open)

    with pytest.raises(PublicationError, match="materialize"):
        _verify_zip(archive, manifest, tmp_path / "extracted")

    assert failed
    assert not list(tmp_path.glob(".extracted.*"))


def test_zip_materialization_never_follows_a_replaced_intermediate_directory(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    model = _model(tmp_path)
    nested = model / "a" / "b"
    nested.mkdir(parents=True)
    (nested / "payload.bin").write_bytes(b"approved payload")
    manifest = tmp_path / "evidence.json"
    write_manifest(model, manifest)
    archive = tmp_path / "raw.zip"
    _zip_from_model(model, archive)
    external = tmp_path / "external"
    external.mkdir()
    real_mkdir = publication.os.mkdir
    swapped = False

    def swap_parent_then_mkdir(
        path: object,
        mode: int = 0o777,
        *,
        dir_fd: int | None = None,
    ) -> None:
        nonlocal swapped
        path_text = os.fspath(path)
        if not swapped and dir_fd is not None:
            opened = Path(os.readlink(f"/proc/self/fd/{dir_fd}"))
            victim: Path | None = None
            if path_text == "a/b":
                victim = opened / "a"
            elif path_text == "b" and opened.name == "a":
                victim = opened
            if victim is not None:
                victim.rmdir()
                victim.symlink_to(external, target_is_directory=True)
                swapped = True
        real_mkdir(path, mode=mode, dir_fd=dir_fd)

    monkeypatch.setattr(publication.os, "mkdir", swap_parent_then_mkdir)

    with pytest.raises(PublicationError, match="materialize|changed|unsafe"):
        _verify_zip(archive, manifest, tmp_path / "extracted")

    assert swapped
    assert not list(external.iterdir())


def test_materialization_supports_the_entry_limit_under_a_small_fd_budget(tmp_path: Path) -> None:
    original_limit = resource.getrlimit(resource.RLIMIT_NOFILE)
    temporary_limit = min(48, original_limit[0])
    if temporary_limit < 24:
        pytest.skip("process file-descriptor limit is already too small for this regression")
    file_count = temporary_limit + 12
    files = tuple(
        publication.ManifestFile(
            PurePosixPath(f"file-{index:03d}.bin"),
            1,
            hashlib.sha256(b"x").hexdigest(),
        )
        for index in range(file_count)
    )
    manifest = publication.PublicationManifest((), files)
    payloads = {item.path: b"x" for item in files}
    output = tmp_path / "materialized"

    try:
        resource.setrlimit(resource.RLIMIT_NOFILE, (temporary_limit, original_limit[1]))
        publication._materialize_verified(output, manifest, payloads)
    finally:
        resource.setrlimit(resource.RLIMIT_NOFILE, original_limit)

    assert len(list(output.iterdir())) == file_count


def test_public_prepare_upload_supports_many_flat_files_under_a_small_fd_budget(tmp_path: Path) -> None:
    original_limit = resource.getrlimit(resource.RLIMIT_NOFILE)
    temporary_limit = min(48, original_limit[0])
    if temporary_limit < 24:
        pytest.skip("process file-descriptor limit is already too small for this regression")
    model = tmp_path / "many-files-model"
    model.mkdir()
    for index in range(temporary_limit + 12):
        (model / f"file-{index:03d}.bin").write_bytes(b"x")
    manifest = tmp_path / "many-files-manifest.json"
    write_manifest(model, manifest)
    output = tmp_path / "upload-stage"

    try:
        resource.setrlimit(resource.RLIMIT_NOFILE, (temporary_limit, original_limit[1]))
        prepare_upload(
            model,
            manifest,
            output,
            manifest_sha256=_manifest_sha256(manifest),
        )
    finally:
        resource.setrlimit(resource.RLIMIT_NOFILE, original_limit)

    assert len(list(output.iterdir())) == temporary_limit + 12
