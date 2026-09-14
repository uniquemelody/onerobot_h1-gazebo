from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
GUIDE = ROOT / "gazebo" / "README.md"
PUBLISHING = ROOT / "gazebo" / "PUBLISHING.md"
ROOT_README = ROOT / "README.md"
SOURCE_COMMIT = "ecf530911284ba0e559f7a24dc222fd8e60d31ed"
POST_PUBLICATION_HEADING = "## 发布后：真实发布后的下载复验"


def _guide() -> str:
    assert GUIDE.is_file(), "missing beginner Gazebo guide: gazebo/README.md"
    return GUIDE.read_text(encoding="utf-8")


def _publishing() -> str:
    assert PUBLISHING.is_file(), "missing advanced Gazebo publication guide: gazebo/PUBLISHING.md"
    return PUBLISHING.read_text(encoding="utf-8")


def test_root_readme_is_a_gazebo_first_beginner_entry_point() -> None:
    text = ROOT_README.read_text(encoding="utf-8")

    beginner = "[Gazebo 详细使用教程](gazebo/README.md)"
    publishing = "[Fuel 发布记录与维护说明](gazebo/PUBLISHING.md)"
    launcher = 'bash "$HOME/桌面/onerobot_h1_gazebo/gazebo/scripts/open_demo.sh"'

    assert text.startswith("# OneRobotics A1 for Gazebo")
    assert launcher in text
    assert "右臂" in text and "左臂" in text and "双臂站架" in text
    assert beginner in text
    assert publishing in text
    assert text.index(beginner) < text.index(publishing)
    assert "Isaac Lab Integration" not in text
    assert "Start RSL-RL training" not in text
    assert "scripts/rsl_rl/train.py" not in text
    assert "MuJoCo sim-to-sim" not in text


def test_guide_opens_with_five_plain_language_definitions() -> None:
    opening = [line for line in _guide().splitlines() if line.strip()][:5]

    assert len(opening) == 5
    assert [term in line for term, line in zip(("repository", "URDF", "SDF", "Gazebo", "Fuel"), opening)] == [
        True,
        True,
        True,
        True,
        True,
    ]


def test_guide_identifies_the_public_source_and_gazebo_repository() -> None:
    text = _guide()

    assert "https://github.com/katazen/onerobot_h1.git" in text
    assert SOURCE_COMMIT in text
    assert "https://github.com/uniquemelody/onerobot_h1_gazebo" in text
    assert "Gazebo implementation is not on the public origin" not in text


def test_guide_contains_copy_paste_setup_and_validation_commands() -> None:
    text = _guide()
    required = (
        "env -u PYTHONPATH uv sync --project gazebo --locked",
        "env -u PYTHONPATH uv run --project gazebo python -m onerobotics_a1_gazebo.source_lock --check",
        "bash gazebo/scripts/install_harmonic_conda.sh",
        "env -u PYTHONPATH uv run --project gazebo python -m onerobotics_a1_gazebo.package --output",
        "env -u PYTHONPATH uv run --project gazebo python -m onerobotics_a1_gazebo.validate",
        "bash gazebo/scripts/check_harmonic.sh",
        "env -u PYTHONPATH uv run --project gazebo python -m onerobotics_a1_gazebo.demo generate --models",
    )

    assert not [fragment for fragment in required if fragment not in text]


def test_all_copy_paste_shell_blocks_fail_fast() -> None:
    for document, text in ((GUIDE, _guide()), (PUBLISHING, _publishing())):
        blocks = re.findall(r"~~~bash\n(.*?)\n~~~", text, flags=re.DOTALL)

        assert blocks, document
        for block in blocks:
            lines = [line for line in block.splitlines() if line.strip()]
            assert lines[0] == "set -euo pipefail", (document, block)


def test_publication_blocks_source_harmonic_environment_before_using_its_helper() -> None:
    for block in re.findall(r"~~~bash\n(.*?)\n~~~", _publishing(), flags=re.DOTALL):
        if "a1_harmonic_run" in block:
            assert "source gazebo/scripts/harmonic_env.sh" in block
            assert block.index("source gazebo/scripts/harmonic_env.sh") < block.index("a1_harmonic_run")


def test_guide_contains_the_one_launcher_path_and_separate_smoke_test() -> None:
    text = _guide()
    daily_launcher = 'bash "$HOME/桌面/onerobot_h1_gazebo/gazebo/scripts/open_demo.sh"'
    required = (
        "/usr/bin/gazebo --version",
        "bash gazebo/scripts/install_harmonic_conda.sh",
        'cd "$HOME/桌面/onerobot_h1_gazebo"',
        daily_launcher,
        "bash gazebo/scripts/run_smoke_tests.sh",
    )

    assert not [fragment for fragment in required if fragment not in text]
    assert "任何目录" in text


def test_beginner_guide_has_one_launcher_command_and_no_publication_procedure() -> None:
    text = _guide()
    daily_launcher = 'bash "$HOME/桌面/onerobot_h1_gazebo/gazebo/scripts/open_demo.sh"'
    assert text.count(daily_launcher) == 1
    assert "bash gazebo/scripts/open_demo.sh" not in text
    daily_section = text.split("## 日常查看：只用这一条启动路径", 1)[1].split("\n## ", 1)[0]
    daily_block = re.findall(r"~~~bash\n(.*?)\n~~~", daily_section, flags=re.DOTALL)
    assert daily_block == [f"set -euo pipefail\n{daily_launcher}"]
    assert "1. 右臂" in text and "2. 左臂" in text and "3. 双臂站架" in text
    assert "SMOKE_TEST_OK: 4 cases across 3 models" in text
    assert "PUBLISHING.md" in text
    assert "Fuel token" not in text
    assert "prepare-upload" not in text
    assert "/usr/bin/curl" not in text
    assert "a1_gui_run" not in text
    assert not re.search(r"(?m)^\s*(?:a1_harmonic_run\s+)?gz sim\b", text)


def test_guide_explains_the_launcher_runtime_safety_contract() -> None:
    text = _guide()
    assert "Gazebo Harmonic / Sim 8" in text
    assert "GZ_SIM_RESOURCE_PATH" in text
    assert "HOME/XDG" in text
    assert "DISPLAY" in text and "WAYLAND_DISPLAY" in text
    assert "本地 load/render" in text


def test_thumbnail_workflow_separates_render_review_and_exact_byte_approval() -> None:
    text = _publishing()

    render = "bash gazebo/scripts/render_thumbnails.sh render"
    approve = "bash gazebo/scripts/render_thumbnails.sh approve"
    assert render in text
    assert approve in text
    assert text.index(render) < text.index(approve)
    assert "review all three candidate PNG files" in text
    assert "does not rerender" in text


def test_guide_states_model_distinction_and_known_limits() -> None:
    text = _guide()
    required = (
        "separate CAD revisions",
        "must not be assembled together",
        "high-poly collision meshes",
        "no gripper",
        "no ROS 2 control",
        "no sensors",
        "no invented dynamics or friction",
    )

    assert not [phrase for phrase in required if phrase not in text]


def test_local_runtime_markers_describe_four_cases_across_three_models() -> None:
    text = _guide()

    assert "four SMOKE_TEST_VALID" in text
    assert "SMOKE_TEST_OK: 4 cases across 3 models" in text
    assert "三个 SMOKE_TEST_VALID 和一个 SMOKE_TEST_OK" not in text


def test_archive_evidence_matches_the_real_tools() -> None:
    text = _publishing()
    archive = (
        'sha256sum "dist/gazebo-fuel/archives/$model_slug-1.0.0.tar.gz" '
        '| tee "$release_evidence/$model_slug-1.0.0.archive.sha256"'
    )

    assert archive in text
    assert text.index('release_evidence="$(mktemp -d)"') < text.index(archive)


def test_publication_gate_hard_stops_the_unsafe_pinned_authenticated_client() -> None:
    text = _publishing()
    upload = "gz fuel --force-version 9 upload"
    download = "gz fuel --force-version 9 download"

    assert "Gazebo Fuel account" in text
    assert "ownership rights" in text
    assert "explicit OneRobotics organizational authorization" in text
    assert "authorized reviewer" in text
    assert upload not in text
    assert download not in text
    assert "CURLOPT_SSL_VERIFYPEER=0" in text
    assert "CURLOPT_FOLLOWLOCATION=1" in text
    assert "environment-harmonic.yml only constrains gz-fuel-tools9=9.*" in text
    assert "the locally audited Fuel Tools 9.1.1" in text
    assert 'tee "$release_evidence/fuel-tools.version.txt"' in text
    assert "all other accepted 9.x clients remain unaudited" in text
    assert "hard stop" in text
    assert text.count('--model "$model_dir"') == 2
    assert '--model "$upload_model_dir"' not in text
    assert text.count('--owner "$fuel_owner"') == 1
    assert text.count("--header @-") == 2
    assert "--private" not in text


def test_publication_checks_license_api_before_snapshot_handoff() -> None:
    text = _publishing()
    verify = "python -m onerobotics_a1_gazebo.publication verify-licenses"
    prepare = "python -m onerobotics_a1_gazebo.publication prepare-upload"

    assert "read -rsp 'Fuel token: ' GZ_FUEL_TOKEN; echo" in text
    assert "export GZ_FUEL_TOKEN" not in text
    assert "unset GZ_FUEL_TOKEN" in text
    assert "https://fuel.gazebosim.org/1.0/licenses" in text
    assert "Creative Commons Attribution 4.0 International" in text
    assert f"env -u PYTHONPATH uv run --project gazebo {verify}" in text
    assert '--response "$release_evidence/fuel-licenses.json"' in text
    assert '--license "Creative Commons Attribution 4.0 International"' in text
    assert "grep -F" not in text
    assert text.index(verify) < text.index(prepare)


def test_reviewer_approval_is_bound_to_the_canonical_manifest_before_snapshot() -> None:
    text = _publishing()
    manifest = "python -m onerobotics_a1_gazebo.publication manifest"
    manifest_digest = 'sha256sum "$manifest_path"'
    approval = "authorized reviewer must approve"
    approved_value = "approved_manifest_sha256='replace-with-authorized-reviewer-approved-manifest-sha256'"
    digest_check = "sha256sum --check -"
    prepare = "python -m onerobotics_a1_gazebo.publication prepare-upload"

    assert "approved_archive_sha256='replace-with-authorized-reviewer-approved-archive-sha256'" in text
    assert "approved_resource_name='replace-with-authorized-reviewer-approved-resource-name'" in text
    assert 'test "$approved_resource_name" = "$fuel_resource_name"' in text
    assert "approved owner, resource name and visibility" in text
    assert "canonical manifest SHA-256 is the authoritative upload-content identity" in text
    assert text.count('--manifest-sha256 "$approved_manifest_sha256"') == 2
    assert text.index(manifest) < text.index(manifest_digest)
    assert text.index(manifest_digest) < text.index(approval)
    assert text.index(approval) < text.index(approved_value)
    assert text.index(approved_value) < text.index(digest_check) < text.index(prepare)


def test_upload_uses_an_independent_read_only_manifest_bound_snapshot() -> None:
    text = _publishing()
    prepare = "python -m onerobotics_a1_gazebo.publication prepare-upload"
    hard_stop = "UPLOAD HANDOFF hard stop"

    assert 'upload_stage_root="$(mktemp -d)"' in text
    assert 'upload_model_dir="$upload_stage_root/$model_slug"' in text
    assert f"env -u PYTHONPATH uv run --project gazebo {prepare}" in text
    assert '--manifest "$manifest_path"' in text
    assert '--manifest-sha256 "$approved_manifest_sha256"' in text
    assert '--output "$upload_model_dir"' in text
    assert "read-only" in text
    assert text.index(prepare) < text.index(hard_stop)


def test_each_model_completes_handoff_and_verification_before_variables_change() -> None:
    text = _publishing()

    assert "现在不要先改 model_slug" in text
    assert "直到执行完 cp -a" in text
    assert "再回到这里换成下一个 slug" in text
    assert "第三个模型复制完成后才继续" in text
    assert "保留同一个 verified_models_root" in text
    assert "把 model_slug 依次改成" not in text


def test_curl_ignores_user_config_and_never_forwards_private_tokens_across_redirects() -> None:
    text = _publishing()

    assert text.count("/usr/bin/curl --disable --fail") == 3
    assert not re.search(r"(?m)^\s*curl\s", text)
    assert "curl --fail" not in text
    assert "--location" not in text
    assert text.count("--proto '=https' --tlsv1.2") == 3
    assert text.count("--header @-") == 2
    assert '--header "Private-Token: ${GZ_FUEL_TOKEN}"' not in text


def test_every_secret_operation_is_scoped_to_a_trap_cleaned_subshell() -> None:
    text = _publishing()

    assert text.count("trap 'unset GZ_FUEL_TOKEN' EXIT HUP INT TERM") == 2
    assert text.count("(\n  set +x\n  trap 'unset GZ_FUEL_TOKEN' EXIT HUP INT TERM") == 2
    assert text.count("printf 'Private-Token: %s\\n' \"$GZ_FUEL_TOKEN\" |") == 2
    assert text.count("\n)\n") >= 2


def test_post_upload_urls_use_the_metadata_resource_name_not_the_local_slug() -> None:
    text = _publishing()

    assert "OneRobotics A1 Right Arm" in text
    assert "OneRobotics A1 Left Arm" in text
    assert "OneRobotics A1 Bimanual Stand" in text
    assert "OneRobotics%20A1%20Right%20Arm" in text
    assert "OneRobotics%20A1%20Left%20Arm" in text
    assert "OneRobotics%20A1%20Bimanual%20Stand" in text
    assert 'resource_url="https://fuel.gazebosim.org/1.0/$fuel_owner/models/$fuel_resource_path"' in text
    assert 'models/$model_slug"' not in text
    assert '-path "*/models/$model_slug/*/model.sdf"' not in text
    assert 'cp -a "$raw_check_root/files" "$verified_models_root/$model_slug"' in text


def test_post_upload_resource_api_is_verified_before_selecting_a_version() -> None:
    text = _publishing()
    api = "python -m onerobotics_a1_gazebo.publication verify-api"

    assert 'fuel_api_json="$release_evidence/$model_slug.resource.json"' in text
    assert '"$resource_url" --output "$fuel_api_json"' in text
    assert f"env -u PYTHONPATH uv run --project gazebo {api}" in text
    assert '--owner "$fuel_owner"' in text
    assert '--name "$fuel_resource_name"' in text
    assert '--license "Creative Commons Attribution 4.0 International"' in text
    assert '--visibility "$expected_visibility"' in text
    assert 'fuel_version_file="$release_evidence/$model_slug.version"' in text
    assert 'test ! -e "$fuel_version_file"' in text
    assert '--version-output "$fuel_version_file"' in text
    assert 'model_version="$(<"$fuel_version_file")"' in text
    assert text.index(api) < text.index("raw_zip_url=")


def test_post_upload_shell_blocks_fail_fast_and_reject_stale_version_evidence() -> None:
    text = _publishing()
    section = text[text.index(POST_PUBLICATION_HEADING) :]
    blocks = re.findall(r"~~~bash\n(.*?)\n~~~", section, flags=re.DOTALL)

    assert len(blocks) == 2
    assert all(block.startswith("set -euo pipefail\n") for block in blocks)
    api_block = blocks[0]
    version_file = 'fuel_version_file="$release_evidence/$model_slug.version"'
    absence_guard = 'test ! -e "$fuel_version_file"'
    verify_api = "python -m onerobotics_a1_gazebo.publication verify-api"
    assert api_block.index(version_file) < api_block.index(absence_guard) < api_block.index(verify_api)


def test_raw_zip_uses_typed_manifest_and_safe_verified_extraction() -> None:
    text = _publishing()
    manifest = "python -m onerobotics_a1_gazebo.publication manifest"
    verify = "python -m onerobotics_a1_gazebo.publication verify-zip"

    assert f"env -u PYTHONPATH uv run --project gazebo {manifest}" in text
    assert '--model "$model_dir"' in text
    assert '--output "$manifest_path"' in text
    assert f"env -u PYTHONPATH uv run --project gazebo {verify}" in text
    assert '--zip "$raw_check_root/$model_slug-$model_version.zip"' in text
    assert '--manifest "$manifest_path"' in text
    assert '--manifest-sha256 "$approved_manifest_sha256"' in text
    assert '--output "$raw_check_root/files"' in text
    assert "unzip " not in text
    assert text.index(verify) < text.index("sha256sum --check", text.index(verify))


def test_post_upload_checks_distinguish_raw_zip_staging_from_a_real_fuel_cache() -> None:
    text = _publishing()
    required = (
        "pre-upload external manifest",
        "versioned raw Fuel ZIP",
        "sha256sum --check",
        "verified raw ZIP",
        "metadata.pbtxt",
        "model.sdf",
        "rerun structural and runtime checks",
        "verified raw-ZIP staging tree, not a real Fuel client cache",
        "does not prove real Fuel client-cache retrieval or URI/XML rewriting",
        "remains blocked until an organizationally approved secure client",
        "native Fuel cache root cannot be passed directly to --cache-models",
        "fresh slug-normalized staging root",
    )

    assert not [phrase for phrase in required if phrase not in text]
    assert ("env -u PYTHONPATH uv run --project gazebo python -m onerobotics_a1_gazebo.demo smoke-cache-all") in text
    assert "--trusted-models dist/gazebo-fuel" in text
    assert '--cache-models "$verified_models_root"' in text
    assert "--worlds gazebo/worlds" in text
    assert "4 条 CACHE_SMOKE_TEST_VALID" in text
    assert "CACHE_SMOKE_TEST_OK: 4 cases across 3 models" in text
    assert "joint_r1" in text and "joint_l1" in text
    assert "三个代表关节" not in text


def test_verified_raw_zip_reaches_native_tools_only_after_the_safe_cache_gate() -> None:
    text = _publishing()
    section = text[text.index(POST_PUBLICATION_HEADING) :]
    blocks = re.findall(r"~~~bash\n(.*?)\n~~~", section, flags=re.DOTALL)
    verification_block = blocks[0]

    assert "gz sdf" not in verification_block
    assert "gz fuel --force-version 9 meta" not in verification_block
    assert "onerobotics_a1_gazebo.harmonic" not in verification_block
    assert verification_block.index("verify-zip") < verification_block.index('cp -a "$raw_check_root/files"')
    assert "smoke-cache-all" in blocks[1]
    assert "后面的 smoke-cache-all 安全门" in text


def test_documentation_contains_no_credentials_private_paths_or_unsafe_version_claims() -> None:
    text = _guide() + _publishing()
    token_placeholder = "<" + "TOKEN>"
    developer_home = "/home/" + "woan"
    forbidden = (
        token_placeholder,
        developer_home,
        "example.internal",
        "company-internal",
        "already uploaded",
        "upload succeeded",
    )

    assert not [value for value in forbidden if value.casefold() in text.casefold()]
    assert not re.search(r"gz sim(?! --force-version 8(?:\s|$))", text)
    uv_lines = [line.strip() for line in text.splitlines() if "uv run --project gazebo" in line]
    assert uv_lines
    assert all(line.startswith("env -u PYTHONPATH uv run --project gazebo") for line in uv_lines)


def test_advanced_publication_guide_retains_the_hard_stop() -> None:
    text = _publishing()
    assert "UPLOAD HANDOFF hard stop" in text
    assert "CURLOPT_SSL_VERIFYPEER=0" in text
    assert "CURLOPT_FOLLOWLOCATION=1" in text
    assert "prepare-upload" in text
    assert text.count("/usr/bin/curl --disable --fail") == 3


def test_publication_guide_starts_at_publication_and_declares_its_local_prerequisites() -> None:
    text = _publishing()

    assert "先完成 [README 的一次性 setup 和本地 4-case smoke](README.md)" in text
    assert "仓库根目录" in text
    assert "dist/gazebo-fuel" in text
    assert "repository（仓库）" not in text
    assert "# A1 Gazebo Harmonic / Fuel 零基础操作指南" not in text
    assert "a1_gui_run" not in text
    assert "gz sim --force-version 8" not in text
