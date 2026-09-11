# Beginner Gazebo Launcher Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Provide a cwd-independent interactive Gazebo model viewer for a beginner, separate advanced publication instructions, and close the missing bimanual-left motion acceptance case.

**Architecture:** A strict Bash launcher reuses the reviewed Harmonic helper, creates and validates a private deterministic model export, then maps a four-choice Chinese menu to three fixed checked-in worlds. Existing Python motion infrastructure gains one additional bimanual-left case. Beginner and authorized-maintainer documentation are split without weakening publication security contracts.

**Tech Stack:** Bash, Python 3.10+, pytest, uv, Gazebo Harmonic / Sim 8, Mamba, Markdown.

## Global Constraints

- Do not modify the locked `source/h1_reach/h1_reach/assets/urdf/A1_2026` bytes.
- Do not install, use sudo, change global shell/Conda/Gazebo state, contact Fuel, upload, push, merge, or request credentials.
- All new behavior follows RED → GREEN → refactor; every regression test must be observed failing before production changes.
- Every native Gazebo call uses `a1_harmonic_run` and `gz sim --force-version 8`.
- The launcher accepts no arbitrary model/world path and resolves the repository with `BASH_SOURCE[0]`.
- The GUI receives only the private model resource root, private HOME/XDG paths, and the five allow-listed display-session variables.
- Ctrl+C status 130 is a normal window close; other nonzero statuses fail.
- No authenticated Fuel Tools 9 upload/download command may appear.
- Do not push or upload any result.

---

### Task 1: Complete both halves of bimanual commanded-motion acceptance

**Files:**
- Modify: `gazebo/tests/test_demo.py`
- Modify: `gazebo/tests/test_runtime.py`
- Modify: `gazebo/onerobotics_a1_gazebo/demo.py`

**Interfaces:**
- Consumes: existing `_LIVE_CASES`,
  `smoke_test(*, model: str, joint: str, target: float, export_root: Path, worlds_root: Path) -> SmokeResult`,
  `smoke_cache_all(*, trusted_models: Path, cache_models: Path, worlds_root: Path) -> tuple[SmokeResult, ...]`,
  and the bounded Task 6 lifecycle.
- Produces: four motion cases across three models, including `joint_r1` and `joint_l1` for `onerobotics_a1_bimanual_stand`; CLI footer says `4 cases across 3 models`.

- [ ] **Step 1: Write the failing live-case contract test**

Add to `gazebo/tests/test_demo.py`:

```python
def test_live_cases_cover_standalone_models_and_both_bimanual_halves() -> None:
    assert demo._LIVE_CASES == (
        ("onerobotics_a1_right_arm", "joint1-a1_r", 0.05),
        ("onerobotics_a1_left_arm", "joint1-a1_l", 0.05),
        ("onerobotics_a1_bimanual_stand", "joint_r1", 0.05),
        ("onerobotics_a1_bimanual_stand", "joint_l1", 0.05),
    )
```

Change the successful cache CLI fake to return one `SmokeResult` per
`demo._LIVE_CASES`, then require four `CACHE_SMOKE_TEST_VALID` lines and the
exact footer `CACHE_SMOKE_TEST_OK: 4 cases across 3 models`.

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```bash
env -u PYTHONPATH uv run --project gazebo pytest \
  gazebo/tests/test_demo.py::test_live_cases_cover_standalone_models_and_both_bimanual_halves \
  gazebo/tests/test_demo.py::test_cache_smoke_cli_prints_only_after_all_motion_passes -q
```

Expected: FAIL because `joint_l1` is absent and the footer still reports three models.

- [ ] **Step 3: Add the minimal production case and truthful CLI wording**

Change `demo.py` to:

```python
_LIVE_CASES = (
    ("onerobotics_a1_right_arm", "joint1-a1_r", 0.05),
    ("onerobotics_a1_left_arm", "joint1-a1_l", 0.05),
    ("onerobotics_a1_bimanual_stand", "joint_r1", 0.05),
    ("onerobotics_a1_bimanual_stand", "joint_l1", 0.05),
)
```

Change parser help to “four real commanded-motion cases across three models”.
Change success footers to:

```python
print(f"SMOKE_TEST_OK: {len(lines)} cases across 3 models")
print(f"CACHE_SMOKE_TEST_OK: {len(lines)} cases across 3 models")
```

- [ ] **Step 4: Extend the real runtime parameterization and decouple render expectations**

Add this fourth parameter in `test_position_command_moves_joint_toward_target`:

```python
("onerobotics_a1_bimanual_stand", "joint_l1", 0.05),
```

The render test must continue expecting exactly the three model slugs from
`load_model_specs()`, not one item per motion case:

```python
assert tuple(captured.png_by_slug) == tuple(spec.slug for spec in load_model_specs())
```

Update failure assertions that mention the old success footer.

- [ ] **Step 5: Verify GREEN with unit and real runtime tests**

Run:

```bash
env -u PYTHONPATH uv run --project gazebo pytest \
  gazebo/tests/test_demo.py gazebo/tests/test_runtime.py -m 'not runtime' -q
env -u PYTHONPATH uv run --project gazebo pytest \
  gazebo/tests/test_runtime.py -m 'runtime and not render_runtime' -q
```

Expected: focused non-runtime tests pass; real runtime reports `4 passed` and
each bimanual half moves by at least `0.02 rad`.

- [ ] **Step 6: Commit Task 1**

```bash
git add gazebo/onerobotics_a1_gazebo/demo.py \
  gazebo/tests/test_demo.py gazebo/tests/test_runtime.py
git commit -m "test: cover both A1 bimanual arms in Gazebo"
```

---

### Task 2: Add the cwd-independent interactive GUI launcher

**Files:**
- Create: `gazebo/scripts/open_demo.sh`
- Create: `gazebo/tests/test_open_demo.py`
- Modify: `gazebo/tests/test_cli_scripts.py`

**Interfaces:**
- Consumes: `gazebo/scripts/harmonic_env.sh`, `.venv/bin/python`, package and validator CLIs, three checked-in world files.
- Produces: no-argument `bash gazebo/scripts/open_demo.sh` menu; selections `1`, `2`, `3`, `0`; fixed Sim 8 launches.

- [ ] **Step 1: Write failing launcher existence and menu tests**

Create `gazebo/tests/test_open_demo.py` with a private fake Mamba root and tests
that run the script from `tmp_path`. The first contracts are:

```python
OPEN_DEMO = Path(__file__).resolve().parents[1] / "scripts" / "open_demo.sh"


def test_launcher_exists_is_executable_and_parses_as_bash() -> None:
    assert OPEN_DEMO.is_file()
    assert os.access(OPEN_DEMO, os.X_OK)
    subprocess.run(["bash", "-n", str(OPEN_DEMO)], check=True)


@pytest.mark.parametrize(
    ("choice", "world"),
    [
        ("1", "onerobotics_a1_right_arm_demo.sdf"),
        ("2", "onerobotics_a1_left_arm_demo.sdf"),
        ("3", "onerobotics_a1_bimanual_stand_demo.sdf"),
    ],
)
def test_menu_choice_launches_only_the_fixed_world_from_any_cwd(
    tmp_path: Path,
    choice: str,
    world: str,
) -> None:
    result, calls = run_launcher(tmp_path, f"{choice}\n0\n")
    assert result.returncode == 0, result.stderr
    assert len(calls) == 1
    assert calls[0][-7:] == [
        "gz", "sim", "--force-version", "8", "-v", "4", str(WORLDS / world)
    ]
```

The helper records every Mamba argument outside the launcher's temporary root,
creates the exact safe `<fake-root>/bin/mamba` and existing environment prefix,
sets `A1_PYTHON_BIN=sys.executable`, and provides `DISPLAY=:99`.

- [ ] **Step 2: Add failing safety and interaction tests**

Add tests that require:

```python
assert "GZ_SIM_RESOURCE_PATH=" in "\n".join(payload)
assert not any(value.startswith("GZ_SIM_RESOURCE_PATH=/hostile") for value in payload)
assert all(name in joined for name in ("HOME=", "XDG_CACHE_HOME=", "XDG_CONFIG_HOME=", "XDG_DATA_HOME="))
assert "DISPLAY=:99" in payload
assert "gz sim --force-version 8" in OPEN_DEMO.read_text(encoding="utf-8")
```

Also cover: extra arguments exit 2; missing display exits before Mamba; invalid
input followed by `0` retries without Mamba; EOF exits 0; fake GUI status 130
returns to the menu and then accepts `0`; status 42 exits nonzero; ambient
`GZ_SIM_RESOURCE_PATH`, HOME, and XDG variables do not enter the inner payload.

- [ ] **Step 3: Run launcher tests and verify RED**

Run:

```bash
env -u PYTHONPATH uv run --project gazebo pytest \
  gazebo/tests/test_open_demo.py gazebo/tests/test_cli_scripts.py \
  -k 'open_demo or launcher or shell_scripts_are_strict or forbidden_global' -q
```

Expected: FAIL because `open_demo.sh` is missing.

- [ ] **Step 4: Implement the minimal strict launcher**

Create executable `gazebo/scripts/open_demo.sh` with this structure:

```bash
#!/usr/bin/env bash
set -euo pipefail

script_dir=$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
gazebo_root=${script_dir%/scripts}
# shellcheck source=harmonic_env.sh
source "$script_dir/harmonic_env.sh"

if (( $# != 0 )); then
  printf '用法：bash %s（不需要参数）\n' "${0##*/}" >&2
  exit 2
fi
if [[ -z ${DISPLAY:-}${WAYLAND_DISPLAY:-} ]]; then
  printf '无法打开 Gazebo：当前终端没有图形桌面 DISPLAY 或 WAYLAND_DISPLAY。\n' >&2
  exit 1
fi

python_bin=${A1_PYTHON_BIN:-$gazebo_root/.venv/bin/python}
if [[ ! -x $python_bin ]]; then
  printf 'Gazebo Python 环境不存在，请先在仓库根目录执行：env -u PYTHONPATH uv sync --project gazebo --locked\n' >&2
  exit 1
fi

session_root=$(mktemp -d "${TMPDIR:-/tmp}/onerobotics-a1-gui.XXXXXX")
cleanup_session() { rm -rf -- "$session_root"; }
trap cleanup_session EXIT
models_root=$session_root/models

if ! build_output=$(env -u PYTHONPATH "$python_bin" -m onerobotics_a1_gazebo.package --output "$models_root" 2>&1); then
  printf '模型生成失败：%s\n' "$build_output" >&2
  exit 1
fi
if ! validation_output=$(env -u PYTHONPATH "$python_bin" -m onerobotics_a1_gazebo.validate "$models_root" 2>&1); then
  printf '模型验证失败：%s\n' "$validation_output" >&2
  exit 1
fi
printf '模型数据验证通过。\n'

if ! prefix_state=$(a1_harmonic_prefix_state); then
  exit 1
fi
if [[ $prefix_state != existing ]]; then
  printf 'Gazebo Harmonic 环境尚未安装，请先运行：bash %s/install_harmonic_conda.sh\n' "$script_dir" >&2
  exit 1
fi

mkdir -p "$session_root/home" "$session_root/cache" "$session_root/config" "$session_root/data"
gui_env=(
  "GZ_SIM_RESOURCE_PATH=$models_root"
  "HOME=$session_root/home"
  "XDG_CACHE_HOME=$session_root/cache"
  "XDG_CONFIG_HOME=$session_root/config"
  "XDG_DATA_HOME=$session_root/data"
)
for variable in DISPLAY WAYLAND_DISPLAY XAUTHORITY XDG_RUNTIME_DIR DBUS_SESSION_BUS_ADDRESS; do
  if [[ -v $variable ]]; then
    gui_env+=("$variable=${!variable}")
  fi
done

while true; do
  printf '\n请选择要查看的 Gazebo 模型：\n1. 右臂\n2. 左臂\n3. 双臂站架\n0. 退出\n'
  if ! IFS= read -r -p '请输入 0、1、2 或 3：' choice; then
    printf '\n'
    exit 0
  fi
  case $choice in
    0) exit 0 ;;
    1) world=onerobotics_a1_right_arm_demo.sdf ;;
    2) world=onerobotics_a1_left_arm_demo.sdf ;;
    3) world=onerobotics_a1_bimanual_stand_demo.sdf ;;
    *) printf '输入无效，请输入 0、1、2 或 3。\n'; continue ;;
  esac
  set +e
  a1_harmonic_run env "${gui_env[@]}" gz sim --force-version 8 -v 4 "$gazebo_root/worlds/$world"
  gui_status=$?
  set -e
  if (( gui_status != 0 && gui_status != 130 )); then
    printf 'Gazebo 启动失败，退出状态：%s\n' "$gui_status" >&2
    exit "$gui_status"
  fi
done
```

Set executable mode with `chmod +x gazebo/scripts/open_demo.sh`.

- [ ] **Step 5: Add the launcher to repository-wide shell safety contracts**

In `test_cli_scripts.py` define `OPEN_DEMO = SCRIPTS / "open_demo.sh"` and add
it to `SCRIPT_PATHS`, so Bash syntax, `BASH_SOURCE[0]`, and forbidden mutation /
Fuel-network assertions apply automatically.

- [ ] **Step 6: Verify GREEN and real Ctrl+C menu survival**

Run:

```bash
env -u PYTHONPATH uv run --project gazebo pytest \
  gazebo/tests/test_open_demo.py gazebo/tests/test_cli_scripts.py -q
bash -n gazebo/scripts/open_demo.sh
```

Then use a PTY on the configured desktop, select `1`, wait for Gazebo to open,
send Ctrl+C, verify the menu reappears, enter `0`, and require final status 0.

- [ ] **Step 7: Commit Task 2**

```bash
git add gazebo/scripts/open_demo.sh \
  gazebo/tests/test_open_demo.py gazebo/tests/test_cli_scripts.py
git commit -m "feat: add beginner Gazebo model launcher"
```

---

### Task 3: Split beginner use from authorized publication

**Files:**
- Modify: `gazebo/tests/test_documentation.py`
- Modify: `gazebo/README.md`
- Create: `gazebo/PUBLISHING.md`
- Modify: `README.md`

**Interfaces:**
- Consumes: launcher command and current fail-closed publication workflow.
- Produces: short beginner guide plus a complete organization-maintainer publication guide.

- [ ] **Step 1: Write failing two-document contracts**

In `test_documentation.py` add:

```python
PUBLISHING = ROOT / "gazebo" / "PUBLISHING.md"


def _publishing() -> str:
    assert PUBLISHING.is_file()
    return PUBLISHING.read_text(encoding="utf-8")


def test_beginner_guide_has_one_launcher_command_and_no_publication_procedure() -> None:
    text = _guide()
    assert "bash gazebo/scripts/open_demo.sh" in text
    assert "1. 右臂" in text and "2. 左臂" in text and "3. 双臂站架" in text
    assert "SMOKE_TEST_OK: 4 cases across 3 models" in text
    assert "PUBLISHING.md" in text
    assert "Fuel token" not in text
    assert "prepare-upload" not in text
    assert "/usr/bin/curl" not in text


def test_advanced_publication_guide_retains_the_hard_stop() -> None:
    text = _publishing()
    assert "UPLOAD HANDOFF hard stop" in text
    assert "CURLOPT_SSL_VERIFYPEER=0" in text
    assert "CURLOPT_FOLLOWLOCATION=1" in text
    assert "prepare-upload" in text
    assert text.count("/usr/bin/curl --disable --fail") == 3
```

Route every existing thumbnail/publication/API/ZIP/cache contract to
`_publishing()`; leave repository/source/install/local/runtime contracts on
`_guide()`.

- [ ] **Step 2: Run documentation tests and verify RED**

Run:

```bash
env -u PYTHONPATH uv run --project gazebo pytest gazebo/tests/test_documentation.py -q
```

Expected: FAIL because `PUBLISHING.md` and the one-command beginner path do not exist.

- [ ] **Step 3: Write the short beginner guide**

Keep the five plain-language definitions, then explain:

```text
代码 1 katazen/onerobot_h1：A1 source assets and vendor-side integrations.
代码 2 T1Amoo/IsaacLab branch：an IsaacLab-repository contribution, not a code-1 branch.
This Gazebo work：a local unpushed code-1 feature branch, not a Gazebo-engine fork.
```

Give one-time setup commands, then the only normal viewing command:

```bash
cd "$HOME/桌面/onerobot_h1-gazebo"
bash gazebo/scripts/open_demo.sh
```

Document the `1/2/3/0` menu, the visual checklist, the separate four-case smoke
command, current limits, and the exact statement that GUI success proves local
load/render only. Link authorized maintainers to `PUBLISHING.md`.

- [ ] **Step 4: Move advanced material without weakening it**

Create `PUBLISHING.md` from the current thumbnail, authorization, manifest,
safe-client handoff, API/raw-ZIP, and strict-cache sections. Add a prerequisite
link back to the beginner guide and retain all command blocks, digest binding,
absolute `/usr/bin/curl`, token-via-stdin, no-redirect, TLS, and Fuel Tools 9
hard-stop wording.

Update root `README.md` to link both guides with beginner first.

- [ ] **Step 5: Verify documentation GREEN and every Bash block parses**

Run:

```bash
env -u PYTHONPATH uv run --project gazebo pytest gazebo/tests/test_documentation.py -q
for document in gazebo/README.md gazebo/PUBLISHING.md; do
  while IFS= read -r -d '' block; do bash -n <<<"$block"; done \
    < <(perl -0777 -ne 'while (/~~~bash\n(.*?)\n~~~/sg) { print $1, "\0" }' "$document")
done
```

Expected: all documentation tests and all fenced Bash blocks pass.

- [ ] **Step 6: Commit Task 3**

```bash
git add README.md gazebo/README.md gazebo/PUBLISHING.md gazebo/tests/test_documentation.py
git commit -m "docs: add beginner Gazebo viewing path"
```

---

### Task 4: Full release-candidate regression and independent review

**Files:**
- Modify only files implicated by a failed gate or reproduced review finding.

**Interfaces:**
- Consumes: Tasks 1–3 commits.
- Produces: a clean local branch with evidence; no remote mutation.

- [ ] **Step 1: Run all static and non-runtime gates**

```bash
env -u PYTHONPATH uv run --project gazebo ruff check gazebo
env -u PYTHONPATH uv run --project gazebo ruff format --check gazebo
env -u PYTHONPATH uv run --project gazebo pytest gazebo/tests -m 'not runtime' -q
while IFS= read -r -d '' script; do bash -n "$script"; done \
  < <(find gazebo/scripts -type f -name '*.sh' -print0)
git diff --check
```

- [ ] **Step 2: Run all real runtime gates serially**

```bash
env -u PYTHONPATH uv run --project gazebo pytest \
  gazebo/tests/test_runtime.py -m 'runtime and not render_runtime' -q
env -u PYTHONPATH uv run --project gazebo pytest \
  gazebo/tests/test_runtime.py -m render_runtime -q
bash gazebo/scripts/run_smoke_tests.sh dist/gazebo-fuel gazebo/worlds
```

Require four successful motion cases, including both bimanual halves, and one
three-model Ogre2 render test.

- [ ] **Step 3: Run the strict versioned-HTTPS cache aggregate**

Create a fresh private staging root from `dist/gazebo-fuel`, rewrite only each
mesh URI to its exact Fuel versioned HTTPS form, and run `smoke-cache-all`.
Require four case lines and `CACHE_SMOKE_TEST_OK: 4 cases across 3 models`.
Clean the exact temporary root after the command.

- [ ] **Step 4: Verify immutable source, repository hygiene, and no residual processes**

```bash
git diff --exit-code ecf530911284ba0e559f7a24dc222fd8e60d31ed -- \
  source/h1_reach/h1_reach/assets/urdf/A1_2026
env -u PYTHONPATH uv run --project gazebo python -m \
  onerobotics_a1_gazebo.source_lock --check gazebo/generated-manifest.json
git status --short --branch
ps -eo pid=,ppid=,pgid=,stat=,args= | \
  rg 'mamba|micromamba|gz sim|gz-server|ruby.*gz|onerobotics-a1-smoke|pytest' || true
```

- [ ] **Step 5: Request independent review and close only reproduced findings**

Ask a fresh reviewer to check the launcher trust boundary, menu/path mapping,
display environment allow-list, Ctrl+C behavior, bimanual-left acceptance,
documentation split, and absence of upload commands. Reproduce every finding,
add a failing regression before a fix, rerun relevant and full gates, then keep
the completed branch local without push/upload.
