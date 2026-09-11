# OneRobotics A1 Gazebo Harmonic / Fuel Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build, validate, render, and archive three self-contained OneRobotics A1 model packages for Gazebo Harmonic and Gazebo Fuel without using private code or credentials.

**Architecture:** A Python 3.10+ tool reads the immutable public A1 URDF/STL baseline, validates a checked-in SHA-256 lock, converts each URDF into native SDF 1.11 while preserving topology, and overlays only the published actuator effort and velocity limits. It exports atomic Fuel directories through a temporary staging area, validates provenance and physics data, produces deterministic archives, and uses separate generated demo worlds for Gazebo-native motion tests so published models stay controller-free.

**Tech Stack:** Python 3.10+, `xml.etree.ElementTree`, PyYAML, pytest, uv, Gazebo Harmonic (Gazebo Sim 8 / SDFormat 14 / Fuel Tools 9), POSIX shell, Git.

## Global Constraints

- Target Gazebo Harmonic: Gazebo Sim 8, SDFormat 14, SDF 1.11.
- Source baseline: `katazen/onerobot_h1@ecf530911284ba0e559f7a24dc222fd8e60d31ed`.
- Source root: `source/h1_reach/h1_reach/assets/urdf/A1_2026/`.
- Export exactly three independent resources: right arm, left arm, bimanual robot with stand.
- Never modify source URDF, STL, YAML, mass, inertia, geometry, joint pose, axis, or position limits.
- Never combine the independent-arm CAD revision with the bimanual-stand revision.
- Overlay joints 1--3 with effort `26.859 N m` and velocity `2.6179938779914944 rad/s`.
- Overlay joints 4--7 with effort `5.975 N m` and velocity `12.566370614359172 rad/s`.
- Preserve bimanual fixed joints `joint_r0` / `joint_l0` and links `Link_r0` / `Link_l0`.
- Anchor each model to SDF `world` with a fixed joint; do not use `<static>true>`.
- Published Fuel models contain no Gazebo controller plugin, ROS 2 dependency, sensor, gripper, invented damping, or invented friction.
- Exported assets are explicitly CC BY 4.0 with source URL, immutable commit, attribution, and changes notice.
- Fuel upload is a documented human action requiring the user's token; implementation must not upload.
- Generated directories contain no symlinks, absolute paths, credentials, or dependency on another model.
- Every task follows red-green TDD and ends in a focused commit.

Tasks 1--6 are the Gazebo core milestone: immutable input, correct models,
validation, a Harmonic installation, and commanded motion. Tasks 7--9 complete
the Fuel publication candidate. A rendering-only failure may be reported
separately after the core milestone, but it may not be hidden or described as a
fully publication-ready result.

---

## File Map

| Path | Responsibility |
|---|---|
| `gazebo/pyproject.toml` | Isolated Python tooling and test dependencies |
| `gazebo/uv.lock` | Reproducible Python dependency resolution |
| `gazebo/config/models.yaml` | Allow-listed source paths, names, topology, and actuator groups |
| `gazebo/generated-manifest.json` | Checked-in SHA-256 lock for the 39-file A1_2026 source inventory |
| `gazebo/onerobotics_a1_gazebo/spec.py` | Typed config and hardware-overlay loading |
| `gazebo/onerobotics_a1_gazebo/source_lock.py` | Source discovery, hashing, lock writing and checking |
| `gazebo/onerobotics_a1_gazebo/sdf.py` | Strict URDF-to-SDF 1.11 conversion |
| `gazebo/onerobotics_a1_gazebo/package.py` | Atomic Fuel directory export and deterministic archives |
| `gazebo/onerobotics_a1_gazebo/validate.py` | Pure-data package and physics validation |
| `gazebo/onerobotics_a1_gazebo/demo.py` | Controller-free model wrapping, demo controller injection, runtime orchestration |
| `gazebo/templates/*` | Fuel metadata, attribution, model README, and config templates |
| `gazebo/scripts/*.sh` | Harmonic CLI checks, runtime smoke tests, and thumbnail capture |
| `gazebo/worlds/*.sdf` | Generated/reviewed control demo worlds |
| `gazebo/tests/*` | Unit, export, determinism, CLI, and runtime tests |
| `gazebo/README.md` | Zero-background build, local use, Fuel upload, and verification guide |
| `.gitignore` | Ignore `/dist/` and transient Gazebo test output only |

---

### Task 1: Isolated tooling, model specifications, and source lock

**Files:**
- Create: `gazebo/pyproject.toml`
- Create: `gazebo/onerobotics_a1_gazebo/__init__.py`
- Create: `gazebo/onerobotics_a1_gazebo/spec.py`
- Create: `gazebo/onerobotics_a1_gazebo/source_lock.py`
- Create: `gazebo/config/models.yaml`
- Create: `gazebo/generated-manifest.json`
- Create: `gazebo/tests/test_spec.py`
- Create: `gazebo/tests/test_source_lock.py`
- Create: `gazebo/uv.lock`

**Interfaces:**
- Produces: `ModelSpec`, `JointGroup`, `load_model_specs()`, `load_hardware_overlay()`, `discover_source_files()`, `build_source_lock()`, `validate_source_lock()`.
- Consumes: public URDF/STL/YAML paths and the Git baseline defined in Global Constraints.

- [ ] **Step 1: Write failing specification and source-lock tests**

Create tests with these exact behavioral assertions:

```python
# gazebo/tests/test_spec.py
from onerobotics_a1_gazebo.spec import load_hardware_overlay, load_model_specs


def test_three_atomic_model_specs_are_complete():
    specs = load_model_specs()
    assert [spec.slug for spec in specs] == [
        "onerobotics_a1_right_arm",
        "onerobotics_a1_left_arm",
        "onerobotics_a1_bimanual_stand",
    ]
    assert [len(spec.physical_links) for spec in specs] == [8, 8, 17]
    assert [len(spec.actuated_joints) for spec in specs] == [7, 7, 14]
    assert specs[2].fixed_joints == ("joint_r0", "joint_l0")
    assert specs[0].actuator_groups == (("joint1-a1_r", "joint2-a1_r", "joint3-a1_r"),
                                         ("joint4-a1_r", "joint5-a1_r", "joint6-a1_r", "joint7-a1_r"))


def test_hardware_overlay_matches_public_yaml():
    overlay = load_hardware_overlay()
    assert overlay[1].effort == 26.859
    assert overlay[1].velocity == 2.6179938779914944
    assert overlay[4].effort == 5.975
    assert overlay[4].velocity == 12.566370614359172
    assert set(overlay) == set(range(1, 8))
```

```python
# gazebo/tests/test_source_lock.py
from pathlib import Path

import pytest

from onerobotics_a1_gazebo.source_lock import build_source_lock, discover_source_files, validate_source_lock


def test_source_inventory_is_exactly_the_public_39_file_set():
    files = discover_source_files()
    assert len(files) == 39
    assert sum(path.suffix.lower() == ".stl" for path in files) == 33
    assert sum(path.suffix == ".urdf" for path in files) == 3
    assert {path.name for path in files} >= {"README.md", "SHA256SUMS", "model_parameters.yaml"}


def test_checked_in_lock_matches_source_bytes():
    validate_source_lock()


def test_lock_rejects_changed_bytes(tmp_path: Path):
    source = tmp_path / "source"
    source.mkdir()
    (source / "a.urdf").write_text("one", encoding="utf-8")
    lock = build_source_lock((source / "a.urdf",), relative_to=source)
    (source / "a.urdf").write_text("two", encoding="utf-8")
    with pytest.raises(ValueError, match="SHA-256 mismatch"):
        validate_source_lock(lock=lock, source_root=source)
```

- [ ] **Step 2: Run tests and record the expected import failure**

Run:

```bash
uv run --project gazebo --with pytest pytest gazebo/tests/test_spec.py gazebo/tests/test_source_lock.py -q
```

Expected: collection fails with `ModuleNotFoundError: No module named 'onerobotics_a1_gazebo'`.

- [ ] **Step 3: Add the isolated project and exact model config**

Create `gazebo/pyproject.toml` with Python 3.10+, `PyYAML>=6.0.2,<7`, and a
`dev` dependency group containing `pytest>=8.4,<9` and `ruff>=0.12,<0.14`.
Configure pytest with `pythonpath = ["."]` and `testpaths = ["tests"]`.

Create `models.yaml` with all explicit names, including these joint groups:

```yaml
source_commit: ecf530911284ba0e559f7a24dc222fd8e60d31ed
source_repository: https://github.com/katazen/onerobot_h1
source_root: source/h1_reach/h1_reach/assets/urdf/A1_2026
models:
  - key: right_arm
    slug: onerobotics_a1_right_arm
    display_name: OneRobotics A1 Right Arm
    urdf: a1_r.urdf
    physical_links: [base_link, Link1, Link2, Link3, Link4, Link5, Link6, Link7]
    fixed_joints: []
    actuator_groups:
      - [joint1-a1_r, joint2-a1_r, joint3-a1_r]
      - [joint4-a1_r, joint5-a1_r, joint6-a1_r, joint7-a1_r]
  - key: left_arm
    slug: onerobotics_a1_left_arm
    display_name: OneRobotics A1 Left Arm
    urdf: a1_l.urdf
    physical_links: [base_link, Link1, Link2, Link3, Link4, Link5, Link6, Link7]
    fixed_joints: []
    actuator_groups:
      - [joint1-a1_l, joint2-a1_l, joint3-a1_l]
      - [joint4-a1_l, joint5-a1_l, joint6-a1_l, joint7-a1_l]
  - key: bimanual_stand
    slug: onerobotics_a1_bimanual_stand
    display_name: OneRobotics A1 Bimanual Stand
    urdf: bimanual_stand/a1_bimanual_stand.urdf
    physical_links: [base_link, Link_r0, Link_r1, Link_r2, Link_r3, Link_r4, Link_r5, Link_r6, Link_r7,
                     Link_l0, Link_l1, Link_l2, Link_l3, Link_l4, Link_l5, Link_l6, Link_l7]
    fixed_joints: [joint_r0, joint_l0]
    actuator_groups:
      - [joint_r1, joint_r2, joint_r3]
      - [joint_r4, joint_r5, joint_r6, joint_r7]
      - [joint_l1, joint_l2, joint_l3]
      - [joint_l4, joint_l5, joint_l6, joint_l7]
```

- [ ] **Step 4: Implement typed loading and fail-closed validation**

Implement immutable dataclasses and validate duplicates, missing files, group lengths, and exact group-to-motor mapping:

```python
@dataclass(frozen=True)
class JointLimitOverlay:
    effort: float
    velocity: float


@dataclass(frozen=True)
class ModelSpec:
    key: str
    slug: str
    display_name: str
    urdf: Path
    physical_links: tuple[str, ...]
    fixed_joints: tuple[str, ...]
    actuator_groups: tuple[tuple[str, ...], ...]

    @property
    def actuated_joints(self) -> tuple[str, ...]:
        return tuple(joint for group in self.actuator_groups for joint in group)


def load_model_specs(config_path: Path | None = None) -> tuple[ModelSpec, ...]: ...
def load_hardware_overlay(parameters_path: Path | None = None) -> dict[int, JointLimitOverlay]: ...
```

`load_hardware_overlay()` must read the published YAML, map indices 1--3 to motor `4340`, map 4--7 to `4310`, use `peak_output_torque_nm` and `rated_output_speed_rad_s`, reject missing/non-finite/non-positive values, and return exactly seven entries.

- [ ] **Step 5: Implement and generate the source lock**

Use sorted POSIX-relative paths and lowercase SHA-256 digests:

```python
def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def build_source_lock(files: Iterable[Path], *, relative_to: Path) -> dict[str, object]:
    return {
        "schema_version": 1,
        "source_repository": SOURCE_REPOSITORY,
        "source_commit": SOURCE_COMMIT,
        "files": {
            path.relative_to(relative_to).as_posix(): sha256_file(path)
            for path in sorted(files)
        },
    }
```

Add a module CLI with `--write`; run:

```bash
uv run --project gazebo python -m onerobotics_a1_gazebo.source_lock --write gazebo/generated-manifest.json
uv run --project gazebo python -m onerobotics_a1_gazebo.source_lock --check gazebo/generated-manifest.json
```

Expected: `SOURCE_LOCK_OK: 39 files (33 STL, 3 URDF, 3 metadata/document files)`.

- [ ] **Step 6: Run focused tests and lock dependencies**

Run:

```bash
uv lock --project gazebo
uv run --project gazebo pytest gazebo/tests/test_spec.py gazebo/tests/test_source_lock.py -q
```

Expected: all tests pass and `gazebo/uv.lock` is created.

- [ ] **Step 7: Commit**

```bash
git add gazebo/pyproject.toml gazebo/uv.lock gazebo/config gazebo/generated-manifest.json \
  gazebo/onerobotics_a1_gazebo gazebo/tests/test_spec.py gazebo/tests/test_source_lock.py
git commit -m "feat: lock public A1 Gazebo source inputs"
```

---

### Task 2: Strict native SDF 1.11 conversion and topology preservation

**Files:**
- Create: `gazebo/onerobotics_a1_gazebo/sdf.py`
- Create: `gazebo/tests/test_sdf_conversion.py`

**Interfaces:**
- Consumes: `ModelSpec`, `JointLimitOverlay`, source URDF paths.
- Produces: `convert_urdf(spec, overlay) -> ElementTree`, `serialize_sdf(tree) -> bytes`, `source_joint_index(name) -> int`.

- [ ] **Step 1: Write failing right/left/bimanual conversion tests**

Test all topological and pose rules directly against the generated XML:

```python
def test_right_arm_has_world_anchor_and_seven_revolute_joints():
    model = converted_model("right_arm")
    assert model.attrib["name"] == "onerobotics_a1_right_arm"
    assert model.attrib["canonical_link"] == "base_link"
    assert model.find("canonical_link") is None
    assert model.find("static") is None
    assert [link.attrib["name"] for link in model.findall("link")] == [
        "base_link", "Link1", "Link2", "Link3", "Link4", "Link5", "Link6", "Link7"
    ]
    anchor = model.find("joint[@name='world_to_base']")
    assert anchor is not None and anchor.attrib["type"] == "fixed"
    assert anchor.findtext("parent") == "world"
    assert anchor.findtext("child") == "base_link"
    assert len(model.findall("joint[@type='revolute']")) == 7


def test_left_arm_drops_only_virtual_world_link():
    model = converted_model("left_arm")
    assert model.find("link[@name='world']") is None
    assert len(model.findall("link")) == 8
    assert model.find("joint[@name='world_to_base']/parent").text == "world"


def test_bimanual_fixed_shoulders_are_not_lumped():
    model = converted_model("bimanual_stand")
    assert len(model.findall("link")) == 17
    assert len(model.findall("joint[@type='revolute']")) == 14
    assert {joint.attrib["name"] for joint in model.findall("joint[@type='fixed']")} == {
        "world_to_base", "joint_r0", "joint_l0"
    }
    assert model.find("link[@name='Link_r0']") is not None
    assert model.find("link[@name='Link_l0']") is not None


def test_joint_pose_is_the_source_origin_and_child_uses_joint_frame():
    model = converted_model("right_arm")
    joint_pose = model.find("joint[@name='joint2-a1_r']/pose")
    assert joint_pose.attrib["relative_to"] == "Link1"
    assert joint_pose.text == "-0.0 0.0 0.12745 1.5708 1.5708 -0.0"
    child_pose = model.find("link[@name='Link2']/pose")
    assert child_pose.attrib["relative_to"] == "joint2-a1_r"
    assert child_pose.text == "0 0 0 0 0 0"


def test_mesh_and_color_are_portable_and_preserved():
    model = converted_model("right_arm")
    visual = model.find("link[@name='Link1']/visual[@name='Link1_visual_0']")
    assert visual.findtext("geometry/mesh/uri") == "meshes/Link1.STL"
    assert visual.findtext("material/ambient") == "0.898039215686275 0.917647058823529 0.929411764705882 1"
    assert visual.findtext("material/diffuse") == visual.findtext("material/ambient")
```

- [ ] **Step 2: Run the tests and verify the missing converter failure**

Run:

```bash
uv run --project gazebo pytest gazebo/tests/test_sdf_conversion.py -q
```

Expected: import fails because `onerobotics_a1_gazebo.sdf` does not exist.

- [ ] **Step 3: Implement strict URDF parsing and allow lists**

Parse with `ElementTree`; reject a root other than `<robot>`, duplicate names,
unknown geometry, and unsupported root children other than `link`, `joint`, and
the two exact source-only `mujoco` compiler hints. Validate every supported
element's attributes, children, and significant text fail-closed. The locked
bimanual source contains the exporter artifact `<axis xyz="0 0 0"/>` on fixed
`joint_r0` and `joint_l0`; accept and omit only that exact form, while rejecting
every other fixed-joint axis and all fixed-joint limits. Use these helpers:

```python
def pose_text(origin: Element | None) -> str:
    xyz = "0 0 0" if origin is None else origin.attrib.get("xyz", "0 0 0")
    rpy = "0 0 0" if origin is None else origin.attrib.get("rpy", "0 0 0")
    return f"{' '.join(xyz.split())} {' '.join(rpy.split())}"


def add_text(parent: Element, tag: str, text: str, **attributes: str) -> Element:
    child = SubElement(parent, tag, attributes)
    child.text = text
    return child


def source_joint_index(name: str) -> int:
    match = re.search(r"([1-7])(?:-a1_[rl])?$", name)
    if match is None:
        raise ValueError(f"cannot map actuated joint to motor index: {name}")
    return int(match.group(1))
```

- [ ] **Step 4: Implement link, geometry, and material conversion**

For each physical link, copy inertial pose/mass/tensor; convert each source
visual and collision in source order; rename them `<link>_visual_<index>` and
`<link>_collision_<index>`; rewrite only the mesh URI to
`meshes/<basename>`; map URDF RGBA to both SDF `ambient` and `diffuse`.

Use the source joint graph to write each non-root link pose as identity relative
to its source joint. Preserve the exact whitespace-normalized source decimal
strings on that joint's pose relative to its parent link.
Fail if a mesh basename collides with another different source path.

- [ ] **Step 5: Implement joint conversion and hardware limits**

Create each source fixed or revolute joint without letting libsdformat perform
fixed-joint reduction. The joint frame is the child-link frame:

```python
joint_out = SubElement(model, "joint", {"name": name, "type": joint_type})
add_text(joint_out, "pose", source_origin, relative_to=parent_name)
add_text(joint_out, "parent", "world" if parent_name == "world" else parent_name)
add_text(joint_out, "child", child_name)
if joint_type == "revolute":
    axis_out = SubElement(joint_out, "axis")
    add_text(axis_out, "xyz", normalized_source_axis)
    limit_out = SubElement(axis_out, "limit")
    add_text(limit_out, "lower", source_lower)
    add_text(limit_out, "upper", source_upper)
    limits = overlay[source_joint_index(name)]
    add_text(limit_out, "effort", format_number(limits.effort))
    add_text(limit_out, "velocity", format_number(limits.velocity))
```

For source models without a world anchor, add `world_to_base` as a fixed joint
whose zero pose is relative to `base_link`; this is the only case where the
joint pose is not expressed relative to its parent, because literal `world` is
valid in `<parent>` but invalid in `pose/@relative_to`. For the left model, map
its virtual `world` parent to SDF `world`, retain the source name
`world_to_base`, and fold any non-identity source world-joint origin into the
root link pose. Never emit a physical `world` link, a child named `world`, or
`relative_to="world"`.

- [ ] **Step 6: Serialize deterministically and run all conversion tests**

Use UTF-8, XML declaration, LF line endings, four-space indentation, and a
single final newline. Run:

```bash
uv run --project gazebo pytest gazebo/tests/test_sdf_conversion.py -q
uv run --project gazebo ruff check gazebo/onerobotics_a1_gazebo/sdf.py gazebo/tests/test_sdf_conversion.py
uv run --project gazebo ruff format --check gazebo/onerobotics_a1_gazebo/sdf.py gazebo/tests/test_sdf_conversion.py
```

Expected: all conversion tests pass; ruff exits 0.

- [ ] **Step 7: Commit**

```bash
git add gazebo/onerobotics_a1_gazebo/sdf.py gazebo/tests/test_sdf_conversion.py
git commit -m "feat: convert A1 URDF models to native SDF"
```

---

### Task 3: Fuel package export, provenance, and deterministic archives

**Files:**
- Create: `gazebo/onerobotics_a1_gazebo/package.py`
- Create: `gazebo/templates/model.config.xml`
- Create: `gazebo/templates/metadata.pbtxt`
- Create: `gazebo/templates/README.md`
- Create: `gazebo/templates/NOTICE`
- Create: `gazebo/tests/test_package_export.py`
- Modify: `.gitignore`

**Interfaces:**
- Consumes: `load_model_specs()`, `convert_urdf()`, source lock.
- Produces: `export_models(output_root: Path) -> tuple[ExportedModel, ...]`, `create_reproducible_archive(model_dir, archive_path) -> str` (SHA-256), CLI `python -m onerobotics_a1_gazebo.package`.

- [ ] **Step 1: Write failing self-contained package tests**

```python
def test_exported_packages_are_atomic_and_self_contained(tmp_path):
    exported = export_models(tmp_path / "dist")
    assert [item.directory.name for item in exported] == [
        "onerobotics_a1_right_arm", "onerobotics_a1_left_arm", "onerobotics_a1_bimanual_stand"
    ]
    assert [len(list((item.directory / "meshes").glob("*.STL"))) for item in exported] == [8, 8, 17]
    for item in exported:
        assert {"model.config", "model.sdf", "metadata.pbtxt", "README.md", "LICENSE", "NOTICE",
                "SOURCE_MANIFEST.json"} <= {path.name for path in item.directory.iterdir()}
        assert not any(path.is_symlink() for path in item.directory.rglob("*"))
        sdf = (item.directory / "model.sdf").read_text(encoding="utf-8")
        assert str(REPO_ROOT) not in sdf
        assert "../" not in sdf


def test_package_provenance_and_license_are_explicit(tmp_path):
    item = export_models(tmp_path / "dist")[0]
    notice = (item.directory / "NOTICE").read_text(encoding="utf-8")
    metadata = (item.directory / "metadata.pbtxt").read_text(encoding="utf-8")
    assert "Copyright © 2026 OneRobotics" in notice
    assert "CC BY 4.0" in notice
    assert "ecf530911284ba0e559f7a24dc222fd8e60d31ed" in notice
    assert "URDF-to-SDF conversion" in notice
    assert "Creative Commons Attribution 4.0" in metadata


def test_archives_are_byte_reproducible(tmp_path):
    first = export_models(tmp_path / "first")
    second = export_models(tmp_path / "second")
    assert [item.archive_sha256 for item in first] == [item.archive_sha256 for item in second]
    assert [item.archive.read_bytes() for item in first] == [item.archive.read_bytes() for item in second]
```

- [ ] **Step 2: Run the tests and verify `export_models` is missing**

Run:

```bash
uv run --project gazebo pytest gazebo/tests/test_package_export.py -q
```

Expected: import fails for `onerobotics_a1_gazebo.package`.

- [ ] **Step 3: Add exact Fuel metadata and attribution templates**

`model.config` must name `model.sdf`, declare SDF 1.11, version `1.0.0`, author
`OneRobotics`, and the model-specific description. Do not invent an author
email.

`metadata.pbtxt` must use the Fuel Tools 9 field names confirmed against
`gz fuel meta --config2pbtxt`, explicitly select `Creative Commons Attribution
4.0`, contain model tags `robot`, `manipulator`, `onerobotics`, and set the
matching model name and version.

`NOTICE` must contain this changes statement:

```text
Changes: Converted the published URDF description to native SDFormat 1.11;
anchored the fixed-base mechanism to the SDF world frame; rewrote mesh paths
for a self-contained Fuel package; overlaid the published peak effort and rated
velocity limits from model_parameters.yaml. Geometry, mass, inertia, joint
poses, axes, and position limits were not changed.
```

Copy `LICENSES/CC-BY-4.0.txt` byte-for-byte into each package as `LICENSE`.
Write Fuel's exact accepted license name in `metadata.pbtxt`:

```protobuf
model {
  file: "model.sdf"
  file_format {
    version { major: 1 minor: 11 }
    name: "sdf"
  }
}
legal {
  copyright: "Copyright © 2026 OneRobotics"
  license: "Creative Commons Attribution 4.0 International"
}
```

Do not use an SPDX shorthand in this field and do not omit `legal`: Fuel Tools
would otherwise select a different license. Keep archives outside each model
root because Fuel uploads every file below the selected model directory.

- [ ] **Step 4: Implement atomic export and per-package manifests**

Create a temporary staging directory inside the output parent, validate the
source lock before reading assets, export all files, then rename the complete
staging tree into place. Refuse output `/`, the repository root, the user's home
directory, or an existing non-generated directory.

`SOURCE_MANIFEST.json` must contain schema version, source URL, source commit,
model key, changes statement, and sorted `{path, sha256}` objects for the
source URDF, model YAML, and copied meshes. Copy mesh bytes with `shutil.copyfile`.

- [ ] **Step 5: Implement deterministic `.tar.gz` creation**

Sort every path; set archive path names to `<slug>-1.0.0/<relative-path>`;
normalize `uid`, `gid`, `uname`, `gname`, and `mtime` to zero; use file mode
`0o644` and directory mode `0o755`; create gzip with `filename=""` and
`mtime=0`:

```python
with archive_path.open("wb") as raw:
    with gzip.GzipFile(filename="", mode="wb", fileobj=raw, mtime=0) as zipped:
        with tarfile.open(fileobj=zipped, mode="w", format=tarfile.PAX_FORMAT) as archive:
            for path in sorted(model_dir.rglob("*")):
                add_normalized_tar_entry(archive, path, arc_root / path.relative_to(model_dir))
```

- [ ] **Step 6: Ignore generated output and run export tests**

Append only `/dist/` to `.gitignore`. Run:

```bash
uv run --project gazebo pytest gazebo/tests/test_package_export.py -q
uv run --project gazebo python -m onerobotics_a1_gazebo.package --output dist/gazebo-fuel
test -z "$(find dist/gazebo-fuel -type l -print -quit)"
```

Expected: three package directories and three `archives/*-1.0.0.tar.gz` files;
no symlink output.

- [ ] **Step 7: Commit**

```bash
git add .gitignore gazebo/templates gazebo/onerobotics_a1_gazebo/package.py gazebo/tests/test_package_export.py
git commit -m "feat: export self-contained A1 Fuel packages"
```

---

### Task 4: Pure-data validation and fail-closed CLI

**Files:**
- Create: `gazebo/onerobotics_a1_gazebo/validate.py`
- Create: `gazebo/tests/test_validation.py`

**Interfaces:**
- Consumes: exported package paths and model specs.
- Produces: `ValidationIssue`, `validate_package(path, spec) -> tuple[ValidationIssue, ...]`, `validate_all(path) -> None`, CLI exit 0 on success / 1 on validation failure.

- [ ] **Step 1: Write failing corruption tests**

Create one valid export per test and mutate a copied output. Assert the
validator catches each exact defect:

```python
@pytest.mark.parametrize(
    ("mutate", "message"),
    [
        (remove_one_mesh, "missing mesh"),
        (zero_bimanual_effort, "effort limit mismatch"),
        (remove_fixed_shoulder, "fixed-joint topology mismatch"),
        (make_inertia_non_positive, "inertia is not positive definite"),
        (insert_absolute_uri, "non-portable URI"),
        (remove_cc_by_metadata, "CC BY 4.0 metadata missing"),
        (corrupt_manifest_digest, "package source digest mismatch"),
    ],
)
def test_validator_rejects_corruption(tmp_path, mutate, message):
    package, spec = exported_bimanual_fixture(tmp_path)
    mutate(package)
    issues = validate_package(package, spec)
    assert any(message in issue.message for issue in issues)
```

- [ ] **Step 2: Run tests and verify the validator import failure**

```bash
uv run --project gazebo pytest gazebo/tests/test_validation.py -q
```

Expected: import fails for `onerobotics_a1_gazebo.validate`.

- [ ] **Step 3: Implement structural and portability checks**

Validate model name, SDF version, expected links, exact actuated and fixed
joints, parent/child topology, reserved world anchor, absence of `<static>`,
mesh existence, package containment after `resolve()`, no symlink, no absolute
URI, no `..`, and no plugin in the published `model.sdf`.

- [ ] **Step 4: Implement physical and provenance checks**

Parse all values as finite floats. Validate positive mass and positive-definite
symmetric inertia using Sylvester's criterion:

```python
minor_1 = ixx
minor_2 = ixx * iyy - ixy * ixy
determinant = (
    ixx * (iyy * izz - iyz * iyz)
    - ixy * (ixy * izz - iyz * ixz)
    + ixz * (ixy * iyz - iyy * ixz)
)
if min(minor_1, minor_2, determinant) <= 0:
    issue("inertia is not positive definite")
```

Validate lower < upper, zero within each position interval, and exact overlay
effort/velocity. Validate license text, metadata license, notice source URL /
commit / changes, and every package manifest digest.

- [ ] **Step 5: Implement CLI summary and run tests**

Success output must be stable and model-specific:

```text
VALID onerobotics_a1_right_arm: 8 links, 7 actuated joints, 8 meshes
VALID onerobotics_a1_left_arm: 8 links, 7 actuated joints, 8 meshes
VALID onerobotics_a1_bimanual_stand: 17 links, 14 actuated joints, 17 meshes
VALIDATION_OK: 3 models, 33 meshes, 28 actuated joints
```

Run:

```bash
uv run --project gazebo pytest gazebo/tests/test_validation.py -q
uv run --project gazebo python -m onerobotics_a1_gazebo.validate dist/gazebo-fuel
```

Expected: all tests pass and CLI prints the exact success footer.

- [ ] **Step 6: Commit**

```bash
git add gazebo/onerobotics_a1_gazebo/validate.py gazebo/tests/test_validation.py
git commit -m "test: validate A1 Fuel package integrity"
```

---

### Task 5: User-space Gazebo Harmonic environment and format gates

The original APT plan is intentionally superseded. The target host already has
Gazebo Classic 11 at `/usr/bin/gz`, passwordless sudo is unavailable, and
Miniforge is writable. Install Harmonic in a named conda-forge environment so
the work is executable without changing APT, `/usr/bin`, shell startup files,
or global Conda configuration.

**Files:**
- Create: `gazebo/environment-harmonic.yml`
- Create: `gazebo/scripts/harmonic_env.sh`
- Create: `gazebo/scripts/install_harmonic_conda.sh`
- Create: `gazebo/scripts/check_harmonic.sh`
- Create: `gazebo/onerobotics_a1_gazebo/harmonic.py`
- Create: `gazebo/tests/test_cli_scripts.py`
- Create: `gazebo/tests/test_harmonic_sdf.py`

**Interfaces:**
- Preserves: the existing Gazebo Classic 11 binary, target, bytes, and version.
- Produces: a named `onerobotics-a1-gz-harmonic` environment containing Gazebo
  Sim 8, SDFormat 14, Fuel Tools 9, and gz-tools 2.
- Runs every Harmonic command through `mamba run --clean-env`; it never activates
  the environment or changes the system/default `gz` command.
- Provides: local-only native-format and parsed-semantic gates. Neither uploads,
  downloads, nor otherwise calls a Fuel network API.

- [ ] **Step 1: Write failing script-contract tests**

Assert the environment file has the exact name, `conda-forge` plus `nodefaults`,
and the four major pins `gz-tools2=2.*`, `gz-sim8=8.*`,
`gz-fuel-tools9=9.*`, and `libsdformat14=14.*`. Assert all scripts use
`set -euo pipefail`, resolve paths relative to themselves, and contain none of
`sudo`, APT/repository mutations, `/etc` or `/usr/bin` writes, shell-RC edits,
`conda config`, environment activation, package removal, upload, download,
token, or Fuel configuration commands.

Use an injected fake `A1_MAMBA_BIN` to prove:

- a missing environment selects `env create` with the checked-in YAML;
- an existing but stale environment selects `env update` without pruning;
- an already-correct environment exits without mutation;
- missing mamba and wrong/missing component majors fail with actionable errors;
- paths containing spaces remain one quoted argument; and
- every execution uses the fixed environment name and `run --clean-env`.

- [ ] **Step 2: Run tests and record the missing-script failure**

```bash
env -u PYTHONPATH uv run --project gazebo pytest \
  gazebo/tests/test_cli_scripts.py gazebo/tests/test_harmonic_sdf.py -q
```

Expected: imports/files are absent and the new tests fail before implementation.

- [ ] **Step 3: Implement the isolated, idempotent installer**

`environment-harmonic.yml` is the single dependency specification. The helper
script exposes `a1_find_mamba` and `a1_harmonic_run`, supports test/user overrides
through `A1_MAMBA_BIN` and `A1_HARMONIC_ENV_NAME`, and does not depend on the
caller's current directory.

The installer supports the verified Linux x86-64 path, uses `mamba env create`
or `mamba env update` with `--no-rc`, the checked-in YAML, and strict
conda-forge-only resolution, and never uses `--prune`. Before and after any
transaction it probes and parses the requested majors rather than trusting exit
status alone:

```bash
a1_harmonic_run gz sim --force-version 8 --versions
a1_harmonic_run gz sdf --force-version 14 --versions
a1_harmonic_run gz fuel --force-version 9 --versions
```

If all probes already pass, print `HARMONIC_ALREADY_READY` and make no change.
Record the resolved `/usr/bin/gz` target, SHA-256 digests of `/usr/bin/gz` and
`/usr/bin/gazebo`, and the Classic version before installation when present;
require all recorded values to be identical afterward before printing
`CLASSIC_UNCHANGED`.

- [ ] **Step 4: Implement native format and parsed-semantic gates**

First run the pure-data validator and require exactly the three locked model
slugs. In a trap-cleaned temporary directory, run for each model:

```bash
a1_harmonic_run gz sdf --force-version 14 -k "$model_dir/model.sdf"
a1_harmonic_run gz sdf --force-version 14 -p "$model_dir/model.sdf" \
  > "$temporary_dir/$slug.parsed.sdf"
a1_harmonic_run gz sdf --force-version 14 -k \
  "$temporary_dir/$slug.parsed.sdf"
a1_harmonic_run gz fuel --force-version 9 meta \
  --pbtxt2config "$model_dir/metadata.pbtxt" \
  > "$temporary_dir/$slug.model.config"
```

Require both generated files to be non-empty. Redirect HOME/XDG cache state to
the temporary directory when invoking Fuel metadata conversion.

Pass each pretty-printed SDF to `harmonic.py`. Because SDFormat may insert
defaults, do not compare its bytes or full XML signature. Require exactly one
correctly named model and compare all expected links, joint names/types,
parent/child topology, the world anchor, both fixed shoulders, and every
actuated lower/upper/effort/velocity limit with the immutable source-derived
expectation. Missing, extra, or mutated semantics must fail closed.

Print exactly one `HARMONIC_FORMAT_VALID <slug>` line per model followed by
`HARMONIC_FORMAT_OK: 3 models`.

- [ ] **Step 5: Install and verify the actual host**

```bash
bash -n gazebo/scripts/harmonic_env.sh \
  gazebo/scripts/install_harmonic_conda.sh gazebo/scripts/check_harmonic.sh
env -u PYTHONPATH uv run --project gazebo pytest \
  gazebo/tests/test_cli_scripts.py gazebo/tests/test_harmonic_sdf.py -q
bash gazebo/scripts/install_harmonic_conda.sh
bash gazebo/scripts/check_harmonic.sh dist/gazebo-fuel
env -u PYTHONPATH uv run --project gazebo pytest gazebo/tests -q
env -u PYTHONPATH uv run --project gazebo ruff check gazebo
env -u PYTHONPATH uv run --project gazebo ruff format --check gazebo
```

Expected: all three exact component-major probes pass, all three packages pass
native format and parsed-semantic validation, Classic's target/hash/version are
unchanged, no global/system configuration is changed, and no Fuel upload occurs.
Record the concrete solved versions in the task report. Runtime physics/plugin
and Ogre rendering remain explicit Tasks 6 and 7 gates.

- [ ] **Step 6: Commit**

```bash
git add gazebo/environment-harmonic.yml gazebo/scripts/harmonic_env.sh \
  gazebo/scripts/install_harmonic_conda.sh gazebo/scripts/check_harmonic.sh \
  gazebo/onerobotics_a1_gazebo/harmonic.py \
  gazebo/tests/test_cli_scripts.py gazebo/tests/test_harmonic_sdf.py
git commit -m "build: install and gate Gazebo Harmonic safely"
```

---

### Task 6: Gazebo-native demo worlds and commanded-motion smoke tests

**Files:**
- Modify: `gazebo/pyproject.toml`
- Create: `gazebo/onerobotics_a1_gazebo/demo.py`
- Create: `gazebo/scripts/run_smoke_tests.sh`
- Create: `gazebo/tests/test_demo.py`
- Create: `gazebo/tests/test_runtime.py`
- Generate: `gazebo/worlds/onerobotics_a1_right_arm_demo.sdf`
- Generate: `gazebo/worlds/onerobotics_a1_left_arm_demo.sdf`
- Generate: `gazebo/worlds/onerobotics_a1_bimanual_stand_demo.sdf`

**Interfaces:**
- Consumes: validated, controller-free exported models and their source limits.
- Produces: one standalone world per model,
  `parse_joint_position(payload, joint_name) -> float`, and
  `smoke_test(world, joints) -> SmokeResult`.

- [ ] **Step 1: Write failing world-generation tests**

Assert each generated world has Physics, UserCommands, and SceneBroadcaster
world systems and exactly one `<include>` whose URI is `model://<slug>`. Every
actuated joint gets one include-scoped controller with exact identifiers:

```xml
<plugin filename="gz-sim-joint-position-controller-system"
        name="gz::sim::systems::JointPositionController">
  <joint_name>joint1-a1_r</joint_name>
  <joint_index>0</joint_index>
  <topic>/a1_demo/onerobotics_a1_right_arm/joint1-a1_r/cmd_pos</topic>
  <use_velocity_commands>true</use_velocity_commands>
  <cmd_max>0.1</cmd_max>
  <cmd_min>-0.1</cmd_min>
  <initial_position>0</initial_position>
</plugin>
```

Add one include-scoped state publisher and explicitly list every actuated joint:

```xml
<plugin filename="gz-sim-joint-state-publisher-system"
        name="gz::sim::systems::JointStatePublisher">
  <joint_name>joint1-a1_r</joint_name>
</plugin>
```

Gazebo Sim 8.10's `JointStatePublisher` supports repeated `joint_name` and an
optional `topic`, but does not consume `update_rate`; do not emit that silently
ignored parameter. Register the `runtime` pytest marker in `pyproject.toml`.

Assert every controller uses the conservative `0.1 rad/s` smoke-test cap and
does not claim that the hardware PD values are validated Gazebo dynamics. Also
assert no plugin is added back to any published `dist/.../model.sdf`.

- [ ] **Step 2: Run tests and record the missing demo module failure**

```bash
uv run --project gazebo pytest gazebo/tests/test_demo.py -q
```

Expected: import fails for `onerobotics_a1_gazebo.demo`.

- [ ] **Step 3: Implement deterministic standalone world generation**

Generate an SDF 1.11 world whose `<include>` uses the local
`model://<slug>` resource and inject controller plugins inside that include.
This is the SDFormat-supported way to attach a model system without editing the
published package. Configure gravity, `max_step_size=0.001`, and
`real_time_update_rate=1000`. Explicitly select
`gz-physics-dartsim-plugin` inside the Physics system; the installed DART plugin
and its dependencies must be checked before the live test. Add a ground plane
and directional light outside the robot. Both must be inline SDF geometry /
light elements; they must not add another `model://` include or network
dependency. Serialize using the same deterministic XML policy as the converter.

Provide a CLI:

```bash
uv run --project gazebo python -m onerobotics_a1_gazebo.demo generate \
  --models dist/gazebo-fuel --output gazebo/worlds
```

- [ ] **Step 4: Write the runtime test before its implementation**

Mark it `runtime` and parameterize one representative revolute joint per
package. It must start a unique server, observe an initial finite position,
publish a small target inside the source position limits, observe a later
finite position, and require motion toward the target:

```python
@pytest.mark.runtime
@pytest.mark.parametrize(
    ("model", "joint", "target"),
    [
        ("onerobotics_a1_right_arm", "joint1-a1_r", 0.05),
        ("onerobotics_a1_left_arm", "joint1-a1_l", 0.05),
        ("onerobotics_a1_bimanual_stand", "joint_r1", 0.05),
    ],
)
def test_position_command_moves_joint_toward_target(model, joint, target):
    result = smoke_test(model=model, joint=joint, target=target)
    assert result.finite
    assert result.moved_toward_target
    assert result.delta >= 0.02
    assert result.lower <= result.final_position <= result.upper
```

Run it now; expected failure is the absent `smoke_test` implementation, not a
silent skip.

Add pure unit cases for `parse_joint_position()` using captured-shape
`gz.msgs.Model` JSON. It must select `joint[]` by exact `name`, then read the
scalar `joint[].axis1.position`. Proto3 JSON omits a scalar whose value is zero,
so a present `axis1` object with no `position` means exactly `0.0`; a missing or
non-object `axis1`, Boolean/non-number, NaN, and infinity must fail. Never rely
on joint array order. A subprocess timeout must surface as
`RuntimeError("joint-state timeout")`.

- [ ] **Step 5: Implement bounded process and transport orchestration**

Use a unique `GZ_PARTITION=onerobotics-a1-smoke-<pid>-<nonce>`, set
`GZ_SIM_RESOURCE_PATH` to the absolute export root, and launch:

```bash
world_path="$PWD/gazebo/worlds/onerobotics_a1_right_arm_demo.sdf"
gz sim --force-version 8 -s -r -v 4 "$world_path"
```

Poll `gz topic --force-version 13 -l` for the command topic and the default state topic
`/world/a1_demo/model/<slug>/joint_state` with a 30-second monotonic deadline.
Confirm the state publisher advertises `gz.msgs.Model`. Capture one state using
the complete command `gz topic --force-version 13 -e -n 1 --json-output -t
"$state_topic"`, then parse it through the unit-tested helper; publish a
`gz.msgs.Double` target using `gz topic --force-version 13 -t ... -m
gz.msgs.Double -p 'data: 0.05'`; then capture states until the joint moves at
least `0.02 rad` toward the target without exceeding its position limits.
Terminate the process first through `/server_control` with the exact Transport
13 request:

```bash
gz service --force-version 13 -s /server_control \
  --reqtype gz.msgs.ServerControl --reptype gz.msgs.Boolean \
  --timeout 2000 --req 'stop: true'
```

Gazebo Transport returns status 0 even when a service call times out, so require
the response to contain `data: true`. Fall back to process-group SIGTERM, wait
five seconds, then SIGKILL only if still alive. On failure include the bounded
server log and topic list; never leave a server running. Every Sim, topic, and
service command must run through the isolated Task 5 helper with the same
`GZ_PARTITION` and absolute `GZ_SIM_RESOURCE_PATH` injected *inside* the clean
Mamba environment.

- [ ] **Step 6: Run unit and live Harmonic tests**

```bash
env -u PYTHONPATH uv run --project gazebo pytest gazebo/tests/test_demo.py -q
env -u PYTHONPATH uv run --project gazebo pytest gazebo/tests/test_runtime.py -m runtime -q -s
bash gazebo/scripts/run_smoke_tests.sh dist/gazebo-fuel gazebo/worlds
```

Expected: each of the three models loads and its representative joint responds
to a Gazebo Transport command. Treat a physics warning, missing plugin, missing
topic, NaN, server crash, or timeout as failure.

- [ ] **Step 7: Commit**

```bash
git add gazebo/pyproject.toml gazebo/onerobotics_a1_gazebo/demo.py \
  gazebo/scripts/run_smoke_tests.sh \
  gazebo/tests/test_demo.py gazebo/tests/test_runtime.py gazebo/worlds
git commit -m "feat: add Gazebo Harmonic A1 motion demos"
```

---

### Task 7: Reproducible Gazebo thumbnails

**Files:**
- Create: `gazebo/onerobotics_a1_gazebo/render.py`
- Create: `gazebo/onerobotics_a1_gazebo/thumbnail.py`
- Create: `gazebo/scripts/render_thumbnails.sh`
- Create: `gazebo/tests/test_render.py`
- Create: `gazebo/config/render.yaml`
- Create: `gazebo/assets/thumbnails/onerobotics_a1_right_arm.png`
- Create: `gazebo/assets/thumbnails/onerobotics_a1_left_arm.png`
- Create: `gazebo/assets/thumbnails/onerobotics_a1_bimanual_stand.png`
- Create: `gazebo/assets/thumbnails/render-manifest.json`
- Modify: `gazebo/pyproject.toml`
- Modify: `gazebo/onerobotics_a1_gazebo/demo.py`
- Modify: `gazebo/onerobotics_a1_gazebo/package.py`
- Modify: `gazebo/onerobotics_a1_gazebo/validate.py`
- Modify: `gazebo/tests/test_runtime.py`
- Modify: `gazebo/tests/test_package_export.py`
- Modify: `gazebo/tests/test_validation.py`
- Generate into each package: `thumbnails/0.png`

**Interfaces:**
- Produces: `build_render_world(model_dir, output_path, frame_dir) -> Path`,
  `select_frame(frame_dir) -> Path`, a render-only candidate command, an
  approval-only command, and three 512x512 PNG thumbnails.

- [ ] **Step 1: Write failing render-world and PNG tests**

Assert each render world contains the controller-free model, a neutral ground
plane, two lights, and exactly one camera sensor with a deterministic pose and:

```xml
<camera>
  <horizontal_fov>0.9</horizontal_fov>
  <image><width>512</width><height>512</height><format>R8G8B8</format></image>
  <save enabled="true"><path>ABSOLUTE_TEMP_FRAME_DIRECTORY</path></save>
</camera>
```

The world must enable the Sensors system with Ogre2. PNG validation must inspect
the PNG signature, exact IHDR, chunk bounds and CRCs, bounded decompressed RGB
scanlines, IDAT, and unique terminal IEND with the standard library. Reject
anything other than complete 512x512 8-bit truecolor. The export test must prove that package
`thumbnails/0.png` exactly matches its checked-in reviewed source thumbnail.
No camera or Sensors plugin may appear in published `model.sdf`.

Strictly parse `render.yaml`: it contains exactly the three locked slugs and
finite, non-Boolean camera, ground, and two-light values. Reject missing,
extra, duplicate-key, non-finite, and wrong-length configuration. Register a
`render_runtime` pytest marker; live render cases carry both `runtime` and
`render_runtime` and never skip silently.

- [ ] **Step 2: Run tests and record the missing render module failure**

```bash
uv run --project gazebo pytest gazebo/tests/test_render.py -q
```

Expected: imports fail for the absent render / thumbnail modules.

- [ ] **Step 3: Write failing runtime, candidate, approval, and package tests**

Before runtime implementation, prove the required ordering and failure
behavior with pure orchestration tests. The server starts paused; an exact
camera topic and `gz.msgs.Image` publisher must appear; a subscriber process
must start and become visible before world unpause. World-control and shutdown
responses succeed only on a semantic `data: true`. Cover deadlines, early
exit, fatal Sensors/Ogre2/mesh/load logs, subscriber and server process-group
cleanup on every exception, incomplete third-frame races, and atomic output
with no success line until all models pass.

Candidate rendering and approval are separate trust boundaries. `render`
creates an exact candidate set and never changes checked-in assets. `approve`
accepts the exact already-reviewed candidate root, never starts Gazebo, takes
one descriptor-safe immutable snapshot, validates its manifest and PNG hashes,
and atomically installs that same byte set. Cover missing / extra entries,
symlinks, FIFOs, path replacement, hash drift, interrupted install rollback,
and prove approval does not call `Popen` or rerender.

Package / validator tests must require exact `thumbnails/0.png` bytes from one
approved-assets snapshot, archive exactness, and deterministic repeat exports.
The validator must reject a missing, extra, unsafe, corrupt, or source-mismatch
thumbnail. This necessarily modifies `validate.py` and its tests; otherwise
the current exact package inventory would reject Task 7's own output.

- [ ] **Step 4: Implement render worlds and bounded headless capture**

Read model-specific public camera and light poses from `render.yaml`, generate
the absolute camera-save path only in a private temporary runtime world, and
run a unique-partition paused server with deterministic software rendering:

```bash
gz sim --force-version 8 -s --headless-rendering -v 4 "$render_world"
```

Gazebo Sim 8.10 disables an unconnected rendering sensor even when camera save
is enabled. Therefore do **not** add `-r`: wait for the exact camera topic,
start `gz topic --force-version 13 -e -n 3 --json-output`, confirm the
subscriber is registered, and only then call `/world/a1_render/control` with
`pause: false`. After at least three complete valid frames, call the same
service with `pause: true` before stopping the server so shutdown cannot race
new frame writes. Select the greatest strict numeric filename suffix from one
stable frame snapshot; never select by mtime or lexicographic order and never
reopen a path after selection.

Every Sim, topic, and service command uses Task 5's helper and Task 6's
reviewed process-group lifecycle. Extend the Task 6 argv helper only with a
backward-compatible fixed inner-environment argument. Because Mamba uses
`env -i --clean-env`, inject these values **inside** `a1_harmonic_run env`:

```text
QT_QPA_PLATFORM=offscreen
LIBGL_ALWAYS_SOFTWARE=1
HOME=<private-run-root>/home
XDG_CACHE_HOME=<private-run-root>/cache
```

Do not set `MESA_LOADER_DRIVER_OVERRIDE=llvmpipe`; this machine does not
provide `llvmpipe_dri.so`. Use zero gravity in the render-only world so a
controller-free fixed-base model remains at its reviewed zero pose. The
45-second monotonic deadline includes topic discovery, subscriber connection,
unpause, and three complete frames. Always clean subscriber, transient helper,
and server groups on success, failure, or interruption.

The default candidate root is a printed, preserved `mktemp` directory. It
contains exactly three `<slug>.png` files plus `render-manifest.json`. The
manifest has no time or absolute path; it records schema, Ogre2, software
backend, render-config SHA-256, each PNG SHA-256 / dimensions, and each rendered
`model.sdf` SHA-256.

- [ ] **Step 5: Render, visually inspect, then approve the same bytes**

```bash
uv run --project gazebo pytest gazebo/tests/test_render.py -q
candidate_parent="$(mktemp -d)"
candidate_root="$candidate_parent/candidates"
bash gazebo/scripts/render_thumbnails.sh render \
  --models dist/gazebo-fuel --candidates "$candidate_root"
# Open and review exactly the three PNGs under "$candidate_root".
bash gazebo/scripts/render_thumbnails.sh approve \
  --candidates "$candidate_root"
uv run --project gazebo python -m onerobotics_a1_gazebo.package --output dist/gazebo-fuel
uv run --project gazebo python -m onerobotics_a1_gazebo.validate dist/gazebo-fuel
```

Open each PNG with the image viewer. Require the complete robot to be in frame,
upright, visibly lit, not clipped, and not blank. If visual review fails, adjust
only the public render-world camera / lights and repeat; never change model
geometry or pose data. Approval must copy the already-viewed candidate snapshot;
it must never rerender an unreviewed replacement.

- [ ] **Step 6: Rebuild archives, revalidate, and commit render tooling**

Require `package.py` to copy the reviewed thumbnail into `thumbnails/0.png`,
and require `validate.py` to compare that package byte-for-byte with the
approved source and with the archive. Verify two complete package/archive builds
compare byte-for-byte. Commit render code, strict PNG / approval code, render
config, manifest, reviewed PNGs, package / validator integration, and tests;
keep temporary worlds, generated Fuel directories, candidates, and frame
sequences outside the repository or ignored.

```bash
git add gazebo/pyproject.toml gazebo/onerobotics_a1_gazebo/demo.py \
  gazebo/onerobotics_a1_gazebo/render.py gazebo/onerobotics_a1_gazebo/thumbnail.py \
  gazebo/onerobotics_a1_gazebo/package.py gazebo/onerobotics_a1_gazebo/validate.py \
  gazebo/scripts/render_thumbnails.sh gazebo/tests/test_render.py \
  gazebo/tests/test_runtime.py gazebo/tests/test_package_export.py \
  gazebo/tests/test_validation.py gazebo/config/render.yaml gazebo/assets/thumbnails
git commit -m "feat: render A1 Gazebo Fuel thumbnails"
```

---

### Task 8: Zero-background operating and publication guide

**Files:**
- Create: `gazebo/README.md`
- Modify: `README.md`
- Create: `gazebo/tests/test_documentation.py`

- [ ] **Step 1: Write failing documentation-contract tests**

Require the guide to contain copy-pasteable commands for source-lock checking,
export, pure-data validation, Harmonic format checking, local insertion, live
motion smoke tests, thumbnail generation, license verification, one-model-at-a-
time private/public Fuel upload, API/raw-ZIP verification, and Fuel-cache
download verification. Reject `<TOKEN>`, `/home/woan`, any company-internal
host, an unpinned `gz sim`, or wording that claims an upload already occurred.

- [ ] **Step 2: Run tests and record the missing-guide failure**

```bash
uv run --project gazebo pytest gazebo/tests/test_documentation.py -q
```

- [ ] **Step 3: Write the beginner-first guide**

Start with a five-line explanation of repository, URDF, SDF, Gazebo, and Fuel.
Then give this workflow in order:

1. `git clone` the public baseline and check its immutable commit.
2. Install Harmonic side-by-side and verify both Gazebo versions.
3. Build all three packages and run source, structure, SDF, Fuel, and runtime
   checks.
4. Add the export parent to `GZ_SIM_RESOURCE_PATH` for local use and open each
   checked-in demo world.
5. Explain that independent arms and the bimanual stand are separate CAD
   revisions and must not be assembled together.
6. State limitations: high-poly collision meshes, no gripper, no ROS 2 control,
   no sensors, and no invented dynamics / friction.
7. Explain that publishing needs a Gazebo Fuel account, a valid token, ownership
   rights, and explicit OneRobotics organizational authorization.

Use an environment variable read without printing it:

```bash
read -rsp 'Fuel token: ' GZ_FUEL_TOKEN; export GZ_FUEL_TOKEN; echo
gz fuel --force-version 9 upload \
  --url https://fuel.gazebosim.org \
  --model "$model_dir" \
  --owner "$fuel_owner" \
  --private \
  --header "Private-Token: ${GZ_FUEL_TOKEN}"
unset GZ_FUEL_TOKEN
```

Document public upload as the same single-model command without `--private`,
only after an authorized reviewer approves the exact archive hashes. Before
upload, query `https://fuel.gazebosim.org/1.0/licenses` and verify the server
still exposes `Creative Commons Attribution 4.0 International`. Never put a
real token in a command, file, test fixture, terminal transcript, or Git commit.

For post-upload verification, document both methods:

- Download the versioned raw Fuel ZIP and compare its file hashes against the
  pre-upload external manifest.
- Download through `gz fuel --force-version 9 download`, then rerun metadata,
  SDF, structural, and runtime checks against the cache. Do not demand bytewise
  equality from the cache because Fuel may rewrite resource URIs / XML.

- [ ] **Step 4: Link the guide and run documentation tests**

Add a short root README section pointing to `gazebo/README.md`; do not rewrite
the upstream project overview.

```bash
uv run --project gazebo pytest gazebo/tests/test_documentation.py -q
```

- [ ] **Step 5: Commit**

```bash
git add README.md gazebo/README.md gazebo/tests/test_documentation.py
git commit -m "docs: explain A1 Gazebo build and Fuel publication"
```

---

### Task 9: Release-candidate verification and independent review

**Files:**
- Modify only files implicated by failed checks or review findings.

- [ ] **Step 1: Verify immutable inputs and repository hygiene**

```bash
git diff --exit-code ecf530911284ba0e559f7a24dc222fd8e60d31ed -- \
  source/h1_reach/h1_reach/assets/urdf/A1_2026
uv run --project gazebo python -m onerobotics_a1_gazebo.source_lock --check gazebo/generated-manifest.json
git diff --check
git status --short
```

The Git diff and source lock jointly confirm that no tracked or untracked source
asset byte changed.

- [ ] **Step 2: Run the complete automated suite**

```bash
uv run --project gazebo ruff check gazebo
uv run --project gazebo ruff format --check gazebo
uv run --project gazebo pytest gazebo/tests -m 'not runtime' -q
uv run --project gazebo pytest gazebo/tests/test_runtime.py -m runtime -q -s
uv run --project gazebo python -m onerobotics_a1_gazebo.package --output dist/gazebo-fuel
uv run --project gazebo python -m onerobotics_a1_gazebo.validate dist/gazebo-fuel
bash gazebo/scripts/check_harmonic.sh dist/gazebo-fuel
bash gazebo/scripts/run_smoke_tests.sh dist/gazebo-fuel gazebo/worlds
candidate_parent="$(mktemp -d)"
candidate_root="$candidate_parent/release-thumbnail-candidates"
bash gazebo/scripts/render_thumbnails.sh render \
  --models dist/gazebo-fuel --candidates "$candidate_root"
```

- [ ] **Step 3: Verify reproducibility, portability, and secrets**

Export twice into two temporary roots and compare every archive with `cmp`.
List every tar member, check gzip integrity, and confirm no member is absolute,
contains `..`, is a symlink, or lives outside the expected versioned root.
Require each archive to contain exactly one
`<slug>-1.0.0/thumbnails/0.png`; archive creation always happens after the
reviewed thumbnails exist in `gazebo/assets/thumbnails` and after the package
exporter has copied them.
Search tracked and generated text for `Private-Token:`, common token prefixes,
`/home/woan`, private hostnames, and absolute mesh paths. Confirm each model
root contains only files intended for Fuel and each thumbnail passes visual
review.

- [ ] **Step 4: Request independent code review and resolve findings**

Ask a fresh reviewer to compare the implementation against the design, this
plan, the 39-file source lock, the three topology signatures, the exact overlay
values, SDFormat 1.11 rules, Fuel metadata/license rules, subprocess cleanup,
and evidence from all checks. Reproduce each technical finding before changing
code, add a regression test first, implement the smallest correction, then
rerun the relevant focused and full suites.

- [ ] **Step 5: Produce the final local handoff record**

Record baseline commit, implementation commit, exact tool versions, archive
SHA-256 values, test counts, runtime joint deltas, thumbnail dimensions, and
the remaining external publication gate in the final response. Do not claim
the models are present on Fuel unless a user-authorized upload and independent
download verification actually occurred.
