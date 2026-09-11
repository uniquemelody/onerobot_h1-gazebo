# OneRobotics A1 Gazebo Harmonic / Fuel Integration Design

Date: 2026-08-31

## 1. Purpose

Create a reviewable, reproducible Gazebo Harmonic integration for the publicly
released OneRobotics A1 assets and prepare three self-contained model packages
for Gazebo Fuel:

1. OneRobotics A1 right arm;
2. OneRobotics A1 left arm;
3. OneRobotics A1 bimanual robot with stand.

The implementation will use only public material from this repository. It will
not depend on company-private code, credentials, hardware, ROS 2, Isaac Lab, or
MuJoCo at runtime.

The target stack is Gazebo Harmonic (Gazebo Sim 8, SDFormat 14) with SDF 1.11.
Gazebo Classic 11 is an optional compatibility probe, not a release target.

## 2. Source baseline and trust boundary

The immutable implementation baseline is repository commit
`ecf530911284ba0e559f7a24dc222fd8e60d31ed` and the files under:

`source/h1_reach/h1_reach/assets/urdf/A1_2026/`

The source inventory is:

- `a1_r.urdf` and eight right-arm STL files;
- `a1_l.urdf` and eight left-arm STL files;
- `bimanual_stand/a1_bimanual_stand.urdf` and seventeen STL files;
- `model_parameters.yaml` for confirmed actuator and simulation metadata.

The generator treats URDF geometry, mass, center of mass, inertia, joint pose,
axis, and position limits as authoritative. It treats
`model_parameters.yaml` as authoritative for velocity and effort overlays.
It will not infer or invent damping, friction, contact parameters, controller
gains for the distributable model, sensors, transmissions, or grippers.

The independent arms and the bimanual stand are distinct CAD revisions. The
generator must never synthesize the bimanual model by combining the independent
arms, nor copy masses or position limits between revisions.

## 3. Considered approaches

### A. Runtime URDF loading

Gazebo can load URDF through libsdformat and convert it at runtime. This is the
smallest implementation, but it hides fixed-joint lumping, does not apply the
hardware parameter overlay, and does not produce the native `model.sdf`
required for a clear Fuel review. This approach is rejected for publication.

### B. ROS 2 package with `ros2_control`

A ROS 2 description and control package would be useful for downstream robot
applications, but it couples the asset to ROS and adds transmissions,
controllers, launch files, and version compatibility outside the requested
Gazebo/Fuel task. This approach is deferred.

### C. Deterministically generated native SDF packages

Generate native SDF 1.11 from the public URDFs with a repository-owned Python
tool, apply only the documented hardware overlay, export self-contained Fuel
directories, and validate them in Gazebo Harmonic. This is the selected
approach because every transformation can be inspected and reproduced while
the original assets remain unchanged.

## 4. Repository layout

The integration will add:

```text
gazebo/
  README.md
  pyproject.toml
  config/
    models.yaml
  templates/
    model.config.xml
    metadata.pbtxt
  worlds/
    right_arm_control_demo.sdf
    left_arm_control_demo.sdf
    bimanual_control_demo.sdf
  scripts/
    export_fuel_models.py
    validate_fuel_models.py
    run_smoke_tests.sh
    render_thumbnails.sh
  tests/
    test_source_assets.py
    test_exported_models.py
    test_deterministic_export.py
  generated-manifest.json
```

Generated upload directories are written to `dist/gazebo-fuel/` and are not
used as editable source. Each exported model has this shape:

```text
onerobotics_a1_right_arm/
  model.config
  model.sdf
  metadata.pbtxt
  README.md
  LICENSE
  NOTICE
  SOURCE_MANIFEST.json
  meshes/
  thumbnails/0.png
```

The equivalent left-arm and bimanual directories contain only their own
meshes. Export output must not contain symlinks, absolute paths, references to
the repository parent, or dependencies on another Fuel model.

Review archives are written to
`dist/gazebo-fuel/archives/<model-slug>-1.0.0.tar.gz`. Fuel CLI uploads use the
validated directories, not the archives.

The committed source tree avoids a second editable copy of all STL files. The
exporter copies the exact public source bytes into each self-contained output
directory. `SOURCE_MANIFEST.json` records source-relative paths, SHA-256
digests, the source repository URL, and the source commit.

`gazebo/pyproject.toml` defines a small Python 3.10+ tooling environment with
declared PyYAML and pytest dependencies. The export and pure-data checks must
run through that environment and must not import Isaac Lab.

## 5. Conversion rules

### 5.1 Names and topology

Public source link and actuated-joint names remain unchanged. Stable model
names are:

- `onerobotics_a1_right_arm`;
- `onerobotics_a1_left_arm`;
- `onerobotics_a1_bimanual_stand`.

The right arm exports eight links and seven revolute joints. The left arm's
empty URDF `world` link is represented by an SDF world anchor, while its eight
physical links and seven revolute joints remain. The bimanual export preserves
all seventeen physical links, fourteen revolute joints, the two fixed shoulder
joints (`joint_r0`, `joint_l0`), and their link names. It must not accept the
default libsdformat conversion that lumps `Link_r0` and `Link_l0` into the
base.

All three robots are fixed-base mechanisms. A fixed joint to the reserved SDF
`world` frame anchors the physical base. The model must not use
`<static>true>`, because that would prevent normal articulated dynamics.

### 5.2 Physical data

The exporter copies every URDF inertial tensor, inertial pose, visual pose,
collision pose, mesh scale, joint pose, joint axis, and position limit without
numerical rewriting beyond a stable decimal serialization.

Effort and velocity limits are overlaid per seven-joint arm from
`model_parameters.yaml`:

- joints 1--3: peak effort `26.859 N m`, rated speed
  `2.6179938779914944 rad/s`;
- joints 4--7: peak effort `5.975 N m`, rated speed
  `12.566370614359172 rad/s`.

The overlay applies independently to the right and left branches of the
bimanual robot. Raw bimanual `effort="0"` and `velocity="0"` placeholders must
never reach an exported SDF.

No automatic collision simplification is included in this first release.
Visual and collision geometry therefore retain the published STL bytes. The
README will disclose that these high-triangle meshes favor source fidelity
over contact performance. Collision optimization requires separately reviewed
derived assets and is outside this design.

### 5.3 Controllers and demonstrations

Fuel model packages remain controller-free and reusable. Gazebo-native
position-controller systems are injected only into generated demonstration
worlds. A demo commands a conservative, in-limit trajectory and verifies that
each actuated joint moves in the intended direction while all positions,
velocities, and simulation times remain finite.

The documented PD gains may seed demo-controller gains only after a smoke test
shows stable behavior. They are not represented as measured Gazebo dynamics
and are never embedded in the distributable robot model as physical truth.

## 6. Licensing and provenance

The three URDFs, thirty-three STL files, and `model_parameters.yaml` are
OneRobotics assets licensed under CC BY 4.0. Every package will explicitly
carry:

- the complete CC BY 4.0 license text;
- `Copyright © 2026 OneRobotics`;
- the public source repository and immutable commit;
- a precise changes statement covering URDF-to-SDF conversion, world anchoring,
  portable mesh paths, and documented effort/velocity overlays;
- retained SolidWorks-to-URDF Exporter attribution where present in the source;
- a statement that redistribution does not imply endorsement by OneRobotics,
  Isaac Lab, NVIDIA, Gazebo, or Open Robotics.

Fuel metadata will explicitly select CC BY 4.0. It must not rely on the Fuel
CLI's fallback license. The integration scripts remain under the repository's
BSD-3-Clause code license.

The source repository does not include a separate signed corporate
authorization document, and its review checksum set does not cover every
left-arm file. This implementation may prepare and validate upload packages,
but public upload under a company or organization identity remains an external
approval gate for an authorized OneRobotics representative.

## 7. Validation strategy

### 7.1 Pure-data tests

Tests run without Gazebo and fail on:

- a changed or missing public source file;
- unresolved mesh paths, path traversal, symlinks, or absolute paths;
- unexpected link or joint counts and names;
- non-finite or non-physical masses and inertia tensors;
- topology changes or fixed-joint lumping;
- effort/velocity values that do not match the hardware overlay;
- joint zero positions outside limits;
- missing CC BY metadata, license text, source URL, commit, or changes notice;
- mismatched manifest hashes;
- nondeterministic output from two clean exports.

### 7.2 SDFormat and Fuel checks

Each package must pass the Harmonic versions of:

```bash
gz sdf -k model.sdf
gz sdf -p model.sdf
gz fuel meta --config2pbtxt model.config
```

The parsed SDF is checked again after `gz sdf -p` so automatic parser upgrades
cannot silently change topology or limits.

### 7.3 Gazebo runtime checks

Server-only smoke tests launch each model with Gazebo Sim 8, advance physics,
and require:

- successful resource and mesh loading with no error-level log messages;
- stable world anchoring;
- the expected entities and joints;
- finite state throughout the run;
- no spontaneous joint-limit violation;
- successful conservative motion in the separate control demo.

At least one authentic headless server-side Ogre2 render is captured per model
and stored as `thumbnails/0.png`. A thumbnail is documentation evidence, not a
substitute for the server-only assertions.

### 7.4 Upload boundary

The implementation stops after producing validated directories and archives.
It does not upload to Fuel automatically. Upload requires a user's Gazebo Fuel
account token and, for organization-owned publication, explicit organization
permission. After an authorized upload, the documented final check downloads
each resource by Fuel URL and reruns the same validation against the downloaded
bytes.

## 8. Error handling and reproducibility

Generation is fail-closed. Unknown URDF elements, unsupported geometry,
duplicate names, invalid XML, missing YAML fields, hash drift, and unexpected
fixed-joint conversion are fatal. The tool never silently substitutes a
default physical value.

All source traversal is allow-listed in `gazebo/config/models.yaml`. Output is
created in a temporary directory, validated, and only then moved into the
requested destination. A failed run leaves the previous valid output intact.
Sorted paths, normalized XML formatting, stable JSON serialization, and fixed
archive timestamps make repeated exports byte-for-byte reproducible.

## 9. Acceptance criteria

The work is complete only when:

1. all three self-contained Fuel directories and reproducible archives exist;
2. all source, topology, physical-overlay, provenance, and determinism tests
   pass;
3. all three packages pass SDFormat and Fuel metadata parsing under Harmonic;
4. all three pass server-only Gazebo runtime tests;
5. right, left, and both bimanual arms demonstrate conservative commanded
   motion without non-finite state or limit violations;
6. each model has an authentic Gazebo thumbnail;
7. documentation gives exact build, validation, local insertion, private Fuel
   upload, public Fuel upload, and post-download verification commands;
8. no private code, private path, credential, or unsupported ownership claim is
   present in source or generated output.
