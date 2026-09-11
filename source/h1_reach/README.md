# h1_reach package

This directory is the Python packaging root for the external **OneRobotics A1 — Isaac Lab Integration** project.

This package is the legacy `release/3.0.0-beta2` external extension. The
current `develop` contribution, public `IsaacContrib-*` task IDs, and exact
review snapshot are documented in the repository-root README; Isaac Lab uses
the `A1_2026` assets here without importing this legacy task implementation.

Project documentation, installation and validation commands are maintained in the [repository README](https://github.com/katazen/onerobot_h1#readme).

The packaged distribution contains code under BSD-3-Clause and OneRobotics A1 robot assets under CC BY 4.0. The wheel and source distribution include the applicable license texts, asset scope and third-party notices.

The current asset manifest contains 77 robot-model files (66 STL, six URDF,
and five MJCF/XML files) plus the CC BY 4.0 A1 2026 model/control metadata YAML.
The A1 2026 inventory itself contains two independent single-arm URDFs with 16
meshes and a distinct whole-robot bimanual stand URDF with 17 meshes. The
revisions are separate CAD sources and do not include a gripper. The packaged
asset README is documentation and is not counted as a robot-model file.
