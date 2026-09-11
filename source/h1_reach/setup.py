# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Installation script for the 'h1_reach' python package."""

import os

import toml
from setuptools import find_packages, setup

# Obtain the extension data from the extension.toml file
EXTENSION_PATH = os.path.dirname(os.path.realpath(__file__))
# Read the extension.toml file
EXTENSION_TOML_DATA = toml.load(os.path.join(EXTENSION_PATH, "config", "extension.toml"))

# Installation operation
setup(
    name="h1_reach",
    packages=find_packages(),
    author=EXTENSION_TOML_DATA["package"]["author"],
    maintainer=EXTENSION_TOML_DATA["package"]["maintainer"],
    url=EXTENSION_TOML_DATA["package"]["repository"],
    version=EXTENSION_TOML_DATA["package"]["version"],
    description=EXTENSION_TOML_DATA["package"]["description"],
    keywords=EXTENSION_TOML_DATA["package"]["keywords"],
    install_requires=[],
    license="BSD-3-Clause AND CC-BY-4.0",
    license_files=[
        "LICENSE",
        "ASSET_LICENSE_STATUS.md",
        "THIRD_PARTY_NOTICES.md",
        "LICENSES/CC-BY-4.0.txt",
        "LICENSES/IsaacLab-BSD-3-Clause.txt",
    ],
    include_package_data=True,
    package_data={"h1_reach": ["assets/**/*"]},
    python_requires=">=3.12",
    project_urls={
        "Company Website": "http://www.onerobot.com/",
        "Source": EXTENSION_TOML_DATA["package"]["repository"],
    },
    classifiers=[
        "Natural Language :: English",
        "Programming Language :: Python :: 3.12",
        "Isaac Sim :: 6.0.0",
    ],
    zip_safe=False,
)
