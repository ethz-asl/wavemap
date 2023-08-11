#!/bin/bash

set -o pipefail

# Change directory to the repository root
cd "$(rospack find wavemap)"/../.. || exit 1

# Build the docs and check all the links
sphinx-build -b linkcheck docs docs/_build/html
