#!/bin/bash

set -o pipefail

# Change directory to the repository root
cd "$(rospack find wavemap_utils)"/../../.. || exit 1

# Build the docs
sphinx-build -b html docs docs/_build/html

# Open in browser
xdg-open docs/_build/html/index.html
