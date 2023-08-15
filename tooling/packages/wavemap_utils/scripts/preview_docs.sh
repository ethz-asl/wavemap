#!/bin/bash

set -o pipefail

# Change directory to the repository root
cd "$(rospack find wavemap)"/../.. || exit 1

# Build the docs
sphinx-build -b html docs docs/_build/html

# Start a small server and open in browser
xdg-open http://0.0.0.0:8000/
python3 -m http.server --directory docs/_build/html/
