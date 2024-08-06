#!/bin/bash

set -o pipefail

# Change directory to the repository root
cd "$(rospack find wavemap_utils)"/../../../docs || exit 1

# Build the docs
rm -rf _doxygen_cpp _doxygen_ros1 cpp_api _build
doxygen Doxyfile_cpp
doxygen Doxyfile_ros1
sphinx-build -b html . _build/html

# Open in browser
xdg-open _build/html/index.html
