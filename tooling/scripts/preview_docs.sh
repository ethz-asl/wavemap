#!/bin/bash

set -o pipefail

# Enter the documentation directory
pushd "$(dirname "$0")/../../docs" >> /dev/null || exit 1

# Build the docs
rm -rf _doxygen_cpp _doxygen_ros1 cpp_api _build
doxygen Doxyfile_cpp
doxygen Doxyfile_ros1
sphinx-build -b html . _build/html

# Open in browser
xdg-open _build/html/index.html

# Return to the original directory
popd >> /dev/null || exit 1
