#!/bin/bash

set -o pipefail

# Enter the documentation directory
pushd "$(dirname "$0")/../../docs" >> /dev/null || exit 1

# Build the docs and check all the links
sphinx-build -b linkcheck . _build/html

# Return to the original directory
popd >> /dev/null || exit 1
