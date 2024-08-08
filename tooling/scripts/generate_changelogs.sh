#!/bin/bash

set -o pipefail

# Enter the repository's root directory
pushd "$(dirname "$0")/../../" >> /dev/null || exit 1

# Generate the change logs
catkin_generate_changelog

# Return to the original directory
popd >> /dev/null || exit 1
