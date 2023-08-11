#!/bin/bash

set -o pipefail

pushd "$(rospack find wavemap)"/../../ >>/dev/null || exit 1

catkin_generate_changelog

popd >>/dev/null || exit 1
