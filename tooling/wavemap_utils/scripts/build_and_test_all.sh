#!/bin/bash

set -o pipefail

flags=""
if [ "$CI" ]; then
  flags="--no-status --force-color"
fi

# shellcheck disable=SC2086
catkin build wavemap_all $flags
# shellcheck disable=SC2086
catkin build wavemap_all $flags --no-deps --catkin-make-args tests

pushd "$(catkin locate --devel wavemap)/../.." >>/dev/null || exit 1
all_tests_passed=1
for f in lib/wavemap*/test_*; do
  $f --gtest_color=yes || all_tests_passed=0
done
popd >>/dev/null || exit 1

if [ $all_tests_passed -ne 1 ]; then
  echo "Not all tests passed!"
  exit 1
fi
