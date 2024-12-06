#!/bin/bash

set -o pipefail

flags=""
if [ "$CI" ]; then
  flags="--no-status --force-color"
fi

# Enter wavemap's home directory
pushd "$(dirname "$0")/../../" >> /dev/null || exit 1

# Build all ROS1 packages, including the ROS examples
# shellcheck disable=SC2086
catkin build wavemap_all $flags
# shellcheck disable=SC2086
catkin build wavemap_all $flags --no-deps --catkin-make-args tests

# Run all ROS1 tests, which indirectly also runs the C++ API's tests
pushd "$(catkin locate --devel wavemap)/../.." >> /dev/null || exit 1
all_tests_passed=1
for f in lib/wavemap*/test_*; do
  $f --gtest_color=yes || all_tests_passed=0
done
popd >> /dev/null || exit 1

if [ $all_tests_passed -ne 1 ]; then
  echo "Not all tests passed!"
  exit 1
fi

# Build and test the Python API
python3 -m pip install -v ./library/python/
pytest -rAv ./library/python/

# Build the C++ examples
pushd examples/cpp >> /dev/null || exit 1
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
