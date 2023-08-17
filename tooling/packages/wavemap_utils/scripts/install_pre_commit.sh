#!/bin/bash

set -o pipefail

command_exists() {
  command -v "$1" &>/dev/null
  return_value="$?"
  [ "$return_value" -eq 0 ]
}

install_if_missing() {
  if command_exists "$1"; then
    echo "Required command '$1' already available."
  else
    first_arg=$1
    shift
    echo "Required command '$first_arg' is not yet available. Installing..."
    for cmd in "$@"; do
      echo "--executing: '$cmd'"
      $cmd
    done
  fi
}

# Install pre-commit
install_if_missing "pip3" "sudo apt-get install python3-pip" "source $HOME/.profile"
install_if_missing "pre-commit" "pip3 install pre-commit"

# Install system packages we use in our pre-commit hooks
install_if_missing "clang-format-11" "sudo apt-get install clang-format-11"
install_if_missing "cpplint" "pip3 install cpplint"
install_if_missing "cppcheck" "sudo apt-get install cppcheck"
install_if_missing "hadolint" \
  "sudo apt-get install wget" \
  "sudo wget -O /bin/hadolint https://github.com/hadolint/hadolint/releases/download/v2.8.0/hadolint-Linux-x86_64" \
  "sudo chmod +x /bin/hadolint"
install_if_missing "xmllint" "sudo apt-get install libxml2-utils"

# Enable pre-commit for wavemap
pushd "$(dirname "$0")" >>/dev/null || exit 1 # Enter wavemap's home directory
echo "Enabling pre-commit for wavemap"
pre-commit install --install-hooks
popd >>/dev/null || exit 1
