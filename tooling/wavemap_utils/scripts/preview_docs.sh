#!/bin/bash

set -o pipefail

cd "$(rospack find wavemap_common)"/../../docs/ || exit 1
make html
cd _build/html/ || exit 1
xdg-open http://0.0.0.0:8000/
python3 -m http.server
