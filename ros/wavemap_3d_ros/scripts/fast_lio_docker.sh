#!/bin/bash
set -e

# Run the script's argument in an ephemeral fast_lio container
docker run --rm --net=host fast_lio "$@"
