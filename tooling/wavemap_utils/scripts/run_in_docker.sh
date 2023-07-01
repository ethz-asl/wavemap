#!/usr/bin/env bash

xhost + local:docker
docker run --rm -it \
  -e DISPLAY="${DISPLAY}" \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  ghcr.io/ethz-asl/wavemap:main \
  /bin/bash
