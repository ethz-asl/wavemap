#!/usr/bin/env bash

host_data_dir_path=/home/$USER/data

xhost + local:docker

if docker info | grep "Runtimes.*nvidia.*" &>/dev/null; then
  docker run --rm -it \
    -e DISPLAY="${DISPLAY}" \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --runtime=nvidia --gpus all \
    -v "$host_data_dir_path":/home/ci/data:ro \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=graphics \
    ghcr.io/ethz-asl/wavemap:main "$@"
else
  docker run --rm -it \
    -e DISPLAY="${DISPLAY}" \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$host_data_dir_path":/home/ci/data:ro \
    ghcr.io/ethz-asl/wavemap:main "$@"
fi
