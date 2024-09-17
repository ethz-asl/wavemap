# Select a git branch, tag or commit
ARG WAVEMAP_VERSION=main

FROM debian:11.10

ARG WAVEMAP_VERSION

# hadolint ignore=DL3008
RUN apt-get update && \
    apt-get install -q -y --no-install-recommends \
    cmake build-essential git ca-certificates \
    libeigen3-dev libgoogle-glog-dev libboost-dev && \
    rm -rf /var/lib/apt/lists/*

RUN git clone --branch ${WAVEMAP_VERSION} https://github.com/ethz-asl/wavemap.git

WORKDIR /wavemap/library/cpp
RUN cmake -S . -B build && \
    cmake --build build -j "$(nproc)" && \
    cmake --install build
