# Select a git branch, tag or commit
ARG WAVEMAP_VERSION=main

FROM alpine:3.20

ARG WAVEMAP_VERSION

# hadolint ignore=DL3018
RUN apk add --no-cache cmake build-base git eigen-dev glog-dev boost-dev

# hadolint ignore=DL3059
RUN git clone --branch ${WAVEMAP_VERSION} https://github.com/ethz-asl/wavemap.git

WORKDIR /wavemap/library/cpp
RUN cmake -S . -B build && \
    cmake --build build -j "$(nproc)" && \
    cmake --install build
