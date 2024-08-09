ARG WAVEMAP_TAG=main

FROM alpine:3.20

ARG WAVEMAP_TAG

# hadolint ignore=DL3018
RUN apk add --no-cache cmake build-base git eigen-dev glog-dev boost-dev

# hadolint ignore=DL3059
RUN git clone --branch ${WAVEMAP_TAG} https://github.com/ethz-asl/wavemap.git

WORKDIR /wavemap/library/cpp
RUN cmake -S . -B build && \
    cmake --build build -j "$(nproc)" && \
    cmake --install build
