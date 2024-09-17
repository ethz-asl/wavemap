# Select a git branch, tag or commit
ARG WAVEMAP_VERSION=main

FROM debian:11.10

ARG WAVEMAP_VERSION

# hadolint ignore=DL3008
RUN apt-get update && \
    apt-get install -q -y --no-install-recommends \
    git build-essential python3-dev python3-pip && \
    rm -rf /var/lib/apt/lists/*

RUN git clone --branch ${WAVEMAP_VERSION} https://github.com/ethz-asl/wavemap.git

WORKDIR /wavemap/library/python
# hadolint ignore=DL3042
RUN pip3 install .
