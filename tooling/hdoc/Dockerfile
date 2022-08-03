FROM ubuntu:20.04

ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Install general dependencies
# hadolint ignore=DL3008
RUN apt-get -q -y update && \
    apt-get install -q -y --no-install-recommends \
      lsb-release wget software-properties-common gnupg ca-certificates \
      build-essential meson ninja-build cmake xxd \
      groff unzip pkg-config libssl-dev git && \
    rm -rf /var/lib/apt/lists/*

# Install Clang and related tooling
RUN wget -qO - https://apt.llvm.org/llvm-snapshot.gpg.key | apt-key add - && \
    echo "deb http://apt.llvm.org/focal/ llvm-toolchain-focal-15 main" >> /etc/apt/sources.list.d/docker.list && \
    echo "deb-src http://apt.llvm.org/focal/ llvm-toolchain-focal-15 main" >> /etc/apt/sources.list.d/docker.list
# hadolint ignore=DL3008,DL3015
RUN apt-get -y update && \
    apt-get install -q -y clang-15 libclang-common-15-dev libclang-15-dev lld-15 && \
    rm -rf /var/lib/apt/lists/*

# Clone and build hdoc
RUN git clone --branch 1.3.0 --depth 1 https://github.com/hdoc/hdoc.git
WORKDIR /hdoc
RUN meson build && \
    ninja -C build
