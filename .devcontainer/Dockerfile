ARG VERSION=latest

FROM ghcr.io/ethz-asl/wavemap:${VERSION}

# Install curl [needed by CLion]
# hadolint ignore=DL3008
RUN apt-get update && apt-get install -yq --no-install-recommends curl && rm -rf /var/lib/apt/lists/*

# Build the package
RUN catkin build wavemap_all
