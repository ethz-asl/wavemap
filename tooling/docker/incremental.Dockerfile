ARG VERSION=latest

FROM ghcr.io/ethz-asl/wavemap:${VERSION}

# Build the package
RUN catkin build wavemap_all
